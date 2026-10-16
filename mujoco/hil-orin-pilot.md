# HIL 跨机仿真：网线连接的 Orin Nano 上跑 pilot 全栈

本机只跑 mujoco 仿真本体；tinynav 侧（pilot 全家桶）由独立编排启动——要么本机容器，
要么网线/USB 直连的 Orin Nano 上的 `tinynav-pilot` compose 栈。两侧通过 looper 契约
话题（`/camera/camera/*`）+ 底盘 rt/* 话题跨网协作，全 CycloneDDS。本文记录这套
"网线连 Orin 仿 pilot"形态的环境配置、编址和实测开销。

架构边界（谁起什么）：

- **mujoco 仿真**：`docker/run-mujoco-sim.sh` 只起 sim 相关容器（dog.py 物理与底盘脸 +
  contract_face.py 契约面），不含任何 tinynav 进程。
- **tinynav 侧**：pilot 仓库的 compose 栈（core_runtime / chassis / sensor / nav 动作
  服务器 / frontend 等 15 容器），独立脚本/编排启动，与本仿真进程零耦合。
- **C++ 桥**（`src/tinynav_cpp`，`tinynav_bridge --hil`）：开销测试工具——与 Python 桥
  （pilot 的 `looper_bridge_node.py`）并行对测用的逐位等价移植。rig 生产路径仍是
  Python 桥；测试方法见文末。

## 环境配置

本机（x86 工作站，GPU 渲染）：

```bash
MJSIM_DDS_IFACE=<网线网卡, 如 enxec1ac311add3> \
  docker/run-mujoco-sim.sh dog -- --profile go2 --iface <同上> --scene map3 --contract
```

- `MJSIM_DDS_IFACE` 生成钉网卡的 Cyclone XML，覆盖镜像里烤死的 loopback 配置；不钉
  则跨机发现不到。
- `--iface` 是 dog.py 底盘 SDK 脸监听的网卡（底盘 rt/* 与 sport RPC 走它）。
- `--contract` 起 contract_face（gausscam 渲染 splat + 发布 looper 契约），bench/CI
  形态不加。

Orin Nano（jetpack + docker，无宿主 ROS）：

- 部署树 `~/navcore-deploy/pilot`（dm 用户；/data 曾 700 root，已放开）。 fleet 编排
  原样复用，机器级差异全部收在 `docker-compose.override.yml`（**机器本地，不提交**）：
  - 所有上总线服务 `FASTDDS_BIND_REQUIRED: enP8p1s0`（网线网卡，替代 fleet 的 looper
    USB 网卡默认值）；
  - traces 停用（8GB Nano 纪律，`--profile traces` 可选开）；traces/webrtc/turn/
    snapshot 复用 `navcore-tinynav:local`（机器上没有 stock ros:humble 镜像且离线拉不到）；
  - frontend 用 `python:3.11-slim` 服 rsync 过去的预编译包（flutter 首建要走 wifi，慢）；
  - sensor 服务 `LOOPER_WAIT_SEC: "5"`——wrapper 默认等真 looper 盒子 180s×2 才强起桥，
    本 rig 没有盒子，白白死 6 分钟。
- `.env`：`ROBOT_TYPE=go2`、`CHASSIS_NET_IFACE=enP8p1s0`。
- 宿主机 shell 没装 ROS：调试探针一律 `docker exec` 进对应容器跑（core-runtime 有
  rclpy 全家桶）。若宿主装了 Humble，才需要 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` +
  钉网卡的 `CYCLONEDDS_URI`（dm/nvidia 的 bashrc 与 `~/hil_env.sh` 已备）。

管理通道与数据通道分离：ssh 走 tailscale（`ssh orin`）或 USB gadget
（192.168.55.1），DDS 数据只走网线。

## 网段

拟真编址完全对齐真狗 123 段：

| 端点 | 地址 | 载体 |
|---|---|---|
| 本机 | `192.168.123.99` | NM 连接 `dog123`（manual 静态；兼真狗网线直连两用） |
| Orin | `192.168.123.103` | `enP8p1s0` 静态 + `never-default`（重启不丢） |

- **`.161` 永远不能用**——那是狗的运控计算单元地址（官方禁占）。
- 点对点网线无 DHCP：两端必须同段且都静态，否则 Orin 掉 169.254、本机 manual 不跟随，
  互不相通。
- 例行检查：`ip route show | grep -c 192.168.123` 必须是 **1**（多出来就是别的连接
  也在路由 123 段，DDS 会走错口）。
- DDS 纪律：全 Cyclone（cyclonedds 0.10.x 单进程只能建一次 domain 0，宇树 SDK 与
  rclpy 同进程必死——底盘/emu 是纯 SDK 进程，pilot 是纯 rclpy 进程，两边都不混）；
  每个上总线的容器由 `deploy/dds_bind.py` 钉网卡；**同名契约话题全系统只容得下一个
  发布源**（起第二套 emu 会混流出"双轨迹交替"流，pilot vio 守门每秒告警——排障签名：
  `/slam/odometry_visual` 采样见两个恒定不同的 xy 交替）。

## 程序开销指标（实测区间）

本机（多核桌面 CPU，单进程百分比）：

| 进程 | CPU |
|---|---|
| contract_face.py（gausscam splat 渲染 + 契约发布） | ~37% |
| dog.py（mujoco 物理 + 底盘 SDK + 策略） | ~23% |

Orin Nano（6 核 / 8GB，容器级 `docker stats`，emu 站立待机态）：

| 容器 | CPU | 说明 |
|---|---|---|
| navcore-sensor（Python 桥） | ~68% | `looper_bridge_node.py` ~53% + wrapper ~6% |
| core-runtime（pilot 行为树） | ~20% | 2Hz tick + vio 守门 + web 面 |
| navcore-server-snapshot | ~13% | 快照环形录制，1.5GB 缓存是设计值 |
| navcore-chassis | ~10% | unitree_control：SDK lowstate/RPC + rt 桥 |
| 六个动作服务器（空闲） | 各 3-6% | 一容器一服务器的架构固定税，合计 ~25-30% |
| host 整体 | ~30% busy | 内存 ~3.7/7.6GB |

- **C++ 桥对比**（`tinynav_bridge --hil`，逐位等价）：~12%，Python 桥的 ~1/6。输出
  parity 实测：odometry 位置/速度偏差 0、四元数 |dot|=1.0、深度/关键帧图 md5 全对；
  关键帧 stamp 集除各自首帧锚点外全同。
- **nav 三进程**（`map_node`+`planning`+`cmd_vel_control`）起后 server-nav 容器
  ~164%；无图时 pilot 的 `WaitNavMap` 守门让 nav 干净等待（不拉进程、零开销）。
- 已知底噪：planning njit 首任务冷编译 ~21s；emu imu 实发 70-160Hz 低于契约 200Hz
  （暂无消费者，未调）。

## 验证工具

- 全链探针：`tools/probes/probe_first_msg.py <topic> <type>`（ros2 CLI 在 Cyclone 钉卡
  下常瞎，以数据面为准）。
- C++/Python 桥对测：C++ 侧输出 `--ros-args -r /slam/...:=/slamtest/...` 重映射并行
  跑，`bridge_hil_build/parity_probe.py`（Orin `~/bridge_hil_build`）按 stamp 逐条
  对质；测完停 C++ 容器即可，rig 无痕。
- 机器狗状态一键：`gazebo/dog_state.sh`（gz 线的，mujoco 线看 `dog.py` 日志或 viser）。

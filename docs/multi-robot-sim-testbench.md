# 多机型适配与 mujoco 仿真测试台设计（AS2 / 四足 / 人形）

状态: 已实现（sim 侧 emu + pilot 侧 as2 适配）· 评审与实作结论已回填
范围: tinynav-pilot 底盘驱动层迁移 + tinynav-sim/mujoco 测试台（本仓）
关联: `tinynav-pilot/docs/pilot-dock.md` §5/§7 · 实测依据 `as2-topic-audit.md`（建议后续拷入本仓 docs/）

**实作对评审结论的落地**：
- as2 `gait_assertion = None`（BalanceStand 是切换开关 + 驱动每次运动启动 re-arm → 无条件断言会把 NORMAL_WALK 打回 STAND_UP；已按"断言只留在 stand_steps"实现，emu 回归用例覆盖）。
- `RobotType(str, Enum)`（运行环境 py3.10，无 StrEnum）。
- g1 的 odom/battery 均为 None（现状即无）；lekiwi 不进枚举，`get_profile` 返回 None → charge_watch 软拒绝。
- AS2_CONFIG **不注册**进 robot_specs 查表（实测几何回填前 `ROBOT_TYPE=as2` 在规划侧 loud failure）。
- AS2 充电阈值独立成 BatterySpec 数据（默认 -100mA，保守；go 的 -1000mA 不可复用——as2 静止 -970mA 贴脸），上桩校准后启用 docking。
- conformance 判据 = curated must/must-not 清单（非全总线 diff），DCPSPublication 枚举实现（`mujoco/robot_emu/conformance.py`，go2/as2 均 PASS）。

---

## 0. TL;DR

1. **抽象**：把散在 `unitree_control.py` / `charge_watch.py` / `robot_specs.py` 的机型分支，收敛为 **`RobotType` 枚举 + `RobotProfile` 注册表**（声明式机型画像，四足/轮足/人形统一），驱动层主体不再出现机型 if。
2. **迁移**：六个机型（go2/go2w/b2/b2w/g1/**as2**）画像落地 + SDK 升级到官方 main + 电量/里程计源按实测结论换绑。pilot 消费面（/battery、/robot_status、/cmd_vel、/service/command）零改动。
3. **测试台**：在 **tinynav-sim/mujoco** 内嵌 **robot_emu（狗/人形模式）**——仿真身体沿用 go2 资产，**DDS 话题面与真机逐项对齐**（类型/频率/FSM 门控），用 conformance diff 做客观验收。L0 单测 → L1 仿真台 → L2 真机冒烟三层，仿真台无硬件依赖可进 CI。

---

## 1. 背景与目标

AS2（宇树紧凑型工业四足）接入 tinynav-pilot。它打破了两个既有假设：`is_quadruped` 旗标的三重含义不再一致（是四足、但没有 go2 的 ClassicWalk API、lowstate 是 hg 族且无电池字段）；同时后续人形机型（g1 之外）会继续进入机队。目标：

- 新机型接入 = **只加一份 profile + 一份 emu 规则**，不改驱动层主体；
- 四足/轮足/人形由同一套抽象承载；
- 办公室内有一条**话题面与真机对齐**的全链路仿真测试台。

## 2. 实测事实基础（AS2 审计，v2）

| 事实 | 依据 |
|---|---|
| `rt/sportmodestate` 498.5Hz、`rt/lf/sportmodestate` 20Hz，字段与 go2 同构（含 imu_state.rpy/quat、range_obstacle、stamp） | DDS 订阅实测 |
| `rt/lowstate` 存在但类型为 **unitree_hg LowState_**（35 电机，**无 bms 字段**，同 G1） | DCPSPublication + hg 订阅 |
| 电量源 = `rt/lf/bmsstate`（hg BmsState_：soc/current/soh/温度），**无 go 版 status 充电位** | 同上，soc=88% 实测 |
| **无任何独立里程计话题**（robot_odom 不存在）；唯一里程计源 = sportmodestate（position+velocity+rpy+stamp） | 61 话题全枚举 |
| FSM 门控：`Move` 仅 NORMAL_WALK(3) 接受；STAND_UP(2) 拒收（服务端 -1）；`BalanceStand()` 是切换开关；`StopMove` 不切 FSM | 真机 RPC 实测 |
| ClassicWalk 是 go2 系专属 API，**AS2 无此方法且 id 撞 BipedStand** | 官方 as2 sport_api 全量比对 |
| DDS 类型不匹配 = 静默收不到（无报错）；ros2 topic list 会漏报 | v1→v2 审计修订 |

点云：`rt/utlidar/cloud_livox_mid360` 9.7Hz / 77k 点/帧 / ~20MB/s，frame_id=`hesai_lidar`。

## 3. 抽象设计

### 3.1 RobotType 枚举（收敛现状，值不变）

仓内现状是字符串元组约定（`unitree_control._SUPPORTED_ROBOT_MODELS`、`robot_specs` 的 `{TYPE}_CONFIG` globals 查表），**没有正式枚举**。收敛为：

```python
# tinynav/platforms/robot_profiles.py
class RobotType(StrEnum):
    GO2 = "go2";  GO2W = "go2w"      # 四足 / 轮足（复用 go2 sport API）
    B2 = "b2";    B2W = "b2w"        # 工业四足 / 轮足（多 SwitchGait）
    G1 = "g1"                        # 人形（loco FSM API, hg 低层, 无电池上报）
    AS2 = "as2"                      # 紧凑工业四足（sport API 子集, hg 低层, bmsstate 独立话题）
```

值与现有 `ROBOT_TYPE` 字符串完全一致——`.env`、docker-compose、robot_specs 的 `{TYPE}_CONFIG` 查表全部兼容。`family` 是 profile 的属性而非枚举维度：`GO2.family = QUADRUPED`、`GO2W = QUADRUPED_WHEELED`、`G1 = HUMANOID`。

### 3.2 RobotProfile（声明式机型画像）

```python
@dataclass(frozen=True)
class RobotProfile:
    type: RobotType
    family: Family                                # QUADRUPED | QUADRUPED_WHEELED | HUMANOID
    motion_api: Literal["sport", "loco"]          # 客户端族：go2/b2/as2 用 sport；g1 用 loco
    client_factory: Callable[[], object]          # 延迟 import，返回未 Init 的 client
    # -- 行走断言：进入"可行走状态"的调用（读线程不可阻塞的应答式 RPC 交给 gait worker）
    gait_assertion: Literal["classic_walk", "balance_stand", "loco_start"] | None
    needs_gait_worker: bool                       # classic_walk 慢(10s)才需要; as2 的 balance_stand 秒回
    # -- 状态面
    lowstate: tuple[str, str] | None              # (IDL 包名, topic)：go2=('unitree_go','rt/lowstate')
                                                  #   as2/g1=('unitree_hg','rt/lowstate')
    lowstate_has_bms: bool                        # go 族 True；hg 族 False
    battery: BatterySpec | None                   # go 族=None(吃 lowstate.bms)
                                                  #   as2=BatterySpec('rt/lf/bmsstate','unitree_hg',policy=CurrentSign)
    odom: Literal["robot_odom", "sportmodestate"]  # as2 无 robot_odom → sportmodestate 重铺
    # -- 动作序列（动作名表, getattr(client, name) 执行）
    stand_steps: tuple[str, ...]                  # go2: ('StandUp','BalanceStand','ClassicWalk')
                                                  #   as2: ('StandUp','BalanceStand'); g1: ('Damp','Squat2StandUp')
    sit_steps: tuple[str, ...]                    # go2/b2: ('StandDown',); g1: ('StandUp2Squat',)
```

要点：

- **g1 今天的不对称被原样吸收**（loco API、hg 无 bms、sit=StandUp2Squat），`is_quadruped`/`has_switch_gait`/`_QUADRUPEDS` 旗标全部退役，SwitchGait 进 b2 的 `stand_steps` 尾部。
- **人形兼容不是加字段，而是字段选值**：后续人形（如带臂型号）= 新 profile + 若需新 API 则在 `motion_api` 加枚举值 + emu 规则加一张 FSM 表；驱动层主体零改动。arm_sdk 等 human-only 通道由 family 门控，四足 profile 不携带。
- `robot_specs.py` 平行加 `AS2_CONFIG`（几何**占位 + 大字警告**，实测后回填——抄错足印=碰撞体错误，122 b2 的前车之鉴）。

### 3.3 两个策略接口

```python
class ChargingPolicy(Protocol):
    def judge(self, status: int | None, median_ma: float | None) -> str: ...
class StatusBitPolicy: ...    # go 族: status 位主判 + 电流中位数判外供电（现逻辑原样）
class CurrentSignPolicy: ...  # as2: 无 status 位，电流符号/阈值判定（复用 EXTERNAL_POWER_CURRENT_MA 窗口机制）

class OdomSource(Protocol):   # 状态回传→/unitree/odometry
    # robot_odom: 现逻辑原样 | sportmodestate: position+imu_state.rpy→quat+stamp 重铺（as2）
```

`charge_watch.py` 的 `note_lowstate` 注入缝、`Sample` 结构、staleness 规则全部保留——现有测试语义不变，只加策略分支。

## 4. tinynav-pilot 迁移改动表（精简版）

| 文件 | 变更 |
|---|---|
| `tinynav/platforms/robot_profiles.py` | **新增**：枚举 + 六机型画像 + 两个策略类（~200 行） |
| `tinynav/platforms/unitree_control.py` | 8 处改造：能力元组→查表；`_build_sport_client`/`_lowstate_type_and_topic` 删除；启动/gait/stand 三处 ClassicWalk→profile 断言；odom 双源；bms 门控（行号锚点见评审记录） |
| `pilot/backend/charge_watch.py` | 策略化：订阅 topic/类型/字段提取/判定从 BatterySpec 取 |
| `tinynav/core/robot_specs.py` | +`AS2_CONFIG` 占位 |
| `tinynav/pyproject.toml` + `uv.lock` | SDK pin → 官方 main sha（fork 可退役；40 新 commit 已核实不碰 pilot 使用面） |
| 测试 | `test_unitree_control`：mock 从 profile 注入，b2 现有用例零回归 + as2 断言；`test_charge_watch`：+CurrentSignPolicy 组；新增 profiles 表驱动测试 |
| 明确不动 | drivers.yml（已参数化）/ Dockerfile / pilot 全部消费面（topics.py、backend domains、node_manager、cmd_vel_control、planning_node 只吃 ROBOT_CONFIG）/ simulator_control / localize |

## 5. mujoco 测试台（robot_emu，本仓）

**方向已定**：实现放 `tinynav-sim/mujoco/`，仿真身体沿用 go2 资产，**话题面与真机对齐**——让 mujoco sim 长出一张真机的 DDS 脸，生产驱动代码原封不动连上来。

### 5.1 链路

```
pilot 栈 → cmd_vel_control → rt/cmd_vel → unitree_control（生产代码, --profile as2）
    → sport RPC rt/api/sport/request
┌─ mujoco sim（go2 身体 + as2 话题语义）──────────────────────┐
│ robot_emu: RPC 应答 + 机型 FSM 门控（STAND_UP 拒 Move 等）       │
│   └→ Move → 注入现有 /cmd_vel 执行通路（RosCmdSource, 零改 plant）│
│ 状态回传（物理引擎真值直填）:                                    │
│   rt/sportmodestate 500Hz · rt/lowstate(go/hg 按 profile)       │
│   rt/lf/bmsstate(as2, 可注入在桩) · rt/utlidar/robot_odom(仅go/b2)│
└────────────────────────────────────────────────┘
```

### 5.2 模块（`mujoco/robot_emu/`）

| 模块 | 职责 | 抽象要求 |
|---|---|---|
| `fsm.py` | **纯逻辑无 DDS**：机型 FSM 迁移表（状态集、api_id→迁移/拒绝码）。AS2: STAND_UP 拒 Move 回 -1、BalanceStand 切 NORMAL_WALK、StopMove 不切；go2: Move 常收、ClassicWalk 有效；humanoid: loco FSM id 空间 | 迁移表数据驱动，family 之间不共享迁移逻辑只共享执行器 |
| `service_<api>.py` | `sport_service`（rt/api/sport/request 应答）/ `loco_service`（g1 族，rt/api/loco/request）——按 `motion_api` 选 | 应答器骨架共享，api 分派表按 profile |
| `state_feed.py` | 物理状态→话题发布：sportmodestate/lowstate/bmsstate/robot_odom **按 profile 的话题面清单发布**（as2 不发 robot_odom） | 话题面清单即 3.2 profile 字段的镜像 |
| `scenarios.py` | 场景注入：在桩充电（bms 电流翻转）、障碍拒动（error_code+range_obstacle）、cmd 断流 | 测试钩子，独立话题/参数 |
| 入口 `dog.py`（独立于 hil.py） | `python3 dog.py --profile as2 --iface lo` | hil 是感知在环，语义不同不混用 |

### 5.3 Conformance 检查（"话题对齐真机"的客观判据）

复用 `as2_topic_enum.py`（DCPSPublication 全量枚举）：**对 sim 跑枚举，与真机审计清单逐 (topic, type) diff**。as2 模式必须出现 `rt/sportmodestate [unitree_go]`、`rt/lowstate [unitree_hg]`、`rt/lf/bmsstate [unitree_hg]`，必须不出现 `rt/utlidar/robot_odom`；go2 模式反之。**DDS 类型不匹配静默收不到**的坑在台架层面永久封死。

### 5.4 测试矩阵

| 用例 | 跑法 | 断言 |
|---|---|---|
| 话题面 conformance | enum diff vs 真机清单 | 逐 (topic,type) 一致 |
| AS2 FSM 门控回归 | dog.py as2 + unitree_control | BalanceStand 后 Move 才成功；启动即 Move 必须 -1（防 ClassicWalk 回潮） |
| go2/b2 零回归 | dog.py go2/b2 + 现有 test_unitree_control | 现有用例原样绿 |
| 电量/充电 | 注入在桩 | charge_watch charging/外供电判定 |
| odom 源 | as2 模式 | /unitree/odometry vs 物理真值 |
| 全链路 | pilot 发目标 → 到位 | ChassisWatch 无停滞告警、trace 正常 |
| （远期）人形 | dog.py g1 + loco_service | loco FSM 序列、hg 状态面 |

### 5.5 边界（明确不做）

- **不做物理起身**：StandUp/StandDown 是 FSM 迁移 +（可选）mujoco keyframe 摆位；步态保真是 plant 的活。
- **go2 身体 + as2 语义的错位**：该模式测协议/链路/FSM，几何是 go2 的——**禁止**用它调 AS2 足印参数（那要等 as2 仿真资产 + T-A 直驱拓扑）。
- 不仿步态动力学、不仿lidar 生成（点云话题如需再议）。

## 6. 依赖与坑

1. mjsim 容器（mjsim:jazzy）需烤入 unitree_sdk2py（官方 main + as2 子包；自带 cyclonedds py 绑定 0.10.2，与 Jazzy 不冲突）——**迁移 Step1 完成后即烤**。
2. 接口隔离：dog-mode 默认绑 `lo`，防 sport 服务漏到 123 网段被真狗听见。
3. 500Hz 状态发布回调只做字段拷贝（与 unitree_control 侧消费习惯对齐）。
4. `mujoco/hil/ros.py` L42-44 限幅为 go2 硬编码，多机型时参数化（小改，可后置）。
5. AS2 仿真模型资产（robot-assets 仓）是 T-A 直驱拓扑的前置，**不阻塞**本测试台。

## 7. 排期顺序

1. SDK 升级（pyproject pin → uv lock → 镜像重建 → 冒烟）＋ 烤入 mjsim 镜像
2. robot_profiles 落地 + unitree_control/charge_watch 改造 + 零回归验证
3. robot_emu（fsm/sport_service/state_feed/scenarios）+ dog.py 入口
4. conformance 检查 + 测试矩阵逐项过
5. AS2_CONFIG 实测回填、真机冒烟清单（audit 落地清单）
6. 文档回填：pilot-dock.md §5/§7、本档状态更新

1-4 与 6 已完成（见文首"实对结论"）。第 5 步的真机冒烟按下面清单执行；emu 侧所有推测位在 `mujoco/robot_emu/fsm.py` 有 `UNVERIFIED` 标记与本清单一一对应。

### 7.1 真机冒烟核对清单（AS2 首次上机）

| # | 核对项 | emu 现状 | 怎么核 |
|---|---|---|---|
| 1 | 坐姿下 BalanceStand 的返回码（emu 猜 -1 拒收） | `UNVERIFIED` | 坐姿 RPC BalanceStand，记录 code |
| 2 | NORMAL_WALK 下再发 StandUp（emu 猜 0 no-op 且不降级） | `UNVERIFIED` | 站立中 RPC StandUp，核 code + 后续 Move 是否仍收 |
| 3 | 在桩充电电流方向与量级（充/放电符号、在桩 mA 值） | 阈值 -100mA 保守占位 | 上桩静置 3 分钟取电流中位数 → 回填 `BatterySpec.threshold_ma` 并启用 docking gate |
| 4 | `rt/lf/bmsstate` 实测频率（emu 按 20Hz 发） | 20Hz | **已答（真机复核）：40Hz**，emu 20Hz 低于真机但 staleness 裕量充足，保持现状 |
| 5 | AS2 几何/速度上限实测 → 录入 `AS2_CONFIG`（**录入前 `ROBOT_TYPE=as2` 的 planning 启动会 loud failure，这是故意的**） | 未注册 | 卷尺+实走，一次录入 |
| 6 | ChassisWatch `gait_type` 在 AS2 上的取值语义（审计遗留项） | 常值 1 | **已答（真机）：闲置即 gait_type=1**，emu 常值与真机一致；行走中取值待实走阶段观察 |
| 7 | `/unitree/odometry` 消费端（localize/pilot）对 sportmodestate 重铺噪声的容忍度 | 物理真值直填 | 室内已知点位走 10m 比对定位漂移 |
| 8 | emu `vel_clip` 占位 (1.0, 0.5, 1.5) | `UNVERIFIED` | 实测 AS2 极速后回填 |
| 9 | **sportmodestate 静置零心跳**（已定位：FSM 闲置时估计器休眠，500Hz 零载荷心跳，lowstate/BMS 不受影响） | 已过滤 | 唤醒状态下复核真实载荷率与静默语义；odom 静默≠离线，消费端不得拿 quiet 当故障 |

## 8. 决策记录

| 决策 | 结论 |
|---|---|
| 实现位置 | mujoco 子文件夹（用户拍板，轻量） |
| 仿真资产 | 先用 go2 这套，话题对齐真机（用户拍板） |
| 测试台形态 | sim 内嵌 emu（合并原"独立 fake_chassis"方案） |
| 枚举来源 | 收敛现有 ROBOT_TYPE 字符串集为 RobotType 枚举（仓内本无枚举类） |
| FSM 规则表 | sim/pilot 双份，不跨仓 import；emu 持全量表（`robot_emu/fsm.py`），pilot 侧只留断言点语义（profile 的 stand_steps/gait_assertion），话题面由 conformance 锁 |
| AS2_CONFIG 占位 | 不注册（查表报 unsupported），实测几何后一次性录入——占位+警告仍有误用面 |
| CurrentSignPolicy 阈值 | 阈值是 BatterySpec 的 per-profile 数据；默认 -100mA 保守值，排期第 5 步上桩校准（执行冒烟者兼校准人），校准前 docking gate 不启用 |
| 状态回传速率 | 20Hz（用户拍板降开销；最低消费端是 charge_watch 3s staleness，ChassisWatch 1s 静默告警裕量充足） |
| bench 网络面 | SDK 模板无 Peers 段 + lo 无 IFF_MULTICAST → loopback 发现不可用；bench 绑 docker0（MULTICAST + 与 123 狗网隔离）。cyclonedds 版本必须 ≤0.10.x（11 的 type object 令 0.10.2 段错误）；mjsim(py3.12) 以 CYCLONEDDS_HOME 指向 jazzy libddsc 源码编 0.10.2，整套已烤入 docker/mujoco.Dockerfile（vendored SDK + sdist 编绑定 + ldconfig 链 iceoryx），重建镜像即生效 |
| 加固项（评审后落地） | 探针/e2e 全入库可重放（probe_as2/go2、e2e_driver、e2e_charge）；emu L0 单测 15 条；flat 场景注册（长跑无坠崖）；scenarios 补 refuse_move/cmd_blackout_s；FSM 推测位 UNVERIFIED 标记 + §7.1 真机冒烟清单；pilot 侧 test_profile_mirror 锁双表；dog.py nv==18 断言 + vel_clip 按机型参数化 |
| 遗留 | ~~SDK pin 是 junlinp fork@800103ea~~ **已清**：emu 侧 SDK 与 pilot 统一 pin 上游 `unitreerobotics@814556d`，`vendor/unitree_sdk2py` 为 pin 源码检出（子模块语义、提交时 `git submodule add` 转正），镜像 pip install。**mjsim 镜像已重建并冒烟全绿**（conformance/probe_as2/go2 均 PASS；根因=CycloneDDS 层尾部 `apt purge+autoremove` 级联删光 /opt/ros，已移除并加农场自检断言）；g1 emu（loco 服务）按原排期后置 |

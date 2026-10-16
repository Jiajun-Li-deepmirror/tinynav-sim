"""State feed: physics truth -> the model's DDS state topics at 20 Hz -- the
rate of the dog's rt/lf/* topics, which is all the stack consumes (charge_watch's
staleness window is 3 s; ChassisWatch alarms past 1 s of silence).

Runs on its own thread; the control thread swaps the Snapshot it reads.
Field orders mirror the vendored IDL dataclasses exactly.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from .profile import EmuProfile

# range_obstacle fill: "nothing near" (emu-defined far value, metres)
RANGE_NONE_M = 30.0
NUM_JOINTS = 12        # go2 body; model-specific lowstates pad with zeros


@dataclass(frozen=True)
class Snapshot:
    """One physics sample, already in dog-body conventions."""
    uptime: float        # dog power-on clock (s since emu boot)
    pos: tuple           # world xyz (m)
    quat_wxyz: tuple     # attitude
    rpy: tuple           # roll pitch yaw (rad, ZYX)
    vel_body: tuple      # body-frame linear velocity
    yaw_rate: float      # body-frame yaw rate (rad/s)
    gyro: tuple
    acc: tuple
    joint_q: tuple       # 12 leg joints (rad)
    joint_dq: tuple      # 12 leg joint velocities


def _timespec(uptime: float):
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import TimeSpec_
    sec = int(uptime)
    return TimeSpec_(sec, int((uptime - sec) * 1e9))


def _ros_time(uptime: float):
    from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_
    sec = int(uptime)
    return Time_(sec, int((uptime - sec) * 1e9))


def _sport_mode_state(snap: Snapshot, mode: int, error_code: int):
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
        IMUState_, PathPoint_, SportModeState_)

    imu = IMUState_(list(snap.quat_wxyz), list(snap.gyro), list(snap.acc),
                    list(snap.rpy), 0)
    return SportModeState_(
        stamp=_timespec(snap.uptime), error_code=int(error_code),
        imu_state=imu, mode=int(mode), progress=0.0, gait_type=1,
        foot_raise_height=0.0, position=list(snap.pos), body_height=0.0,
        velocity=list(snap.vel_body), yaw_speed=float(snap.yaw_rate),
        range_obstacle=[RANGE_NONE_M] * 4, foot_force=[0] * 4,
        foot_position_body=[0.0] * 12, foot_speed_body=[0.0] * 12,
        path_point=[PathPoint_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    for _ in range(10)])


def _go_lowstate(snap: Snapshot, bms_status: int, bms_soc: int, bms_ma: int,
                 tick: int):
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
        BmsState_, IMUState_, LowState_, MotorState_)

    imu = IMUState_(list(snap.quat_wxyz), list(snap.gyro), list(snap.acc),
                    list(snap.rpy), 0)
    motors = []
    for i in range(20):
        q = float(snap.joint_q[i]) if i < NUM_JOINTS else 0.0
        dq = float(snap.joint_dq[i]) if i < NUM_JOINTS else 0.0
        motors.append(MotorState_(0, q, dq, 0.0, 0.0, q, dq, 0.0, 0, 0,
                                  [0, 0]))
    bms = BmsState_(1, 0, int(bms_status), int(bms_soc), int(bms_ma), 0,
                    [0, 0], [0, 0], [0] * 15)
    return LowState_(
        head=[0, 0], level_flag=0, frame_reserve=0, sn=[0, 0],
        version=[0, 0], bandwidth=0, imu_state=imu, motor_state=motors,
        bms_state=bms, foot_force=[0] * 4, foot_force_est=[0] * 4,
        tick=int(tick), wireless_remote=[0] * 40, bit_flag=0, adc_reel=0.0,
        temperature_ntc1=0, temperature_ntc2=0, power_v=0.0, power_a=0.0,
        fan_frequency=[0] * 4, reserve=0, crc=0)


def _hg_lowstate(snap: Snapshot, tick: int):
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
        IMUState_, LowState_, MotorState_)

    imu = IMUState_(list(snap.quat_wxyz), list(snap.gyro), list(snap.acc),
                    list(snap.rpy), 0)
    motors = []
    for i in range(35):
        q = float(snap.joint_q[i]) if i < NUM_JOINTS else 0.0
        dq = float(snap.joint_dq[i]) if i < NUM_JOINTS else 0.0
        motors.append(MotorState_(0, q, dq, 0.0, 0.0, [0, 0], 0.0, [0, 0],
                                  0, [0, 0, 0, 0]))
    return LowState_(version=[0, 0], mode_pr=0, mode_machine=0,
                     tick=int(tick), imu_state=imu, motor_state=motors,
                     wireless_remote=[0] * 40, reserve=[0, 0, 0, 0], crc=0)


def _hg_bmsstate(bms_soc: int, bms_ma: int, cycle: int = 10):
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_

    return BmsState_(1, 0, 0, [0] * 40, [0, 0, 0], int(bms_ma), int(bms_soc),
                     100, [250] * 12, int(cycle), 0, [0] * 5, [0, 0, 0])


def _odometry(snap: Snapshot):
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import (
        Point_, Pose_, PoseWithCovariance_, Quaternion_, Twist_,
        TwistWithCovariance_, Vector3_)
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_

    w, x, y, z = (float(v) for v in snap.quat_wxyz)
    pose = Pose_(position=Point_(x=float(snap.pos[0]), y=float(snap.pos[1]),
                                 z=float(snap.pos[2])),
                 orientation=Quaternion_(x=x, y=y, z=z, w=w))
    twist = Twist_(linear=Vector3_(x=float(snap.vel_body[0]),
                                   y=float(snap.vel_body[1]),
                                   z=float(snap.vel_body[2])),
                   angular=Vector3_(x=0.0, y=0.0, z=float(snap.yaw_rate)))
    return Odometry_(
        header=Header_(stamp=_ros_time(snap.uptime), frame_id="odom"),
        child_frame_id="base_link",
        pose=PoseWithCovariance_(pose=pose, covariance=[0.0] * 36),
        twist=TwistWithCovariance_(twist=twist, covariance=[0.0] * 36))


class StateFeed(threading.Thread):
    """Publishes the profile's topic surface from the latest snapshot."""

    def __init__(self, profile: EmuProfile, fsm, scenarios, snapshot_provider,
                 hz: float = 20.0) -> None:
        super().__init__(daemon=True, name="emu-state-feed")
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
            LowState_ as GoLowState_, SportModeState_)
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
            BmsState_ as HgBmsState_, LowState_ as HgLowState_)

        self._profile = profile
        self._fsm = fsm
        self._sc = scenarios
        self._snapshot = snapshot_provider
        self._period = 1.0 / max(hz, 1.0)
        self._stop_evt = threading.Event()
        self._tick = 0
        self.pub_count = 0

        self._pub_sport = ChannelPublisher("rt/lf/sportmodestate",
                                           SportModeState_)
        self._pub_sport.Init()
        self._pub_bms = None
        if profile.lowstate_idl == "unitree_go":
            # The battery rides inside the lf lowstate the watch subscribes:
            # one publisher covers both.
            self._pub_low = ChannelPublisher(profile.battery_topic, GoLowState_)
        else:
            self._pub_low = ChannelPublisher("rt/lowstate", HgLowState_)
            if profile.battery_topic:
                self._pub_bms = ChannelPublisher(profile.battery_topic,
                                                 HgBmsState_)
                self._pub_bms.Init()
        self._pub_low.Init()
        self._pub_odom = None
        if profile.odom_topic:
            self._pub_odom = ChannelPublisher(profile.odom_topic, Odometry_)
            self._pub_odom.Init()

    def run(self) -> None:
        next_t = time.perf_counter()
        while not self._stop_evt.is_set():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(next_t - now, 0.02))
                continue
            next_t = max(next_t + self._period, time.perf_counter())
            snap = self._snapshot()
            if snap is None:
                continue
            self._tick += 1
            sc = self._sc
            self._pub_sport.Write(_sport_mode_state(
                snap, self._fsm.mode, sc.get("error_code")))
            if self._profile.lowstate_idl == "unitree_go":
                self._pub_low.Write(_go_lowstate(
                    snap, sc.get("bms_status"), sc.get("bms_soc"),
                    sc.get("bms_ma"), self._tick))
            else:
                self._pub_low.Write(_hg_lowstate(snap, self._tick))
            if self._pub_bms is not None:
                self._pub_bms.Write(_hg_bmsstate(sc.get("bms_soc"),
                                                 sc.get("bms_ma")))
            if self._pub_odom is not None:
                self._pub_odom.Write(_odometry(snap))
            self.pub_count += 1

    def stop(self) -> None:
        self._stop_evt.set()
        self.join(timeout=2.0)

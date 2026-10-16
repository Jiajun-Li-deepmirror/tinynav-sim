"""go2 conformance probe against a running emu (or a real go2).

Checks the go-family wire surface: unitree_go lowstate WITH a bms field,
rt/utlidar/robot_odom present, Move accepted from any state, ClassicWalk
valid (api id 2049 in the go2 client).

Run inside a container that has unitree_sdk2py:
    python3 -m robot_emu.probe_go2 --iface docker0
"""
from __future__ import annotations

import argparse
import time

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelSubscriber)
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default="docker0")
    ap.add_argument("--window", type=float, default=3.0)
    args = ap.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    counts = {"sport": 0, "low": 0, "odom": 0}
    last = {}

    def mk(key, cb):
        def _h(msg):
            counts[key] += 1
            last[key] = cb(msg)
        return _h

    sub_sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub_sport.Init(mk("sport", lambda m: {"mode": int(m.mode)}), 10)
    sub_low = ChannelSubscriber("rt/lowstate", LowState_)
    sub_low.Init(mk("low", lambda m: {
        "n_motor": len(m.motor_state),
        "bms_soc": int(m.bms_state.soc),
        "bms_ma": int(m.bms_state.current),
        "bms_status": int(m.bms_state.status)}), 10)
    sub_odom = ChannelSubscriber("rt/utlidar/robot_odom", Odometry_)
    sub_odom.Init(mk("odom", lambda m: {
        "frame": m.header.frame_id,
        "pos": [round(float(v), 3) for v in (
            m.pose.pose.position.x, m.pose.pose.position.y,
            m.pose.pose.position.z)]}), 10)

    time.sleep(args.window)
    print(f"topic counts over {args.window:.0f}s:", counts)
    for k, v in last.items():
        print(f"last[{k}]:", v)

    c = SportClient()
    c.SetTimeout(5.0)
    c.Init()
    r = {"move_while_sitting": c.Move(0.3, 0.0, 0.0),
         "classicwalk": c.ClassicWalk(True)}
    print("RPC results:", r)
    ok = r["move_while_sitting"] == 0 and r["classicwalk"] == 0
    print("PROBE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

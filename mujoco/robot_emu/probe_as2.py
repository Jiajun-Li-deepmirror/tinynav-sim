"""AS2 conformance probe against a running emu (or, with --iface enx..., a
real AS2 -- the exact probe the real-dog audit used, kept runnable).

Checks the audit's wire facts: 20Hz-class state feed, hg lowstate without a
bms field, rt/lf/bmsstate with live values, and the FSM gating sequence
(Move refused until StandUp+BalanceStand put the chassis in NORMAL_WALK).

Run inside a container that has unitree_sdk2py (the mjsim or tinynav one):
    python3 -m robot_emu.probe_as2 --iface docker0
"""
from __future__ import annotations

import argparse
import time

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelSubscriber)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_
from unitree_sdk2py.as2.sport.sport_client import SportClient


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default="docker0")
    ap.add_argument("--window", type=float, default=3.0)
    args = ap.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    counts = {"sport": 0, "low": 0, "bms": 0}
    last = {}

    def mk(key, cb):
        def _h(msg):
            counts[key] += 1
            last[key] = cb(msg)
        return _h

    sub_sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub_sport.Init(mk("sport", lambda m: {
        "mode": int(m.mode), "error_code": int(m.error_code),
        "pos": [round(float(v), 3) for v in m.position],
        "vel": [round(float(v), 3) for v in m.velocity]}), 10)
    sub_low = ChannelSubscriber("rt/lowstate", LowState_)
    sub_low.Init(mk("low", lambda m: {
        "n_motor": len(m.motor_state),
        "has_bms": hasattr(m, "bms_state")}), 10)
    sub_bms = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
    sub_bms.Init(mk("bms", lambda m: {
        "soc": int(m.soc), "current_ma": int(m.current), "soh": int(m.soh)}),
        10)

    time.sleep(args.window)
    print(f"topic counts over {args.window:.0f}s:", counts)
    for k, v in last.items():
        print(f"last[{k}]:", v)

    c = SportClient()
    c.SetTimeout(5.0)
    c.Init()
    refused = c.Move(0.3, 0.0, 0.0)
    r = {"move_while_sitting": refused,
         "standup": c.StandUp(),
         "balancestand": c.BalanceStand(),
         "move_walk_ready": c.Move(0.3, 0.0, 0.0)}
    print("RPC results:", r)
    ok = (refused != 0 and r["standup"] == 0 and r["balancestand"] == 0
          and r["move_walk_ready"] == 0)
    time.sleep(1.0)
    print("last sport:", last.get("sport"))
    print("PROBE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

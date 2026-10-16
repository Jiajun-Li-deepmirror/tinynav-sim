"""E2E charge watch: pilot ChargeWatch (production) against the emu.

Resting draw must read discharging / not externally powered; injecting an
on-pad current via the emu scenario topic must flip it to charging /
externally powered; clearing must restore it. Runs in the pilot-side
container:
    python3 -m robot_emu.e2e_charge --iface docker0
"""
from __future__ import annotations

import argparse
import sys
import time

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelPublisher)
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

sys.path.insert(0, ".")  # robot_emu import when run from the repo tree


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default="docker0")
    ap.add_argument("--robot-type", default="as2")
    args = ap.parse_args()

    from pilot.backend.charge_watch import ChargeWatch

    ChannelFactoryInitialize(0, args.iface)
    watch = ChargeWatch(args.iface, args.robot_type, log=print)
    if not watch.start():
        print("charge watch failed to start")
        return 1
    pub = ChannelPublisher("rt/emu/scenario", String_)
    pub.Init()

    ok = True
    time.sleep(4.5)
    resting = watch.fields()
    print("fields resting:", resting)
    if resting["chargeState"] != "discharging" or resting["externallyPowered"]:
        ok = False

    pub.Write(String_('{"bms_ma": 1500}'))
    time.sleep(4.0)
    on_pad = watch.fields()
    print("fields on-pad: ", on_pad)
    if on_pad["chargeState"] != "charging" or not on_pad["externallyPowered"]:
        ok = False

    pub.Write(String_('{"clear": true}'))
    time.sleep(4.0)
    cleared = watch.fields()
    print("fields cleared:", cleared)
    if cleared["chargeState"] != "discharging" or cleared["externallyPowered"]:
        ok = False

    print("CHARGE E2E", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

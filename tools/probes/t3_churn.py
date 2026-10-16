#!/usr/bin/env python3
"""T3 churn participant: bare SDK participant + fall-watch-like readers,
exit after CHURN_ALIVE_S so the container restart re-appears it on the dog's
DDS network (SPDP appear/disappear churn). Nothing is logged; this process
exists only to be born and die on the dog's network.

Usage: t3_churn.py <iface>
"""
import os
import sys
import time

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

ALIVE_S = float(os.environ.get("CHURN_ALIVE_S", "3"))


def _count(msg):
    pass


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
    ChannelFactoryInitialize(0, iface)
    ChannelSubscriber("rt/lf/bmsstate", BmsState_).Init(_count, 10)
    sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sport.Init(_count, 25)  # 1/25 like fall-watch
    t0 = time.time()
    print(f"[t3-churn] participant up iface={iface} pid={os.getpid()} "
          f"alive={ALIVE_S}s", flush=True)
    time.sleep(ALIVE_S)
    print(f"[t3-churn] exiting after {time.time() - t0:.1f}s "
          f"(docker restart will re-appear me)", flush=True)


if __name__ == "__main__":
    main()

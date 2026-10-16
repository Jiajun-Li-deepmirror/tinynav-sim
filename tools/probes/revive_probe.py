#!/usr/bin/env python3
"""revive_probe.py — try to un-stick a prone/fake-online AS2 via sport RPCs.
Read-only intent, motion-class calls only (dog is already down; worst case = stays down).
"""
import sys
import time

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.as2.sport.sport_client import SportClient

iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
ChannelFactoryInitialize(0, iface)
c = SportClient()
c.SetTimeout(5.0)
c.Init()

def show(tag):
    st = {}
    code = c.GetState(st)
    print(f"[{tag}] GetState code={code} {st}", flush=True)

show("before")

for name, fn in [
    ("RecoveryStand", lambda: c.RecoveryStand()),
    ("StopMove", lambda: c.StopMove()),
    ("Damp", lambda: c.Damp()),
]:
    try:
        code = fn()
        print(f"{name} -> code {code}", flush=True)
    except Exception as e:
        print(f"{name} -> EXC {e}", flush=True)
    time.sleep(2.0)
    show(name)

# Damp 之后再试站起链
for name, fn in [
    ("StandUp(after Damp)", lambda: c.StandUp()),
    ("BalanceStand", lambda: c.BalanceStand()),
    ("RecoveryStand(again)", lambda: c.RecoveryStand()),
]:
    try:
        code = fn()
        print(f"{name} -> code {code}", flush=True)
    except Exception as e:
        print(f"{name} -> EXC {e}", flush=True)
    time.sleep(2.5)
    show(name)

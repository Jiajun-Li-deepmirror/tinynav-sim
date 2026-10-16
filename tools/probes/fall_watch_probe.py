#!/usr/bin/env python3
"""fall_watch_probe.py — high-rate dog-state recorder for fall forensics.

Subscribes (read-only) and CSV-logs:
  rt/lf/bmsstate     ~17-40Hz -> every sample      (voltage/current/soc/temps)
  rt/sportmodestate  ~500Hz   -> every SPORT_EVERYth (~20Hz)
                                 mode/gait/error_code/velocity/position

Wall-clock timestamps (the AS2's own sport stamp is known-zero). Runs inside
the tinynav container, writes to /out (bind-mounted host dir).

Usage: fall_watch_probe.py <iface> <out_dir> [sport_every]
"""
import os
import sys
import time

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


def numeric_fields(msg):
    out = {}
    for name in dir(msg):
        if name.startswith("_"):
            continue
        try:
            v = getattr(msg, name)
        except AttributeError:
            continue
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            out[name] = v
        elif isinstance(v, (list, tuple)) and v and isinstance(v[0], (int, float)):
            out[name] = ";".join(str(x) for x in v[:24])
    return out


class Csv:
    def __init__(self, path):
        self.path = path
        self.f = open(path, "a", buffering=1)
        self.keys = None

    def write(self, t, fields):
        if self.keys is None:
            self.keys = sorted(fields.keys())
            if os.path.getsize(self.path) == 0:
                self.f.write("wall_ts," + ",".join(self.keys) + "\n")
        self.f.write("%.3f," % t + ",".join(str(fields.get(k, "")) for k in self.keys) + "\n")


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "/out"
    sport_every = max(1, int(sys.argv[3])) if len(sys.argv) > 3 else 25
    os.makedirs(out_dir, exist_ok=True)
    tag = time.strftime("%H%M%S")
    bms_csv = Csv(os.path.join(out_dir, f"fallwatch_bms_{tag}.csv"))
    sport_csv = Csv(os.path.join(out_dir, f"fallwatch_sport_{tag}.csv"))

    state = {"n_bms": 0, "n_sport": 0, "t0": time.time()}

    def on_bms(msg: BmsState_):
        state["n_bms"] += 1
        bms_csv.write(time.time(), numeric_fields(msg))

    def on_sport(msg: SportModeState_):
        state["n_sport"] += 1
        if state["n_sport"] % sport_every == 0:
            sport_csv.write(time.time(), numeric_fields(msg))

    ChannelFactoryInitialize(0, iface)
    sub_bms = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
    sub_bms.Init(on_bms, 100)
    sub_sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub_sport.Init(on_sport, 200)

    print(f"[fallwatch] logging to {out_dir} (tag={tag}), sport 1/{sport_every}", flush=True)
    while True:
        time.sleep(10)
        dt = time.time() - state["t0"]
        print(f"[fallwatch] bms={state['n_bms']} ({state['n_bms']/dt:.1f}Hz) "
              f"sport={state['n_sport']} ({state['n_sport']/dt:.0f}Hz)", flush=True)


if __name__ == "__main__":
    main()

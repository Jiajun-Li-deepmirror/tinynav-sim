#!/usr/bin/env python3
"""VUI brightness staircase: 0 -> 20 -> 40 -> 60 -> 80 -> 100 -> 80 -> ... -> 0,
one step every 5 s. Watch the dog's head light and note at which step (if any)
the visible light changes.

Usage: vui_brightness_sweep.py <netiface> [step_seconds] [max_level]
"""
import sys
import time

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

SET_BRIGHTNESS = 1005
GET_BRIGHTNESS = 1006


class VuiProbe(Client):
    def __init__(self):
        super().__init__("vui", False)

    def Init(self):
        self._SetApiVerson("1.0.0.1")
        self._RegistApi(SET_BRIGHTNESS, 0)
        self._RegistApi(GET_BRIGHTNESS, 0)

    def set_brightness(self, level):
        import json
        return self._Call(SET_BRIGHTNESS, json.dumps({"brightness": level}))[0]

    def get_brightness(self):
        import json
        code, data = self._Call(GET_BRIGHTNESS, "{}")
        return json.loads(data)["brightness"] if code == 0 else None


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
    step_s = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    max_level = int(sys.argv[3]) if len(sys.argv) > 3 else 100
    up = list(range(0, max_level + 1, max_level // 5))          # 0..100 in 6 steps
    down = list(range(max_level - max_level // 5, -1, -max_level // 5))  # back to 0
    ChannelFactoryInitialize(0, iface)
    c = VuiProbe()
    c.SetTimeout(3.0)
    c.Init()
    for level in up + down:
        code = c.set_brightness(level)
        time.sleep(step_s)
        print(f"[vui] SetBrightness({level}) code={code} now={c.get_brightness()}", flush=True)


if __name__ == "__main__":
    main()

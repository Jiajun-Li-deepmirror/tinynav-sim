#!/usr/bin/env python3
"""VUI service probe for AS2 (and any unitree dog hosting the generic `vui` RPC).

The go2/b2 SDK ships a VuiClient (service "vui", api 1001-1006) but the pinned
SDK on the rig may lack it, and AS2 has no named client at all. The wire
protocol is model-agnostic JSON-RPC over the base Client, so this probe talks
to whatever hosts `vui` — brightness/volume/switch read first (safe), set only
with --set.

Usage:
  vui_probe.py <netiface>            # read-only: GetBrightness + GetVolume + GetSwitch
  vui_probe.py <netiface> --set B V  # SetBrightness(B) SetVolume(V) then re-read
  vui_probe.py <netiface> --switch 0|1
"""
import json
import sys

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

VUI_API_ID_SETSWITCH = 1001
VUI_API_ID_GETSWITCH = 1002
VUI_API_ID_SETVOLUME = 1003
VUI_API_ID_GETVOLUME = 1004
VUI_API_ID_SETBRIGHTNESS = 1005
VUI_API_ID_GETBRIGHTNESS = 1006


class VuiProbe(Client):
    def __init__(self):
        super().__init__("vui", False)

    def Init(self):
        self._SetApiVerson("1.0.0.1")
        for api in (VUI_API_ID_SETSWITCH, VUI_API_ID_GETSWITCH,
                    VUI_API_ID_SETVOLUME, VUI_API_ID_GETVOLUME,
                    VUI_API_ID_SETBRIGHTNESS, VUI_API_ID_GETBRIGHTNESS):
            self._RegistApi(api, 0)

    def get_brightness(self):
        code, data = self._Call(VUI_API_ID_GETBRIGHTNESS, "{}")
        return (code, json.loads(data)["brightness"]) if code == 0 else (code, None)

    def get_volume(self):
        code, data = self._Call(VUI_API_ID_GETVOLUME, "{}")
        return (code, json.loads(data)["volume"]) if code == 0 else (code, None)

    def get_switch(self):
        code, data = self._Call(VUI_API_ID_GETSWITCH, "{}")
        return (code, json.loads(data)["enable"]) if code == 0 else (code, None)

    def set_brightness(self, level):
        return self._Call(VUI_API_ID_SETBRIGHTNESS, json.dumps({"brightness": level}))[0]

    def set_volume(self, level):
        return self._Call(VUI_API_ID_SETVOLUME, json.dumps({"volume": level}))[0]

    def set_switch(self, enable):
        return self._Call(VUI_API_ID_SETSWITCH, json.dumps({"enable": enable}))[0]


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
    args = sys.argv[2:]
    ChannelFactoryInitialize(0, iface)
    c = VuiProbe()
    c.SetTimeout(3.0)
    c.Init()
    print(f"[vui] iface={iface} service=vui (read-only pass)")
    print(f"[vui] GetBrightness -> {c.get_brightness()}")
    print(f"[vui] GetVolume     -> {c.get_volume()}")
    print(f"[vui] GetSwitch     -> {c.get_switch()}")
    if "--set" in args:
        i = args.index("--set")
        b, v = int(args[i + 1]), int(args[i + 2])
        print(f"[vui] SetBrightness({b}) -> code {c.set_brightness(b)}")
        print(f"[vui] SetVolume({v})     -> code {c.set_volume(v)}")
        print(f"[vui] re-read brightness -> {c.get_brightness()}")
    if "--switch" in args:
        i = args.index("--switch")
        en = int(args[i + 1])
        print(f"[vui] SetSwitch({en}) -> code {c.set_switch(en)}")
        print(f"[vui] re-read switch  -> {c.get_switch()}")


if __name__ == "__main__":
    main()

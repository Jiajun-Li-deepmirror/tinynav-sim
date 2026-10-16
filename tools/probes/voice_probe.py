#!/usr/bin/env python3
"""Voice service probe (g1-style AudioClient protocol, service name is model-agnostic).

g1's AudioClient speaks service `voice` v1.0.0.0: TTS(1001), ASR(1002, no wrapper
in SDK), StartPlay(1003), StopPlay(1004), GetVolume(1005), SetVolume(1006),
SetRgbLed(1010). AS2/go2/b2 have no named client in the pinned SDK; this probe
talks to whatever hosts `voice`.

Usage:
  voice_probe.py <netiface>                 # read-only: GetVolume
  voice_probe.py <netiface> --set 1         # SetVolume(level) then re-read
  voice_probe.py <netiface> --tts "你好"     # TtsMaker at current volume (SET LOW FIRST)
  voice_probe.py <netiface> --stop          # PlayStop
"""
import json
import sys

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

TTS = 1001
STOP_PLAY = 1004
GET_VOLUME = 1005
SET_VOLUME = 1006


class VoiceProbe(Client):
    def __init__(self):
        super().__init__("voice", False)

    def Init(self):
        self._SetApiVerson("1.0.0.0")
        for api in (TTS, STOP_PLAY, GET_VOLUME, SET_VOLUME):
            self._RegistApi(api, 0)

    def get_volume(self):
        code, data = self._Call(GET_VOLUME, "{}")
        return (code, json.loads(data)["volume"]) if code == 0 else (code, None)

    def set_volume(self, level):
        return self._Call(SET_VOLUME, json.dumps({"volume": level}))[0]

    def tts(self, text, speaker_id=0):
        p = {"index": 1, "text": text, "speaker_id": speaker_id}
        return self._Call(TTS, json.dumps(p))[0]

    def stop(self):
        return self._Call(STOP_PLAY, json.dumps({"app_name": "probe"}))[0]


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
    args = sys.argv[2:]
    ChannelFactoryInitialize(0, iface)
    c = VoiceProbe()
    c.SetTimeout(3.0)
    c.Init()
    print(f"[voice] iface={iface} service=voice (read pass)")
    print(f"[voice] GetVolume -> {c.get_volume()}")
    if "--set" in args:
        i = args.index("--set")
        level = int(args[i + 1])
        print(f"[voice] SetVolume({level}) -> code {c.set_volume(level)}")
        print(f"[voice] re-read volume -> {c.get_volume()}")
    if "--tts" in args:
        i = args.index("--tts")
        text = args[i + 1]
        print(f"[voice] TtsMaker({text!r}) -> code {c.tts(text)}")
    if "--stop" in args:
        print(f"[voice] PlayStop -> code {c.stop()}")


if __name__ == "__main__":
    main()

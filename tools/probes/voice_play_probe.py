#!/usr/bin/env python3
"""Voice PlayStream probe: play a raw PCM file straight through api 1003,
bypassing the TTS engine. g1 protocol: service `voice`, StartPlay=1003,
param {"app_name", "stream_id"}, binary body = list(pcm bytes).

Usage:
  voice_play_probe.py <netiface> <pcm_path> [volume]
    volume given -> SetVolume first (restore to 100 etc.), then play.
"""
import json
import sys

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

START_PLAY = 1003
STOP_PLAY = 1004
SET_VOLUME = 1006


class VoicePlayProbe(Client):
    def __init__(self):
        super().__init__("voice", False)

    def Init(self):
        self._SetApiVerson("1.0.0.0")
        for api in (START_PLAY, STOP_PLAY, SET_VOLUME):
            self._RegistApi(api, 0)

    def set_volume(self, level):
        return self._Call(SET_VOLUME, json.dumps({"volume": level}))[0]

    def play(self, pcm: bytes, app="probe", stream="1"):
        param = json.dumps({"app_name": app, "stream_id": stream})
        return self._CallRequestWithParamAndBin(START_PLAY, param, list(pcm))[0]

    def stop(self, app="probe"):
        return self._Call(STOP_PLAY, json.dumps({"app_name": app}))[0]


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
    pcm_path = sys.argv[2]
    volume = int(sys.argv[3]) if len(sys.argv) > 3 else None
    with open(pcm_path, "rb") as f:
        pcm = f.read()
    ChannelFactoryInitialize(0, iface)
    c = VoicePlayProbe()
    c.SetTimeout(10.0)  # binary upload can be slow over frp-linked rig
    c.Init()
    if volume is not None:
        print(f"[voice] SetVolume({volume}) -> code {c.set_volume(volume)}")
    print(f"[voice] PlayStream({len(pcm)} bytes) -> code {c.play(pcm)}")
    print("[voice] sound should be playing now (2 s beep)")


if __name__ == "__main__":
    main()

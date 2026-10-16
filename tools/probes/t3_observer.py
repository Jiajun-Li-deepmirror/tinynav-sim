#!/usr/bin/env python3
"""T3 steady observer: ONE stable SDK participant that only watches.

- Subscribes rt/sportmodestate (@full rate) + rt/lf/bmsstate, counts msgs/s.
- Raw AF_PACKET sniffer on the interface tallies every packet from/to the
  the dog per second -- passive wire view, no DDS speaking.
- CSV 1 row/s: wall_ts, sport_n, bms_n, wire_from_dog, wire_to_dog,
  sport_gap_s, mode, error_code, vx | ALERT lines on stream silence.

This container must stay up for the whole test: its own death/restart would
add participant churn and contaminate the experiment.

Usage: t3_observer.py <iface> <out_csv>
"""
import csv
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, "/opt/venv/lib/python3.10/site-packages")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

DOG_IP = "192.168.123.161"

_counters = {"sport": 0, "bms": 0, "wire_from": 0, "wire_to": 0}
_latest = {"mode": -1, "err": -1, "vx": 0.0}
_lock = threading.Lock()


def _sport(msg):
    with _lock:
        _counters["sport"] += 1
        _latest["mode"] = int(msg.mode)
        _latest["err"] = int(msg.error_code)
        _latest["vx"] = float(msg.velocity[0])


def _bms(msg):
    with _lock:
        _counters["bms"] += 1


def _sniffer(iface):
    try:
        s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
        s.bind((iface, 0))
        s.settimeout(1.0)
    except OSError as e:
        print(f"[t3-obs] sniffer disabled: {e}", flush=True)
        return
    while True:
        try:
            pkt = s.recv(65535)
        except socket.timeout:
            continue
        except OSError:
            return
        if len(pkt) < 34:
            continue
        eth_proto = struct.unpack("!H", pkt[12:14])[0]
        if eth_proto != 0x0800:
            continue
        src = socket.inet_ntoa(pkt[26:30])
        dst = socket.inet_ntoa(pkt[30:34])
        with _lock:
            if src == DOG_IP:
                _counters["wire_from"] += 1
            if dst == DOG_IP:
                _counters["wire_to"] += 1


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
    out = sys.argv[2] if len(sys.argv) > 2 else "/out/t3_observer.csv"

    ChannelFactoryInitialize(0, iface)
    ChannelSubscriber("rt/lf/bmsstate", BmsState_).Init(_bms, 10)
    ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(_sport, 1)
    print(f"[t3-obs] stable participant up iface={iface} csv={out}", flush=True)

    threading.Thread(target=_sniffer, args=(iface,), daemon=True).start()

    last_sport_wall = None
    last_sport_n = 0
    t_start = time.time()
    with open(out, "a", buffering=1) as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(["wall_ts", "sport_n", "bms_n", "wire_from_dog",
                        "wire_to_dog", "sport_gap_s", "mode", "error_code", "vx"])
        while True:
            time.sleep(1.0)
            now = time.time()
            with _lock:
                snap = dict(_counters)
                lat = dict(_latest)
                for k in _counters:
                    _counters[k] = 0
            if snap["sport"] > 0:
                last_sport_wall = now
                last_sport_n = snap["sport"]
            gap = 0.0 if last_sport_wall is None else now - last_sport_wall
            w.writerow([f"{now:.3f}", snap["sport"], snap["bms"],
                        snap["wire_from"], snap["wire_to"],
                        f"{gap:.1f}", lat["mode"], lat["err"], f"{lat['vx']:.3f}"])
            if last_sport_wall is not None and gap > 2.0:
                print(f"[t3-obs] ALERT sportmodestate silent {gap:.1f}s "
                      f"(was {last_sport_n}/s)", flush=True)
            if int(now - t_start) % 10 == 0:
                print(f"[t3-obs] t={now - t_start:5.0f}s sport={snap['sport']}/s "
                      f"bms={snap['bms']}/s wire_from_dog={snap['wire_from']}/s "
                      f"mode={lat['mode']} err={lat['err']}", flush=True)


if __name__ == "__main__":
    main()

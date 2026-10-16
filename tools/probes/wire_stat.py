#!/usr/bin/env python3
"""wire_stat.py — passive byte accounting on the dog link.

AF_PACKET sniffer: per-second totals + per-flow (src,dst,proto-port) byte
ranking, split by direction (dog side vs navcore side).

Usage: wire_stat.py <iface> <seconds>
"""
import os
import socket
import struct
import sys
import time
from collections import defaultdict

DOG = "192.168.123.161"

iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
s.bind((iface, 0))
s.settimeout(1.0)

flows = defaultdict(int)   # (src,dst,sport,dport) -> bytes
sec_bytes = defaultdict(lambda: [0, 0])  # sec -> [from_dog, to_dog]
OUT = os.environ.get("WIRE_CSV")  # per-second CSV when set
_fout = open(OUT, "a", buffering=1) if OUT else None
if _fout and os.path.getsize(OUT) == 0:
    _fout.write("wall_ts,from_dog_B,to_dog_B\n")
t0 = time.time()
end = t0 + dur

def _flush(done_before):
    for sec in sorted(k for k in sec_bytes if k < done_before):
        v = sec_bytes.pop(sec)
        if _fout:
            _fout.write(f"{t0 + sec:.3f},{v[0]},{v[1]}\n")


while time.time() < end:
    try:
        pkt = s.recv(65535)
    except socket.timeout:
        _flush(int(time.time() - t0))
        continue
    if len(pkt) < 34:
        continue
    if struct.unpack("!H", pkt[12:14])[0] != 0x0800:  # IPv4 only
        continue
    src = socket.inet_ntoa(pkt[26:30])
    dst = socket.inet_ntoa(pkt[30:34])
    n = len(pkt)
    sec = int(time.time() - t0)
    if src == DOG:
        sec_bytes[sec][0] += n
    elif dst == DOG:
        sec_bytes[sec][1] += n
    else:
        continue
    sport = dport = 0
    if len(pkt) > 42 and pkt[23] == 17:  # UDP
        sport, dport = struct.unpack("!HH", pkt[34:38])
    elif len(pkt) > 44 and pkt[23] == 6:  # TCP
        sport, dport = struct.unpack("!HH", pkt[34:38])
    flows[(src, dst, sport, dport)] += n

_flush(int(time.time() - t0) + 1)
if _fout:
    _fout.close()
tot_from = sum(v[0] for v in sec_bytes.values())
tot_to = sum(v[1] for v in sec_bytes.values())
print(f"== {dur:.0f}s totals ==")
print(f"dog -> navcore: {tot_from/dur/1024:.1f} KB/s ({tot_from/dur*8/1e6:.2f} Mbps)")
print(f"navcore -> dog: {tot_to/dur/1024:.1f} KB/s ({tot_to/dur*8/1e6:.2f} Mbps)")
print(f"\n== per-second from-dog KB/s (min/avg/max) ==", end=" ")
vals = [v[0] for v in sec_bytes.values()]
print(f"{min(vals)/1024:.0f}/{tot_from/dur/1024:.0f}/{max(vals)/1024:.0f}")
print("\n== top flows by bytes (src -> dst sport/dport) ==")
for (src, dst, sport, dport), b in sorted(flows.items(), key=lambda x: -x[1])[:15]:
    print(f"  {b/dur/1024:8.1f} KB/s  {src} -> {dst}  {sport}/{dport}")

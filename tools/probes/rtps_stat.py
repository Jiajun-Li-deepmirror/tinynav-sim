#!/usr/bin/env python3
"""rtps_stat.py — RTPS submessage census on the dog link.

Counts RTPS submessages by (direction, submessage id) per second:
  0x06=ACKNACK (reliable reader -> writer)   0x07=HEARTBEAT (writer -> reader)
  0x15=DATA  0x16=DATA_FRAG  0x12=GAP
ACKNACK presence from navcore proves reliable reader sessions exist;
their volume during/after a link flap is the retransmission pressure.

Usage: rtps_stat.py <iface> <seconds>
"""
import socket
import struct
import sys
import time
from collections import Counter

DOG = "192.168.123.161"
IDS = {0x06: "ACKNACK", 0x07: "HEARTBEAT", 0x15: "DATA", 0x16: "DATA_FRAG",
       0x12: "GAP", 0x01: "PAD", 0x08: "NACK_FRAG", 0x03: "INFO_TS"}

iface = sys.argv[1] if len(sys.argv) > 1 else "dog0"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
s.bind((iface, 0))
s.settimeout(1.0)

counts = Counter()
end = time.time() + dur
while time.time() < end:
    try:
        pkt = s.recv(65535)
    except socket.timeout:
        continue
    if len(pkt) < 60:
        continue
    if struct.unpack("!H", pkt[12:14])[0] != 0x0800 or pkt[23] != 17:
        continue
    src = socket.inet_ntoa(pkt[26:30])
    dst = socket.inet_ntoa(pkt[30:34])
    ihl = (pkt[14] & 0xF) * 4
    ulen = 8 + struct.unpack("!H", pkt[14 + ihl + 4:14 + ihl + 6])[0]
    payload = pkt[14 + ihl + 8: 14 + ihl + ulen]
    if payload[:4] != b"RTPS":
        continue
    direction = "dog->us" if src == DOG else ("us->dog" if dst == DOG else "other")
    off = 20  # skip RTPS header (magic+version+guidprefix)
    while off + 4 <= len(payload):
        sid = payload[off]
        ln = struct.unpack("<H", payload[off + 2:off + 4])[0]
        if ln == 0 or off + 4 + ln > len(payload):
            break
        counts[(direction, IDS.get(sid, hex(sid)))] += 1
        off += 4 + ln

print("== RTPS submessage census (%ds) ==" % dur)
for (d, name), n in sorted(counts.items(), key=lambda x: -x[1]):
    print(f"  {n:8d}  {d:8s}  {name}  ({n/dur:.0f}/s)")

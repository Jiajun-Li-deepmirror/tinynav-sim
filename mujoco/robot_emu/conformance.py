"""Conformance: enumerate the bus (DCPSPublication, the audit's method) and
diff the emu's rt/* topic surface against the curated real-dog list.

Objective gate for "topic surface matches the real robot" -- the audit's
core lesson was that a DDS type mismatch fails SILENTLY, so this never
relies on subscribing data; endpoint info is enough.

Run from anywhere with the SDK DDS visible, e.g. inside the mjsim container:
    python3 -m robot_emu.conformance --profile as2 --iface docker0
Exit 0 = pass.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# (topic, idl typename) the emu MUST publish, per profile -- mirrors
# docs/../as2-topic-audit.md + go2 known surface. SEDP reports typenames in
# the ::msg::dds_:: form (observed on both the real dog and the emu).
MUST = {
    "as2": {
        ("rt/lf/sportmodestate", "unitree_go::msg::dds_::SportModeState_"),
        ("rt/lowstate", "unitree_hg::msg::dds_::LowState_"),
        ("rt/lf/bmsstate", "unitree_hg::msg::dds_::BmsState_"),
        ("rt/api/sport/response", "unitree_api::msg::dds_::Response_"),
    },
    "go2": {
        ("rt/lf/sportmodestate", "unitree_go::msg::dds_::SportModeState_"),
        ("rt/lf/lowstate", "unitree_go::msg::dds_::LowState_"),
        ("rt/utlidar/robot_odom", "nav_msgs::msg::dds_::Odometry_"),
        ("rt/api/sport/response", "unitree_api::msg::dds_::Response_"),
    },
    # b2 probe (hangzhou rig): the lf pair carries real data at
    # ~20 Hz (battery rides in rt/lf/lowstate's bms_state); no robot_odom
    # publisher exists.
    "b2": {
        ("rt/lf/sportmodestate", "unitree_go::msg::dds_::SportModeState_"),
        ("rt/lf/lowstate", "unitree_go::msg::dds_::LowState_"),
        ("rt/api/sport/response", "unitree_api::msg::dds_::Response_"),
    },
}

MUST_NOT = {
    "as2": {("rt/utlidar/robot_odom", "nav_msgs::msg::dds_::Odometry_")},
    "go2": {("rt/lf/bmsstate", "unitree_hg::msg::dds_::BmsState_")},
    "b2": {
        ("rt/utlidar/robot_odom", "nav_msgs::msg::dds_::Odometry_"),
        ("rt/lf/bmsstate", "unitree_hg::msg::dds_::BmsState_"),
    },
}


def enumerate_publications(domain_id: int, window_s: float) -> dict:
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
    from cyclonedds.domain import DomainParticipant

    dp = DomainParticipant(domain_id=domain_id)
    rdr = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
    seen = {}
    deadline = time.time() + window_s
    while time.time() < deadline:
        for s in rdr.take():
            key = (s.topic_name, s.type_name)
            seen[key] = seen.get(key, 0) + 1
        time.sleep(0.3)
    return seen


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", required=True, choices=sorted(MUST))
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--window", type=float, default=12.0)
    ap.add_argument("--iface", default=None,
                    help="bind the probe participant to this interface "
                         "(set when the emu is not on the default route)")
    args = ap.parse_args()

    if args.iface:
        os.environ["CYCLONEDDS_URI"] = (
            "<CycloneDDS><Domain><General><Interfaces>"
            f"<NetworkInterface name=\"{args.iface}\" multicast=\"default\"/>"
            "</Interfaces></General></Domain></CycloneDDS>")

    pubs = enumerate_publications(args.domain, args.window)
    rt_topics = {k for k in pubs if k[0].startswith("rt/")}

    ok = True
    for topic, typ in sorted(MUST[args.profile]):
        if (topic, typ) in rt_topics:
            print(f"  MUST     ok   {topic} [{typ}]")
        else:
            near = [k for k in sorted(rt_topics) if k[0] == topic]
            print(f"  MUST     MISS {topic} [{typ}]"
                  + (f"  (saw {near})" if near else ""))
            ok = False
    for topic, typ in sorted(MUST_NOT[args.profile]):
        if (topic, typ) in rt_topics:
            print(f"  MUST-NOT FAIL {topic} [{typ}] published")
            ok = False
        else:
            print(f"  MUST-NOT ok   {topic} [{typ}] absent")

    print(f"--- rt/* surface seen: {len(rt_topics)} topics ---")
    for (t, ty) in sorted(rt_topics):
        print(f"    {t:40s} [{ty}]")
    print("CONFORMANCE PASS" if ok else "CONFORMANCE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

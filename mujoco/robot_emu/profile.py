"""Emu model profiles: the DDS topic surface each model exposes.

This table is a DELIBERATE mirror of the pilot-side RobotProfile topic fields
(tinynav-pilot tinynav/platforms/robot_profiles.py) -- the conformance check
locks the two against the real-dog audit list. Keep them in sync by hand;
do not import across repos.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EmuProfile:
    name: str
    fsm_table: str            # key into fsm.TABLES
    lowstate_idl: str         # 'unitree_go' | 'unitree_hg'
    battery_topic: str | None  # topic the battery arrives on: go family =
                              #   rt/lf/lowstate (bms_state rides inside);
                              #   AS2 = the standalone hg BmsState topic;
                              #   None = no battery (g1)
    odom_topic: str | None    # 'rt/utlidar/robot_odom' (go-family only)
    initial_mode: int
    vel_clip: tuple = (0.5, 0.5, 1.2)   # (vx, vy, wz) caps on accepted Move;
                                        # go2's, per hil/ros.py


PROFILES: dict[str, EmuProfile] = {
    "go2": EmuProfile("go2", "sport_permissive", "unitree_go", "rt/lf/lowstate",
                      "rt/utlidar/robot_odom", initial_mode=0),
    "go2w": EmuProfile("go2w", "sport_permissive", "unitree_go", "rt/lf/lowstate",
                       "rt/utlidar/robot_odom", initial_mode=0),
    # Real-b2 probe (hangzhou rig): no rt/utlidar/robot_odom
    # publisher exists (10 s of silence, standing dog) -- the driver takes
    # chassis odometry from rt/lf/sportmodestate instead, so the emu must
    # not publish it. b2w is still unprobed and keeps the go2 surface.
    "b2": EmuProfile("b2", "sport_permissive", "unitree_go", "rt/lf/lowstate",
                     None, initial_mode=0),
    "b2w": EmuProfile("b2w", "sport_permissive", "unitree_go", "rt/lf/lowstate",
                      "rt/utlidar/robot_odom", initial_mode=0),
    # AS2 audit: sportmodestate is unitree_go; lowstate is
    # unitree_hg (35 motors, NO bms field); battery is rt/lf/bmsstate (hg);
    # no robot_odom publisher exists at all.
    "as2": EmuProfile("as2", "as2", "unitree_hg", "rt/lf/bmsstate",
                      None, initial_mode=0,
                      # UNVERIFIED: placeholder caps, not measured on an AS2.
                      vel_clip=(1.0, 0.5, 1.5)),
    # g1: humanoid, loco client + hg lowstate, no battery -- needs the loco
    # service responder; deferred with the g1 emulation rules.
}


def get_profile(name: str) -> EmuProfile:
    try:
        return PROFILES[name]
    except KeyError:
        raise SystemExit(
            f"unknown emu profile {name!r}; choose {sorted(PROFILES)}") from None


def topic_surface(profile: EmuProfile) -> dict[str, str]:
    """topic -> idl typename the emu must publish (the lf state topics the
    stack consumes + the model-specific ones). RPC request/response and
    rt/emu/* are transport, not part of this surface."""
    surface = {"rt/lf/sportmodestate": "unitree_go.msg.dds_.SportModeState_"}
    if profile.lowstate_idl == "unitree_go":
        surface[profile.battery_topic] = "unitree_go.msg.dds_.LowState_"
    else:
        surface["rt/lowstate"] = "unitree_hg.msg.dds_.LowState_"
        if profile.battery_topic:
            surface[profile.battery_topic] = "unitree_hg.msg.dds_.BmsState_"
    if profile.odom_topic:
        surface[profile.odom_topic] = "nav_msgs.msg.dds_.Odometry_"
    return surface

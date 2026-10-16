"""Emu profile table unit tests -- the topic surface each model must expose.

This table is a deliberate mirror of the pilot-side RobotProfile (see
profile.py's docstring); pilot/tests/test_profile_mirror.py locks the two
from the other side. Pure logic, no SDK needed.
Run from mujoco/:  python3 -m unittest robot_emu.tests.test_profile -v
"""
from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, str(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))

from robot_emu import profile as P


class TestProfiles(unittest.TestCase):
    def test_every_profile_has_a_table(self):
        for name, prof in P.PROFILES.items():
            self.assertEqual(name, prof.name)

    def test_go_family_surface(self):
        for name in ("go2", "go2w", "b2w"):
            prof = P.get_profile(name)
            self.assertEqual(prof.lowstate_idl, "unitree_go", name)
            self.assertEqual(prof.battery_topic, "rt/lf/lowstate", name)
            self.assertEqual(prof.odom_topic, "rt/utlidar/robot_odom", name)
            self.assertEqual(prof.fsm_table, "sport_permissive", name)

    def test_b2_surface_matches_real_dog_probe(self):
        """The real b2 has no rt/utlidar/robot_odom publisher (
        rig probe), so neither does the emu."""
        prof = P.get_profile("b2")
        self.assertEqual(prof.lowstate_idl, "unitree_go")
        self.assertEqual(prof.battery_topic, "rt/lf/lowstate")
        self.assertIsNone(prof.odom_topic)
        self.assertEqual(prof.fsm_table, "sport_permissive")

    def test_as2_surface_matches_audit(self):
        prof = P.get_profile("as2")
        self.assertEqual(prof.lowstate_idl, "unitree_hg")
        self.assertEqual(prof.battery_topic, "rt/lf/bmsstate")
        self.assertIsNone(prof.odom_topic)
        self.assertEqual(prof.fsm_table, "as2")

    def test_topic_surface_as2(self):
        surface = P.topic_surface(P.get_profile("as2"))
        self.assertEqual(surface, {
            "rt/lf/sportmodestate": "unitree_go.msg.dds_.SportModeState_",
            "rt/lowstate": "unitree_hg.msg.dds_.LowState_",
            "rt/lf/bmsstate": "unitree_hg.msg.dds_.BmsState_",
        })

    def test_topic_surface_go2(self):
        surface = P.topic_surface(P.get_profile("go2"))
        self.assertEqual(surface, {
            "rt/lf/sportmodestate": "unitree_go.msg.dds_.SportModeState_",
            "rt/lf/lowstate": "unitree_go.msg.dds_.LowState_",
            "rt/utlidar/robot_odom": "nav_msgs.msg.dds_.Odometry_",
        })

    def test_unknown_profile_exits(self):
        with self.assertRaises(SystemExit):
            P.get_profile("g3")


if __name__ == "__main__":
    unittest.main()

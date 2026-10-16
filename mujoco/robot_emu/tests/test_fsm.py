"""FSM table unit tests -- the audit's behavioural claims, pinned.

Pure logic, no DDS/SDK needed (L0 of the test bench's L0->L1->L2 ladder).
Run from mujoco/:  python3 -m unittest robot_emu.tests.test_fsm -v
"""
from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, str(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))

from robot_emu import fsm as F


class TestAs2Gating(unittest.TestCase):
    """Real-dog audit: Move only in NORMAL_WALK(3); BalanceStand
    toggles 2<->3; StopMove never switches."""

    def setUp(self):
        self.fsm = F.EmuFSM("as2")

    def test_move_refused_until_normal_walk(self):
        self.assertEqual(self.fsm.mode, F.SITTING)
        self.assertEqual(self.fsm.handle(F.MOVE), -1)
        self.fsm.set_mode(F.STAND_UP)
        self.assertEqual(self.fsm.handle(F.MOVE), -1)
        self.fsm.set_mode(F.NORMAL_WALK)
        self.assertEqual(self.fsm.handle(F.MOVE), 0)

    def test_stand_sequence_sitting_to_walk_ready(self):
        self.assertEqual(self.fsm.handle(F.STAND_UP_CALL), 0)
        self.assertEqual(self.fsm.mode, F.STAND_UP)
        self.assertEqual(self.fsm.handle(F.BALANCE_STAND), 0)
        self.assertEqual(self.fsm.mode, F.NORMAL_WALK)

    def test_balancestand_is_a_toggle_not_an_assert(self):
        """The exact regression the pilot profile guards: an unconditional
        per-motion BalanceStand would walk NORMAL_WALK back to STAND_UP."""
        self.fsm.set_mode(F.NORMAL_WALK)
        self.assertEqual(self.fsm.handle(F.BALANCE_STAND), 0)
        self.assertEqual(self.fsm.mode, F.STAND_UP)
        self.assertEqual(self.fsm.handle(F.MOVE), -1)

    def test_stopmove_never_switches(self):
        for mode in (F.SITTING, F.STAND_UP, F.NORMAL_WALK):
            self.fsm.set_mode(mode)
            self.assertEqual(self.fsm.handle(F.STOP_MOVE), 0)
            self.assertEqual(self.fsm.mode, mode)

    def test_stand_down_from_any_state(self):
        for mode in (F.STAND_UP, F.NORMAL_WALK):
            self.fsm.set_mode(mode)
            self.assertEqual(self.fsm.handle(F.STAND_DOWN), 0)
            self.assertEqual(self.fsm.mode, F.SITTING)


class TestPermissiveTable(unittest.TestCase):
    """go2/b2: Move accepted from any state; ClassicWalk valid in both client
    id spaces (go2 2049, b2 1049); stand/sit switch modes."""

    def setUp(self):
        self.fsm = F.EmuFSM("sport_permissive")

    def test_move_always_accepted(self):
        for mode in (F.SITTING, F.STAND_UP, F.NORMAL_WALK):
            self.fsm.set_mode(mode)
            self.assertEqual(self.fsm.handle(F.MOVE), 0)

    def test_classicwalk_both_id_spaces(self):
        self.assertEqual(self.fsm.handle(F.CLASSIC_WALK_GO2), 0)
        self.assertEqual(self.fsm.mode, F.NORMAL_WALK)
        self.fsm.set_mode(F.SITTING)
        self.assertEqual(self.fsm.handle(F.CLASSIC_WALK_B2), 0)
        self.assertEqual(self.fsm.mode, F.NORMAL_WALK)

    def test_unknown_api_is_a_noop_success(self):
        self.assertEqual(self.fsm.handle(9999), 0)
        self.assertEqual(self.fsm.mode, F.SITTING)


class TestUnknownTable(unittest.TestCase):
    def test_rejected(self):
        with self.assertRaises(ValueError):
            F.EmuFSM("no-such-model")


if __name__ == "__main__":
    unittest.main()

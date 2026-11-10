"""Depth monitor unit tests (synthetic navcore frames). Run from mujoco/:
  python3 -m unittest sim.tests.test_depth_monitor -v
"""
from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from sim import contract_v2 as V  # noqa: E402
from sim import depth_monitor as M  # noqa: E402

SPEC = V.DepthSpec("navcore", 68, 80, 38.76875, True, 3.0,
                   min_range_m=V.NAVCORE_MIN_RANGE_M)


def floor_frame() -> np.ndarray:
    """A level camera over a floor: the upper half misses (beyond 3 m), the
    lower half reads the floor nearer towards the bottom."""
    frame = np.zeros((80, 68), np.float32)
    frame[40:] = np.linspace(2.5, 0.4, 40, dtype=np.float32)[:, None]
    return frame


def feed(monitor, frame, n):
    for _ in range(n):
        monitor.update(frame)
    return monitor.fault


class TestDepthMonitor(unittest.TestCase):
    def test_normal_frames_are_healthy(self):
        m = M.DepthMonitor(SPEC)
        self.assertIsNone(feed(m, floor_frame(), 50))
        self.assertEqual(m.near, 0.0)
        self.assertLess(m.blind, 0.1)

    def test_something_up_close_then_clear(self):
        m = M.DepthMonitor(SPEC)
        near = np.full((80, 68), 0.20, np.float32)
        self.assertIsNone(m.update(near))                   # one frame is not enough
        self.assertEqual(m.update(near), "near")
        clear_frames = int(round(M.CLEAR_S / M.FRAME_DT))
        self.assertEqual(feed(m, floor_frame(), clear_frames - 1), "near")
        self.assertIsNone(m.update(floor_frame()))          # 1 s healthy

    def test_a_relapse_restarts_the_clearing(self):
        m = M.DepthMonitor(SPEC)
        near = np.full((80, 68), 0.20, np.float32)
        feed(m, near, 2)
        feed(m, floor_frame(), 5)
        m.update(near)                                      # one bad frame again
        self.assertEqual(feed(m, floor_frame(), 9), "near")
        self.assertIsNone(m.update(floor_frame()))

    def test_covered_lens(self):
        m = M.DepthMonitor(SPEC)
        covered = np.zeros((80, 68), np.float32)            # all misses
        self.assertIsNone(feed(m, covered, M.BLIND_FRAMES - 1))
        self.assertEqual(m.update(covered), "blind")
        # nearer than the minimum range reads as nothing too
        m = M.DepthMonitor(SPEC)
        self.assertEqual(feed(m, np.full((80, 68), 0.05, np.float32), M.BLIND_FRAMES), "blind")

    def test_a_hand_sized_patch_is_left_to_the_student(self):
        m = M.DepthMonitor(SPEC)
        frame = floor_frame()
        frame[40:72, 20:47] = 0.25                          # 16% of the frame
        self.assertIsNone(feed(m, frame, 30))
        self.assertGreater(m.near, 0.1)

    def test_reset(self):
        m = M.DepthMonitor(SPEC)
        feed(m, np.full((80, 68), 0.20, np.float32), 3)
        m.reset()
        self.assertIsNone(m.fault)
        self.assertIsNone(m.update(floor_frame()))


if __name__ == "__main__":
    unittest.main()

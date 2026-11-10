#!/usr/bin/env python3
"""Depth health monitor: is the navcore frame something the depth student was
trained on?

The students learnt terrain: floors, stairs, slopes. Something filling the
view up close (a person or a box right at the lens) reads to them as a wall or
a riser, and they step at it even on a zero command -- in tinynav-sim a frame
held at 0.20 m made the Go2 student roll 22-25 deg and the B2W student walk
38 cm on its own (2026-10-29). A lens covered or blinded leaves them on
proprioception alone. The runtime then drives the robot's blind policy
instead (sim/runtime.py fallback), back on the student once the frame has
been healthy for CLEAR_S.

Per frame (the raw optical-Z render, metres, 0 = miss; the student's
DepthSpec says what reads as nothing):
  near   share of the frame read nearer than NEAR_M: at least NEAR_SHARE for
         NEAR_FRAMES frames in a row is a fault. Normal running stays at or
         under 0.28 (the Go2 student climbing map3's L0 at 0.3 m/s, risers
         close to the lens); a hand-sized patch is 0.16 and is left to the
         student, which shrugged it off.
  blind  share of the lower half read as nothing: at least BLIND_SHARE for
         BLIND_FRAMES frames in a row is a fault. The lower half sees the
         floor within range in normal running (at most 0.36 missing).
"""
from __future__ import annotations

import numpy as np

from .contract import CONTROL_DT, DEPTH_UPDATE_PERIOD_STEPS

NEAR_M = 0.35
NEAR_SHARE = 0.5
NEAR_FRAMES = 2       # 0.2 s at the 10 Hz depth rate
BLIND_SHARE = 0.9
BLIND_FRAMES = 3
CLEAR_S = 1.0         # healthy this long before the student drives again
FRAME_DT = CONTROL_DT * DEPTH_UPDATE_PERIOD_STEPS   # s between depth frames


class DepthMonitor:
    """Feed every depth frame to update(); `fault` is None (healthy) or the
    reason ("near" / "blind") while the frame is not to be trusted."""

    def __init__(self, spec) -> None:
        self.spec = spec
        self.fault: str | None = None
        self.near = 0.0     # the last frame's shares, for status lines
        self.blind = 0.0
        self._near_run = 0
        self._blind_run = 0
        self._healthy_s = 0.0

    def reset(self) -> None:
        self.fault = None
        self._near_run = self._blind_run = 0
        self._healthy_s = 0.0

    def update(self, depth_z: np.ndarray) -> str | None:
        depth = np.asarray(depth_z, dtype=np.float32)
        seen = (np.isfinite(depth) & (depth > 0.0) & (depth >= self.spec.min_range_m)
                & (depth < self.spec.max_m))
        self.near = float((seen & (depth < NEAR_M)).mean())
        self.blind = float(1.0 - seen[depth.shape[0] // 2:].mean())
        self._near_run = self._near_run + 1 if self.near >= NEAR_SHARE else 0
        self._blind_run = self._blind_run + 1 if self.blind >= BLIND_SHARE else 0
        reason = ("near" if self._near_run >= NEAR_FRAMES
                  else "blind" if self._blind_run >= BLIND_FRAMES else None)
        if reason is not None:
            self.fault, self._healthy_s = reason, 0.0
        elif self.fault is not None:
            bad = self._near_run > 0 or self._blind_run > 0
            self._healthy_s = 0.0 if bad else self._healthy_s + FRAME_DT
            if self._healthy_s >= CLEAR_S - 1e-9:
                self.fault = None
        return self.fault

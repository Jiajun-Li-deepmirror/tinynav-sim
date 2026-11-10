#!/usr/bin/env python3
"""Wall guard: no forward speed into a wall the depth camera sees.

The depth students take a wall right in front of them for a stair riser and
try to climb it (2026-10-28: Go2 reared and rolled, AS2 and B2 fell when held
against a map3 wall). WallGuard caps the forward command at
GAIN * (D - STOP_M), D the distance from the camera to the nearest wall
straight ahead, so a robot driven at a wall slows down and stands STOP_M in
front of it (~0.5 m: the camera is the robot's front); backing off,
turning and strafing stay free.

A wall is told from stairs by its height. Of the points the policy's newest
depth frame shows inside the robot's corridor (|lateral| <= HALF_WIDTH
around the heading) and within a band from FLOOR_CLEAR above the ground under
the feet (BAND_LOW below the camera when the feet are unknown) to BAND_HIGH
above the camera (gravity-aligned), the ones within WALL_DEPTH of one forward
distance must span WALL_SPAN of height: a wall or a box does, a stair riser
(14-19 cm on map3), a kerb or a tread never does, and the next riser is a
tread further on. The band reaches down to the ground since 2026-10-29: from
just under the camera it missed a 0.6 m box in front of AS2, B2 and B2-W,
whose cameras sit higher, and AS2 climbed onto it and fell.
The frame is the policy's (contract_v2.DepthSpec: nearer than the camera's
minimum range reads as a miss), so the guard sees what the robot's own
camera would. Between frames (10 Hz) D shrinks by the distance walked since.
Navcore frames only: the PIE camera looks 20 deg down and sees too little
above itself.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

STOP_M = 0.45       # camera to wall where the forward command reaches zero: the
                    # students start to climb ~0.5 m out (0.30 still let Go2
                    # rear up to 29 deg; 0.45 kept all three under 6 deg); a
                    # robot's own Robot.guard_stop_m wins (B2: 0.60)
GAIN = 1.5          # 1/s: forward speed allowed per metre beyond STOP_M
REACH_M = 1.5       # walls nearer than this count (1.5 m/s is allowed at 1.3 m)
HALF_WIDTH = 0.20   # m, the corridor the robot sweeps
BAND_LOW = 0.10     # m below the camera (or from FLOOR_CLEAR above the ground) ...
BAND_HIGH = 0.20    # ... to this much above it
FLOOR_CLEAR = 0.05  # m above the lowest foot: the floor itself is no wall
WALL_SPAN = 0.22    # m of height at one distance: more than a riser
WALL_DEPTH = 0.05   # m, "one distance"
MIN_POINTS = 20
_BIN = 0.01         # m


def wall_distance(frame_m: np.ndarray, spec, cam_pos, cam_xmat, heading,
                  floor_z: float | None = None) -> float | None:
    """Distance (m) along the horizontal unit `heading` from the camera to the
    nearest wall in `frame_m` (height x width, metres; spec.max_m = a miss),
    or None. spec: the frame's contract_v2.DepthSpec; cam_pos / cam_xmat: the
    camera's world pose (MuJoCo camera axes: x right, y up, looking along -z);
    floor_z: the world height of the ground under the feet, if known."""
    frame = np.asarray(frame_m, dtype=np.float64)
    h, w = frame.shape
    u = (np.arange(w) - 0.5 * (w - 1)) / spec.focal_px
    v = (np.arange(h) - 0.5 * (h - 1)) / spec.focal_px
    x, y = np.meshgrid(u, v)                  # optical: x right, y down, z ahead
    seen = frame < spec.max_m - 1e-3
    z = frame if spec.optical_z else frame / np.sqrt(x * x + y * y + 1.0)
    optical = np.stack((x * z, y * z, z), axis=-1)[seen]
    rel = (optical * (1.0, -1.0, -1.0)) @ np.asarray(cam_xmat, dtype=np.float64).reshape(3, 3).T
    fwd = np.array((heading[0], heading[1], 0.0))
    side = np.array((-heading[1], heading[0], 0.0))
    r, lat, up = rel @ fwd, rel @ side, rel[:, 2]
    low = -BAND_LOW if floor_z is None else min(-BAND_LOW, floor_z + FLOOR_CLEAR - float(cam_pos[2]))
    keep = ((r > 0.0) & (r < REACH_M) & (np.abs(lat) <= HALF_WIDTH)
            & (up >= low) & (up <= BAND_HIGH))
    if np.count_nonzero(keep) < MIN_POINTS:
        return None
    r, up = r[keep], up[keep]
    b = (r / _BIN).astype(np.intp)
    n = int(math.ceil(REACH_M / _BIN)) + 1
    count = np.bincount(b, minlength=n)
    low = np.full(n, np.inf)
    high = np.full(n, -np.inf)
    near = np.full(n, np.inf)
    np.minimum.at(low, b, up)
    np.maximum.at(high, b, up)
    np.minimum.at(near, b, r)
    k = int(round(WALL_DEPTH / _BIN))
    span = sliding_window_view(high, k).max(-1) - sliding_window_view(low, k).min(-1)
    points = sliding_window_view(count, k).sum(-1)
    wall = np.flatnonzero((span >= WALL_SPAN) & (points >= MIN_POINTS))
    # the nearest point of the first window that holds a wall
    return float(near[wall[0]:wall[0] + k].min()) if wall.size else None


class WallGuard:
    """Command source: `source`'s command with its forward part capped in front
    of walls. runtime() gives the PieRuntime driving now (its depth frame,
    camera and base); engaged() says whether the guard applies (hil: while
    the keys or the pad drive). `enabled` is the panel's switch; `distance`
    the last wall distance (None: none seen) and `limiting` whether it cut
    the command, for status lines."""

    def __init__(self, source, runtime, engaged=None) -> None:
        self.source = source
        self._runtime = runtime
        self._engaged = engaged
        self.enabled = True
        self.distance: float | None = None
        self.limiting = False
        self._rt = None
        self._feet = None         # the runtime's foot geoms
        self._stop_m = STOP_M     # its robot's stop distance
        self._raw = None          # the depth capture ...
        self._captured = None     # ... its wall distance, base position and heading

    def vector(self) -> np.ndarray:
        cmd = np.asarray(self.source.vector(), dtype=np.float32).copy()
        self.distance = self._wall()
        self.limiting = False
        if (self.distance is not None and self.enabled and cmd[0] > 0.0
                and (self._engaged is None or self._engaged())):
            allowed = max(0.0, GAIN * (self.distance - self._stop_m))
            if cmd[0] > allowed:
                cmd[0] = allowed
                self.limiting = True
        return cmd

    def _wall(self) -> float | None:
        """The wall distance now: the newest frame's, less the distance walked
        along its heading since it was captured."""
        rt = self._runtime()
        if rt is not self._rt:
            self._rt, self._raw, self._captured = rt, None, None
            self._feet = _foot_geoms(rt)
            robot = getattr(getattr(rt, "b", None), "robot", None)
            self._stop_m = getattr(robot, "guard_stop_m", None) or STOP_M
        spec = getattr(rt, "depth_spec", None)
        if rt is None or not rt.uses_depth or spec is None or spec.pie:
            return None
        root = rt.b.root_qpos_adr
        pos = rt.data.qpos[root:root + 2]
        raw = rt.latest_raw_depth
        if raw is not None and raw is not self._raw:
            self._raw = raw
            w, x, y, z = rt.data.qpos[root + 3:root + 7]
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            heading = np.array((math.cos(yaw), math.sin(yaw)))
            cam = rt.b.depth_camera_id
            floor_z = None
            if self._feet:
                g = np.asarray(self._feet)
                floor_z = float(np.min(rt.data.geom_xpos[g, 2] - rt.model.geom_size[g, 0]))
            distance = wall_distance(
                rt.depth_frame() * spec.max_m, spec, rt.data.cam_xpos[cam],
                rt.data.cam_xmat[cam], heading, floor_z)
            self._captured = None if distance is None else (distance, pos.copy(), heading)
        if self._captured is None:
            return None
        distance, start, heading = self._captured
        return distance - float((pos - start) @ heading)

    def consume_reset(self) -> bool:
        return self.source.consume_reset()

    def zero(self) -> None:
        self.source.zero()

    def reset(self) -> None:
        """Back at the spawn: the source's reset too (AccelLimit), and no wall
        until the next frame."""
        reset = getattr(self.source, "reset", None)
        if reset is not None:
            reset()
        self._raw = self._captured = None

    def start(self):
        return self

    def stop(self) -> None:
        self.source.stop()


def _foot_geoms(rt) -> list[int]:
    """The robot's foot (or wheel) collision geoms: the ground under them is
    their lowest point. Empty when the runtime has no model (tests)."""
    model = getattr(rt, "model", None)
    if model is None:
        return []
    import mujoco
    return [g for g in range(model.ngeom)
            if "foot" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]


__all__ = ["STOP_M", "WallGuard", "wall_distance"]

"""Posture reset: march in place when a standing robot's legs are off its stance.

The sim's copy of the real controller's ThermalGuard posture reset (go2-depth
robot_ctrl deploy/include/FSM/ThermalGuard.h, 2026-11-09; the user: after a
hard stop with a payload the Go2-W legs stood off the default stance and the
next move nearly fell; marching in place puts the stance back). Same rule and
the same per-robot settings as each robot's config.yaml (ThermalGuard
posture_march_rad / posture_settle_s / posture_march_s / posture_march_max):

  once the zero command has lasted `settle_s`, a leg joint more than `rad` off
  the policy's default angle starts `march_s` of marching in place, at most
  `max_tries` times per stop (each checked `settle_s` after the last march);
  any non-zero command ends it and starts the count again.

Only policies trained to march (contract.gait_march) march, as on the robot.
The leg joints are the policy's position-controlled ones (wheels excluded),
measured against the angles its actions are offset from.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sim import contract as C


@dataclass(frozen=True)
class PostureResetCfg:
    rad: float = 0.15        # posture_march_rad
    settle_s: float = 1.0    # posture_settle_s
    march_s: float = 1.0     # posture_march_s (2.0 until 2026-11-10)
    max_tries: int = 2       # posture_march_max


# = robot_ctrl deploy/robots/<robot>/config/config.yaml ThermalGuard (posture_march_rad 0 / absent: off).
# Keep the two in step: a robot turned on there is turned on here.
ROBOT_CONFIG: dict[str, PostureResetCfg] = {
    "go2w": PostureResetCfg(0.15, 1.0, 1.0, 2),
}
DEFAULT_CFG = PostureResetCfg()   # what the panel's checkbox uses on a robot that has it off


class PostureReset:
    """Per control tick: step(rt) after rt.tick() returns whether to march."""

    def __init__(self, robot: str) -> None:
        self.robot = robot
        self.cfg = ROBOT_CONFIG.get(robot, DEFAULT_CFG)
        self.enabled = robot in ROBOT_CONFIG    # as on the robot
        self._idle = 0.0
        self._until = 0.0
        self.tries = 0
        self.marching = False
        self.deviation = 0.0     # the leg joints' largest distance from the default (rad)
        self.worst_joint = ""

    def reset(self) -> None:
        self._idle = self._until = 0.0
        self.tries = 0
        self.marching = False

    @staticmethod
    def leg_deviation(rt) -> tuple[float, int]:
        """(largest |q - default| over the position-controlled joints, its index in those joints)."""
        worst, worst_i, i = 0.0, -1, 0
        for _ids, _part, _scale, offset, _dof, qadr in rt._action_map:
            if offset is None:
                continue
            e = np.abs(rt.data.qpos[qadr] - offset)
            j = int(np.argmax(e))
            if e[j] > worst:
                worst, worst_i = float(e[j]), i + j
            i += len(qadr)
        return worst, worst_i

    def step(self, rt) -> bool:
        zero = not np.any(rt.command)
        self._idle = self._idle + C.CONTROL_DT if zero else 0.0
        self.deviation, worst_i = self.leg_deviation(rt)
        names = [j for g in rt.contract.action_groups if g.kind == "position" for j in g.joints]
        self.worst_joint = names[worst_i] if 0 <= worst_i < len(names) else ""
        if not getattr(rt.contract, "gait_march", False):
            self.marching = False
            return False
        if self._idle <= 0.0:
            self._until, self.tries = 0.0, 0
        elif self.enabled and self.cfg.rad > 0.0:
            c = self.cfg
            # half a step of slack: summed float steps fall just short of whole seconds
            if self._idle >= self._until + c.settle_s - 0.5 * C.CONTROL_DT and self.tries < c.max_tries:
                if self.deviation > c.rad:
                    self._until = self._idle + c.march_s
                    self.tries += 1
                    print(f"[POSTURE] reset {self.tries} / {c.max_tries}: {self.worst_joint or worst_i} "
                          f"{self.deviation:.2f} rad off its default, marching {c.march_s:g} s", flush=True)
        else:
            self._until = 0.0
        marching = self.enabled and self._idle > 0.0 and self._idle < self._until
        if marching != self.marching:
            print(f"[POSTURE] {'start' if marching else 'stop'} marching in place "
                  f"(idle {self._idle:.1f} s, deviation {self.deviation:.2f} rad)", flush=True)
        self.marching = marching
        return marching

    def status(self) -> str:
        """Status-line suffix: the stance deviation, and the reset while it marches."""
        s = f" · 站姿偏 {self.deviation:.2f} rad"
        if self.marching:
            s += f" · 踏步复位 {self.tries}/{self.cfg.max_tries}"
        return s


def hip_out_deg(rt) -> tuple[tuple[str, ...], np.ndarray]:
    """The hips' angle minus the policy's default (deg), away from the body +
    (left hips +dq, right ones -dq), as the real controller's thermal CSV
    columns hip*_out_deg: (labels FL / FR / RL / RR, values)."""
    labels, values = [], []
    for group, (_ids, _part, _scale, offset, _dof, qadr) in zip(rt.contract.action_groups, rt._action_map):
        if offset is None:
            continue
        for j, name in enumerate(group.joints):
            if name.endswith("_hip_joint"):
                leg = name.split("_")[0]
                side = 1.0 if leg[1] == "L" else -1.0
                labels.append(leg)
                values.append(side * np.degrees(float(rt.data.qpos[qadr[j]] - offset[j])))
    return tuple(labels), np.asarray(values)

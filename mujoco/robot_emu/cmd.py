"""EmuCmdSource: the cmd_source PieRuntime consumes, fed by sport Move RPCs.

Same contract as hil/ros.RosCmdSource (vector/consume_reset/zero/start/stop)
so the plant and runtime stay untouched. Latest accepted Move wins; 0.5 s
without a Move the chassis holds zero (real sport Move is a setpoint, and
pilot's cmd_vel_control streams anyway).
"""
from __future__ import annotations

import threading
import time

import numpy as np


class EmuCmdSource:
    WATCHDOG_S = 0.5

    def __init__(self, vel_clip=(0.5, 0.5, 1.2)) -> None:
        self._lock = threading.Lock()
        self._cmd = np.zeros(3, dtype=np.float32)
        self._t_last = 0.0
        self._clip = vel_clip
        self._clipped = 0
        self.moves_accepted = 0
        self.moves_refused = 0

    def accept(self, vx: float, vy: float, wz: float, code: int) -> int:
        """Service layer hands in one Move RPC result."""
        if code != 0:
            with self._lock:
                self.moves_refused += 1
            return code
        vx = float(np.clip(vx, -self._clip[0], self._clip[0]))
        vy = float(np.clip(vy, -self._clip[1], self._clip[1]))
        wz = float(np.clip(wz, -self._clip[2], self._clip[2]))
        with self._lock:
            self._cmd = np.asarray((vx, vy, wz), dtype=np.float32)
            self._t_last = time.monotonic()
            self.moves_accepted += 1
        return 0

    def stop_move(self) -> None:
        with self._lock:
            self._cmd = np.zeros(3, dtype=np.float32)
            self._t_last = 0.0

    def vector(self) -> np.ndarray:
        with self._lock:
            if self._t_last and time.monotonic() - self._t_last > self.WATCHDOG_S:
                self._cmd = np.zeros(3, dtype=np.float32)
                self._t_last = 0.0
            return self._cmd

    def consume_reset(self) -> bool:
        return False

    def zero(self) -> None:
        self.stop_move()

    @property
    def clipped(self) -> int:
        return self._clipped

    def start(self):
        return self

    def stop(self) -> None:
        pass

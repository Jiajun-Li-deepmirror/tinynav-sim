"""Scenario injection: test hooks that bend the state feed / command gate
the way a real world would. Commands arrive as JSON on rt/emu/scenario
(unitree std_msgs String_) -- an emu-internal topic, not part of the model
conformance surface.

- bms_ma / bms_soc / bms_status / error_code: state-feed values (on-pad
  charging, fault codes).
- refuse_move: answer every Move with -1 while set (obstacle refusal /
  motion watchdog path -- the ChassisWatch "not executing" alarm becomes
  reachable from the bench).
- cmd_blackout_s: refuse every Move for the next N seconds (a dropped
  command link mid-motion).

Defaults mirror a resting dog: discharging, soc 88 (the audited AS2 value).
"""
from __future__ import annotations

import json
import threading
import traceback

TOPIC = "rt/emu/scenario"

# go-family bms_state.status values (unitree_go BmsState)
STATUS_CHG = 7
STATUS_DCHG = 8

DEFAULTS = {
    "bms_ma": -970,        # resting discharge current (mA), AS2 measured
    "bms_soc": 88,
    "bms_status": STATUS_DCHG,
    "error_code": 0,
    "refuse_move": False,
}


class ScenarioState:
    def __init__(self, clock=None) -> None:
        import time as _time
        self._clock = clock or _time.monotonic
        self._lock = threading.Lock()
        self._s = dict(DEFAULTS)
        self._blackout_until = 0.0
        self._reset = False

    def get(self, key: str):
        with self._lock:
            return self._s.get(key)

    def blackout_active(self) -> bool:
        with self._lock:
            return self._clock() < self._blackout_until

    def apply(self, cmd: dict) -> None:
        with self._lock:
            if cmd.get("clear"):
                self._s = dict(DEFAULTS)
                self._blackout_until = 0.0
            if cmd.get("reset"):
                self._reset = True
            if "cmd_blackout_s" in cmd:
                self._blackout_until = self._clock() + float(cmd["cmd_blackout_s"])
            for k in DEFAULTS:
                if k in cmd:
                    self._s[k] = cmd[k]

    def consume_reset(self) -> bool:
        with self._lock:
            was, self._reset = self._reset, False
            return was


class ScenarioService:
    def __init__(self, state: ScenarioState) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        self._state = state
        self._sub = ChannelSubscriber(TOPIC, String_)
        self._sub.Init(self._on_cmd, 10)

    def _on_cmd(self, msg) -> None:
        try:
            self._state.apply(json.loads(msg.data))
        except Exception:      # noqa: BLE001 -- reader thread must stay alive
            traceback.print_exc()

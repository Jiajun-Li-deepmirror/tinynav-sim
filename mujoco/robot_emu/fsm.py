"""Model FSM tables: which sport API calls each chassis accepts per state,
and what a call does to the state. Pure logic, no DDS.

State ids 2/3 are the real AS2 FSM ids (real-dog audit); 0 is the emu's
name for sitting/not-standing (real id unmeasured -- pilot only logs it).
The AS2 column is audit-backed: Move only in NORMAL_WALK (else -1),
BalanceStand toggles STAND_UP<->NORMAL_WALK, StopMove never switches.
go2/b2 are permissive like the real dogs (Move always accepted).
"""
from __future__ import annotations

import threading
from dataclasses import dataclass

SITTING = 0
STAND_UP = 2
NORMAL_WALK = 3

# sport api ids (go2/as2/b2 share the 1001-1034 space; ClassicWalk lives in
# different id spaces per client: go2 2049, b2 1049)
DAMP = 1001
BALANCE_STAND = 1002
STOP_MOVE = 1003
STAND_UP_CALL = 1004
STAND_DOWN = 1005
RECOVERY_STAND = 1006
MOVE = 1008
SWITCH_GAIT = 1011
GET_STATE = 1034
CLASSIC_WALK_GO2 = 2049
CLASSIC_WALK_B2 = 1049


@dataclass(frozen=True)
class Rule:
    code: int = 0            # RPC answer code (0 ok, -1 refused)
    to: int | None = None    # state after the call; None = stay


# table: profile -> api_id -> (default rule, {state: rule})
# UNVERIFIED marks emu-side guesses the real-dog audit did not cover; the
# smoke checklist (docs/multi-robot-sim-testbench.md) carries them to the rig.
TABLES: dict[str, dict[int, tuple[Rule, dict[int, Rule]]]] = {
    "as2": {
        DAMP: (Rule(), {}),
        # UNVERIFIED: BalanceStand from SITTING refused (-1) -- the audit only
        # showed the toggle between the two standing states.
        BALANCE_STAND: (Rule(-1), {
            STAND_UP: Rule(0, NORMAL_WALK),
            NORMAL_WALK: Rule(0, STAND_UP),
        }),
        STOP_MOVE: (Rule(), {}),
        # UNVERIFIED: StandUp away from SITTING is a no-op success; the audit
        # only exercised it from sitting.
        STAND_UP_CALL: (Rule(), {SITTING: Rule(0, STAND_UP)}),
        STAND_DOWN: (Rule(0, SITTING), {}),
        RECOVERY_STAND: (Rule(), {SITTING: Rule(0, STAND_UP)}),
        MOVE: (Rule(-1), {NORMAL_WALK: Rule()}),
        SWITCH_GAIT: (Rule(), {}),
        GET_STATE: (Rule(), {}),
    },
    "sport_permissive": {   # go2/go2w/b2/b2w
        DAMP: (Rule(), {}),
        BALANCE_STAND: (Rule(0, NORMAL_WALK), {}),
        STOP_MOVE: (Rule(), {}),
        STAND_UP_CALL: (Rule(0, STAND_UP), {}),
        STAND_DOWN: (Rule(0, SITTING), {}),
        RECOVERY_STAND: (Rule(0, STAND_UP), {}),
        MOVE: (Rule(), {}),
        SWITCH_GAIT: (Rule(), {}),
        GET_STATE: (Rule(), {}),
        CLASSIC_WALK_GO2: (Rule(0, NORMAL_WALK), {}),
        CLASSIC_WALK_B2: (Rule(0, NORMAL_WALK), {}),
    },
}


class EmuFSM:
    """Thread-safe enough for the bench: SDK reader thread calls handle(),
    the state-feed thread reads mode."""

    def __init__(self, table: str) -> None:
        if table not in TABLES:
            raise ValueError(f"unknown FSM table {table!r}")
        self._rules = TABLES[table]
        self._mode = SITTING
        self._lock = threading.Lock()

    @property
    def mode(self) -> int:
        with self._lock:
            return self._mode

    def set_mode(self, mode: int) -> None:
        with self._lock:
            self._mode = mode

    def handle(self, api_id: int) -> int:
        """Apply one sport API call; returns the RPC answer code."""
        default, per_state = self._rules.get(api_id, (Rule(), {}))
        with self._lock:
            rule = per_state.get(self._mode, default)
            if rule.to is not None:
                self._mode = rule.to
            return rule.code

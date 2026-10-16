"""robot_emu: a Unitree-DDS face for the MuJoCo dog.

The sim grows the real robot's DDS surface (docs/multi-robot-sim-testbench.md
§5): sport RPC service (rt/api/sport/request + response) with per-model FSM
gating, and state feedback (the lf topics the stack consumes:
rt/lf/sportmodestate; rt/lf/lowstate go family / rt/lowstate + rt/lf/bmsstate hg;
rt/utlidar/robot_odom as the model dictates) straight from the physics truth.
Production pilot code connects unmodified; conformance against the real-dog
audit list is the objective gate.

SDK availability: unitree_sdk2py comes from site-packages — the mjsim image
pip-installs the pinned source checkout (mujoco/robot_emu/vendor/
unitree_sdk2py, upstream unitree_sdk2_python @814556d, the same commit
tinynav/pyproject.toml pins; submodule-style, converted at commit time).
This package is import-pure: only the modules that touch DDS need the SDK,
so L0 logic tests run with no install at all. For an in-tree host run of
the DDS modules, pip-install that checkout first.
"""
from __future__ import annotations


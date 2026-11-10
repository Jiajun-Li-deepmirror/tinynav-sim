#!/usr/bin/env python3
"""PIE 50 Hz control runtime over a native-MuJoCo plant.

Adapted from parkour_mjlab deploy/pie/sim2sim/go2_pie_sim2sim.py
(train/orient-ablation): depth push every DEPTH_UPDATE_PERIOD_STEPS ticks
(10 Hz), observation -> term-major history -> ONNX -> ctrl = q_target ->
PHYSICS_STEPS_PER_CONTROL sub-steps (PD lives in the actuator model).
The command source is injected (keyboard for the live host, scripted for
headless smokes); the plant is pie_model's map3 or flat builder.

The policy's contract (sim/contract_v2.py) decides the observation layout and
which actuators its actions drive (leg position targets, wheel velocity
targets), so any registered robot runs here; a policy whose joints are not
the plant's is refused. Blind policies (no depth input) skip the camera.
"""

from __future__ import annotations

import time

import mujoco
import numpy as np

from . import contract as C
from .contract_v2 import PIE_DEPTH
from .depth_monitor import DepthMonitor
from .plant import Bindings, place_depth_shell, reset_to_spawn

# Joint-target safety clamp around the default pose, as the real-robot deploy
# applies it (go2-blind deploy.yaml: default +- 2.0 rad = raw action +- 8).
# Training had none and it bites on ~0.005% of steps; last_action stays the
# raw policy output.
ACTION_CLIP_RAD = 2.0

# A policy may stand within this much (rad, per joint) of the robot's registry
# stance; the plant then takes the policy's (go2-blind's finals of 2026-10-27
# stand their hips at 0, the earlier exports at +-0.1).
DEFAULT_POSE_TOLERANCE = 0.25


class DepthCamera:
    """Native MuJoCo optical-Z renderer matching the raw PIE camera (height x
    width: PIE's 60 x 106 by default, a navcore student's frame otherwise)."""

    def __init__(self, model: mujoco.MjModel, camera_id: int,
                 height: int = C.DEPTH_HEIGHT, width: int = C.DEPTH_RAW_WIDTH) -> None:
        self._model = model
        self._camera_id = camera_id
        self._shape = (int(height), int(width))
        # Single-sample buffer: a multisampled depth buffer resolves to one
        # off-centre sample per pixel (NVIDIA EGL: ~0.5 cm median error, cm
        # at stair edges); without MSAA the depth equals mj_ray's optical Z.
        # offsamples is read when the context is made, so restore it after.
        offsamples = int(model.vis.quality.offsamples)
        model.vis.quality.offsamples = 0
        try:
            self._renderer = mujoco.Renderer(model, height=self._shape[0], width=self._shape[1])
        finally:
            model.vis.quality.offsamples = offsamples
        self._renderer.enable_depth_rendering()
        self._scene_option = mujoco.MjvOption()
        self._scene_option.geomgroup[:] = 0
        self._scene_option.geomgroup[:3] = 1   # visuals + map3 boxes (group 0/2)
        self._renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self._renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        self._far_clip = float(model.vis.map.zfar * model.stat.extent)
        near_clip = float(model.vis.map.znear * model.stat.extent)
        if not 0.0 < near_clip < C.DEPTH_MIN_M:
            raise ValueError(
                f"depth near plane {near_clip} must be below {C.DEPTH_MIN_M}"
            )
        if self._far_clip * 0.99 <= C.DEPTH_MAX_M:
            raise ValueError("depth far plane does not exceed the PIE cutoff")

    def capture(self, data: mujoco.MjData) -> np.ndarray:
        self._renderer.update_scene(
            data, camera=self._camera_id, scene_option=self._scene_option
        )
        depth = np.asarray(self._renderer.render(), dtype=np.float32)
        if depth.shape != self._shape:
            raise ValueError(f"rendered depth has invalid shape {depth.shape}")
        misses = (
            ~np.isfinite(depth)
            | (depth <= 0.0)
            | (depth >= self._far_clip * 0.99)
        )
        result = depth.copy()
        result[misses] = 0.0
        return np.ascontiguousarray(result, dtype=np.float32)

    def close(self) -> None:
        self._renderer.close()


def _sensor_address(model: mujoco.MjModel, name: str) -> int:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sid < 0:
        raise ValueError(f"model is missing sensor {name!r}")
    return int(model.sensor_adr[sid])


class _Lane:
    """A second policy riding along (the fallback): its contract, the joint
    addresses in its joint order, and its own proprioceptive history, heading
    error and gait clock."""

    def __init__(self, policy, b: Bindings) -> None:
        self.policy = policy
        self.contract = k = policy.contract
        idx = np.asarray([b.joint_names.index(j) for j in k.joint_names])
        self.qpos_adr, self.dof_adr = b.joint_qpos_adr[idx], b.joint_dof_adr[idx]
        self.proprio_history = k.ProprioHistory()
        self.heading = k.HeadingError()
        self.gait = k.GaitClock()

    def reset(self) -> None:
        self.proprio_history.reset()
        for state in (self.heading, self.gait):
            if state is not None:
                state.reset()
        self.policy.reset()


class PieRuntime:
    def __init__(self, model: mujoco.MjModel, b: Bindings,
                 data: mujoco.MjData, policy, depth_camera: DepthCamera,
                 cmd_source, spawn_pos=None, spawn_yaw_deg=None,
                 substep_hook=None, fallback_policy=None) -> None:
        self.model = model
        self.b = b
        self.data = data
        self.policy = policy
        self.camera = depth_camera
        self.cmd_source = cmd_source
        self._spawn_pos = spawn_pos
        self._spawn_yaw_deg = spawn_yaw_deg
        # Optional per-physics-substep callback (called right after each
        # mj_step, inside the control thread -- safe to read MjData here,
        # NOT from other threads). Used by the transport face to sample the
        # IMU and base pose at the full 200 Hz physics rate.
        self.substep_hook = substep_hook
        # Contract of the loaded policy (PIE or a depth student).
        self.contract = policy.contract
        self.uses_depth = bool(getattr(policy, "uses_depth", True))
        self._bind_contract()
        self.last_action = np.zeros(self.contract.NUM_ACTIONS, dtype=np.float32)
        self.proprio_history = self.contract.ProprioHistory()
        # The depth frame the policy was trained on: PIE's camera at the robot's
        # mount, or its metadata's (a navcore student: its mount, field of view
        # and frame size). The shared model's camera is set for it on binding,
        # and a renderer of that size made here (close() closes it).
        self.depth_spec = getattr(self.contract, "depth", None) or PIE_DEPTH
        self._own_camera = None
        if self.uses_depth:
            self._place_depth_camera()
            if not self.depth_spec.pie:
                self._own_camera = self.camera = DepthCamera(
                    model, b.depth_camera_id, *self.depth_spec.frame_shape)
        # heading_error state (heading contracts; None otherwise)
        self.heading = self.contract.HeadingError()
        # go2-blind's speed clock (None: the fixed clock counted by control_step)
        self.gait = self.contract.GaitClock()
        # march in place on a zero command (only policies trained for it,
        # contract.gait_march; set by the host)
        self.march = False
        # The robot's blind policy riding along with a navcore student: its
        # observation and action are computed every tick, so its histories are
        # warm, and it drives while the depth monitor distrusts the frame
        # (sim/depth_monitor.py). None: the student alone.
        # fallback_enabled: the host's switch (the monitor runs either way).
        self.fallback = self.monitor = None
        self.fallback_enabled = True
        self.fallback_active = False
        if (fallback_policy is not None and self.uses_depth and not self.depth_spec.pie
                and self._fallback_fits(fallback_policy.contract)):
            self.fallback = _Lane(fallback_policy, b)
            self.monitor = DepthMonitor(self.depth_spec)
        # the fallback's PD gains, set while it drives (None: the student's)
        self._fallback_gains = (self._pd_gains(fallback_policy, fallback_policy.contract)
                                if self.fallback is not None else None)
        self.depth_history = C.DepthHistory(frame_shape=self.depth_spec.frame_shape)
        self.control_step = 0
        self.latest_raw_depth = None
        self.command = np.zeros(3, dtype=np.float32)   # last command the policy saw
        self.policy_ms = 0.0
        self.physics_ms = 0.0
        self._last_data_time = float(data.time)

    def _place_depth_camera(self) -> None:
        """The plant's depth camera where this policy's was in training: the
        robot's mount and PIE's field of view, or its metadata's. A navcore
        camera also gets the PIE shell out of its view (to the unit's box:
        training has no shell)."""
        spec, cam, robot = self.depth_spec, self.b.depth_camera_id, self.b.robot
        pie = spec.pie or spec.camera_pos is None
        self.model.cam_pos[cam] = robot.camera_pos if pie else spec.camera_pos
        self.model.cam_quat[cam] = robot.camera_quat if pie else spec.camera_quat
        self.model.cam_fovy[cam] = C.DEPTH_FOVY_DEG if spec.pie else spec.fovy_deg
        place_depth_shell(self.model, robot, navcore=not spec.pie)
        # the camera's world pose now (a swap keeps the state: no step runs
        # before the first capture)
        mujoco.mj_camlight(self.model, self.data)

    def close(self) -> None:
        """Release what this runtime made (a navcore student's renderer)."""
        if self._own_camera is not None:
            self._own_camera.close()
            self._own_camera = None

    def _bind_contract(self) -> None:
        """Map the contract's joints and action groups onto the plant, and
        refuse a policy trained for another robot (joints or default pose
        differ); the position actuators take the policy's PD gains (metadata
        joint_stiffness / joint_damping), as the robot does its deploy.yaml's
        (go2-blind's B2W polish 9000 trains its legs at 1.5x, "legkp15").
        A policy standing within DEFAULT_POSE_TOLERANCE of
        the robot's stance is bound and the plant takes its stance: the spawn
        pose and settle targets of reset_to_spawn (the Bindings arrays, shared
        with the host's world)."""
        k, b = self.contract, self.b
        names = b.joint_names
        if sorted(k.joint_names) != sorted(names):
            raise ValueError(
                f"policy contract {k.name!r} joints {k.joint_names} do not "
                f"match robot {b.robot.name!r} joints {names}")
        idx = np.asarray([names.index(j) for j in k.joint_names])
        self._qpos_adr = b.joint_qpos_adr[idx]
        self._dof_adr = b.joint_dof_adr[idx]
        default = np.asarray(k.default_joint_pos, dtype=np.float32)
        if not np.allclose(default, b.default_joint_pos[idx], atol=1e-3):
            if np.max(np.abs(default - b.default_joint_pos[idx])) > DEFAULT_POSE_TOLERANCE:
                raise ValueError(f"policy {k.name!r} default pose does not match "
                                 f"robot {b.robot.name!r}")
            position = self.model.actuator_biasprm[b.actuator_ids[idx], 1] != 0.0
            b.default_joint_pos[idx] = default
            b.ctrl_default[idx] = np.where(position, default, b.ctrl_default[idx])
        # (actuator ids, slice of the action vector, scale, default or None,
        # joint dof addresses, joint qpos addresses)
        self._action_map = []
        start = 0
        for group in k.action_groups:
            jidx = [k.joint_names.index(j) for j in group.joints]
            ids = b.actuator_ids[[names.index(j) for j in group.joints]]
            stop = start + len(ids)
            offset = default[jidx] if group.kind == "position" else None
            self._action_map.append((ids, slice(start, stop),
                                     np.float32(group.scale), offset,
                                     self._dof_adr[jidx], self._qpos_adr[jidx]))
            start = stop
        self._gains_now = None
        self._main_gains = self._pd_gains(self.policy, k)
        self._apply_gains(self._main_gains)
        # The contact solver the policy was trained with, on the shared model:
        # each binding sets it, MuJoCo's pyramidal cone at impratio 1 without
        # the metadata.
        cone = getattr(k, "sim_cone", None) or "pyramidal"
        self.model.opt.cone = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL,
                               "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}[cone]
        self.model.opt.impratio = getattr(k, "sim_impratio", None) or 1.0
        self._reset_brake()

    def _reset_brake(self) -> None:
        """Release the wheel brake, as training does on a reset."""
        # per action group: braked last tick, the angles it holds
        self._braked = [False] * len(self._action_map)
        self._brake_hold = [None] * len(self._action_map)
        self._zero_command_s = 0.0
        # the legs' standstill hold (contract leg_hold_*): held, the targets, its timer
        self._leg_held = [False] * len(self._action_map)
        self._leg_hold_target = [None] * len(self._action_map)
        self._leg_still_s = 0.0

    # -- state -------------------------------------------------------------
    def base_pos(self) -> np.ndarray:
        return np.array(self.data.qpos[self.b.root_qpos_adr:self.b.root_qpos_adr + 3])

    def base_uprightness(self) -> float:
        root = self.b.root_qpos_adr
        quat = self.data.qpos[root + 3:root + 7]
        gz = C.projected_gravity(quat)[2]
        return float(-gz)   # 1.0 upright, ~0 on its side, -1 upside down

    def is_fallen(self) -> bool:
        return self.base_uprightness() < 0.5

    def reset(self) -> None:
        reset_to_spawn(self.model, self.data, self.b, settle=True)
        self.last_action.fill(0.0)
        self.proprio_history.reset()
        if self.heading is not None:
            self.heading.reset()
        if self.gait is not None:
            self.gait.reset()
        self.depth_history.reset()
        self.policy.reset()
        self.control_step = 0
        self.latest_raw_depth = None
        self._reset_brake()
        if self.fallback is not None:
            self.fallback.reset()
            self.monitor.reset()
            self.fallback_active = False
            self._apply_gains(self._main_gains)

    # -- observation (mirrors upstream build_observation) -------------------
    def build_observation(self, command: np.ndarray) -> np.ndarray:
        root = self.b.root_qpos_adr
        quaternion = self.data.qpos[root + 3:root + 7]
        angular_velocity = self.data.sensordata[
            self.b.gyro_adr:self.b.gyro_adr + 3
        ]
        joint_pos = self.data.qpos[self._qpos_adr]
        joint_vel = self.data.qvel[self._dof_adr]
        # Students: training command envelope + deadzone, and the gait phase
        # clock counted from the last reset (control_step; the speed clock
        # advances per call). PIE: unshaped.
        self.command = self.contract.shape_command(command)
        heading_error = (None if self.heading is None
                         else self.heading(quaternion, self.command[2]))
        gait_phase = (None if self.gait is None else
                      self.gait(self.command, march=self.march and self.contract.gait_march))
        return self.contract.build_proprioception(
            angular_velocity, quaternion, self.command,
            joint_pos, joint_vel, self.last_action, self.control_step,
            heading_error=heading_error, gait_phase=gait_phase,
        )

    def _lane_observation(self, lane: _Lane, command: np.ndarray) -> np.ndarray:
        """build_observation for the fallback: the same sensors and applied
        action, in its contract's joint order and terms."""
        root = self.b.root_qpos_adr
        quaternion = self.data.qpos[root + 3:root + 7]
        k = lane.contract
        shaped = k.shape_command(command)
        heading_error = None if lane.heading is None else lane.heading(quaternion, shaped[2])
        gait_phase = (None if lane.gait is None else
                      lane.gait(shaped, march=self.march and k.gait_march))
        return k.build_proprioception(
            self.data.sensordata[self.b.gyro_adr:self.b.gyro_adr + 3], quaternion, shaped,
            self.data.qpos[lane.qpos_adr], self.data.qvel[lane.dof_adr], self.last_action,
            self.control_step, heading_error=heading_error, gait_phase=gait_phase)

    def _pd_gains(self, policy, k):
        """(actuator ids, kp, kd) of the contract's position groups from the
        policy's metadata, or None without joint_stiffness / joint_damping."""
        meta = getattr(policy, "metadata", {})
        if not all(key in meta for key in ("joint_stiffness", "joint_damping")):
            return None
        kp_all = np.array([float(v) for v in meta["joint_stiffness"].split(",")])
        kd_all = np.array([float(v) for v in meta["joint_damping"].split(",")])
        names = self.b.joint_names
        ids, kp, kd = [], [], []
        for group in k.action_groups:
            if group.kind != "position":
                continue
            jidx = [k.joint_names.index(j) for j in group.joints]
            ids.append(self.b.actuator_ids[[names.index(j) for j in group.joints]])
            kp.append(kp_all[jidx])
            kd.append(kd_all[jidx])
        return (np.concatenate(ids), np.concatenate(kp), np.concatenate(kd)) if ids else None

    def _apply_gains(self, gains) -> None:
        """Set the position actuators' PD gains (kp (target - q) - kd dq)."""
        if gains is None or gains is self._gains_now:
            return
        ids, kp, kd = gains
        if not (np.allclose(self.model.actuator_gainprm[ids, 0], kp, atol=1e-3)
                and np.allclose(-self.model.actuator_biasprm[ids, 2], kd, atol=1e-3)):
            print(f"[RT] PD gains kp {kp.min():g}-{kp.max():g}, kd {kd.min():g}-{kd.max():g}", flush=True)
        self.model.actuator_gainprm[ids, 0] = kp
        self.model.actuator_biasprm[ids, 1] = -kp
        self.model.actuator_biasprm[ids, 2] = -kd
        self._gains_now = gains

    def _fallback_fits(self, k) -> bool:
        """The fallback must drive the same actuators the same way: one action
        vector serves both policies (e.g. the heading students stand their hips
        at +-0.1, the blind finals at 0: no fallback for those)."""
        mine = self.contract
        layout = lambda c: [(g.kind, tuple(g.joints), float(g.scale)) for g in c.action_groups]
        pose = lambda c: dict(zip(c.joint_names, np.asarray(c.default_joint_pos, float)))
        if (sorted(mine.joint_names) != sorted(k.joint_names) or layout(mine) != layout(k)
                or any(abs(pose(mine)[j] - pose(k)[j]) > 1e-3 for j in mine.joint_names)):
            print(f"[RT] no depth fallback: {k.name!r} does not drive the robot as "
                  f"{mine.name!r} does (joints, action layout or default pose)", flush=True)
            return False
        return True

    def depth_frame(self) -> np.ndarray | None:
        """Newest policy depth frame (60x86, ray depth / DEPTH_MAX_M), or
        None for a blind policy / before the first capture."""
        if not self.uses_depth or self.latest_raw_depth is None:
            return None
        return self.depth_history.array[-1]

    def _resettle_after_viewer_reset(self) -> None:
        """Recover from the viewer's Reset button: mj_resetData jumps to
        model.qpos0 (the COMPILE-time attach frame -- a different spot and
        straight legs), so re-place the dog at the exact startup spawn and
        settle, then clear policy state (stale GRU/histories)."""
        reset_to_spawn(self.model, self.data, self.b, settle=True,
                       pos=self._spawn_pos, yaw_deg=self._spawn_yaw_deg)
        self.last_action.fill(0.0)
        self.proprio_history.reset()
        if self.heading is not None:
            self.heading.reset()
        if self.gait is not None:
            self.gait.reset()
        self.depth_history.reset()
        self.policy.reset()
        self.control_step = 0
        self.latest_raw_depth = None
        self._reset_brake()
        if self.fallback is not None:
            self.fallback.reset()
            self.monitor.reset()
            self.fallback_active = False
            self._apply_gains(self._main_gains)

    # -- one 50 Hz control tick ---------------------------------------------
    def tick(self) -> None:
        # The viewer's Reset button mj_resetData's the shared state behind
        # our back: data.time jumps backward. The GRU/history are then stale
        # and the joints sit at qpos0 (straight legs) -- resettle and clear.
        if float(self.data.time) < self._last_data_time - 1e-9:
            print(f"[RT] viewer reset detected (t {self._last_data_time:.1f}"
                  f"->{float(self.data.time):.1f}), resettling", flush=True)
            self._resettle_after_viewer_reset()
        self._last_data_time = float(self.data.time)

        if self.cmd_source.consume_reset():
            self.reset()
            return

        if (self.uses_depth
                and self.control_step % C.DEPTH_UPDATE_PERIOD_STEPS == 0):
            raw_depth = self.camera.capture(self.data)
            self.latest_raw_depth = raw_depth
            if self.monitor is not None:
                self.monitor.update(raw_depth)
            self.depth_history.append(self.depth_spec.preprocess(raw_depth))

        raw_command = self.cmd_source.vector()
        observation = self.build_observation(raw_command)
        self.proprio_history.append(observation)
        t0 = time.perf_counter()
        action = self.policy(
            observation,
            self.proprio_history.array,
            self.depth_history.array if self.uses_depth else None,
        )
        if self.fallback is not None:
            lane = self.fallback
            fallback_observation = self._lane_observation(lane, raw_command)
            lane.proprio_history.append(fallback_observation)
            fallback_action = lane.policy(fallback_observation, lane.proprio_history.array, None)
            self.fallback_active = self.fallback_enabled and self.monitor.fault is not None
            if self.fallback_active:
                action = fallback_action
            self._apply_gains(self._fallback_gains if self.fallback_active and self._fallback_gains is not None
                              else self._main_gains)
        self.policy_ms = (time.perf_counter() - t0) * 1e3
        k = self.contract
        brake = getattr(k, "wheel_brake_rad_s", None)
        hold_kp = getattr(k, "wheel_brake_hold_kp", None)
        ratio = getattr(k, "wheel_brake_release_ratio", None)
        after_s = getattr(k, "wheel_brake_after_s", None)
        zero = not np.any(self.command)
        # the brake's idle command: zero, or (brake_max_forward) a forward command up to that
        max_forward = getattr(k, "wheel_brake_max_forward", None)
        idle = zero if max_forward is None else abs(float(self.command[0])) <= max_forward
        self._zero_command_s = self._zero_command_s + C.CONTROL_DT if idle else 0.0
        # the leg hold of the policy that drives this step (the fallback's when it does)
        hk = self.fallback.contract if self.fallback is not None and self.fallback_active else k
        hold_after = getattr(hk, "leg_hold_after_s", None)
        still = zero and not (self.march and getattr(hk, "gait_march", False))
        self._leg_still_s = self._leg_still_s + C.CONTROL_DT if still else 0.0
        # Half a step of slack: summed float steps fall just short of whole seconds.
        timed_out = after_s is not None and self._zero_command_s >= after_s - 0.5 * C.CONTROL_DT
        holds = []   # (actuator ids, qpos addresses, held angles, target per rad)
        for i, (ids, part, scale, offset, dof, qadr) in enumerate(self._action_map):
            # position: default pose + scale * action; velocity: scale * action
            target = scale * action[part]
            if offset is not None:
                target = np.clip(offset + target, offset - ACTION_CLIP_RAD,
                                 offset + ACTION_CLIP_RAD)
                if hold_after is not None:
                    # Standstill hold, as in training (HeldJointPositionAction).
                    speed = float(np.max(np.abs(self.data.qvel[dof])))
                    due = self._leg_still_s >= hold_after - 0.5 * C.CONTROL_DT
                    engage = still and not self._leg_held[i] and due and speed < hk.leg_hold_speed
                    self._leg_held[i] = still and ((self._leg_held[i] and speed <= hk.leg_hold_release_speed)
                                                   or engage)
                    if engage:
                        self._leg_hold_target[i] = target.copy()
                    if self._leg_held[i]:
                        target = self._leg_hold_target[i]
                else:
                    self._leg_held[i] = False
            elif brake is not None:
                # Low-speed brake at a zero command, as in training: a braked
                # wheel (or any wheel once the timer ran out) lets go only
                # above the wider limit.
                limit = brake * ratio if ratio is not None and (self._braked[i] or timed_out) else brake
                braked = idle and bool(np.all(np.abs(self.data.qvel[dof]) < limit))
                if braked:
                    target = np.zeros_like(target)
                    if hold_kp:
                        # Hold the angles the brake engaged at. The velocity
                        # servo pushes kv (ctrl - dq), so a target of
                        # kp / kv (q_hold - q), renewed every physics step, is
                        # the motors' PD lock kp (q_hold - q) - kv dq.
                        if not self._braked[i]:
                            self._brake_hold[i] = self.data.qpos[qadr].copy()
                        holds.append((ids, qadr, self._brake_hold[i],
                                      hold_kp / self.model.actuator_gainprm[ids, 0]))
                self._braked[i] = braked
            if not np.all(np.isfinite(target)):
                raise FloatingPointError("policy actuator target is non-finite")
            self.data.ctrl[ids] = target
        t0 = time.perf_counter()
        for _ in range(C.PHYSICS_STEPS_PER_CONTROL):
            for ids, qadr, held, gain in holds:
                self.data.ctrl[ids] = gain * (held - self.data.qpos[qadr])
            mujoco.mj_step(self.model, self.data)
            if self.substep_hook is not None:
                self.substep_hook(self.data)
        self.physics_ms = (time.perf_counter() - t0) * 1e3
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(
                np.isfinite(self.data.qvel)):
            raise FloatingPointError("MuJoCo state became non-finite")
        self.last_action = action
        self.control_step += 1

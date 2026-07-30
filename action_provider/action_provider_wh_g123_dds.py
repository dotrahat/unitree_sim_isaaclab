# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Wholebody action provider for the G1 23DOF + BrainCo robot.

Runs a single policy: assets/model/policy-23dof.onnx, trained in mjlab/rsl_rl
(see unitree_rl_mjlab, task g1_23dof_brainco_velocity). Unlike the earlier
LSTM/padded-MLP experiments this file used to carry, the policy's full
observation/action contract (joint order, default pose, per-joint action
scale, trained PD gains) is embedded in the ONNX file's own metadata_props
and is transcribed verbatim into the constants below -- see that metadata
if any of these numbers ever need to be re-derived.

The policy outputs targets for all 23 joints (12 legs, waist_yaw, 10 arm
joints), matching what is visible when playing the policy in MuJoCo: a
human-like arm swing while walking. This provider only *applies* the leg +
waist targets, though -- the arm targets are overwritten by the operator's
teleop command (G1RobotDDS motor_cmd) whenever one is available, matching
the working G129 wholebody provider (DDSRLActionProvider) and because arms
are how the operator manipulates the object. The policy still *observes*
the arms' true position/velocity and its own raw 23-action output feeds
back into the next step's observation, exactly as during training -- only
the arm *targets* actually written to the sim are substituted.

This does forgo the policy's trained arm-swing contribution to yaw
stability while walking, and lets teleop push the arm observation well
outside the pose reward's training distribution (see the "Cost of
overriding the arms" note in the accompanying plan). If walking proves
unstable, the fix is `_compute_arm_targets`: gate arm authority by command
magnitude (teleop below the same 0.1 standing threshold `_get_twist_command`
and the phase observation already use, policy above it) rather than
handing arms to the policy unconditionally or blending continuously.
"""
from action_provider.action_base import ActionProvider
from typing import Optional
import math
import os
import ast
import traceback
import torch
from dds.dds_master import dds_manager

project_root = os.environ.get("PROJECT_ROOT")


class DDSRLActionProviderG123(ActionProvider):
    """Wholebody (locomotion policy + teleop upper body) action provider for G1 23DOF."""

    # --- policy-23dof.onnx contract (transcribed from the ONNX metadata_props) ---
    POLICY_JOINT_NAMES = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint",
    ]
    NUM_POLICY_JOINTS = len(POLICY_JOINT_NAMES)  # 23
    # First 13 slots (legs + waist) of POLICY_JOINT_NAMES are policy-applied;
    # the remaining 10 (arms) are teleop-applied -- see module docstring.
    NUM_LEG_WAIST = 13

    # Teleop's DDS command (unitree_hg LowCmd_.motor_cmd, see xr_teleoperate's
    # G1_23_Num_Motors / g1_robot_dds.py's dds_subscriber) has 35 motor slots -- a different,
    # larger convention than the 29-slot G1-29 *lowstate* layout arm_joint_mapping's indices
    # (15-26) are drawn from below. Do not conflate the two.
    NUM_DDS_CMD_MOTORS = 35

    # Arm entries are zeroed (not the ONNX metadata's original 0.35/0.18/0/0.87/0) to match
    # G123_CFG_WITH_BRAINCO_WHOLEBODY's spawn pose in robots/unitree.py, which spawns arms at
    # zero so the robot starts in the teleop-ready L-shape posture. Since the arms are teleop-
    # applied, not policy-applied (see module docstring), this only affects the policy's arm
    # *observation* offset and its pre-teleop fallback arm target -- it does not change what the
    # policy actually controls (legs + waist).
    DEFAULT_JOINT_POS = [
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,      # left leg
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,      # right leg
        0.0,                                  # waist_yaw
        0.0, 0.0, 0.0, 0.0, 0.0,             # left arm
        0.0, 0.0, 0.0, 0.0, 0.0,             # right arm
    ]
    ACTION_SCALE = [
        0.548, 0.351, 0.548, 0.351, 0.439, 0.439,   # left leg
        0.548, 0.351, 0.548, 0.351, 0.439, 0.439,   # right leg
        0.548,                                        # waist_yaw
        0.439, 0.439, 0.439, 0.439, 0.439,           # left arm
        0.439, 0.439, 0.439, 0.439, 0.439,           # right arm
    ]

    GAIT_PERIOD = 0.6
    CONTROL_DT = 0.02  # matches env decimation(4) * sim.dt(0.005)
    STAND_CMD_THRESHOLD = 0.1

    # Diagnostic: per-leg-waist-joint breakdown (obs terms, raw policy action,
    # commanded target, resulting actual position) for the first N control
    # steps, to distinguish a directional/sign bug (one joint's error grows
    # with the wrong sign from step 1) from a generic instability (all joints
    # drift together, magnitude-only). See project_g123_policy23dof_forward_fall_debug.
    DENSE_JOINT_LOG_STEPS = 12

    # arm target source: "teleop" applies G1RobotDDS motor_cmd whenever present,
    # falling back to the policy's own arm targets until the first command
    # arrives. "gated" is the fallback described in the module docstring, not
    # implemented here yet -- flip this only after the walking test shows the
    # teleop-arm setup is unstable.
    ARM_MODE = "teleop"

    def __init__(self, env, args_cli):
        super().__init__("DDSActionProviderG123Wholebody")
        self.enable_robot = args_cli.robot_type
        self.enable_brainco = args_cli.enable_brainco_dds
        self.policy_path = f"{project_root}/" + args_cli.model_path
        self.env = env

        self.robot_dds = None
        self.brainco_dds = None
        self.run_command_dds = None
        self._setup_dds()
        self._setup_joint_mapping()
        self.policy = self.load_policy(self.policy_path)

        device = self.env.device
        self._full_action_buf = torch.zeros(len(self.all_joint_names), device=device, dtype=torch.float32)
        self._last_action = torch.zeros(self.NUM_POLICY_JOINTS, device=device, dtype=torch.float32)
        self._positions_buf = torch.empty(self.NUM_DDS_CMD_MOTORS, device=device, dtype=torch.float32)
        if self.enable_brainco:
            self._brainco_buf = torch.empty(12, device=device, dtype=torch.float32)

        self._default_joint_pos_t = torch.tensor(self.DEFAULT_JOINT_POS, device=device, dtype=torch.float32)
        self._action_scale_t = torch.tensor(self.ACTION_SCALE, device=device, dtype=torch.float32)
        self.sim_step_counter = 0

        # diagnostics: log every step for the first 20 steps (startup transient),
        # then every N steps (N=50 -> 1/sim-second)
        self._log_interval = 50
        self._log_dense_until = 20
        self._log_counter = 0
        try:
            body_names = self.env.scene["robot"].data.body_names
            self._foot_body_idx = torch.tensor(
                [body_names.index("left_ankle_roll_link"), body_names.index("right_ankle_roll_link")],
                dtype=torch.long, device=self.env.device,
            )
        except (ValueError, AttributeError):
            self._foot_body_idx = None
        self._log_startup_state()

    def _log_startup_state(self):
        robot = self.env.scene["robot"]
        kp = robot.data.joint_stiffness[0, self._leg_waist_target_idx_t].tolist()
        kd = robot.data.joint_damping[0, self._leg_waist_target_idx_t].tolist()
        q0 = robot.data.joint_pos[0, self._leg_waist_target_idx_t].tolist()
        print(f"[{self.name}] leg+waist joints (policy order): {self.POLICY_JOINT_NAMES[:self.NUM_LEG_WAIST]}")
        print(f"[{self.name}] leg+waist kp in sim: {[round(v, 1) for v in kp]}")
        print(f"[{self.name}] leg+waist kd in sim: {[round(v, 2) for v in kd]}")
        print(f"[{self.name}] leg+waist q at start: {[round(v, 3) for v in q0]}")
        # mass parity check against mjlab's trained model (33.107 kg total, see
        # project_g123_policy23dof_forward_fall_debug memory) -- sagittal stance
        # joints (hip_pitch, knee, ankle_pitch) failing to track while roll/yaw
        # joints track fine is the signature of the sim body being effectively
        # heavier than what the policy was trained against.
        if hasattr(robot.data, "default_mass") and robot.data.default_mass is not None:
            body_masses = robot.data.default_mass[0].tolist()
            total_mass = sum(body_masses)
            print(f"[{self.name}] TOTAL MASS in sim: {total_mass:.3f} kg (mjlab trained on 33.107 kg)")
            for name, mass in zip(robot.data.body_names, body_masses):
                if mass > 0.01:
                    print(f"[{self.name}]   {name:30s} {mass:.4f} kg")
        print(f"[{self.name}] root height at start: {robot.data.root_pos_w[0, 2].item():.3f}, "
              f"projected gravity: {[round(v, 3) for v in robot.data.projected_gravity_b[0].tolist()]}")

    def _log_step_state(self, target_leg_waist_pos):
        robot = self.env.scene["robot"]
        h = robot.data.root_pos_w[0, 2].item()
        g = robot.data.projected_gravity_b[0]
        w = robot.data.root_ang_vel_b[0]
        q = robot.data.joint_pos[0, self._leg_waist_target_idx_t]
        track_err = (target_leg_waist_pos - q).abs()
        foot = ""
        if self._foot_body_idx is not None:
            fz = robot.data.body_link_pos_w[0, self._foot_body_idx, 2]
            foot = f" footz=({fz[0].item():.3f},{fz[1].item():.3f})"
        print(
            f"[WB-23DOF #{self._log_counter}] h={h:.3f} "
            f"g=({g[0]:.2f},{g[1]:.2f},{g[2]:.2f}) "
            f"w=({w[0]:.2f},{w[1]:.2f},{w[2]:.2f}) "
            f"trackerr[max={track_err.max().item():.3f},mean={track_err.mean().item():.3f}]"
            f"{foot}"
        )

    def _log_dense_joint_state(self, raw_action, targets):
        """Per-leg-waist-joint breakdown: obs terms that produced this action, the
        raw policy output, the commanded target, and the actual position the
        joint reached after this control step's physics substeps. Run for the
        first DENSE_JOINT_LOG_STEPS steps only -- see that constant's comment.
        """
        robot = self.env.scene["robot"]
        q_after = robot.data.joint_pos[0].index_select(0, self._leg_waist_target_idx_t)
        names = self.POLICY_JOINT_NAMES[: self.NUM_LEG_WAIST]
        av, gr, cmd, ph = self._dbg_ang_vel, self._dbg_gravity, self._dbg_command, self._dbg_phase
        print(
            f"[WB-23DOF-JOINT #{self._log_counter}] "
            f"ang_vel=({av[0]:.2f},{av[1]:.2f},{av[2]:.2f}) "
            f"gravity=({gr[0]:.2f},{gr[1]:.2f},{gr[2]:.2f}) "
            f"command=({cmd[0]:.2f},{cmd[1]:.2f},{cmd[2]:.2f}) "
            f"phase=({ph[0]:.2f},{ph[1]:.2f})"
        )
        for i, name in enumerate(names):
            print(
                f"    {name:26s} qpos_rel={self._dbg_qj_obs[i].item():+.3f} "
                f"qvel={self._dbg_dqj_obs[i].item():+.3f} "
                f"raw_act={raw_action[i].item():+.3f} "
                f"target={targets[i].item():+.3f} "
                f"q_after={q_after[i].item():+.3f} "
                f"err={(targets[i] - q_after[i]).item():+.3f}"
            )

    def _setup_dds(self):
        try:
            if self.enable_robot == "g123":
                self.robot_dds = dds_manager.get_object("g123")
            if self.enable_brainco:
                self.brainco_dds = dds_manager.get_object("brainco")
            self.run_command_dds = dds_manager.get_object("run_command")
            print(f"[{self.name}] DDS communication initialized")
        except Exception as e:
            print(f"[{self.name}] DDS initialization failed: {e}")

    def _setup_joint_mapping(self):
        # DDS source indices follow the same 29-slot G1 convention as
        # tasks/common_observations/g1_23dof_state.py / action_provider_dds.py's g123 branch.
        self.arm_joint_mapping = {
            "left_shoulder_pitch_joint": 15,
            "left_shoulder_roll_joint": 16,
            "left_shoulder_yaw_joint": 17,
            "left_elbow_joint": 18,
            "left_wrist_roll_joint": 19,
            "right_shoulder_pitch_joint": 22,
            "right_shoulder_roll_joint": 23,
            "right_shoulder_yaw_joint": 24,
            "right_elbow_joint": 25,
            "right_wrist_roll_joint": 26,
        }

        if self.enable_brainco:
            # Combined 12-motor format: left[0-5] + right[6-11]
            # Motor 0/6 ("thumb") = thumb flexion  -> thumb_proximal_joint
            # Motor 1/7 ("thumb-aux") = rotation   -> thumb_metacarpal_joint
            self.brainco_hand_joint_mapping = {
                "left_thumb_proximal_joint": 0,
                "left_thumb_metacarpal_joint": 1,
                "left_index_proximal_joint": 2,
                "left_middle_proximal_joint": 3,
                "left_ring_proximal_joint": 4,
                "left_pinky_proximal_joint": 5,
                "right_thumb_proximal_joint": 6,
                "right_thumb_metacarpal_joint": 7,
                "right_index_proximal_joint": 8,
                "right_middle_proximal_joint": 9,
                "right_ring_proximal_joint": 10,
                "right_pinky_proximal_joint": 11,
            }
            _FINGER_SCALE = 1.693 / 1.4661
            _THUMB_SCALE = 1.0
            self.brainco_special_joint_mapping = {
                "left_thumb_distal_joint": [0, _THUMB_SCALE],
                "left_index_distal_joint": [2, _FINGER_SCALE],
                "left_middle_distal_joint": [3, _FINGER_SCALE],
                "left_ring_distal_joint": [4, _FINGER_SCALE],
                "left_pinky_distal_joint": [5, _FINGER_SCALE],
                "right_thumb_distal_joint": [6, _THUMB_SCALE],
                "right_index_distal_joint": [8, _FINGER_SCALE],
                "right_middle_distal_joint": [9, _FINGER_SCALE],
                "right_ring_distal_joint": [10, _FINGER_SCALE],
                "right_pinky_distal_joint": [11, _FINGER_SCALE],
            }

        self.all_joint_names = self.env.scene["robot"].data.joint_names
        self.joint_to_index = {name: i for i, name in enumerate(self.all_joint_names)}

        device = self.env.device
        self._policy_target_idx_t = torch.tensor(
            [self.joint_to_index[n] for n in self.POLICY_JOINT_NAMES], dtype=torch.long, device=device
        )
        self._leg_waist_target_idx_t = self._policy_target_idx_t[: self.NUM_LEG_WAIST]
        self._arm_target_idx_t = torch.tensor(
            [self.joint_to_index[n] for n in self.arm_joint_mapping.keys()], dtype=torch.long, device=device
        )
        self._arm_source_idx_t = torch.tensor(list(self.arm_joint_mapping.values()), dtype=torch.long, device=device)

        if self.enable_brainco:
            self._brainco_target_idx_t = torch.tensor(
                [self.joint_to_index[n] for n in self.brainco_hand_joint_mapping.keys()], dtype=torch.long, device=device
            )
            self._brainco_source_idx_t = torch.tensor(
                list(self.brainco_hand_joint_mapping.values()), dtype=torch.long, device=device
            )
            self._brainco_special_target_idx_t = torch.tensor(
                [self.joint_to_index[n] for n in self.brainco_special_joint_mapping.keys()], dtype=torch.long, device=device
            )
            self._brainco_special_source_idx_t = torch.tensor(
                [spec[0] for spec in self.brainco_special_joint_mapping.values()], dtype=torch.long, device=device
            )
            self._brainco_special_scales_t = torch.tensor(
                [spec[1] for spec in self.brainco_special_joint_mapping.values()], dtype=torch.float32, device=device
            )

    def load_policy(self, path):
        ext = os.path.splitext(path)[1].lower()
        if ext != ".onnx":
            raise ValueError(f"Unsupported policy file extension: {ext} (expected .onnx)")

        import onnxruntime as ort
        model = ort.InferenceSession(path)
        input_meta = model.get_inputs()[0]
        output_meta = model.get_outputs()[0]
        obs_dim = int(input_meta.shape[-1])
        action_dim = int(output_meta.shape[-1])
        if obs_dim != 80 or action_dim != self.NUM_POLICY_JOINTS:
            raise ValueError(
                f"Unsupported ONNX policy '{path}': expected obs_dim=80, action_dim="
                f"{self.NUM_POLICY_JOINTS} (policy-23dof.onnx contract), got obs_dim={obs_dim}, "
                f"action_dim={action_dim}."
            )

        def run_inference(input_tensor):
            ort_inputs = {input_meta.name: input_tensor.cpu().numpy()}
            ort_outs = model.run(None, ort_inputs)
            return torch.tensor(ort_outs[0], device=self.env.device)

        return run_inference

    def _get_twist_command(self):
        """Read [vx, vy, wz] from the shared run_command DDS object (defaults to standing still)."""
        cmd = [0.0, 0.0, 0.0]
        if self.run_command_dds is not None:
            run_command = self.run_command_dds.get_run_command()
            if run_command and "run_command" in run_command:
                data = run_command["run_command"]
                try:
                    values = ast.literal_eval(data) if isinstance(data, str) else data
                    if len(values) >= 3:
                        cmd = [float(values[0]), float(values[1]), float(values[2])]
                except (ValueError, SyntaxError, TypeError, IndexError) as e:
                    print(f"[WARNING] cannot parse run_command data: {data}, error: {e}")
                self.run_command_dds.write_run_command([0.0, 0.0, 0.0, 0.8])
        return torch.tensor(cmd, device=self.env.device, dtype=torch.float32)

    def compute_observation(self, command):
        robot = self.env.scene["robot"]
        ang_vel = robot.data.root_ang_vel_b[0]
        gravity_orientation = robot.data.projected_gravity_b[0]

        standing = torch.linalg.norm(command) < self.STAND_CMD_THRESHOLD
        self.sim_step_counter += 1
        if standing:
            sin_cos_phase = torch.zeros(2, device=self.env.device, dtype=torch.float32)
        else:
            count = self.sim_step_counter * self.CONTROL_DT
            phase = (count % self.GAIT_PERIOD) / self.GAIT_PERIOD
            sin_cos_phase = torch.tensor(
                [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)],
                device=self.env.device, dtype=torch.float32,
            )

        qj = robot.data.joint_pos[0].index_select(0, self._policy_target_idx_t)
        dqj = robot.data.joint_vel[0].index_select(0, self._policy_target_idx_t)
        qj_obs = qj - self._default_joint_pos_t
        dqj_obs = dqj

        # stashed for _log_dense_joint_state -- see DENSE_JOINT_LOG_STEPS
        self._dbg_ang_vel = ang_vel.detach()
        self._dbg_gravity = gravity_orientation.detach()
        self._dbg_command = command.detach()
        self._dbg_phase = sin_cos_phase.detach()
        self._dbg_qj_obs = qj_obs.detach()
        self._dbg_dqj_obs = dqj_obs.detach()

        obs = torch.cat([ang_vel, gravity_orientation, command, sin_cos_phase, qj_obs, dqj_obs, self._last_action])
        return obs.unsqueeze(0)

    def run_policy(self, command):
        obs = self.compute_observation(command)
        with torch.no_grad():
            action = self.policy(obs)
        action = action.squeeze(0)
        self._last_action = action.detach().clone()
        return action

    def _compute_arm_targets(self, policy_targets):
        """Return the 10 arm joint targets to apply, or None to leave them untouched.

        ARM_MODE == "teleop": always follow the operator, falling back to the
        policy's own arm targets until the first teleop command arrives. See
        the module docstring for the "gated" alternative if this proves
        unstable while walking.
        """
        arm_vals = self._read_arm_override()
        if arm_vals is not None:
            return arm_vals
        return policy_targets[self.NUM_LEG_WAIST:]

    def _read_arm_override(self):
        """Return the 10 real-arm-joint target angles from teleop DDS, or None if no command yet."""
        if not self.robot_dds:
            return None
        cmd_data = self.robot_dds.get_robot_command()
        if not cmd_data or "motor_cmd" not in cmd_data:
            return None
        positions = cmd_data["motor_cmd"]["positions"]
        if len(positions) < 27:
            return None
        # The arm source indices (15-26) fit well within the buffer; clamp defensively so a
        # differently-sized payload (e.g. a future DDS layout change) degrades gracefully
        # instead of crashing get_action() and silently halting the policy.
        n = min(len(positions), self.NUM_DDS_CMD_MOTORS)
        self._positions_buf[:n].copy_(
            torch.tensor(positions[:n], dtype=torch.float32, device=self.env.device)
        )
        return self._positions_buf.index_select(0, self._arm_source_idx_t)

    def _apply_brainco_override(self, full_action):
        if not self.brainco_dds:
            return
        brainco_cmds = self.brainco_dds.get_brainco_hand_command()
        if not brainco_cmds or "positions" not in brainco_cmds:
            return
        brainco_positions = brainco_cmds["positions"]
        if len(brainco_positions) < 12:
            return
        self._brainco_buf.copy_(torch.tensor(brainco_positions[:12], dtype=torch.float32, device=self.env.device))
        base_vals = self._brainco_buf.index_select(0, self._brainco_source_idx_t)
        full_action.index_copy_(0, self._brainco_target_idx_t, base_vals)
        special_vals = (
            self._brainco_buf.index_select(0, self._brainco_special_source_idx_t) * self._brainco_special_scales_t
        )
        full_action.index_copy_(0, self._brainco_special_target_idx_t, special_vals)

    def get_action(self, env) -> Optional[torch.Tensor]:
        try:
            full_action = self._full_action_buf
            full_action.zero_()

            command = self._get_twist_command()
            raw_action = self.run_policy(command)
            targets = raw_action * self._action_scale_t + self._default_joint_pos_t

            full_action.index_copy_(0, self._policy_target_idx_t, targets)

            arm_vals = self._compute_arm_targets(targets)
            if arm_vals is not None:
                full_action.index_copy_(0, self._arm_target_idx_t, arm_vals)

            self._apply_brainco_override(full_action)

            self._log_counter += 1
            if self._log_counter <= self._log_dense_until or self._log_counter % self._log_interval == 0:
                self._log_step_state(targets[: self.NUM_LEG_WAIST])

            for _ in range(4):
                self.env.scene["robot"].set_joint_position_target(full_action)
                self.env.scene.write_data_to_sim()
                self.env.sim.step(render=False)
                self.env.scene.update(dt=self.env.physics_dt)

            if self._log_counter <= self.DENSE_JOINT_LOG_STEPS:
                self._log_dense_joint_state(raw_action, targets)

            self.env.sim.render()
            self.env.observation_manager.compute()

        except Exception as e:
            print(f"[{self.name}] Get DDS action failed: {e}")
            traceback.print_exc()
            return None

    def cleanup(self):
        try:
            if self.robot_dds:
                self.robot_dds.stop_communication()
            if self.brainco_dds:
                self.brainco_dds.stop_communication()
        except Exception as e:
            print(f"[{self.name}] Clean up DDS resources failed: {e}")

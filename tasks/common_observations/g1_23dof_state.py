# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
G1 23-DOF robot state publisher.

The DDS lowstate message has 29 motor slots (matching the G1-29 layout).
G1-23 is missing 6 joints: waist_roll, waist_pitch, left/right wrist_pitch,
left/right wrist_yaw.  Those 6 DDS slots are written with placeholder values
(joint 0) since xr_teleoperate never reads them for G1-23.

Critical: the index mapping must be built dynamically from
env.scene["robot"].data.joint_names because the Isaac Lab USD articulation
ordering is not the same as the DDS ordering.
"""
from __future__ import annotations

import time
import sys
import os
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# DDS slot index -> joint name for G1-23 (None = slot not present in G1-23)
_G123_DDS_SLOT_NAMES = [
    "left_hip_pitch_joint",     # 0
    "left_hip_roll_joint",      # 1
    "left_hip_yaw_joint",       # 2
    "left_knee_joint",          # 3
    "left_ankle_pitch_joint",   # 4
    "left_ankle_roll_joint",    # 5
    "right_hip_pitch_joint",    # 6
    "right_hip_roll_joint",     # 7
    "right_hip_yaw_joint",      # 8
    "right_knee_joint",         # 9
    "right_ankle_pitch_joint",  # 10
    "right_ankle_roll_joint",   # 11
    "waist_yaw_joint",          # 12
    None,                       # 13 waist_roll  – not in G1-23
    None,                       # 14 waist_pitch – not in G1-23
    "left_shoulder_pitch_joint",  # 15
    "left_shoulder_roll_joint",   # 16
    "left_shoulder_yaw_joint",    # 17
    "left_elbow_joint",           # 18
    "left_wrist_roll_joint",      # 19
    None,                         # 20 left_wrist_pitch  – not in G1-23
    None,                         # 21 left_wrist_yaw   – not in G1-23
    "right_shoulder_pitch_joint", # 22
    "right_shoulder_roll_joint",  # 23
    "right_shoulder_yaw_joint",   # 24
    "right_elbow_joint",          # 25
    "right_wrist_roll_joint",     # 26
    None,                         # 27 right_wrist_pitch – not in G1-23
    None,                         # 28 right_wrist_yaw  – not in G1-23
]

_obs_cache: dict = {
    "device": None,
    "batch": None,
    "idx_t": None,
    "idx_batch": None,
    "pos_buf": None,
    "vel_buf": None,
    "torque_buf": None,
    "combined_buf": None,
    "dds_last_ms": 0,
    "dds_min_interval_ms": 20,
}

_robot_dds = None
_dds_initialized = False


def _get_dds_instance():
    global _robot_dds, _dds_initialized
    if not _dds_initialized or _robot_dds is None:
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "dds"))
            from dds.dds_master import dds_manager
            for key in ("g123", "g129"):
                _robot_dds = dds_manager.get_object(key)
                if _robot_dds is not None:
                    break
            print("[g1_23dof_state] DDS instance obtained")
        except Exception as e:
            print(f"[g1_23dof_state] Failed to get DDS instance: {e}")
            _robot_dds = None
        _dds_initialized = True
    return _robot_dds


def _build_index_mapping(joint_names: list[str], device) -> torch.Tensor:
    """Build a 29-element tensor mapping each DDS slot to its sim joint index."""
    name_to_idx = {name: i for i, name in enumerate(joint_names)}
    indices = []
    for slot_name in _G123_DDS_SLOT_NAMES:
        if slot_name is not None and slot_name in name_to_idx:
            indices.append(name_to_idx[slot_name])
        else:
            indices.append(0)  # placeholder – value will be ignored by xr_teleoperate
    return torch.tensor(indices, dtype=torch.long, device=device)


def get_robot_boy_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
) -> torch.Tensor:
    """Publish G1-23 joint state to DDS and return a (batch, 87) observation tensor.

    Layout: [29 positions | 29 velocities | 29 torques] where the 29 slots
    follow the G1 DDS motor index convention.  Slots for joints that don't
    exist in G1-23 contain the value of sim joint 0 (harmless placeholder).
    """
    joint_pos = env.scene["robot"].data.joint_pos
    joint_vel = env.scene["robot"].data.joint_vel
    joint_torque = env.scene["robot"].data.applied_torque
    device = joint_pos.device
    batch = joint_pos.shape[0]

    global _obs_cache
    if _obs_cache["device"] != device or _obs_cache["idx_t"] is None:
        joint_names = list(env.scene["robot"].data.joint_names)
        _obs_cache["idx_t"] = _build_index_mapping(joint_names, device)
        _obs_cache["device"] = device
        _obs_cache["batch"] = None  # force buffer realloc

    idx_t = _obs_cache["idx_t"]
    n = idx_t.numel()  # 29

    if _obs_cache["batch"] != batch or _obs_cache["idx_batch"] is None:
        _obs_cache["idx_batch"] = idx_t.unsqueeze(0).expand(batch, n)
        _obs_cache["pos_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["vel_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["torque_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["combined_buf"] = torch.empty(batch, n * 3, device=device, dtype=joint_pos.dtype)
        _obs_cache["batch"] = batch

    idx_batch = _obs_cache["idx_batch"]
    pos_buf = _obs_cache["pos_buf"]
    vel_buf = _obs_cache["vel_buf"]
    torque_buf = _obs_cache["torque_buf"]
    combined_buf = _obs_cache["combined_buf"]

    try:
        torch.gather(joint_pos, 1, idx_batch, out=pos_buf)
        torch.gather(joint_vel, 1, idx_batch, out=vel_buf)
        torch.gather(joint_torque, 1, idx_batch, out=torque_buf)
    except TypeError:
        pos_buf.copy_(torch.gather(joint_pos, 1, idx_batch))
        vel_buf.copy_(torch.gather(joint_vel, 1, idx_batch))
        torque_buf.copy_(torch.gather(joint_torque, 1, idx_batch))

    combined_buf[:, 0:n].copy_(pos_buf)
    combined_buf[:, n:2*n].copy_(vel_buf)
    combined_buf[:, 2*n:3*n].copy_(torque_buf)

    if enable_dds and batch > 0:
        try:
            now_ms = int(time.time() * 1000)
            if now_ms - _obs_cache["dds_last_ms"] >= _obs_cache["dds_min_interval_ms"]:
                robot_dds = _get_dds_instance()
                if robot_dds:
                    from tasks.common_observations.g1_29dof_state import get_robot_imu_data
                    imu_data = get_robot_imu_data(env)
                    if imu_data.shape[0] > 0:
                        robot_dds.write_robot_state(
                            pos_buf[0].contiguous().cpu().numpy(),
                            vel_buf[0].contiguous().cpu().numpy(),
                            torque_buf[0].contiguous().cpu().numpy(),
                            imu_data[0].contiguous().cpu().numpy(),
                        )
                        _obs_cache["dds_last_ms"] = now_ms
        except Exception as e:
            print(f"[g1_23dof_state] Error writing state to DDS: {e}")

    return combined_buf

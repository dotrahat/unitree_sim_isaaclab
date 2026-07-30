# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
BrainCo hand joint state observation.

Reads 12 motor-driving joint positions from the robot, publishes them to the
BrainCo DDS shared memory so brainco_dds.py can forward them to xr_teleoperate.

Combined 12-motor format written to shared memory (left[0-5] + right[6-11]):
  0 / 6  : thumb_proximal_joint    (motor "thumb":     thumb flexion)
  1 / 7  : thumb_metacarpal_joint  (motor "thumb-aux": thumb rotation)
  2 / 8  : index_proximal_joint
  3 / 9  : middle_proximal_joint
  4 / 10 : ring_proximal_joint
  5 / 11 : pinky_proximal_joint
"""

from __future__ import annotations

import os
import sys
import time
import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# Joint names in the combined 12-motor order used by brainco_dds.py
_BRAINCO_MOTOR_JOINT_NAMES = [
    "left_thumb_proximal_joint",     # motor 0 (thumb: flexion)
    "left_thumb_metacarpal_joint",   # motor 1 (thumb-aux: rotation)
    "left_index_proximal_joint",     # motor 2
    "left_middle_proximal_joint",    # motor 3
    "left_ring_proximal_joint",      # motor 4
    "left_pinky_proximal_joint",     # motor 5
    "right_thumb_proximal_joint",    # motor 6 (thumb: flexion)
    "right_thumb_metacarpal_joint",  # motor 7 (thumb-aux: rotation)
    "right_index_proximal_joint",    # motor 8
    "right_middle_proximal_joint",   # motor 9
    "right_ring_proximal_joint",     # motor 10
    "right_pinky_proximal_joint",    # motor 11
]

_obs_cache: dict = {
    "device": None,
    "batch": None,
    "brainco_idx_t": None,
    "brainco_idx_batch": None,
    "pos_buf": None,
    "vel_buf": None,
    "torque_buf": None,
    "dds_last_ms": 0,
    "dds_min_interval_ms": 20,
}

_brainco_dds = None
_dds_initialized = False


def _get_brainco_dds_instance():
    global _brainco_dds, _dds_initialized
    if not _dds_initialized or _brainco_dds is None:
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'dds'))
            from dds.dds_master import dds_manager
            _brainco_dds = dds_manager.get_object("brainco")
            print("[brainco_state] DDS instance obtained")

            import atexit
            def _cleanup():
                try:
                    if _brainco_dds:
                        dds_manager.unregister_object("brainco")
                except Exception:
                    pass
            atexit.register(_cleanup)
        except Exception as e:
            print(f"[brainco_state] Failed to get DDS instance: {e}")
            _brainco_dds = None
        _dds_initialized = True
    return _brainco_dds


def _build_joint_index_tensor(env: "ManagerBasedRLEnv", device):
    """Look up joint indices by name from the robot's joint list."""
    all_joint_names = env.scene["robot"].data.joint_names
    name_to_idx = {name: i for i, name in enumerate(all_joint_names)}
    indices = []
    for name in _BRAINCO_MOTOR_JOINT_NAMES:
        if name in name_to_idx:
            indices.append(name_to_idx[name])
        else:
            print(f"[brainco_state] WARNING: joint '{name}' not found in robot joint list. Using 0.")
            indices.append(0)
    return torch.tensor(indices, dtype=torch.long, device=device)


def get_robot_brainco_joint_states(
    env: "ManagerBasedRLEnv",
    enable_dds: bool = True,
) -> torch.Tensor:
    """Gather the 12 BrainCo motor-driving joint states and optionally push to DDS.

    Returns:
        Tensor of shape (num_envs, 12) with raw joint positions.
    """
    joint_pos = env.scene["robot"].data.joint_pos
    joint_vel = env.scene["robot"].data.joint_vel
    joint_torque = env.scene["robot"].data.applied_torque
    device = joint_pos.device
    batch = joint_pos.shape[0]

    global _obs_cache
    # Build index tensor on first call or device change
    if _obs_cache["device"] != device or _obs_cache["brainco_idx_t"] is None:
        _obs_cache["brainco_idx_t"] = _build_joint_index_tensor(env, device)
        _obs_cache["device"] = device
        _obs_cache["batch"] = None  # force batch realloc

    idx_t = _obs_cache["brainco_idx_t"]
    n = idx_t.numel()

    if _obs_cache["batch"] != batch or _obs_cache["brainco_idx_batch"] is None:
        _obs_cache["brainco_idx_batch"] = idx_t.unsqueeze(0).expand(batch, n)
        _obs_cache["pos_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["vel_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["torque_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["batch"] = batch

    idx_batch = _obs_cache["brainco_idx_batch"]
    pos_buf = _obs_cache["pos_buf"]
    vel_buf = _obs_cache["vel_buf"]
    torque_buf = _obs_cache["torque_buf"]

    try:
        torch.gather(joint_pos, 1, idx_batch, out=pos_buf)
        torch.gather(joint_vel, 1, idx_batch, out=vel_buf)
        torch.gather(joint_torque, 1, idx_batch, out=torque_buf)
    except TypeError:
        pos_buf.copy_(torch.gather(joint_pos, 1, idx_batch))
        vel_buf.copy_(torch.gather(joint_vel, 1, idx_batch))
        torque_buf.copy_(torch.gather(joint_torque, 1, idx_batch))

    # Publish to DDS at limited rate (first env only)
    if enable_dds and len(pos_buf) > 0:
        try:
            now_ms = int(time.time() * 1000)
            if now_ms - _obs_cache["dds_last_ms"] >= _obs_cache["dds_min_interval_ms"]:
                brainco_dds = _get_brainco_dds_instance()
                if brainco_dds:
                    pos = pos_buf[0].contiguous().cpu().numpy()
                    vel = vel_buf[0].contiguous().cpu().numpy()
                    torque = torque_buf[0].contiguous().cpu().numpy()
                    brainco_dds.write_brainco_state(pos, vel, torque)
                    _obs_cache["dds_last_ms"] = now_ms
        except Exception as e:
            print(f"[brainco_state] DDS write error: {e}")

    return pos_buf

# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
BrainCo hand DDS communication class
Handles state publishing and command receiving for the BrainCo Revo2 hand.

Topic convention matches xr_teleoperate/teleop/robot_control/robot_hand_brainco.py:
  - Commands IN:  rt/brainco/left/cmd  and rt/brainco/right/cmd  (MotorCmds_,  6 motors each)
  - State   OUT: rt/brainco/left/state and rt/brainco/right/state (MotorStates_, 6 motors each)

Normalization convention (matches brainco hardware official docs):
  0.0 = fully open, 1.0 = fully closed

Combined 12-motor internal format (left[0-5] + right[6-11]):
  Idx  0 / 6  : thumb_metacarpal   range [0, 1.52]   rad
  Idx  1 / 7  : thumb_proximal     range [0, 1.0472] rad
  Idx  2 / 8  : index_proximal     range [0, 1.4661] rad
  Idx  3 / 9  : middle_proximal    range [0, 1.4661] rad
  Idx  4 / 10 : ring_proximal      range [0, 1.4661] rad
  Idx  5 / 11 : pinky_proximal     range [0, 1.4661] rad
"""

import threading
from typing import Any, Dict, Optional
from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_, unitree_go_msg_dds__MotorState_
import numpy as np

# Joint angle ranges (sim/URDF limits) for the 12 combined motors
# Index: [left_thumb_meta, left_thumb_prox, left_idx, left_mid, left_ring, left_pinky,
#         right_thumb_meta, right_thumb_prox, right_idx, right_mid, right_ring, right_pinky]
_MOTOR_MIN = [0.0] * 12
_MOTOR_MAX = [
    1.52,    # 0  left  thumb_metacarpal
    1.0472,  # 1  left  thumb_proximal
    1.4661,  # 2  left  index_proximal
    1.4661,  # 3  left  middle_proximal
    1.4661,  # 4  left  ring_proximal
    1.4661,  # 5  left  pinky_proximal
    1.52,    # 6  right thumb_metacarpal
    1.0472,  # 7  right thumb_proximal
    1.4661,  # 8  right index_proximal
    1.4661,  # 9  right middle_proximal
    1.4661,  # 10 right ring_proximal
    1.4661,  # 11 right pinky_proximal
]


class BraincoDDS(DDSObject):
    """BrainCo hand DDS communication class.

    Publishes hand joint states to separate left/right state topics and
    subscribes to separate left/right command topics, matching the convention
    used by xr_teleoperate's Brainco_Controller.
    """

    def __init__(self, node_name: str = "brainco"):
        if hasattr(self, '_initialized'):
            return

        super().__init__()
        self.node_name = node_name

        # State messages: one per hand (6 motors each)
        self._left_state_msg = MotorStates_()
        self._left_state_msg.states = [unitree_go_msg_dds__MotorState_() for _ in range(6)]
        self._right_state_msg = MotorStates_()
        self._right_state_msg.states = [unitree_go_msg_dds__MotorState_() for _ in range(6)]

        # Internal combined command buffer (12 motors: left[0-5] + right[6-11])
        self._cmd_positions = [0.0] * 12
        self._cmd_velocities = [0.0] * 12
        self._cmd_torques = [0.0] * 12
        self._cmd_kp = [0.0] * 12
        self._cmd_kd = [0.0] * 12
        self._cmd_lock = threading.Lock()

        self._initialized = True

        self.setup_shared_memory(
            input_shm_name="isaac_brainco_state",
            input_size=1024,
            output_shm_name="isaac_brainco_cmd",
            output_size=1024,
        )
        print(f"[{self.node_name}] BrainCo DDS node initialized")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _normalize(val: float, min_val: float, max_val: float) -> float:
        """Map sim joint angle → [0=open, 1=closed] (brainco convention)."""
        if max_val <= min_val:
            return 0.0
        return float(np.clip((val - min_val) / (max_val - min_val), 0.0, 1.0))

    @staticmethod
    def _denormalize(norm_val: float, min_val: float, max_val: float) -> float:
        """Map [0=open, 1=closed] → sim joint angle."""
        return float(min_val + np.clip(norm_val, 0.0, 1.0) * (max_val - min_val))

    # ------------------------------------------------------------------
    # DDSObject interface
    # ------------------------------------------------------------------

    def setup_publisher(self) -> bool:
        try:
            self.left_publisher = ChannelPublisher("rt/brainco/left/state", MotorStates_)
            self.left_publisher.Init()
            self.right_publisher = ChannelPublisher("rt/brainco/right/state", MotorStates_)
            self.right_publisher.Init()
            print(f"[{self.node_name}] BrainCo state publishers initialized (left/right)")
            return True
        except Exception as e:
            print(f"[{self.node_name}] BrainCo publisher init failed: {e}")
            return False

    def setup_subscriber(self) -> bool:
        try:
            self.left_subscriber = ChannelSubscriber("rt/brainco/left/cmd", MotorCmds_)
            self.left_subscriber.Init(lambda msg: self.dds_subscriber(msg, "left"), 32)
            self.right_subscriber = ChannelSubscriber("rt/brainco/right/cmd", MotorCmds_)
            self.right_subscriber.Init(lambda msg: self.dds_subscriber(msg, "right"), 32)
            print(f"[{self.node_name}] BrainCo command subscribers initialized (left/right)")
            return True
        except Exception as e:
            print(f"[{self.node_name}] BrainCo subscriber init failed: {e}")
            return False

    def dds_publisher(self) -> None:
        """Read sim joint states from shared memory and publish to left/right state topics."""
        try:
            data = self.input_shm.read_data()
            if data is None:
                return
            positions = data.get("positions", [])
            velocities = data.get("velocities", [])
            torques = data.get("torques", [])
            if len(positions) < 12:
                return

            # Left hand: combined indices 0-5
            for i in range(6):
                self._left_state_msg.states[i].q = self._normalize(
                    float(positions[i]), _MOTOR_MIN[i], _MOTOR_MAX[i])
                self._left_state_msg.states[i].dq = float(velocities[i]) if i < len(velocities) else 0.0
                self._left_state_msg.states[i].tau_est = float(torques[i]) if i < len(torques) else 0.0
            self.left_publisher.Write(self._left_state_msg)

            # Right hand: combined indices 6-11
            for i in range(6):
                j = i + 6
                self._right_state_msg.states[i].q = self._normalize(
                    float(positions[j]), _MOTOR_MIN[j], _MOTOR_MAX[j])
                self._right_state_msg.states[i].dq = float(velocities[j]) if j < len(velocities) else 0.0
                self._right_state_msg.states[i].tau_est = float(torques[j]) if j < len(torques) else 0.0
            self.right_publisher.Write(self._right_state_msg)

        except Exception as e:
            print(f"[{self.node_name}] dds_publisher error: {e}")

    def dds_subscriber(self, msg: MotorCmds_, side: str) -> None:
        """Receive left or right hand command and merge into the 12-motor combined buffer."""
        try:
            offset = 0 if side == "left" else 6
            with self._cmd_lock:
                for i in range(min(6, len(msg.cmds))):
                    j = offset + i
                    self._cmd_positions[j] = self._denormalize(
                        float(msg.cmds[i].q), _MOTOR_MIN[j], _MOTOR_MAX[j])
                    self._cmd_velocities[j] = float(msg.cmds[i].dq)
                    self._cmd_torques[j] = float(msg.cmds[i].tau)
                    self._cmd_kp[j] = float(msg.cmds[i].kp)
                    self._cmd_kd[j] = float(msg.cmds[i].kd)
                cmd_data = {
                    "positions": list(self._cmd_positions),
                    "velocities": list(self._cmd_velocities),
                    "torques": list(self._cmd_torques),
                    "kp": list(self._cmd_kp),
                    "kd": list(self._cmd_kd),
                }
                self.output_shm.write_data(cmd_data)
        except Exception as e:
            print(f"[{self.node_name}] dds_subscriber ({side}) error: {e}")

    # ------------------------------------------------------------------
    # Public helpers used by action_provider and observation modules
    # ------------------------------------------------------------------

    def get_brainco_hand_command(self) -> Optional[Dict[str, Any]]:
        """Return the latest combined 12-motor command dict, or None."""
        if self.output_shm:
            return self.output_shm.read_data()
        return None

    def write_brainco_state(self, positions, velocities, torques) -> None:
        """Write sim joint states (raw angles) to shared memory for DDS publishing.

        Args:
            positions:  12-element array [left0-5, right6-11] of sim joint angles
            velocities: 12-element array of joint velocities
            torques:    12-element array of joint torques
        """
        try:
            brainco_data = {
                "positions": positions.tolist() if hasattr(positions, 'tolist') else list(positions),
                "velocities": velocities.tolist() if hasattr(velocities, 'tolist') else list(velocities),
                "torques": torques.tolist() if hasattr(torques, 'tolist') else list(torques),
            }
            if self.input_shm:
                self.input_shm.write_data(brainco_data)
        except Exception as e:
            print(f"[{self.node_name}] write_brainco_state error: {e}")

# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""Configuration for Unitree robots."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
import os
project_root = os.environ.get("PROJECT_ROOT")
G129_CFG_WITH_DEX3_BASE_FIX = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-29dof-dex3-base-fix-usd/g1_29dof_with_dex3_base_fix.usd", #original
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, 
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,

        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.05,
            "left_knee_joint": 0.2,
            "left_ankle_pitch_joint": -0.15,
            "left_ankle_roll_joint": 0.0,
            
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.05,
            "right_knee_joint": 0.2,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,
            
            # waist joints
            "waist_yaw_joint": 0.0,
            "waist_roll_joint": 0.0,
            "waist_pitch_joint": 0.0,
            
            # arms joints
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
            
            # fingers joints
            "left_hand_index_0_joint": 0.0,
            "left_hand_middle_0_joint": 0.0,
            "left_hand_thumb_0_joint": 0.0,
            "left_hand_index_1_joint": 0.0,
            "left_hand_middle_1_joint": 0.0,
            "left_hand_thumb_1_joint": 0.0,
            "left_hand_thumb_2_joint": 0.0,
            
            "right_hand_index_0_joint": 0.0,
            "right_hand_middle_0_joint": 0.0,
            "right_hand_thumb_0_joint": 0.0,
            "right_hand_index_1_joint": 0.0,
            "right_hand_middle_1_joint": 0.0,
            "right_hand_thumb_1_joint": 0.0,
            "right_hand_thumb_2_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,

    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint", 
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint", 
                ".*_knee_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            stiffness=None,
            damping=None,
            armature=None,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[
                "waist_yaw_joint",
                "waist_roll_joint",
                "waist_pitch_joint"
            ],  
            effort_limit=1000.0,  # set a large torque limit
            velocity_limit=0.0,   # set the velocity limit to 0
            stiffness={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            damping={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            armature=None,
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=None,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness=None,
            damping=None,
            # armature=0.001,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_.*_joint"
            ],
            effort_limit=None,
            velocity_limit=None,
             stiffness={  # increase the stiffness (kp)
                 ".*_shoulder_.*_joint": 300.0,
                 ".*_elbow_joint": 400.0,
                 ".*_wrist_.*_joint": 400.0,
            },
             damping={    # increase the damping (kd)
                 ".*_shoulder_.*_joint": 3.0,
                 ".*_elbow_joint": 2.5,
                 ".*_wrist_.*_joint": 2.5,
             },
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hand_index_.*_joint",
                ".*_hand_middle_.*_joint",
                ".*_hand_thumb_.*_joint"
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*": 100.0,
            },
            damping={
                ".*": 10.0,
            },
            armature={
                ".*": 0.1
            },
        ),
    },
)




G129_CFG_WITH_DEX1_BASE_FIX = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-29dof-dex1-base-fix-usd/g1_29dof_with_dex1_base_fix1.usd",
        # usd_path=f"{project_root}/assets/robots/g1-29dof-dex1-base-fix-usd-new/usd/g1_29dof_mode_15_with_dex1_1.urdf.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, 
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4
        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.05,
            "left_knee_joint": 0.2,
            "left_ankle_pitch_joint": -0.15,
            "left_ankle_roll_joint": 0.0,
            
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.05,
            "right_knee_joint": 0.2,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,
            
            # waist joints
            "waist_yaw_joint": 0.0,
            "waist_roll_joint": 0.0,
            "waist_pitch_joint": 0.0,
            
            # arms joints
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
            
            # fingers joints
            "left_hand_Joint1_1": 0.0,
            "left_hand_Joint2_1": 0.0,
            "right_hand_Joint1_1": 0.0,
            "right_hand_Joint2_1": 0.0,

            # 'left_dex1_finger_joint_1': 0.0,
            # 'left_dex1_finger_joint_2': 0.0,
            # 'right_dex1_finger_joint_1': 0.0,
            # 'right_dex1_finger_joint_2': 0.0
            
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint", 
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint", 
                ".*_knee_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            stiffness=None,
            damping=None,
            armature=None,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[
                "waist_yaw_joint",
                "waist_roll_joint",
                "waist_pitch_joint"
            ],  
            effort_limit=1000.0,  # set a large torque limit
            velocity_limit=0.0,   # set the velocity limit to 0
            stiffness={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            damping={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            armature=None,
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=None,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness=None,
            damping=None,
            # armature=0.001,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_.*_joint"
            ],
            effort_limit=None,
            velocity_limit=None,
            stiffness={  # increase the stiffness (kp)
                 ".*_shoulder_.*_joint": 25.0,
                 ".*_elbow_joint": 50.0,
                 ".*_wrist_.*_joint": 40.0,
            },
             damping={    # increase the damping (kd)
                 ".*_shoulder_.*_joint": 2.0,
                 ".*_elbow_joint": 2.0,
                 ".*_wrist_.*_joint": 2.0,
             },
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                "left_hand_Joint1_1",
                "left_hand_Joint2_1",
                "right_hand_Joint1_1",
                "right_hand_Joint2_1",
            ],
            # joint_names_expr=[
            #     'left_dex1_finger_joint_1',
            #     'left_dex1_finger_joint_2',
            #     'right_dex1_finger_joint_1',
            #     'right_dex1_finger_joint_2'
            # ],
            effort_limit=None,  # increase the torque limit
            velocity_limit=None,  # set the velocity limit to 0
            stiffness=800.0,    # increase the stiffness (kp)
            damping=3.0,        # increase the damping (kd)
            friction=200.0,
            armature=None,
        ),

    },
)



G129_CFG_WITH_INSPIRE_HAND = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/original_g1-29dof-inspire-base-fix-usd/g1-29dof-inspire-base-fix-usd/g1_29dof_with_inspire_rev_1_0.usd", #original
        # usd_path=f"{project_root}/assets/robots/g1-29dof-inspire-base-fix-usd/a/g1_29dof_rev_1_0_with_inspire_hand_DFQ.usd",
        # usd_path=f"{project_root}/assets/robots/g1-29dof-inspire-base-fix-usd/b/g1_29dof_rev_1_0_with_inspire_hand_DFQ_usd.usd",
        # usd_path=f"{project_root}/assets/robots/g1-29dof-inspire-base-fix-usd/2/g1_29dof_with_inspire_rev_1_0.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, 
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4
        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.05,
            "left_knee_joint": 0.2,
            "left_ankle_pitch_joint": -0.15,
            "left_ankle_roll_joint": 0.0,
            
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.05,
            "right_knee_joint": 0.2,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,
            
            # waist joints
            "waist_yaw_joint": 0.0,
            "waist_roll_joint": 0.0,
            "waist_pitch_joint": 0.0,
            
            # arms joints
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
            
            # fingers joints
            "L_index_proximal_joint": 0.0,
            "L_index_intermediate_joint": 0.0,
            "L_middle_proximal_joint": 0.0,
            "L_middle_intermediate_joint": 0.0,
            "L_pinky_proximal_joint":0.0,
            "L_pinky_intermediate_joint":0.0,
            "L_ring_proximal_joint":0.0,
            "L_ring_intermediate_joint":0.0,
            "L_thumb_proximal_yaw_joint":0.0,
            "L_thumb_proximal_pitch_joint":0.0,
            "L_thumb_intermediate_joint":0.0,
            "L_thumb_distal_joint":0.0,

            "R_index_proximal_joint": 0.0,
            "R_index_intermediate_joint": 0.0,
            "R_middle_proximal_joint": 0.0,
            "R_middle_intermediate_joint": 0.0,
            "R_pinky_proximal_joint":0.0,
            "R_pinky_intermediate_joint":0.0,
            "R_ring_proximal_joint":0.0,
            "R_ring_intermediate_joint":0.0,
            "R_thumb_proximal_yaw_joint":0.0,
            "R_thumb_proximal_pitch_joint":0.0,
            "R_thumb_intermediate_joint":0.0,
            "R_thumb_distal_joint":0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint", 
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint", 
                ".*_knee_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            stiffness=None,
            damping=None,
            armature=None,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[
                "waist_yaw_joint",
                "waist_roll_joint",
                "waist_pitch_joint"
            ],  
            effort_limit=1000.0,  # set a large torque limit
            velocity_limit=0.0,   # set the velocity limit to 0
            stiffness={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            damping={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            armature=None,
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=None,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness=None,
            damping=None,
            # armature=0.001,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_.*_joint"
            ],
            effort_limit=None,
            velocity_limit=None,
             stiffness={  # increase the stiffness (kp)
                 ".*_shoulder_.*_joint": 25.0,
                 ".*_elbow_joint": 50.0,
                 ".*_wrist_.*_joint": 40.0,
            },
             damping={    # increase the damping (kd)
                 ".*_shoulder_.*_joint": 2.0,
                 ".*_elbow_joint": 2.0,
                 ".*_wrist_.*_joint": 2.0,
             },
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_index_proximal_joint",
                ".*_index_intermediate_joint",
                ".*_middle_proximal_joint",
                ".*_middle_intermediate_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_intermediate_joint",
                ".*_ring_proximal_joint",
                ".*_ring_intermediate_joint",
                ".*_thumb_proximal_yaw_joint",
                ".*_thumb_proximal_pitch_joint",
                ".*_thumb_intermediate_joint",
                ".*_thumb_distal_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_index_proximal_joint":1000.0,
                ".*_index_intermediate_joint":1000.0,
                ".*_middle_proximal_joint":1000.0,
                ".*_middle_intermediate_joint":1000.0,
                ".*_pinky_proximal_joint":1000.0,
                ".*_pinky_intermediate_joint":1000.0,
                ".*_ring_proximal_joint":1000.0,
                ".*_ring_intermediate_joint":1000.0,
                ".*_thumb_proximal_yaw_joint":1000.0,
                ".*_thumb_proximal_pitch_joint":1000.0,
                ".*_thumb_intermediate_joint":1000.0,
                ".*_thumb_distal_joint":1000.0,
            },
            damping={
                ".*_index_proximal_joint":15,
                ".*_index_intermediate_joint":15,
                ".*_middle_proximal_joint":15,
                ".*_middle_intermediate_joint":15,
                ".*_pinky_proximal_joint":15,
                ".*_pinky_intermediate_joint":15,
                ".*_ring_proximal_joint":15,
                ".*_ring_intermediate_joint":15,
                ".*_thumb_proximal_yaw_joint":15,
                ".*_thumb_proximal_pitch_joint":15,
                ".*_thumb_intermediate_joint":15,
                ".*_thumb_distal_joint":15,
            },
            armature={
                ".*": 0.0
            },
        ),

    },
)



G129_CFG_WITH_BRAINCO_HAND = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-29dof-brainco-base-fix-usd/USds/2/g1_29dof_mode_15_brainco_hand.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, 
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4
        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.05,
            "left_knee_joint": 0.2,
            "left_ankle_pitch_joint": -0.15,
            "left_ankle_roll_joint": 0.0,
            
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.05,
            "right_knee_joint": 0.2,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,
            
            # waist joints
            "waist_yaw_joint": 0.0,
            "waist_roll_joint": 0.0,
            "waist_pitch_joint": 0.0,
            
            # arms joints
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
            
            # left hand finger joints (tip joints excluded)
            "left_index_proximal_joint": 0.0,
            "left_index_distal_joint": 0.0,
            "left_middle_proximal_joint": 0.0,
            "left_middle_distal_joint": 0.0,
            "left_ring_proximal_joint": 0.0,
            "left_ring_distal_joint": 0.0,
            "left_pinky_proximal_joint": 0.0,
            "left_pinky_distal_joint": 0.0,
            "left_thumb_metacarpal_joint": 0.0,
            "left_thumb_proximal_joint": 0.0,
            "left_thumb_distal_joint": 0.0,

            # right hand finger joints (tip joints excluded)
            "right_thumb_metacarpal_joint": 0.0,
            "right_thumb_proximal_joint": 0.0,
            "right_thumb_distal_joint": 0.0,
            "right_index_proximal_joint": 0.0,
            "right_index_distal_joint": 0.0,
            "right_middle_proximal_joint": 0.0,
            "right_middle_distal_joint": 0.0,
            "right_ring_proximal_joint": 0.0,
            "right_ring_distal_joint": 0.0,
            "right_pinky_proximal_joint": 0.0,
            "right_pinky_distal_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint", 
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint", 
                ".*_knee_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            stiffness=None,
            damping=None,
            armature=None,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[
                "waist_yaw_joint",
                "waist_roll_joint",
                "waist_pitch_joint"
            ],  
            effort_limit=1000.0,  # set a large torque limit
            velocity_limit=0.0,   # set the velocity limit to 0
            stiffness={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            damping={
                "waist_yaw_joint": 10000.0,
                "waist_roll_joint": 10000.0,
                "waist_pitch_joint": 10000.0
            },
            armature=None,
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=None,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness=None,
            damping=None,
            # armature=0.001,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_.*_joint"
            ],
            effort_limit=None,
            velocity_limit=None,
             stiffness={  # increase the stiffness (kp)
                 ".*_shoulder_.*_joint": 25.0,
                 ".*_elbow_joint": 50.0,
                 ".*_wrist_.*_joint": 40.0,
            },
             damping={    # increase the damping (kd)
                 ".*_shoulder_.*_joint": 2.0,
                 ".*_elbow_joint": 2.0,
                 ".*_wrist_.*_joint": 2.0,
             },
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_thumb_metacarpal_joint",
                ".*_thumb_proximal_joint",
                ".*_thumb_distal_joint",
                ".*_index_proximal_joint",
                ".*_index_distal_joint",
                ".*_middle_proximal_joint",
                ".*_middle_distal_joint",
                ".*_ring_proximal_joint",
                ".*_ring_distal_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_distal_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_thumb_metacarpal_joint":1000.0,
                ".*_thumb_proximal_joint":1000.0,
                ".*_thumb_distal_joint":1000.0,
                ".*_index_proximal_joint":1000.0,
                ".*_index_distal_joint":1000.0,
                ".*_middle_proximal_joint":1000.0,
                ".*_middle_distal_joint":1000.0,
                ".*_ring_proximal_joint":1000.0,
                ".*_ring_distal_joint":1000.0,
                ".*_pinky_proximal_joint":1000.0,
                ".*_pinky_distal_joint":1000.0,
            },
            damping={
                ".*_thumb_metacarpal_joint":15,
                ".*_thumb_proximal_joint":15,
                ".*_thumb_distal_joint":15,
                ".*_index_proximal_joint":15,
                ".*_index_distal_joint":15,
                ".*_middle_proximal_joint":15,
                ".*_middle_distal_joint":15,
                ".*_ring_proximal_joint":15,
                ".*_ring_distal_joint":15,
                ".*_pinky_proximal_joint":15,
                ".*_pinky_distal_joint":15,
            },
            armature={
                ".*": 0.0
            },
        ),

    },
)




G123_CFG_WITH_BRAINCO_HAND = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-23dof-brainco-base-fix-usd/with_wrist_cams/g1_23dof_mode_10_with_brainco_wrist_cams.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.05,
            "left_knee_joint": 0.2,
            "left_ankle_pitch_joint": -0.15,
            "left_ankle_roll_joint": 0.0,

            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.05,
            "right_knee_joint": 0.2,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,

            # waist (23dof only has yaw)
            "waist_yaw_joint": 0.0,

            # arms (23dof: no wrist_pitch / wrist_yaw)
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,

            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,

            # left hand finger joints (tip joints excluded)
            "left_index_proximal_joint": 0.0,
            "left_index_distal_joint": 0.0,
            "left_middle_proximal_joint": 0.0,
            "left_middle_distal_joint": 0.0,
            "left_ring_proximal_joint": 0.0,
            "left_ring_distal_joint": 0.0,
            "left_pinky_proximal_joint": 0.0,
            "left_pinky_distal_joint": 0.0,
            "left_thumb_metacarpal_joint": 0.0,
            "left_thumb_proximal_joint": 0.0,
            "left_thumb_distal_joint": 0.0,

            # right hand finger joints (tip joints excluded)
            "right_thumb_metacarpal_joint": 0.0,
            "right_thumb_proximal_joint": 0.0,
            "right_thumb_distal_joint": 0.0,
            "right_index_proximal_joint": 0.0,
            "right_index_distal_joint": 0.0,
            "right_middle_proximal_joint": 0.0,
            "right_middle_distal_joint": 0.0,
            "right_ring_proximal_joint": 0.0,
            "right_ring_distal_joint": 0.0,
            "right_pinky_proximal_joint": 0.0,
            "right_pinky_distal_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            stiffness=None,
            damping=None,
            armature=None,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit=1000.0,
            velocity_limit=0.0,
            stiffness={"waist_yaw_joint": 10000.0},
            damping={"waist_yaw_joint": 10000.0},
            armature=None,
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=None,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness=None,
            damping=None,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_roll_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            # Matched to G129_CFG_WITH_INSPIRE_HAND. The higher gains previously used here
            # (200/20) were compensating for a zero-mass-link bug (fingertip links with 0 mass
            # defaulted to 1kg each in PhysX, inflating arm load) -- that bug is fixed in the
            # current USD, so Inspire's gains apply directly.
            stiffness={
                ".*_shoulder_.*_joint": 25.0,
                ".*_elbow_joint": 50.0,
                ".*_wrist_roll_joint": 40.0,
            },
            damping={
                ".*_shoulder_.*_joint": 2.0,
                ".*_elbow_joint": 2.0,
                ".*_wrist_roll_joint": 2.0,
            },
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_thumb_metacarpal_joint",
                ".*_thumb_proximal_joint",
                ".*_thumb_distal_joint",
                ".*_index_proximal_joint",
                ".*_index_distal_joint",
                ".*_middle_proximal_joint",
                ".*_middle_distal_joint",
                ".*_ring_proximal_joint",
                ".*_ring_distal_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_distal_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_thumb_metacarpal_joint": 1000.0,
                ".*_thumb_proximal_joint": 1000.0,
                ".*_thumb_distal_joint": 1000.0,
                ".*_index_proximal_joint": 1000.0,
                ".*_index_distal_joint": 1000.0,
                ".*_middle_proximal_joint": 1000.0,
                ".*_middle_distal_joint": 1000.0,
                ".*_ring_proximal_joint": 1000.0,
                ".*_ring_distal_joint": 1000.0,
                ".*_pinky_proximal_joint": 1000.0,
                ".*_pinky_distal_joint": 1000.0,
            },
            damping={
                ".*_thumb_metacarpal_joint": 15,
                ".*_thumb_proximal_joint": 15,
                ".*_thumb_distal_joint": 15,
                ".*_index_proximal_joint": 15,
                ".*_index_distal_joint": 15,
                ".*_middle_proximal_joint": 15,
                ".*_middle_distal_joint": 15,
                ".*_ring_proximal_joint": 15,
                ".*_ring_distal_joint": 15,
                ".*_pinky_proximal_joint": 15,
                ".*_pinky_distal_joint": 15,
            },
            armature={".*": 0.0},
        ),
    },
)


G129_CFG_WITH_DEX1_WHOLEBODY = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd", #f"{project_root}/assets/robots/g1/g1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=1
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        joint_pos={
            ".*_hip_pitch_joint": -0.20,
            ".*_knee_joint": 0.42,
            ".*_ankle_pitch_joint": -0.23,
            ".*_elbow_joint": 0.87,
            "left_shoulder_roll_joint": 0.18,
            "left_shoulder_pitch_joint": 0.35,
            "right_shoulder_roll_joint": -0.18,
            "right_shoulder_pitch_joint": 0.35,
            # fingers joints
            "left_hand_Joint1_1": 0.024,
            "left_hand_Joint2_1": 0.024,
            "right_hand_Joint1_1": 0.024,
            "right_hand_Joint2_1": 0.024,

        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.90,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
                ".*waist.*",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 88.0,
                ".*_hip_roll_joint": 139.0,
                ".*_hip_pitch_joint": 88.0,
                ".*_knee_joint": 139.0,
                ".*waist_yaw_joint": 88.0,
                ".*waist_roll_joint": 35.0,
                ".*waist_pitch_joint": 35.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_pitch_joint": 32.0,
                ".*_knee_joint": 20.0,
                ".*waist_yaw_joint": 32.0,
                ".*waist_roll_joint": 30.0,
                ".*waist_pitch_joint": 30.0,
            },
            stiffness={
                ".*_hip_yaw_joint": 150.0,
                ".*_hip_roll_joint": 150.0,
                ".*_hip_pitch_joint": 200.0,
                ".*_knee_joint": 200.0,
                ".*waist.*": 200.0,
            },
            damping={
                ".*_hip_yaw_joint": 5.0,
                ".*_hip_roll_joint": 5.0,
                ".*_hip_pitch_joint": 5.0,
                ".*_knee_joint": 5.0,
                ".*waist.*": 5.0,
            },
            armature=0.01,
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 35.0,
                ".*_ankle_roll_joint": 35.0,
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 30.0,
                ".*_ankle_roll_joint": 30.0,
            },
            stiffness=20.0,
            damping=2.0,
            armature=0.01,
        ),
        "shoulders": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 25.0,
                ".*_shoulder_roll_joint": 25.0,
            },
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 37.0,
                ".*_shoulder_roll_joint": 37.0,
            },
            stiffness=100.0,
            damping=2.0,
            armature=0.01,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_yaw_joint": 25.0,
                ".*_elbow_joint": 25.0,
            },
            velocity_limit_sim={
                ".*_shoulder_yaw_joint": 37.0,
                ".*_elbow_joint": 37.0,
            },
            stiffness=50.0,
            damping=2.0,
            armature=0.01,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_.*",
            ],
            effort_limit_sim={
                ".*_wrist_yaw_joint": 5.0,
                ".*_wrist_roll_joint": 25.0,
                ".*_wrist_pitch_joint": 5.0,
            },
            velocity_limit_sim={
                ".*_wrist_yaw_joint": 22.0,
                ".*_wrist_roll_joint": 37.0,
                ".*_wrist_pitch_joint": 22.0,
            },
            stiffness=40.0,
            damping=2.0,
            armature=0.01,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                "left_hand_Joint1_1",
                "left_hand_Joint2_1",
                "right_hand_Joint1_1",
                "right_hand_Joint2_1",
            ],
            effort_limit=None,  # increase the torque limit
            velocity_limit=None,  # set the velocity limit to 0
            stiffness=800.0,    # increase the stiffness (kp)
            damping=3.0,        # increase the damping (kd)
            friction=200.0,
            armature=None,
        ),
    },
)


G129_CFG_WITH_DEX3_WHOLEBODY = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-29dof_wholebody_dex3/g1_29dof_with_dex3_rev_1_0.usd", #f"{project_root}/assets/robots/g1/g1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=1
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        joint_pos={
            ".*_hip_pitch_joint": -0.20,
            ".*_knee_joint": 0.42,
            ".*_ankle_pitch_joint": -0.23,
            ".*_elbow_joint": 0.87,
            "left_shoulder_roll_joint": 0.18,
            "left_shoulder_pitch_joint": 0.35,
            "right_shoulder_roll_joint": -0.18,
            "right_shoulder_pitch_joint": 0.35,
       
            # fingers joints
            "left_hand_index_0_joint": 0.0,
            "left_hand_middle_0_joint": 0.0,
            "left_hand_thumb_0_joint": 0.0,
            "left_hand_index_1_joint": 0.0,
            "left_hand_middle_1_joint": 0.0,
            "left_hand_thumb_1_joint": 0.0,
            "left_hand_thumb_2_joint": 0.0,
            
            "right_hand_index_0_joint": 0.0,
            "right_hand_middle_0_joint": 0.0,
            "right_hand_thumb_0_joint": 0.0,
            "right_hand_index_1_joint": 0.0,
            "right_hand_middle_1_joint": 0.0,
            "right_hand_thumb_1_joint": 0.0,
            "right_hand_thumb_2_joint": 0.0,

        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.90,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
                ".*waist.*",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 88.0,
                ".*_hip_roll_joint": 139.0,
                ".*_hip_pitch_joint": 88.0,
                ".*_knee_joint": 139.0,
                ".*waist_yaw_joint": 88.0,
                ".*waist_roll_joint": 35.0,
                ".*waist_pitch_joint": 35.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_pitch_joint": 32.0,
                ".*_knee_joint": 20.0,
                ".*waist_yaw_joint": 32.0,
                ".*waist_roll_joint": 30.0,
                ".*waist_pitch_joint": 30.0,
            },
            stiffness={
                ".*_hip_yaw_joint": 150.0,
                ".*_hip_roll_joint": 150.0,
                ".*_hip_pitch_joint": 200.0,
                ".*_knee_joint": 200.0,
                ".*waist.*": 200.0,
            },
            damping={
                ".*_hip_yaw_joint": 5.0,
                ".*_hip_roll_joint": 5.0,
                ".*_hip_pitch_joint": 5.0,
                ".*_knee_joint": 5.0,
                ".*waist.*": 5.0,
            },
            armature=0.01,
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 35.0,
                ".*_ankle_roll_joint": 35.0,
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 30.0,
                ".*_ankle_roll_joint": 30.0,
            },
            stiffness=20.0,
            damping=2.0,
            armature=0.01,
        ),
        "shoulders": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 25.0,
                ".*_shoulder_roll_joint": 25.0,
            },
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 37.0,
                ".*_shoulder_roll_joint": 37.0,
            },
            stiffness=100.0,
            damping=2.0,
            armature=0.01,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_yaw_joint": 25.0,
                ".*_elbow_joint": 25.0,
            },
            velocity_limit_sim={
                ".*_shoulder_yaw_joint": 37.0,
                ".*_elbow_joint": 37.0,
            },
            stiffness=50.0,
            damping=2.0,
            armature=0.01,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_.*",
            ],
            effort_limit_sim={
                ".*_wrist_yaw_joint": 5.0,
                ".*_wrist_roll_joint": 25.0,
                ".*_wrist_pitch_joint": 5.0,
            },
            velocity_limit_sim={
                ".*_wrist_yaw_joint": 22.0,
                ".*_wrist_roll_joint": 37.0,
                ".*_wrist_pitch_joint": 22.0,
            },
            stiffness=40.0,
            damping=2.0,
            armature=0.01,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hand_index_.*_joint",
                ".*_hand_middle_.*_joint",
                ".*_hand_thumb_.*_joint"
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*": 100.0,
            },
            damping={
                ".*": 10.0,
            },
            armature={
                ".*": 0.1
            },
        ),
    },
)


G129_CFG_WITH_INSPIRE_WHOLEBODY = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-29dof_wholebody_inspire/g1_29dof_with_inspire_rev_1_0.usd", #f"{project_root}/assets/robots/g1/g1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=1
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        joint_pos={
            ".*_hip_pitch_joint": -0.20,
            ".*_knee_joint": 0.42,
            ".*_ankle_pitch_joint": -0.23,
            ".*_elbow_joint": 0.87,
            "left_shoulder_roll_joint": 0.18,
            "left_shoulder_pitch_joint": 0.35,
            "right_shoulder_roll_joint": -0.18,
            "right_shoulder_pitch_joint": 0.35,

            
            # fingers joints
            "L_index_proximal_joint": 0.0,
            "L_index_intermediate_joint": 0.0,
            "L_middle_proximal_joint": 0.0,
            "L_middle_intermediate_joint": 0.0,
            "L_pinky_proximal_joint":0.0,
            "L_pinky_intermediate_joint":0.0,
            "L_ring_proximal_joint":0.0,
            "L_ring_intermediate_joint":0.0,
            "L_thumb_proximal_yaw_joint":0.0,
            "L_thumb_proximal_pitch_joint":0.0,
            "L_thumb_intermediate_joint":0.0,
            "L_thumb_distal_joint":0.0,

            "R_index_proximal_joint": 0.0,
            "R_index_intermediate_joint": 0.0,
            "R_middle_proximal_joint": 0.0,
            "R_middle_intermediate_joint": 0.0,
            "R_pinky_proximal_joint":0.0,
            "R_pinky_intermediate_joint":0.0,
            "R_ring_proximal_joint":0.0,
            "R_ring_intermediate_joint":0.0,
            "R_thumb_proximal_yaw_joint":0.0,
            "R_thumb_proximal_pitch_joint":0.0,
            "R_thumb_intermediate_joint":0.0,
            "R_thumb_distal_joint":0.0,

        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.90,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
                ".*waist.*",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 88.0,
                ".*_hip_roll_joint": 139.0,
                ".*_hip_pitch_joint": 88.0,
                ".*_knee_joint": 139.0,
                ".*waist_yaw_joint": 88.0,
                ".*waist_roll_joint": 35.0,
                ".*waist_pitch_joint": 35.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_pitch_joint": 32.0,
                ".*_knee_joint": 20.0,
                ".*waist_yaw_joint": 32.0,
                ".*waist_roll_joint": 30.0,
                ".*waist_pitch_joint": 30.0,
            },
            stiffness={
                ".*_hip_yaw_joint": 150.0,
                ".*_hip_roll_joint": 150.0,
                ".*_hip_pitch_joint": 200.0,
                ".*_knee_joint": 200.0,
                ".*waist.*": 200.0,
            },
            damping={
                ".*_hip_yaw_joint": 5.0,
                ".*_hip_roll_joint": 5.0,
                ".*_hip_pitch_joint": 5.0,
                ".*_knee_joint": 5.0,
                ".*waist.*": 5.0,
            },
            armature=0.01,
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 35.0,
                ".*_ankle_roll_joint": 35.0,
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 30.0,
                ".*_ankle_roll_joint": 30.0,
            },
            stiffness=20.0,
            damping=2.0,
            armature=0.01,
        ),
        "shoulders": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 25.0,
                ".*_shoulder_roll_joint": 25.0,
            },
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 37.0,
                ".*_shoulder_roll_joint": 37.0,
            },
            stiffness=100.0,
            damping=2.0,
            armature=0.01,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_yaw_joint": 25.0,
                ".*_elbow_joint": 25.0,
            },
            velocity_limit_sim={
                ".*_shoulder_yaw_joint": 37.0,
                ".*_elbow_joint": 37.0,
            },
            stiffness=50.0,
            damping=2.0,
            armature=0.01,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_.*",
            ],
            effort_limit_sim={
                ".*_wrist_yaw_joint": 5.0,
                ".*_wrist_roll_joint": 25.0,
                ".*_wrist_pitch_joint": 5.0,
            },
            velocity_limit_sim={
                ".*_wrist_yaw_joint": 22.0,
                ".*_wrist_roll_joint": 37.0,
                ".*_wrist_pitch_joint": 22.0,
            },
            stiffness=40.0,
            damping=2.0,
            armature=0.01,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_index_proximal_joint",
                ".*_index_intermediate_joint",
                ".*_middle_proximal_joint",
                ".*_middle_intermediate_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_intermediate_joint",
                ".*_ring_proximal_joint",
                ".*_ring_intermediate_joint",
                ".*_thumb_proximal_yaw_joint",
                ".*_thumb_proximal_pitch_joint",
                ".*_thumb_intermediate_joint",
                ".*_thumb_distal_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_index_proximal_joint":1000.0,
                ".*_index_intermediate_joint":1000.0,
                ".*_middle_proximal_joint":1000.0,
                ".*_middle_intermediate_joint":1000.0,
                ".*_pinky_proximal_joint":1000.0,
                ".*_pinky_intermediate_joint":1000.0,
                ".*_ring_proximal_joint":1000.0,
                ".*_ring_intermediate_joint":1000.0,
                ".*_thumb_proximal_yaw_joint":1000.0,
                ".*_thumb_proximal_pitch_joint":1000.0,
                ".*_thumb_intermediate_joint":1000.0,
                ".*_thumb_distal_joint":1000.0,
            },
            damping={
                ".*_index_proximal_joint":15,
                ".*_index_intermediate_joint":15,
                ".*_middle_proximal_joint":15,
                ".*_middle_intermediate_joint":15,
                ".*_pinky_proximal_joint":15,
                ".*_pinky_intermediate_joint":15,
                ".*_ring_proximal_joint":15,
                ".*_ring_intermediate_joint":15,
                ".*_thumb_proximal_yaw_joint":15,
                ".*_thumb_proximal_pitch_joint":15,
                ".*_thumb_intermediate_joint":15,
                ".*_thumb_distal_joint":15,
            },
            armature={
                ".*": 0.0
            },
        ),
    },
)


# Floating-base (fix_base=False) conversion of the same G1 23DOF + BrainCo robot used by
# G123_CFG_WITH_BRAINCO_HAND, for the wholebody task driven by policy-23dof.onnx
# (assets/model/policy-23dof.onnx, trained in mjlab/rsl_rl -- see
# action_provider/action_provider_wh_g123_dds.py). Leg/waist/ankle stiffness & damping
# below are transcribed from that ONNX file's own metadata (its trained PD gains, in
# mjlab's armature-scaled convention), since matching them is what lets the policy's leg
# tracking behave as it did in training. Arms are teleop position targets, not
# policy-controlled (the policy's own arm actions are observed but discarded -- see the
# action provider's module docstring). The forward-lean fall this task originally showed
# turned out to be a mass bug, not a gain bug: the URDF declared several links (IMU/camera
# mounts, all 10 fingertips) with exactly 0.0 mass, which PhysX silently defaulted to
# 1.0 kg each -- 14 kg of unmodeled mass, ~42% over the mjlab-trained total. Fixed by
# giving those links a small positive mass in the source URDF and reconverting this USD
# (see project_g123_policy23dof_forward_fall_debug memory). Arm gains are back to the
# pre-gravity-droop-fix values (25/50/40 kp) now that the phantom fingertip mass -- the
# likely real cause of that original droop -- is gone; see the "arms" actuator group
# comment below and [[project_g123_arm_gravity_droop]].
G123_CFG_WITH_BRAINCO_WHOLEBODY = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-23dof_wholebody_brainco/g1_23dof_mode_10_with_brainco_wrist_cams_wholebody.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=1
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        # Leg/waist spawn values match policy-23dof.onnx's own default_joint_pos (its action
        # offset and obs reference), so the robot starts with zero policy tracking error under
        # its trained gains.
        #
        # Arms are zeroed (not the policy's 0.35/0.18/0/0.87/0 default) so the robot spawns in
        # the teleop-ready L-shape posture: FK on the URDF shows all-zero arm joints already put
        # the forearms horizontal/forward with fingers forward and thumbs up, whereas the
        # policy's own arm default points the forearms down. The policy doesn't control the arms
        # (teleop overrides them via DDSRLActionProviderG123's arm-override path, see
        # action_provider_wh_g123_dds.py) but does *observe* their true position, so
        # DEFAULT_JOINT_POS there must be kept at these same zeroed arm values or the policy's
        # arm observation would be offset from what it was trained on.
        joint_pos={
            "left_hip_pitch_joint": -0.1,
            "left_hip_roll_joint": 0.0,
            "left_hip_yaw_joint": 0.0,
            "left_knee_joint": 0.3,
            "left_ankle_pitch_joint": -0.2,
            "left_ankle_roll_joint": 0.0,

            "right_hip_pitch_joint": -0.1,
            "right_hip_roll_joint": 0.0,
            "right_hip_yaw_joint": 0.0,
            "right_knee_joint": 0.3,
            "right_ankle_pitch_joint": -0.2,
            "right_ankle_roll_joint": 0.0,

            "waist_yaw_joint": 0.0,

            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,

            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,

            # hand finger joints (tip joints excluded)
            "left_index_proximal_joint": 0.0,
            "left_index_distal_joint": 0.0,
            "left_middle_proximal_joint": 0.0,
            "left_middle_distal_joint": 0.0,
            "left_ring_proximal_joint": 0.0,
            "left_ring_distal_joint": 0.0,
            "left_pinky_proximal_joint": 0.0,
            "left_pinky_distal_joint": 0.0,
            "left_thumb_metacarpal_joint": 0.0,
            "left_thumb_proximal_joint": 0.0,
            "left_thumb_distal_joint": 0.0,

            "right_thumb_metacarpal_joint": 0.0,
            "right_thumb_proximal_joint": 0.0,
            "right_thumb_distal_joint": 0.0,
            "right_index_proximal_joint": 0.0,
            "right_index_distal_joint": 0.0,
            "right_middle_proximal_joint": 0.0,
            "right_middle_distal_joint": 0.0,
            "right_ring_proximal_joint": 0.0,
            "right_ring_distal_joint": 0.0,
            "right_pinky_proximal_joint": 0.0,
            "right_pinky_distal_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # Stiffness/damping/armature/effort below are transcribed from
        # policy-23dof.onnx's metadata (its trained PD gains, mjlab's
        # armature-scaled convention) -- see the class-level comment above.
        "legs_7520_14": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint",
                ".*_hip_yaw_joint",
                "waist_yaw_joint",
            ],
            effort_limit=88.0,
            velocity_limit=32.0,
            stiffness={
                ".*_hip_pitch_joint": 40.179,
                ".*_hip_yaw_joint": 40.179,
                "waist_yaw_joint": 40.179,
            },
            damping={
                ".*_hip_pitch_joint": 2.558,
                ".*_hip_yaw_joint": 2.558,
                "waist_yaw_joint": 2.558,
            },
            armature=0.0101775,
        ),
        "legs_7520_22": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint", ".*_knee_joint"],
            effort_limit=139.0,
            velocity_limit=20.0,
            stiffness={
                ".*_hip_roll_joint": 99.098,
                ".*_knee_joint": 99.098,
            },
            damping={
                ".*_hip_roll_joint": 6.309,
                ".*_knee_joint": 6.309,
            },
            armature=0.0251019,
        ),
        "feet": ImplicitActuatorCfg(
            # ankle_pitch diagnostically doubled from the ONNX-metadata trained gain
            # (28.501/1.814) -- logging isolated ankle_pitch as the only leg/waist joint
            # with a large, load-dependent tracking error from step 1 onward (droops
            # under body-weight loading even though the PD formula and gain numbers
            # match mjlab's exactly), most likely a PhysX-vs-MuJoCo gain-authority gap
            # at this specific load-bearing joint. ankle_roll left at the trained value
            # as a control -- it was tracking cleanly. See
            # project_g123_policy23dof_forward_fall_debug memory for the log evidence.
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit=50.0,
            velocity_limit=37.0,
            stiffness={
                ".*_ankle_pitch_joint": 57.0,
                ".*_ankle_roll_joint": 28.501,
            },
            damping={
                ".*_ankle_pitch_joint": 3.6,
                ".*_ankle_roll_joint": 1.814,
            },
            armature=0.0072194,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_roll_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            # Reverted to the pre-gravity-droop-fix 25/50/40 kp, kd=2 pattern (2026-07-10,
            # see [[project_g123_arm_gravity_droop]] and project_g123_policy23dof_forward_fall_debug
            # memory). The 200/20/80 gains were raised to fix a "hand won't lift" droop, but
            # that USD had the same URDF mass bug fixed here: 14 phantom-mass links
            # (imu/camera mounts + all 10 fingertips) each defaulted to 1.0 kg in PhysX
            # because the URDF declared exactly 0.0 mass rather than a small positive value
            # -- ~5 kg of unmodeled mass sitting on each hand's fingertips. Now that the
            # wholebody USD is regenerated from the mass-corrected URDF (total ~32.8 kg,
            # matching mjlab), the original real cause of the droop is gone, so the inflated
            # gains are no longer needed here. Do NOT revert G123_CFG_WITH_BRAINCO_HAND's
            # arms group the same way until its own USD
            # (assets/robots/g1-23dof-brainco-base-fix-usd/with_wrist_cams/...) is
            # reconverted from this URDF -- verified 2026-07-10 that USD still has
            # physics:mass=0.0 on the same 14 links, so it likely still has the real droop.
            stiffness={
                ".*_shoulder_.*_joint": 25.0,
                ".*_elbow_joint": 50.0,
                ".*_wrist_roll_joint": 40.0,
            },
            damping={
                ".*_shoulder_.*_joint": 2.0,
                ".*_elbow_joint": 2.0,
                ".*_wrist_roll_joint": 2.0,
            },
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_thumb_metacarpal_joint",
                ".*_thumb_proximal_joint",
                ".*_thumb_distal_joint",
                ".*_index_proximal_joint",
                ".*_index_distal_joint",
                ".*_middle_proximal_joint",
                ".*_middle_distal_joint",
                ".*_ring_proximal_joint",
                ".*_ring_distal_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_distal_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_thumb_metacarpal_joint": 1000.0,
                ".*_thumb_proximal_joint": 1000.0,
                ".*_thumb_distal_joint": 1000.0,
                ".*_index_proximal_joint": 1000.0,
                ".*_index_distal_joint": 1000.0,
                ".*_middle_proximal_joint": 1000.0,
                ".*_middle_distal_joint": 1000.0,
                ".*_ring_proximal_joint": 1000.0,
                ".*_ring_distal_joint": 1000.0,
                ".*_pinky_proximal_joint": 1000.0,
                ".*_pinky_distal_joint": 1000.0,
            },
            damping={
                ".*_thumb_metacarpal_joint": 15,
                ".*_thumb_proximal_joint": 15,
                ".*_thumb_distal_joint": 15,
                ".*_index_proximal_joint": 15,
                ".*_index_distal_joint": 15,
                ".*_middle_proximal_joint": 15,
                ".*_middle_distal_joint": 15,
                ".*_ring_proximal_joint": 15,
                ".*_ring_distal_joint": 15,
                ".*_pinky_proximal_joint": 15,
                ".*_pinky_distal_joint": 15,
            },
            armature={".*": 0.0},
        ),
    },
)


H12_CFG_WITH_INSPIRE_HAND = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/h1_2-26dof-inspire-base-fix-usd/h1_2_26dof_with_inspire_rev_1_0.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, 
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4
        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.05,
            "left_knee_joint": 0.2,
            "left_ankle_pitch_joint": -0.15,
            "left_ankle_roll_joint": 0.0,
            
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.05,
            "right_knee_joint": 0.2,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,
            
            
            # arms joints
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
            
            # fingers joints
            "L_index_proximal_joint": 0.0,
            "L_index_intermediate_joint": 0.0,
            "L_middle_proximal_joint": 0.0,
            "L_middle_intermediate_joint": 0.0,
            "L_pinky_proximal_joint":0.0,
            "L_pinky_intermediate_joint":0.0,
            "L_ring_proximal_joint":0.0,
            "L_ring_intermediate_joint":0.0,
            "L_thumb_proximal_yaw_joint":0.0,
            "L_thumb_proximal_pitch_joint":0.0,
            "L_thumb_intermediate_joint":0.0,
            "L_thumb_distal_joint":0.0,

            "R_index_proximal_joint": 0.0,
            "R_index_intermediate_joint": 0.0,
            "R_middle_proximal_joint": 0.0,
            "R_middle_intermediate_joint": 0.0,
            "R_pinky_proximal_joint":0.0,
            "R_pinky_intermediate_joint":0.0,
            "R_ring_proximal_joint":0.0,
            "R_ring_intermediate_joint":0.0,
            "R_thumb_proximal_yaw_joint":0.0,
            "R_thumb_proximal_pitch_joint":0.0,
            "R_thumb_intermediate_joint":0.0,
            "R_thumb_distal_joint":0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint", 
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint", 
                ".*_knee_joint",
            ],
            effort_limit=None,
            velocity_limit=None,
            stiffness=None,
            damping=None,
            armature=None,
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=None,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness=None,
            damping=None,
            # armature=0.001,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_.*_joint"
            ],
            effort_limit=None,
            velocity_limit=None,
             stiffness={  # increase the stiffness (kp)
                 ".*_shoulder_.*_joint": 25.0,
                 ".*_elbow_joint": 50.0,
                 ".*_wrist_.*_joint": 40.0,
            },
             damping={    # increase the damping (kd)
                 ".*_shoulder_.*_joint": 2.0,
                 ".*_elbow_joint": 2.0,
                 ".*_wrist_.*_joint": 2.0,
             },
            armature=None,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_index_proximal_joint",
                ".*_index_intermediate_joint",
                ".*_middle_proximal_joint",
                ".*_middle_intermediate_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_intermediate_joint",
                ".*_ring_proximal_joint",
                ".*_ring_intermediate_joint",
                ".*_thumb_proximal_yaw_joint",
                ".*_thumb_proximal_pitch_joint",
                ".*_thumb_intermediate_joint",
                ".*_thumb_distal_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_index_proximal_joint":1000.0,
                ".*_index_intermediate_joint":1000.0,
                ".*_middle_proximal_joint":1000.0,
                ".*_middle_intermediate_joint":1000.0,
                ".*_pinky_proximal_joint":1000.0,
                ".*_pinky_intermediate_joint":1000.0,
                ".*_ring_proximal_joint":1000.0,
                ".*_ring_intermediate_joint":1000.0,
                ".*_thumb_proximal_yaw_joint":1000.0,
                ".*_thumb_proximal_pitch_joint":1000.0,
                ".*_thumb_intermediate_joint":1000.0,
                ".*_thumb_distal_joint":1000.0,
            },
            damping={
                ".*_index_proximal_joint":15,
                ".*_index_intermediate_joint":15,
                ".*_middle_proximal_joint":15,
                ".*_middle_intermediate_joint":15,
                ".*_pinky_proximal_joint":15,
                ".*_pinky_intermediate_joint":15,
                ".*_ring_proximal_joint":15,
                ".*_ring_intermediate_joint":15,
                ".*_thumb_proximal_yaw_joint":15,
                ".*_thumb_proximal_pitch_joint":15,
                ".*_thumb_intermediate_joint":15,
                ".*_thumb_distal_joint":15,
            },
            armature={
                ".*": 0.0
            },
        ),

    },
)
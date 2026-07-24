
G129_CFG_WITH_BRAINCO_HAND = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{project_root}/assets/robots/g1-29dof-brainco-base-fix-usd/g1_29dof_with_brainco_rev_1_0.usd",
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
            
            # left hand finger joints
            "left_thumb_metacarpal_joint": 0.0,
            "left_thumb_proximal_joint": 0.0,
            "left_thumb_distal_joint": 0.0,
            "left_thumb_tip_joint": 0.0,
            "left_index_proximal_joint": 0.0,
            "left_index_distal_joint": 0.0,
            "left_index_tip_joint": 0.0,
            "left_middle_proximal_joint": 0.0,
            "left_middle_distal_joint": 0.0,
            "left_middle_tip_joint": 0.0,
            "left_ring_proximal_joint": 0.0,
            "left_ring_distal_joint": 0.0,
            "left_ring_tip_joint": 0.0,
            "left_pinky_proximal_joint": 0.0,
            "left_pinky_distal_joint": 0.0,
            "left_pinky_tip_joint": 0.0,

            # right hand finger joints
            "right_thumb_metacarpal_joint": 0.0,
            "right_thumb_proximal_joint": 0.0,
            "right_thumb_distal_joint": 0.0,
            "right_thumb_tip": 0.0,
            "right_index_proximal_joint": 0.0,
            "right_index_distal_joint": 0.0,
            "right_index_tip_joint": 0.0,
            "right_middle_proximal_joint": 0.0,
            "right_middle_distal_joint": 0.0,
            "right_middle_tip_joint": 0.0,
            "right_ring_proximal_joint": 0.0,
            "right_ring_distal_joint": 0.0,
            "right_ring_tip_joint": 0.0,
            "right_pinky_proximal_joint": 0.0,
            "right_pinky_distal_joint": 0.0,
            "right_pinky_tip_joint": 0.0,
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
                ".*_thumb_tip.*",
                ".*_index_proximal_joint",
                ".*_index_distal_joint",
                ".*_index_tip_joint",
                ".*_middle_proximal_joint",
                ".*_middle_distal_joint",
                ".*_middle_tip_joint",
                ".*_ring_proximal_joint",
                ".*_ring_distal_joint",
                ".*_ring_tip_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_distal_joint",
                ".*_pinky_tip_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_thumb_metacarpal_joint":1000.0,
                ".*_thumb_proximal_joint":1000.0,
                ".*_thumb_distal_joint":1000.0,
                ".*_thumb_tip.*":1000.0,
                ".*_index_proximal_joint":1000.0,
                ".*_index_distal_joint":1000.0,
                ".*_index_tip_joint":1000.0,
                ".*_middle_proximal_joint":1000.0,
                ".*_middle_distal_joint":1000.0,
                ".*_middle_tip_joint":1000.0,
                ".*_ring_proximal_joint":1000.0,
                ".*_ring_distal_joint":1000.0,
                ".*_ring_tip_joint":1000.0,
                ".*_pinky_proximal_joint":1000.0,
                ".*_pinky_distal_joint":1000.0,
                ".*_pinky_tip_joint":1000.0,
            },
            damping={
                ".*_thumb_metacarpal_joint":15,
                ".*_thumb_proximal_joint":15,
                ".*_thumb_distal_joint":15,
                ".*_thumb_tip.*":15,
                ".*_index_proximal_joint":15,
                ".*_index_distal_joint":15,
                ".*_index_tip_joint":15,
                ".*_middle_proximal_joint":15,
                ".*_middle_distal_joint":15,
                ".*_middle_tip_joint":15,
                ".*_ring_proximal_joint":15,
                ".*_ring_distal_joint":15,
                ".*_ring_tip_joint":15,
                ".*_pinky_proximal_joint":15,
                ".*_pinky_distal_joint":15,
                ".*_pinky_tip_joint":15,
            },
            armature={
                ".*": 0.0
            },
        ),

    },
)


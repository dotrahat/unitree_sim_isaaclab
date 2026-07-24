# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
import os
import torch

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, RigidObjectCfg, DeformableObjectCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from . import mdp
# use Isaac Lab native event system

from tasks.common_config import  G1RobotPresets, CameraPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager

# import public scene configuration
from tasks.common_scene.base_scene_pickplace_cylindercfg import TableCylinderSceneCfg

project_root = os.environ.get("PROJECT_ROOT")

##
# Scene definition
##

@configclass
class ObjectTableSceneCfg(TableCylinderSceneCfg):
    """object table scene configuration class
    inherits from TableCylinderSceneCfg, gets the complete shared table/room/light scene,
    but overrides the graspable object with a real-world asset (tomato soup can) instead
    of the abstract cylinder primitive. Future steps will add more objects here.
    """

    # Humanoid robot w/ arms higher
    # 5. humanoid robot configuration
    robot: ArticulationCfg = G1RobotPresets.g1_23dof_brainco_base_fix()


    # 6. add camera configuration
    front_camera = CameraPresets.g1_front_camera()
    left_wrist_camera = CameraPresets.left_brainco_wrist_camera()
    right_wrist_camera = CameraPresets.right_brainco_wrist_camera()

    # 7. object override (Step 1: tomato soup can, replacing the cylinder)
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-0.35, 0.40, 0.85],
            rot=[0.7071, -0.7071, 0.0, 0.0],
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/multiobject_assets/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.35),
        ),
    )

    # 8. object_2 override (Step 2: mustard bottle, added alongside the soup can)
    object_2 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object_2",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-0.15, 0.40, 0.892],
            rot=[0.7071, -0.7071, 0.0, 0.0],
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/multiobject_assets/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.6),
        ),
    )

    # 9. object_3 override (Step 3: cracker box, added alongside the soup can and mustard bottle)
    # Note: swapped from the generic Props/Food/mac_n_cheese_centered.usd, which ships with no
    # authored PhysX collision/rigid-body schema (UsdFileCfg.collision_props can only tune an
    # existing collider, not create one) -- the YCB Axis_Aligned_Physics cracker box gives the
    # same "box" shape variety with physics already authored, matching object/object_2.
    object_3 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object_3",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[0.05, 0.454, 0.90],
            rot=[0.7071, -0.7071, 0.0, 0.0],
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/multiobject_assets/YCB/Axis_Aligned_Physics/003_cracker_box.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.4),
        ),
    )

    # 10. object_4 override (Step 4: banana, added alongside the soup can, mustard bottle, cracker box)
    # Note: the downloaded banana only exists as Axis_Aligned (no _Physics variant -- no authored
    # collision/rigid-body/mass schema at all, unlike the boxes/cans/bottle). Rather than falling
    # back to another box shape, we authored physics onto it ourselves (RigidBodyAPI + MassAPI on
    # the root prim, CollisionAPI + MeshCollisionAPI(convexHull) on its mesh) via a one-time
    # offline script and saved the result as YCB/Axis_Aligned_Physics/011_banana.usd, mirroring how
    # NVIDIA prepared the other YCB _Physics variants. This gives us the organic/non-convex shape
    # for grasp variety that no downloaded physics-ready asset covers.
    object_4 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object_4",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=[-0.35, 0.55, 0.83],
            rot=[1, 0, 0, 0],
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/multiobject_assets/YCB/Axis_Aligned/011_banana_physics.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.15),
        ),
    )

    # 11. object_5 (Step 5: deformable tube -- the soft/deformable object)
    # Note: this is a DeformableObjectCfg, not RigidObjectCfg -- a completely different asset type
    # (PhysX FEM soft body), the first deformable object in this repo. Deformable simulation only
    # runs on the GPU PhysX backend (unlike rigid bodies, which work on both CPU and GPU) -- this
    # task must be launched with --device cuda (not --device cpu, used by every other step so far).
    # The tube usd already ships with pre-authored deformable schema (found via a "softBodyCrc"
    # token check), so no manual physics authoring was needed here, unlike the banana.
    object_5 = DeformableObjectCfg(
        prim_path="/World/envs/env_.*/Object_5",
        init_state=DeformableObjectCfg.InitialStateCfg(
            pos=[-0.15, 0.55, 0.80035],
            rot=[1, 0, 0, 0],
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/multiobject_assets/DeformableTube/tube.usd",
            deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
        ),
    )

##
# MDP settings
##
@configclass
class ActionsCfg:
    """defines the action configuration related to robot control, using direct joint angle control
    """
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True)



@configclass
class ObservationsCfg:
    """
    defines all available observation information
    """
    @configclass
    class PolicyCfg(ObsGroup):
        """policy group observation configuration class
        defines all state observation values for policy decision
        inherit from ObsGroup base class
        """

        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        robot_brainco_state = ObsTerm(func=mdp.get_robot_brainco_joint_states)

        camera_image = ObsTerm(func=mdp.get_camera_image)

        def __post_init__(self):
            """post initialization function
            set the basic attributes of the observation group
            """
            self.enable_corruption = False  # disable observation value corruption
            self.concatenate_terms = False  # disable observation item connection

    # observation groups
    # create policy observation group instance
    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    # check if the object is out of the working range
    success = DoneTerm(func=mdp.reset_object_estimate)# use task completion check function

@configclass
class RewardsCfg:
    reward = RewTerm(func=mdp.compute_reward,weight=1.0)

@configclass
class EventCfg:
    reset_object = EventTermCfg(
        func=mdp.reset_root_state_uniform,  # use uniform distribution reset function
        mode="reset",   # set event mode to reset
        params={
            # position range parameter
            "pose_range": {
                "x": [-0.05, 0.05],  # x axis position range: -0.05 to 0.0 meter
                "y": [-0.05, 0.05],   # y axis position range: 0.0 to 0.05 meter
            },
            # speed range parameter (empty dictionary means using default value)
            "velocity_range": {},
            # specify the object to reset
            "asset_cfg": SceneEntityCfg("object"),
        },
    )


@configclass
class PickPlaceMultiObjectG123BraincoEnvCfg(ManagerBasedRLEnvCfg):
    """
    inherits from ManagerBasedRLEnvCfg, defines all configuration parameters for the entire environment
    """

    # 1. scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=1, # environment number: 1
                                                     env_spacing=2.5, # environment spacing: 2.5 meter
                                                     replicate_physics=True # enable physics replication
                                                     )
    # basic settings
    observations: ObservationsCfg = ObservationsCfg()   # observation configuration
    actions: ActionsCfg = ActionsCfg()                  # action configuration
    # MDP settings

    terminations: TerminationsCfg = TerminationsCfg()    # termination configuration
    events = EventCfg()                                  # event configuration
    commands = None # command manager
    rewards: RewardsCfg = RewardsCfg()  # reward manager
    curriculum = None # curriculum manager
    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        # create event manager
        self.event_manager = SimpleEventManager()

        # register "reset object" event
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=lambda env: base_mdp.reset_root_state_uniform(
                env,
                torch.arange(env.num_envs, device=env.device),
                pose_range={"x": [-0.05, 0.05], "y": [0.0, 0.05]},
                velocity_range={},
                asset_cfg=SceneEntityCfg("object"),
            )
        ))

        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device))
        ))

# /home/cvl/convert_urdf_full.py

import argparse
from omni.isaac.lab.app import AppLauncher

# Launch IsaacSim headless first
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

# Now safe to import isaaclab converters
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.sim.converters.urdf_converter_cfg import JointDriveCfg, JointDriveGainsCfg

urdf_cfg = UrdfConverterCfg(
    asset_path                            = "/home/cvl/URDFs/g1_with_brainco_hand/your_robot.urdf",
    usd_dir                               = "/home/cvl/URDFs/g1_with_brainco_hand_usd",
    usd_file_name                         = "your_robot.usd",
    force_usd_conversion                  = True,
    make_instanceable                     = True,
    fix_base                              = True,
    root_link_name                        = None,
    link_density                          = 0.0,
    merge_fixed_joints                    = False,
    convert_mimic_joints_to_normal_joints = False,
    collider_type                         = "convex_hull",
    self_collision                        = False,
    replace_cylinders_with_capsules       = False,
    collision_from_visuals                = False,
    joint_drive                           = JointDriveCfg(
        drive_type  = "force",
        target_type = "position",
        gains       = JointDriveGainsCfg(
            stiffness = 100.0,
            damping   = 1.0,
        ),
    ),
)

print(f"Converting: {urdf_cfg.asset_path}")
converter = UrdfConverter(urdf_cfg)
print(f"Done! USD saved to: {urdf_cfg.usd_dir}/{urdf_cfg.usd_file_name}")

simulation_app.close()
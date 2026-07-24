# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Convert g1_23dof_with_brainco.urdf to USD format."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Convert g1_23dof_with_brainco URDF to USD")
parser.add_argument("--merge-joints", action="store_true", default=False)
parser.add_argument("--fix-base", action="store_true", default=True)
parser.add_argument("--joint-stiffness", type=float, default=100.0)
parser.add_argument("--joint-damping", type=float, default=1.0)
parser.add_argument(
    "--joint-target-type",
    type=str,
    default="position",
    choices=["position", "velocity", "none"],
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import contextlib
import os

import carb
import isaacsim.core.utils.stage as stage_utils
import omni.kit.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import check_file_path
from isaaclab.utils.dict import print_dict


def main():
    urdf_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "assets/robots/g1-23dof-brainco-base-fix-usd/G1_23DOF_Brainco_URDFs/g1_23dof_with_brainco_wrist_restored.urdf",
    )
    if not os.path.isabs(urdf_path):
        urdf_path = os.path.abspath(urdf_path)
    if not check_file_path(urdf_path):
        raise ValueError(f"Invalid file path: {urdf_path}")

    dest_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "assets/robots/g1-23dof-brainco-base-fix-usd/g1_23dof_with_brainco_wrist_restored.usd",
    )
    if not os.path.isabs(dest_path):
        dest_path = os.path.abspath(dest_path)

    urdf_converter_cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=os.path.dirname(dest_path),
        usd_file_name=os.path.basename(dest_path),
        fix_base=args_cli.fix_base,
        merge_fixed_joints=args_cli.merge_joints,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=args_cli.joint_stiffness,
                damping=args_cli.joint_damping,
            ),
            target_type=args_cli.joint_target_type,
        ),
    )

    print("-" * 80)
    print(f"Input URDF: {urdf_path}")
    print("UrdfConverter config:")
    print_dict(urdf_converter_cfg.to_dict(), nesting=0)
    print("-" * 80)

    urdf_converter = UrdfConverter(urdf_converter_cfg)
    print(f"Generated USD: {urdf_converter.usd_path}")
    print("-" * 80)

    carb_settings_iface = carb.settings.get_settings()
    local_gui = carb_settings_iface.get("/app/window/enabled")
    livestream_gui = carb_settings_iface.get("/app/livestream/enabled")

    if local_gui or livestream_gui:
        stage_utils.open_stage(urdf_converter.usd_path)
        app = omni.kit.app.get_app_interface()
        with contextlib.suppress(KeyboardInterrupt):
            while app.is_running():
                app.update()


if __name__ == "__main__":
    main()
    simulation_app.close()

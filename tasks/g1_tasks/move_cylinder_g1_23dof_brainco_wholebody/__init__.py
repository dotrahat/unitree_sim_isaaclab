
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import move_cylinder_g1_23dof_brainco_hw_env_cfg


gym.register(
    id="Isaac-Move-Cylinder-G123-Brainco-Wholebody",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": move_cylinder_g1_23dof_brainco_hw_env_cfg.MoveCylinderG123BraincoWholebodyEnvCfg,
    },
    disable_env_checker=True,
)

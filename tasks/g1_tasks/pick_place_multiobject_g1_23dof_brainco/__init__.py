
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import pickplace_multiobject_g1_23dof_brainco_env_cfg


gym.register(
    id="Isaac-PickPlace-MultiObject-G123-Brainco-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": pickplace_multiobject_g1_23dof_brainco_env_cfg.PickPlaceMultiObjectG123BraincoEnvCfg,
    },
    disable_env_checker=True,
)

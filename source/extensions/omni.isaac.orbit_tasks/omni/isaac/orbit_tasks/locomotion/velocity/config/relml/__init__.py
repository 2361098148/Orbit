# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents, flat_env_cfg, rough_env_cfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Velocity-Flat-ReLML-v0",
    entry_point="omni.isaac.orbit.envs:RLTaskEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.ReLMLFlatEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.ReLMLFlatPPORunnerCfg,
    },
)

gym.register(
    id="Isaac-Velocity-Flat-ReLML-Play-v0",
    entry_point="omni.isaac.orbit.envs:RLTaskEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.ReLMLFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.ReLMLFlatPPORunnerCfg,
    },
)

gym.register(
    id="Isaac-Velocity-Rough-ReLML-v0",
    entry_point="omni.isaac.orbit.envs:RLTaskEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.ReLMLRoughEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.ReLMLRoughPPORunnerCfg,
    },
)

gym.register(
    id="Isaac-Velocity-Rough-ReLML-Play-v0",
    entry_point="omni.isaac.orbit.envs:RLTaskEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.ReLMLRoughEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.ReLMLRoughPPORunnerCfg,
    },
)

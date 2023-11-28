# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.orbit.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Flat-ReLML-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""


import gymnasium as gym
import os
import torch
import traceback

import carb
from rsl_rl.runners import OnPolicyRunner

import omni.isaac.contrib_tasks  # noqa: F401
from omni.isaac.orbit.utils.logger import Logger
import omni.isaac.orbit_tasks  # noqa: F401
from omni.isaac.orbit_tasks.utils import get_checkpoint_path, parse_env_cfg
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_onnx,
)


def main():
    """Play with RSL-RL agent."""
    # parse configuration
    env_cfg = parse_env_cfg(args_cli.task, use_gpu=not args_cli.cpu, num_envs=args_cli.num_envs)
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    # resume_path = "/home/ubuntu2004/DATA/Projects/IsaacSim/Orbit-devel/Orbit/logs/rsl_rl/relml_flat/2023-11-27_19-31-45/model_1999.pt"
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    ppo_runner.load(resume_path)
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # export policy to onnx
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_onnx(ppo_runner.alg.actor_critic, export_model_dir, filename="policy.onnx")

    # reset environment
    obs, _ = env.get_observations()

    logger = Logger(env.env.step_dt)
    robot_index = 0  # which robot is used for logging
    stop_state_log = 100  # number of steps before plotting states

    for i in range(10 * int(env.max_episode_length)):
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, _, _ = env.step(actions)

            if i < stop_state_log:
                logger.log_states(
                    {
                        'dof_pos_target_0': env.env.action_manager.action[robot_index, 0].item() * 0.45,
                        'dof_pos_0': env.env.scene.articulations['robot'].data.joint_pos[robot_index, env.env.scene.articulations['robot'].ordered_actuator_index[0]].item(),
                        'dof_pos_target_1': env.env.action_manager.action[robot_index, 1].item() * 0.09,
                        'dof_pos_1': env.env.scene.articulations['robot'].data.joint_pos[robot_index, env.env.scene.articulations['robot'].ordered_actuator_index[1]].item(),
                        'dof_pos_target_2': env.env.action_manager.action[robot_index, 2].item() * 0.45,
                        'dof_pos_2': env.env.scene.articulations['robot'].data.joint_pos[robot_index, env.env.scene.articulations['robot'].ordered_actuator_index[2]].item(),
                        'dof_vel_0': env.env.scene.articulations['robot'].data.joint_vel[robot_index, env.env.scene.articulations['robot'].ordered_actuator_index[0]].item(),
                        'dof_vel_1': env.env.scene.articulations['robot'].data.joint_vel[robot_index, env.env.scene.articulations['robot'].ordered_actuator_index[1]].item(),
                        'dof_vel_2': env.env.scene.articulations['robot'].data.joint_vel[robot_index, env.env.scene.articulations['robot'].ordered_actuator_index[2]].item(),
                    }
                )
            elif i == stop_state_log:
                logger.plot_states()
    
    # close the simulator
    env.close()


if __name__ == "__main__":
    try:
        # run the main execution
        main()
    except Exception as err:
        carb.log_error(err)
        carb.log_error(traceback.format_exc())
        raise
    finally:
        # close sim app
        simulation_app.close()

# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.orbit.utils import configclass

from omni.isaac.orbit_tasks.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##
from omni.isaac.orbit.assets.config.relml import RELML_CFG  # isort: skip


@configclass
class ReLMLFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to Relml
        self.scene.robot = RELML_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # reduce action scale
        # {'.*dof1': 0.5, '.*dof2': 0.1, '.*dof3': 0.5}
        self.actions.joint_pos.scale = {'.*dof1': 0.45, '.*dof2': 0.09, '.*dof3': 0.45}  
        # override rewards
        self.rewards.lin_vel_z_l2.weight = -2.0   # -2.0
        self.rewards.ang_vel_xy_l2.weight = 0.0  # -0.05
        self.rewards.dof_torques_l2.weight = 0.0  # -1.0e-5
        self.rewards.dof_acc_l2.weight = 0.0  # -2.5e-7
        self.rewards.action_rate_l2.weight = -0.01  # -0.01
        self.rewards.feet_air_time.weight = 0.0  # 0.5
        self.rewards.undesired_contacts.weight = 0.0   # -1.0
        self.rewards.flat_orientation_l2.weight = -2.0    # 0.0
        self.rewards.dof_pos_limits.weight = 0.0  # 0.0
        self.rewards.stand_still.weight = -0.0  # -0.5 先不考虑
 
        # change body and joint names
        # TODO: Change to .*foot once we make a new USD for the robot
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*MidLeg4"
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [".*LeftLeg4", ".*RightLeg4"]

        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None
        # disable pushing for now
        self.physics_material = None
        self.randomization.add_base_mass = None
        self.base_external_force_torque = None
        self.randomization.push_robot = None

class ReLMLFlatEnvCfg_PLAY(ReLMLFlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.randomization.base_external_force_torque = None
        self.randomization.push_robot = None

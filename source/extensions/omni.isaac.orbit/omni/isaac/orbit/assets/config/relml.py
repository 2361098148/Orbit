# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Unitree robots.

The following configurations are available:

* :obj:`UNITREE_A1_CFG`: Unitree A1 robot with simple PD controller for the legs

Reference: https://github.com/unitreerobotics/unitree_ros
"""

from __future__ import annotations

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.actuators import DCMotorCfg

from ..articulation import ArticulationCfg

##
# Configuration
##

_RELML_INSTANCEABLE_USD = "/home/ubuntu2004/DATA/Projects/IsaacSim/ModelWs/src/Relml/usd/Relmls2Omn/Relmls3Omn.usd"


RELML_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_RELML_INSTANCEABLE_USD,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.7),
        joint_pos={
            ".*dof1": 0.5,
            ".*dof2": 0.1,
            ".*dof3": 0.5,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "side_legs": DCMotorCfg(
            joint_names_expr=[".*dof1", ".*dof3"],
            effort_limit=375,
            saturation_effort=375,
            velocity_limit=3.98,
            stiffness=200.0,
            damping=5,
        ),
        "main_legs": DCMotorCfg(
            joint_names_expr=[".*dof2"],
            effort_limit=10000,
            saturation_effort=10000,
            velocity_limit=1.0,
            stiffness=10000.0,
            damping=200,
        ),
    },
)
"""Configuration of Unitree A1 using DC motor.

Note: Specifications taken from: https://www.trossenrobotics.com/a1-quadruped#specifications
"""

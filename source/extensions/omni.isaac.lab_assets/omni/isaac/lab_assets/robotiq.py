# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Robotiq robots.

The following configurations are available:

* :obj:`ROBOTIQ_2F_140_CFG`: 2F-140 with implicit actuator model.

Reference:

* https://robotiq.com/products/2f85-140-adaptive-robot-gripper 

"""


import math

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab_assets import ISAACLAB_ASSETS_DATA_DIR

##
# Configuration
##

ROBOTIQ_2F_140_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/Robotiq/2F-140/2f140_instanceable.usd",
        # activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            # retain_accelerations=False,
            # enable_gyroscopic_forces=False,
            # angular_damping=0.01,
            # max_linear_velocity=1000.0,
            # max_angular_velocity=64 / math.pi * 180.0,
            max_depenetration_velocity=1000.0,
            # max_contact_impulse=1e32,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        rot=(0.257551, 0.283045, 0.683330, -0.621782),
        joint_pos={".*": 0.0},
    ),
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[
                "finger_joint", 
                "(left|right)_inner_knuckle_joint",
                "right_outer_knuckle_joint", 
                "(left|right)_inner_finger_joint", 
                # "(left|right)_inner_finger_pad_joint"
                ],
            # effort_limit={
            #     "finger_joint": 1.0, 
            #     "(left|right)_(inner|outer)_knuckle_joint": 1.0, 
            #     "(left|right)_(inner|outer)_finger_joint": 1.0, 
            #     "(left|right)_inner_finger_pad_joint": 1.0,
            # },
            # velocity_limit=100.0,
            stiffness=800.0,    # not set in the usd file
            damping=40.0,   # not set in the usd file
            # friction=0.01,
        ),
    },
    # soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Robotiq 2F-140 robot."""

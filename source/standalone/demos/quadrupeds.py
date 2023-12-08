# Copyright (c) 2022-2023, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different legged robots.

.. code-block:: bash

    # Usage
    ./orbit.sh -p source/standalone/demos/quadrupeds.py

"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.orbit.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import torch
import traceback

import carb
import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import Articulation
from omni.isaac.orbit.assets.config.anymal import ANYMAL_B_CFG, ANYMAL_C_CFG
from omni.isaac.orbit.assets.config.unitree import UNITREE_A1_CFG


def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)
    # create a grid of origins
    num_cols = np.floor(np.sqrt(num_origins))
    num_rows = np.ceil(num_origins / num_cols)
    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")
    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0
    # return the origins
    return env_origins.tolist()


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2", "Origin3"
    # Each group will have a mount and a robot on top of it
    origins = define_origins(num_origins=4, spacing=1.25)

    # Origin 1 with Anymal B
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # -- Robot
    anymal_b = Articulation(ANYMAL_B_CFG.replace(prim_path="/World/Origin1/Robot"))

    # Origin 2 with Anymal C
    prim_utils.create_prim("/World/Origin2", "Xform", translation=origins[1])
    # -- Robot
    anymal_c = Articulation(ANYMAL_C_CFG.replace(prim_path="/World/Origin2/Robot"))

    # Origin 3 with Unitree A1
    prim_utils.create_prim("/World/Origin3", "Xform", translation=origins[2])
    # -- Robot
    unitree_a = Articulation(UNITREE_A1_CFG.replace(prim_path="/World/Origin3/Robot"))

    # Origin 4 with Unitree Go1
    prim_utils.create_prim("/World/Origin4", "Xform", translation=origins[2])
    # -- Robot
    unitree_go1 = Articulation(UNITREE_A1_CFG.replace(prim_path="/World/Origin4/Robot"))

    # return the scene information
    scene_entities = {"anymal_b": anymal_b, "anymal_c": anymal_c, "unitree_a": unitree_a, "unitree_go1": unitree_go1}
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulator by applying actions to the robot at every time-step"""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count % 200 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset robots
            for index, robot in enumerate(entities.values()):
                # root state
                root_state = robot.data.default_root_state.clone()
                root_state[:, :3] += origins[index]
                robot.write_root_state_to_sim(root_state)
                # joint state
                joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(joint_pos, joint_vel)
                # reset the internal state
                robot.reset()
            print("[INFO]: Resetting robots state...")
        # apply default actions to the quadrupedal robots
        for robot in entities.values():
            # generate random joint positions
            joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.01
            # apply action to the robot
            robot.set_joint_position_target(joint_pos_target)
            # write data to sim
            robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""

    # Initialize the simulation context
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01, substeps=1))
    # Set main camera
    sim.set_camera_view(eye=[2.5, 2.5, 2.5], target=[0.0, 0.0, 0.0])
    # design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


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

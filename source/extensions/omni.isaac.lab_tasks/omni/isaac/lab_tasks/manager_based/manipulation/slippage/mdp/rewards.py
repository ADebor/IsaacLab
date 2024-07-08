from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import RigidObject, Articulation
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor, FrameTransformer
from omni.isaac.lab.envs import ManagerBasedRLEnv

if TYPE_CHECKING:
    from omni.isaac.orbit.envs import RLTaskEnv

# discrete rewards

def object_has_slipped(
        env: ManagerBasedRLEnv, 
        threshold: float, 
        object_cfg: SceneEntityCfg = SceneEntityCfg("object")
    ) -> torch.Tensor:
    """Give a discrete, negative reward to the agent for letting the manipulated object slip and fall down."""
    object: RigidObject = env.scene[object_cfg.name]
    return torch.where(object.data.root_pos_w[:, 2] < threshold, 1.0, 0.0)

# continuous rewards

def grip_span(
        env: ManagerBasedRLEnv, 
        robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")
    ) -> torch.Tensor:
    """Penalize the agent for gripping too narrow."""
    robot: RigidObject = env.scene[robot_cfg.name]
    return robot.data.inter_finger_distance # type: ignore #TODO

def object_ee_distance(
    env: ManagerBasedRLEnv,
    std: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),    # only the end-effector is used here
) -> torch.Tensor:
    
    # extract the used quantities 
    object: RigidObject = env.scene[object_cfg.name]
    robot: RigidObject = env.scene[robot_cfg.name]
    
    # object position: (num_envs, 3)
    cube_pos_w = object.data.root_pos_w

    # End-effector position: (num_envs, 3)
    ee_w = robot.data.root_pos_w
    
    # Distance of the end-effector to the object: (num_envs,)
    object_ee_distance = torch.norm(cube_pos_w - ee_w, dim=1)

    return 1 - torch.tanh(object_ee_distance / std)
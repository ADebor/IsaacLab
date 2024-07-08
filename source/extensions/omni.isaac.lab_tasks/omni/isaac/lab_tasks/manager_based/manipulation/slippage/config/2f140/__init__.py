import gymnasium as gym
# from . import agents, robotiq_env_cfg
from . import agents
from .robotiq_env_cfg import Robotiq2FSlipEnvCfg, Robotiq2FSlipEnvCfg_PLAY

# ##
# # Register Gym environments.
# ##
gym.register(
    id="Isaac-Slip-2F140-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": Robotiq2FSlipEnvCfg,
        "sb3_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Slip-2F140-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": Robotiq2FSlipEnvCfg_PLAY,
        "sb3_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
    },
)
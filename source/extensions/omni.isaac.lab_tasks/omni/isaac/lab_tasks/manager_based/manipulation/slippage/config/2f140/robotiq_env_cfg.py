from omni.isaac.lab_tasks.manager_based.manipulation.slippage import slippage_env_cfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.assets import RigidObjectCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg, MassPropertiesCfg

##
# Pre-defined configs
##

from omni.isaac.lab_assets import ROBOTIQ_2F_140_CFG

##
# Environment configuration
##

@configclass
class Robotiq2FSlipEnvCfg(slippage_env_cfg.SlipEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to Robotiq 2F140
        self.scene.robot = ROBOTIQ_2F_140_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot") # type: ignore

        # only control two joints
        self.actions.finger_joint_pos.joint_names = ["(left|right)_inner_knuckle_joint"]

        # define object to be manipulated
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0, 0.055), rot=(1, 0, 0, 0)),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                mass_props=MassPropertiesCfg(density=400.0),
                rigid_props=RigidBodyPropertiesCfg( 
                    kinematic_enabled=False,    
                    disable_gravity=False,
                    enable_gyroscopic_forces=True,
                    solver_position_iteration_count=8,
                    solver_velocity_iteration_count=1,
                    sleep_threshold=0.005,
                    stabilization_threshold=0.0025,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                ),
            ),
        )


@configclass
class Robotiq2FSlipEnvCfg_PLAY(Robotiq2FSlipEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50

        # disable randomization for play
        self.observations.policy.enable_corruption = False

        # remove termination due to timeouts
        self.terminations.time_out = None # type: ignore
        
##
# Environment configuration with no velocity observations.
##

# note adebor: would be better to have a factory method

@configclass
class Robotiq2FSlipNoVelObsEnvCfg(Robotiq2FSlipEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # swicth observation group to no velocity group
        self.observations.policy = slippage_env_cfg.ObservationsCfg.PolicyNoVelCfg()
        self.observations.value = slippage_env_cfg.ObservationsCfg.ValueNoVelCfg()


@configclass
class Robotiq2FSlipNoVelObsEnvCfg_PLAY(Robotiq2FSlipNoVelObsEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50

        # disable randomization for play
        self.observations.policy.enable_corruption = False

        # remove termination due to timeouts
        self.terminations.time_out = None # type: ignore

##
# Environment configuration with no object info observations.
##

# note adebor: would be better to have a factory method

@configclass
class Robotiq2FSlipNoExtObsEnvCfg(Robotiq2FSlipEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # swicth observation group to no velocity group
        self.observations.policy = slippage_env_cfg.ObservationsCfg.PolicyNoExtCfg()
        self.observations.value = slippage_env_cfg.ObservationsCfg.ValueNoExtCfg()


@configclass
class Robotiq2FSlipNoExtObsEnvCfg_PLAY(Robotiq2FSlipNoExtObsEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50

        # disable randomization for play
        self.observations.policy.enable_corruption = False

        # remove termination due to timeouts
        self.terminations.time_out = None # type: ignore
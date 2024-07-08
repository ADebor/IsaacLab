from dataclasses import MISSING

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg

from omni.isaac.lab.managers import ActionTermCfg as ActionTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm

from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg

from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.utils.noise import AdditiveGaussianNoiseCfg as Gnoise


# from omni.isaac.lab.utils.noise import AdditiveUniformNoiseCfg as Unoise

# import omni.isaac.lab_tasks.manipulation.reach.mdp as 
import omni.isaac.lab_tasks.manager_based.manipulation.slippage.mdp as mdp

##
# Scene definition
##


@configclass
class SlipSceneCfg(InteractiveSceneCfg):

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING # type: ignore 
    # manipulated object: will be populated by agent env cfg
    object: RigidObjectCfg = MISSING # type: ignore
    
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Commands terms for the MDP."""
    # no commands for the slip detection task
    null = mdp.NullCommandCfg()

@configclass
class ActionsCfg:
    finger_joint_pos = mdp.JointPositionToLimitsActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        rescale_to_limits=True,
    ) 
    # finger_joint_pos = mdp.EMAJointPositionToLimitsActionCfg(
    #     asset_name="robot",
    #     joint_names=[".*"],
    #     alpha=0.95,
    #     rescale_to_limits=True,
    # ) 
    
@configclass
class ObservationsCfg:

    # Full observations
    @configclass
    class PolicyCfg(ObsGroup):
        """Observation for policy group."""
        
        # -- object terms
        object_pos_b = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        object_pos = ObsTerm(
            func=mdp.root_pos_w, noise=Gnoise(std=0.002), params={"asset_cfg": SceneEntityCfg("object")}
        )
        object_quat = ObsTerm(
            func=mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object"), "make_quat_unique": False}
        )
        object_lin_vel = ObsTerm(
            func=mdp.root_lin_vel_w, noise=Gnoise(std=0.002), params={"asset_cfg": SceneEntityCfg("object")}
        )
        object_ang_vel = ObsTerm(
            func=mdp.root_ang_vel_w,
            scale=0.2,
            noise=Gnoise(std=0.002),
            params={"asset_cfg": SceneEntityCfg("object")},
        )
        
        # -- robot terms
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, 
            # noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel, 
            # noise=Unoise(n_min=-0.01, n_max=0.01)
        )

        # contacts = ObsTerm(func=TODO)

        # -- action terms
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class ValueCfg(ObsGroup):
        """Observation for value group."""
        # -- object terms
        object_pos_b = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        object_pos = ObsTerm(
            func=mdp.root_pos_w, noise=Gnoise(std=0.002), params={"asset_cfg": SceneEntityCfg("object")}
        )
        object_quat = ObsTerm(
            func=mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object"), "make_quat_unique": False}
        )
        object_lin_vel = ObsTerm(
            func=mdp.root_lin_vel_w, noise=Gnoise(std=0.002), params={"asset_cfg": SceneEntityCfg("object")}
        )
        object_ang_vel = ObsTerm(
            func=mdp.root_ang_vel_w,
            scale=0.2,
            noise=Gnoise(std=0.002),
            params={"asset_cfg": SceneEntityCfg("object")},
        )
        
        # -- robot terms
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, 
            # noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel, 
            # noise=Unoise(n_min=-0.01, n_max=0.01)
        )

        # contacts = ObsTerm(func=TODO)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # No velocity observations
    @configclass
    class PolicyNoVelCfg(PolicyCfg):
        def __post_init__(self):
            super().__post_init__()
            # set unused terms to None
            self.joint_vel = None
            self.object_lin_vel = None
            self.object_ang_vel = None

    @configclass
    class ValueNoVelCfg(ValueCfg):
        def __post_init__(self):
            super().__post_init__()
            # set unused terms to None
            self.joint_vel = None
            self.object_lin_vel = None
            self.object_ang_vel = None

    # No object info observations
    @configclass
    class PolicyNoExtCfg(PolicyCfg):
        def __post_init__(self):
            super().__post_init__()
            # set unused terms to None
            self.object_pos = None
            self.object_pos_b = None
            self.object_lin_vel = None
            self.object_ang_vel = None
            self.object_quat = None

    @configclass
    class ValueNoExtCfg(ValueCfg):
        def __post_init__(self):
            super().__post_init__()
            # set unused terms to None
            self.object_pos = None
            self.object_pos_b = None
            self.object_lin_vel = None
            self.object_ang_vel = None
            self.object_quat = None
    
    # observation groups
    policy: PolicyCfg = PolicyCfg()
    value: ValueCfg = ValueCfg()

@configclass
class EventCfg:
    # reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")   # TODO: need to remimplement this in local mdp
    # reset_object_pos = EventTerm(
    #     func=mdp.reset_root_state_uniform,
    #     mode="reset",
    #     params={
    #         "pos_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
    #         "velocity_range": {},
    #         "asset_cfg": SceneEntityCfg("object", body_names="Object"),
    #     },
    # )

    """Configuration for randomization."""

    # startup
    # -- robot
    robot_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.7, 1.3),
            "dynamic_friction_range": (0.7, 1.3),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 250, # number of materials
        },
    )
    robot_scale_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "mass_distribution_params": (0.95, 1.05),
            "operation": "scale",
        },
    )
    robot_joint_stiffness_and_damping = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.3, 3.0),  # default: 3.0
            "damping_distribution_params": (0.75, 1.5),  # default: 0.1
            "operation": "scale",
            "distribution": "log_uniform",
        },
    )

    # -- object
    object_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("object", body_names=".*"),
            "static_friction_range": (0.7, 1.3),
            "dynamic_friction_range": (0.7, 1.3),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 250,
        },
    )
    object_scale_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("object"),
            "mass_distribution_params": (0.4, 1.6),
            "operation": "scale",
        },
    )

    # reset
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_object = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": [-0.01, 0.01], "y": [-0.01, 0.01], "z": [-0.01, 0.01]},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object", body_names=".*"),
        },
    )
    # reset_robot_joints = EventTerm(
    #     func=mdp.reset_joints_within_limits_range,
    #     mode="reset",
    #     params={
    #         "position_range": {".*": [0.2, 0.2]},
    #         "velocity_range": {".*": [0.0, 0.0]},
    #         "use_default_offset": True,
    #         "operation": "scale",
    #     },
    # )

@configclass
class RewardsCfg:

    # -- task
    # fall penalty
    fall = RewTerm(
        func=mdp.object_has_slipped,
        # func=mdp.base_height_l2
        weight=-0.01,
        params={
            # "asset_cfg": SceneEntityCfg("robot"),
            "threshold": 0.5, 
        },
    )

    # slipping penalty
    slip = RewTerm(
        func=mdp.object_ee_distance,
        weight=-0.01,
        params={
            "std": 0.1,
        },
    )

    # -- penalties
    # span = RewTerm(
    #     func=mdp.grip_span,
    #     weight=-0.01,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #     },
    # )
    action_l2 = RewTerm(func=mdp.action_l2, weight=-0.0001)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-1e-3)
    joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        # params={"asset_cfg": SceneEntityCfg("robot")},
    )

    # -- anti-sparsity
    is_alive = RewTerm(func=mdp.is_alive, weight=1.0)

@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.5, "asset_cfg": SceneEntityCfg("object")}
        )
    # object_dropping = DoneTerm(
    #     func=mdp.object_away_from_robot,
    #     params={
    #         "threshold": 0.5,
    #     }
    # )

##
# Environment configuration
##


@configclass
class SlipEnvCfg(ManagerBasedRLEnvCfg):

    # Scene settings
    scene: SlipSceneCfg = SlipSceneCfg(num_envs=4, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        """Post initialization"""
        # general settings
        self.decimation = 2
        self.episode_length_s = 12.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 60.0

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

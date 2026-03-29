"""Environment configuration for Adelino balance task.

The robot stands on a kinematic platform that tilts randomly.
It must learn to adjust its joints to keep its CoM over the support polygon.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab.envs.mdp as mdp
import adelino_lab.tasks.locomotion.mdp as adelino_mdp

ADELINO_USD_PATH = "C:/repo/nuttymoves/embodiments/adelino/v1/usd/adelino_fixed.usd"

# Joint names for the 5 actuated DOFs (excludes wobble joints)
ADELINO_ACTUATED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]

# Platform height — enough to clear ground plane at max tilt
PLATFORM_HEIGHT = 0.3

##
# Robot articulation config
##



ADELINO_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=ADELINO_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, PLATFORM_HEIGHT + 0.005),
        joint_pos={
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
        },
    ),
    actuators={
        # Per-joint actuators with real servo torque limits (kg·cm → N·m: multiply by 0.098)
        "joint_1_act": ImplicitActuatorCfg(
            joint_names_expr=["joint_1"],
            stiffness=500.0, damping=10.0,
            effort_limit_sim=2.45,   # HK15338: 25.0 kg·cm
        ),
        "joint_2_act": ImplicitActuatorCfg(
            joint_names_expr=["joint_2"],
            stiffness=500.0, damping=10.0,
            effort_limit_sim=1.96,   # HK15298B: 20.0 kg·cm
        ),
        "joint_3_act": ImplicitActuatorCfg(
            joint_names_expr=["joint_3"],
            stiffness=500.0, damping=10.0,
            effort_limit_sim=1.25,   # HK15328A: 12.8 kg·cm
        ),
        "joint_4_act": ImplicitActuatorCfg(
            joint_names_expr=["joint_4"],
            stiffness=500.0, damping=10.0,
            effort_limit_sim=0.42,   # HK15138: 4.3 kg·cm
        ),
        "joint_5_act": ImplicitActuatorCfg(
            joint_names_expr=["joint_5"],
            stiffness=500.0, damping=10.0,
            effort_limit_sim=0.14,   # HK15178: 1.4 kg·cm
        ),
        # Passive wobble joints
        "wobble_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*wobble.*"],
            stiffness=0.0,
            damping=50.0,
            effort_limit_sim=0.0,
        ),
    },
)

##
# Platform config — kinematic rigid body that tilts under the robot
##

PLATFORM_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Platform",
    spawn=sim_utils.CuboidCfg(
        size=(0.5, 0.5, 0.02),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=5.0,
            dynamic_friction=5.0,
            friction_combine_mode="max",
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.6, 0.3)),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, PLATFORM_HEIGHT)),
)


##
# Scene
##


@configclass
class AdelinoBalanceSceneCfg(InteractiveSceneCfg):
    """Scene with robot on a tilting platform."""

    # Ground plane (robot is above it on the platform)
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # Robot
    robot: ArticulationCfg = ADELINO_CFG

    # Tilting platform
    platform: RigidObjectCfg = PLATFORM_CFG

    # Lighting
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


##
# MDP: Actions
##


@configclass
class ActionsCfg:
    """Joint position targets for the 5 actuated joints."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=ADELINO_ACTUATED_JOINTS,
        scale=0.25,
        use_default_offset=True,
    )


##
# MDP: Observations
##


@configclass
class ObservationsCfg:
    """Observation specification for the balance policy."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for the actor/critic."""

        # Base orientation relative to gravity
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        # Base linear velocity (detects sliding/falling)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        # Base angular velocity (detects tipping)
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        )
        # CoM vertical projection onto support plane — XY offset from base center
        # This correctly detects CoM drift even on tilted platforms
        com_projection = ObsTerm(
            func=adelino_mdp.com_projection_on_support,
            noise=Unoise(n_min=-0.005, n_max=0.005),
        )
        # Joint positions — actuated joints only
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=ADELINO_ACTUATED_JOINTS)},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        # Joint velocities — actuated joints only
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=ADELINO_ACTUATED_JOINTS)},
            noise=Unoise(n_min=-1.5, n_max=1.5),
        )
        # Previous actions
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


##
# MDP: Rewards
##


@configclass
class RewardsCfg:
    """Reward terms for balance task."""

    # --- Task rewards ---
    # Positive reward for staying alive (not terminated)
    is_alive = RewTerm(func=mdp.is_alive, weight=2.0)
    # Stay upright: penalize base tilt from horizontal
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
    # Keep CoM projection centered on support polygon (primary balance signal)
    com_projection = RewTerm(func=adelino_mdp.com_projection_penalty, weight=-50.0)

    # --- Regularization penalties (kept light so lower joints aren't discouraged) ---
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2, weight=-1.0e-8,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ADELINO_ACTUATED_JOINTS)},
    )
    # Per-joint torque penalty: lower joints (balance) are cheap, upper joints (expression) are expensive
    # joint_1 (yaw) and joint_2 (pitch) are the primary balance actuators
    dof_torques_weighted = RewTerm(
        func=adelino_mdp.weighted_joint_torques,
        weight=-1.0e-6,
        params={
            "joint_weights": {
                "joint_1": 0.1,   # Base yaw — free to use for balance
                "joint_2": 0.2,   # Lower pitch — free to use for balance
                "joint_3": 0.5,   # Mid pitch — moderate
                "joint_4": 2.0,   # Upper roll — prefer quiet
                "joint_5": 5.0,   # Head — strongly prefer quiet
            },
        },
    )
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.001)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.01)
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.5)


##
# MDP: Terminations
##


@configclass
class TerminationsCfg:
    """Termination conditions."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # Terminate if robot tilts beyond ~69° (it has fallen)
    fallen = DoneTerm(func=adelino_mdp.base_too_tilted, params={"max_tilt_rad": 1.2})


##
# MDP: Events
##


@configclass
class EventCfg:
    """Perturbation events — tilting platform + pushes."""

    # --- On reset ---
    # Reset robot pose
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.1, 0.1),
            "velocity_range": (0.0, 0.0),
            "asset_cfg": SceneEntityCfg("robot", joint_names=ADELINO_ACTUATED_JOINTS),
        },
    )
    # Reset platform to level so robot spawns cleanly on it
    reset_platform = EventTerm(
        func=adelino_mdp.reset_platform,
        mode="reset",
    )

    # --- Interval ---
    # Pick a new random tilt target every 2-5s (the actual motion is smooth)
    new_tilt_target = EventTerm(
        func=adelino_mdp.tilt_platform,
        mode="interval",
        interval_range_s=(2.0, 5.0),
        params={"max_tilt_deg": 30.0},
    )
    # Smoothly interpolate platform toward target every step
    # interval_range_s near dt ensures this runs every physics step
    smooth_tilt = EventTerm(
        func=adelino_mdp.tilt_platform_step,
        mode="interval",
        interval_range_s=(0.02, 0.02),
        params={"lerp_rate": 0.02, "max_lerp_multiplier": 5.0},
    )
    # Push the robot occasionally
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(5.0, 10.0),
        params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}},
    )


##
# Full environment config
##


@configclass
class AdelinoBalanceFlatEnvCfg(ManagerBasedRLEnvCfg):
    """Adelino balance on tilting platform."""

    scene: AdelinoBalanceSceneCfg = AdelinoBalanceSceneCfg(num_envs=256, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation


@configclass
class AdelinoBalanceFlatEnvCfg_PLAY(AdelinoBalanceFlatEnvCfg):
    """Play/evaluation variant."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None

"""Environment configuration for Adelino balance task on flat ground.

The goal is for the 5-DOF serial-chain robot to learn to balance on its base,
maintaining an upright posture while responding to perturbations.
"""

import math

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab.envs.mdp as mdp

##
# Path to the Adelino USD asset (relative to nuttymoves repo root).
# TODO: Update this path once the asset is in a stable location accessible
#       to the training environment (e.g. copied into the extension or
#       referenced via an absolute path / nucleus server).
##
ADELINO_USD_PATH = "C:/repo/nuttymoves/embodiments/adelino/v1/usd/adelino.usd"

# Joint names for the 5 actuated DOFs (excludes wobble joints)
ADELINO_ACTUATED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]

##
# Robot articulation config
##

ADELINO_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=ADELINO_USD_PATH,
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
        pos=(0.0, 0.0, 0.15),  # Start slightly above ground
        joint_pos={
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
        },
    ),
    actuators={
        "servo_actuators": ImplicitActuatorCfg(
            joint_names_expr=ADELINO_ACTUATED_JOINTS,
            stiffness=500.0,
            damping=10.0,
            effort_limit=50.0,
        ),
    },
)


##
# Scene
##


@configclass
class AdelinoBalanceSceneCfg(InteractiveSceneCfg):
    """Scene configuration for Adelino balance task."""

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # Robot
    robot: ArticulationCfg = ADELINO_CFG

    # Contact sensor on the base (for detecting falls)
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        history_length=3,
        track_air_time=False,
    )

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
    """Action specification: joint position targets for the 5 actuated joints."""

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

        # Base orientation (gravity projection tells the policy which way is up)
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        # Base angular velocity
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        )
        # Joint positions (relative to default)
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        # Joint velocities
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
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
    # Stay upright: penalize deviation from flat orientation
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)

    # --- Regularization penalties ---
    # Penalize joint accelerations (smooth motion)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    # Penalize joint torques (energy efficiency)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)
    # Penalize rapid action changes (smooth control)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    # Penalize angular velocity (prefer stillness)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    # Penalize vertical velocity
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)


##
# MDP: Terminations
##


@configclass
class TerminationsCfg:
    """Termination conditions."""

    # Episode timeout
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # Terminate if the base contacts the ground (robot fell over)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"),
            "threshold": 1.0,
        },
    )


##
# MDP: Events (domain randomization)
##


@configclass
class EventCfg:
    """Domain randomization events."""

    # --- On reset ---
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.8, 1.2),
            "velocity_range": (0.0, 0.0),
        },
    )

    # --- Interval: push the robot occasionally ---
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}},
    )


##
# Full environment config
##


@configclass
class AdelinoBalanceFlatEnvCfg(ManagerBasedRLEnvCfg):
    """Adelino balance on flat ground."""

    scene: AdelinoBalanceSceneCfg = AdelinoBalanceSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # Simulation parameters
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005  # 200 Hz physics
        self.sim.render_interval = self.decimation

        # Contact sensor update period
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt


@configclass
class AdelinoBalanceFlatEnvCfg_PLAY(AdelinoBalanceFlatEnvCfg):
    """Play/evaluation variant: fewer envs, no randomization."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # Disable observation noise
        self.observations.policy.enable_corruption = False
        # Disable perturbations
        self.events.push_robot = None

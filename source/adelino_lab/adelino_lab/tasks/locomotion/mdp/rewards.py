"""Custom reward functions for Adelino balance task."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg

from .observations import com_projection_on_support


def com_projection_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize the CoM projection being far from the base center.

    Uses the vertical projection of the CoM onto the support plane.
    Returns the squared XY distance from the base center.

    Returns:
        Squared distance of CoM projection from base center (num_envs,).
    """
    com_xy = com_projection_on_support(env, asset_cfg)
    return com_xy[:, 0] ** 2 + com_xy[:, 1] ** 2


def com_counterbalance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward shifting CoM uphill on a tilted platform.

    Computes the inner product between the CoM offset (in base frame XY)
    and the platform tilt direction (derived from projected gravity).
    When the platform tilts, gravity in the body frame gains XY components
    pointing downhill. The robot should shift its CoM in the opposite
    direction (uphill), so a negative inner product means good counterbalance.

    Returns the negative dot product so that positive values = good counterbalance.

    Returns:
        Counterbalance score (num_envs,). Positive = CoM shifted uphill.
    """
    robot = env.scene[asset_cfg.name]
    com_xy = com_projection_on_support(env, asset_cfg)

    # Projected gravity in body frame: XY components point downhill when tilted
    grav_b = robot.data.projected_gravity_b  # (num_envs, 3)
    downhill_xy = grav_b[:, :2]  # (num_envs, 2)

    # Inner product: negative when CoM is opposite to downhill = uphill
    dot = (com_xy * downhill_xy).sum(dim=-1)  # (num_envs,)

    # Return negative so that uphill shift → positive reward
    return -dot


def head_height_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward keeping the head (last body) high above the ground.

    Uses the world-frame Z position of the last body in the kinematic chain.
    Returns the height directly so higher = more reward.

    Returns:
        Head height in world frame (num_envs,).
    """
    robot = env.scene[asset_cfg.name]
    # Last body in the chain is the head (child of joint_5)
    head_pos_w = robot.data.body_pos_w[:, -1, :]  # (num_envs, 3)
    return head_pos_w[:, 2]  # Z height


def weighted_joint_torques(
    env: ManagerBasedRLEnv,
    joint_weights: dict[str, float] | None = None,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Per-joint weighted torque penalty for actuated joints only.

    Allows penalizing upper joint torques more heavily than lower ones,
    so the policy prefers using lower joints for balance and keeps
    upper joints quiet for expression.

    Only considers joints listed in joint_weights. Wobble joints and
    other unlisted joints are ignored.

    Args:
        joint_weights: Dict mapping joint name to penalty multiplier.
            Higher values = more penalized = less likely to be used.
            Only joints listed here are included in the penalty.

    Returns:
        Weighted sum of squared torques (num_envs,).
    """
    robot = env.scene[asset_cfg.name]
    torques = robot.data.applied_torque  # (num_envs, num_joints)

    if joint_weights is None:
        return torch.sum(torques**2, dim=-1)

    # Only sum torques for joints explicitly listed in joint_weights
    result = torch.zeros(torques.shape[0], device=torques.device)
    for i, name in enumerate(robot.joint_names):
        if name in joint_weights:
            result += torques[:, i] ** 2 * joint_weights[name]

    return result

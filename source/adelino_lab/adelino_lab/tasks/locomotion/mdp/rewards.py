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

"""Custom reward functions for Adelino balance task."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply_inverse


def com_projection_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize the CoM projection being far from the base center.

    Computes the whole-body center of mass, projects it vertically onto the
    base's local XY plane, and returns the squared distance from the base center.
    A robot that extends its arm sideways will have a large penalty because
    the CoM shifts away from the support polygon center.

    Returns:
        Squared distance of CoM projection from base center in base frame (num_envs,).
    """
    robot = env.scene[asset_cfg.name]

    # Per-body CoM positions in world frame: (num_envs, num_bodies, 3)
    body_com_w = robot.data.body_com_pos_w
    # Per-body masses: (num_envs, num_bodies)
    masses = robot.data.default_mass

    # Ensure masses are on the same device as positions
    masses = masses.to(body_com_w.device)

    # Whole-body CoM in world frame: (num_envs, 3)
    total_mass = masses.sum(dim=-1, keepdim=True)  # (num_envs, 1)
    whole_com_w = (body_com_w * masses.unsqueeze(-1)).sum(dim=1) / total_mass  # (num_envs, 3)

    # Base position in world frame
    base_pos_w = robot.data.root_pos_w  # (num_envs, 3)

    # Vector from base to CoM in world frame
    com_offset_w = whole_com_w - base_pos_w  # (num_envs, 3)

    # Rotate into base frame
    com_offset_b = quat_apply_inverse(robot.data.root_quat_w, com_offset_w)  # (num_envs, 3)

    # XY distance in base frame (how far CoM projects from base center)
    com_xy_dist_sq = com_offset_b[:, 0] ** 2 + com_offset_b[:, 1] ** 2

    return com_xy_dist_sq

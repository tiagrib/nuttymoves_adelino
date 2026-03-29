"""Custom observation functions for Adelino balance task."""

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse


def com_projection_on_support(
    env: ManagerBasedEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Vertical projection of the whole-body CoM onto the support plane, in base frame.

    Projects the CoM straight down (world gravity direction) onto the plane
    defined by the base's position and orientation. Returns the XY offset
    of that projected point from the base center, in the base's local frame.

    On a tilted platform with a straight robot, this correctly shows the CoM
    projection shifting toward the downhill edge — even if the CoM is directly
    above the base in 3D space.

    Returns:
        CoM projection XY in base frame. Shape: (num_envs, 2).
    """
    robot = env.scene[asset_cfg.name]

    # --- Compute whole-body CoM in world frame ---
    body_com_w = robot.data.body_com_pos_w  # (num_envs, num_bodies, 3)
    masses = robot.data.default_mass.to(body_com_w.device)  # (num_envs, num_bodies)
    total_mass = masses.sum(dim=-1, keepdim=True)
    com_w = (body_com_w * masses.unsqueeze(-1)).sum(dim=1) / total_mass  # (num_envs, 3)

    # --- Get the base (support) plane ---
    base_pos_w = robot.data.root_pos_w  # (num_envs, 3)
    base_quat_w = robot.data.root_quat_w  # (num_envs, 4)

    # The base plane normal is the base's local Z-axis in world frame
    local_z = torch.tensor([[0.0, 0.0, 1.0]], device=base_pos_w.device).expand(base_pos_w.shape[0], -1)
    plane_normal_w = quat_apply(base_quat_w, local_z)  # (num_envs, 3)

    # --- Project CoM vertically onto the base plane ---
    # The projection ray is straight down: direction = (0, 0, -1)
    # We want the point where a vertical line through the CoM hits the base plane.
    #
    # Plane equation: dot(plane_normal, P - base_pos) = 0
    # Ray: P = com + t * (0, 0, -1)
    # Solving: t = dot(plane_normal, com - base_pos) / plane_normal_z
    #
    com_to_base = com_w - base_pos_w  # (num_envs, 3)
    numerator = (plane_normal_w * com_to_base).sum(dim=-1)  # dot product
    denominator = plane_normal_w[:, 2].clamp(min=0.1)  # plane_normal_z, avoid div by zero

    t = numerator / denominator  # (num_envs,)

    # Projected point in world frame
    projected_w = com_w.clone()
    projected_w[:, 2] -= t  # move straight down by t

    # --- Convert to base frame ---
    offset_w = projected_w - base_pos_w
    offset_b = quat_apply_inverse(base_quat_w, offset_w)

    # Return XY in base frame
    return offset_b[:, :2]

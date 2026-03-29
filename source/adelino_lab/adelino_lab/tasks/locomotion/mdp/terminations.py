"""Custom termination functions for Adelino balance task."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg


def base_too_tilted(
    env: ManagerBasedRLEnv,
    max_tilt_rad: float = 1.2,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Terminate when the robot's base is tilted beyond a threshold.

    Uses projected_gravity_b: when upright, z ≈ -1.0.
    When tilted by angle θ, z ≈ -cos(θ).
    At 1.2 rad (~69°), cos(1.2) ≈ 0.36, so z > -0.36 means fallen.

    Args:
        max_tilt_rad: Maximum allowed tilt angle in radians.
    """
    robot = env.scene[asset_cfg.name]
    # z-component of projected gravity in body frame
    grav_z = robot.data.projected_gravity_b[:, 2]
    # Robot is too tilted when grav_z > -cos(max_tilt)
    threshold = -torch.cos(torch.tensor(max_tilt_rad, device=grav_z.device))
    return grav_z > threshold

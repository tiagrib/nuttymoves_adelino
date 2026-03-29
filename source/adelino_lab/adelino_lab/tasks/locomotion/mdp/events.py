"""Custom event functions for Adelino balance task."""

import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_from_euler_xyz, quat_slerp, sample_uniform

# Persistent state for smooth interpolation
_platform_state = {}


def _get_platform_state(platform: RigidObject, device):
    """Get or initialize persistent tilt state for smooth interpolation."""
    key = id(platform)
    if key not in _platform_state:
        num_envs = platform.num_instances
        _platform_state[key] = {
            "current_roll": torch.zeros(num_envs, device=device),
            "current_pitch": torch.zeros(num_envs, device=device),
            "target_roll": torch.zeros(num_envs, device=device),
            "target_pitch": torch.zeros(num_envs, device=device),
        }
    return _platform_state[key]


def tilt_platform(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    max_tilt_deg: float = 30.0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("platform"),
):
    """Set a new random tilt target for the platform.

    This is called as an interval event to pick a new target orientation.
    The actual interpolation happens in tilt_platform_step() every physics step.
    """
    platform: RigidObject = env.scene[asset_cfg.name]
    device = platform.device
    state = _get_platform_state(platform, device)

    if env_ids is None:
        env_ids = torch.arange(platform.num_instances, device=device)

    max_rad = torch.deg2rad(torch.tensor(max_tilt_deg, device=device))

    # Set new random targets for selected envs
    num = len(env_ids)
    state["target_roll"][env_ids] = sample_uniform(-max_rad, max_rad, (num,), device=device)
    state["target_pitch"][env_ids] = sample_uniform(-max_rad, max_rad, (num,), device=device)


def tilt_platform_step(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    lerp_rate: float = 0.02,
    max_lerp_multiplier: float = 5.0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("platform"),
):
    """Smoothly interpolate platform orientation toward the target.

    Call this every step (as a high-frequency interval event) to smoothly
    move the platform toward its target tilt.

    The lerp speed increases as training progresses:
        0-66%:   1x lerp_rate
        66-90%:  ramp 1x → 3x
        90-100%: ramp 3x → max_lerp_multiplier (5x)
    """
    platform: RigidObject = env.scene[asset_cfg.name]
    device = platform.device
    state = _get_platform_state(platform, device)

    # Compute speed multiplier based on training progress
    # Use sim step counter as a proxy for training progress
    step = env._sim_step_counter // env.cfg.decimation
    # Estimate total steps: max_iterations * num_steps_per_env * num_envs
    # We don't know max_iterations, so use step-based milestones.
    # With 5000 iters, 24 steps/env, 256 envs: ~30M steps total
    # Scale-independent: use fraction of steps where behavior changes
    # We'll use a simple schedule based on absolute step counts
    # that works across different num_envs/max_iterations
    if not hasattr(tilt_platform_step, "_total_steps_estimate"):
        # Estimate once from the runner config if available, otherwise use a default
        tilt_platform_step._total_steps_estimate = 30_000_000  # reasonable default

    progress = min(step / max(tilt_platform_step._total_steps_estimate, 1), 1.0)

    if progress < 0.66:
        multiplier = 1.0
    elif progress < 0.90:
        # Ramp from 1x to 3x
        t = (progress - 0.66) / 0.24
        multiplier = 1.0 + t * 2.0
    else:
        # Ramp from 3x to max
        t = (progress - 0.90) / 0.10
        multiplier = 3.0 + t * (max_lerp_multiplier - 3.0)

    effective_rate = lerp_rate * multiplier

    # Lerp current toward target (all envs, every step)
    state["current_roll"] += (state["target_roll"] - state["current_roll"]) * effective_rate
    state["current_pitch"] += (state["target_pitch"] - state["current_pitch"]) * effective_rate

    # Build quaternion from current angles
    yaw = torch.zeros(platform.num_instances, device=device)
    quat = quat_from_euler_xyz(state["current_roll"], state["current_pitch"], yaw)

    # Keep current world position, update orientation
    current_pos = platform.data.root_pos_w.clone()
    platform.write_root_pose_to_sim(torch.cat([current_pos, quat], dim=-1))


def reset_platform(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("platform"),
):
    """Reset platform to level on episode reset."""
    platform: RigidObject = env.scene[asset_cfg.name]
    device = platform.device
    state = _get_platform_state(platform, device)

    if env_ids is None:
        env_ids = torch.arange(platform.num_instances, device=device)

    # Reset tilt state to zero for these envs
    state["current_roll"][env_ids] = 0.0
    state["current_pitch"][env_ids] = 0.0
    state["target_roll"][env_ids] = 0.0
    state["target_pitch"][env_ids] = 0.0

    # Write level orientation
    identity_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=device).expand(len(env_ids), -1)
    current_pos = platform.data.root_pos_w[env_ids].clone()
    platform.write_root_pose_to_sim(
        torch.cat([current_pos, identity_quat], dim=-1),
        env_ids=env_ids,
    )

"""Interactive play script with a physical tilting platform.

A kinematic platform tilts under the robot using arrow keys.
The robot must physically balance on the tilting surface.

Controls:
    Arrow Up/Down   - Tilt platform pitch
    Arrow Left/Right - Tilt platform roll
    R               - Reset platform to level
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Interactive play with tilting platform.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--task", type=str, default="Adelino-Balance-Flat-v0", help="Task name.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Agent config entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed.")
parser.add_argument("--tilt_speed", type=float, default=0.3, help="Tilt speed (deg/step).")
parser.add_argument("--max_tilt", type=float, default=45.0, help="Max tilt angle (degrees).")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import os

import importlib.metadata as metadata

import gymnasium as gym
import torch
from packaging import version
from rsl_rl.runners import OnPolicyRunner

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.math import quat_from_euler_xyz

from isaaclab_rl.rsl_rl import (
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)

installed_version = metadata.version("rsl-rl-lib")

import adelino_lab  # noqa: F401

from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

# Platform height — must clear ground plane even at max tilt
# At 45deg, a 0.5m platform edge drops ~0.18m, so 0.3m is safe
PLATFORM_HEIGHT = 0.3

# Keyboard state
key_states = {"up": False, "down": False, "left": False, "right": False, "reset": False, "reset_robot": False}


def setup_keyboard():
    """Set up keyboard listener for platform control."""
    import carb.input
    import omni.appwindow

    input_iface = carb.input.acquire_input_interface()
    app_window = omni.appwindow.get_default_app_window()
    keyboard = app_window.get_keyboard()

    key_map = {
        carb.input.KeyboardInput.UP: "up",
        carb.input.KeyboardInput.DOWN: "down",
        carb.input.KeyboardInput.LEFT: "left",
        carb.input.KeyboardInput.RIGHT: "right",
        carb.input.KeyboardInput.R: "reset",
        carb.input.KeyboardInput.SPACE: "reset_robot",
    }

    def on_key_event(event, *args, **kwargs):
        if event.input in key_map:
            name = key_map[event.input]
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                key_states[name] = True
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                key_states[name] = False
        return True

    input_iface.subscribe_to_keyboard_events(keyboard, on_key_event)
    print("\n=== Interactive Platform Controls ===")
    print("  Arrow Up/Down   : Tilt pitch")
    print("  Arrow Left/Right: Tilt roll")
    print("  R               : Reset platform to level")
    print("  SPACE           : Reset robot (upright on platform)")
    print("=====================================\n")


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Interactive play with tilting platform."""
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # --- Strip everything that could interfere ---
    # No randomization, no pushes, no gravity changes
    env_cfg.events.reset_base = None
    env_cfg.events.reset_robot_joints = None
    env_cfg.events.reset_gravity = None
    env_cfg.events.tilt_gravity = None
    env_cfg.events.push_robot = None
    # No observation noise
    env_cfg.observations.policy.enable_corruption = False
    # Very long episode so it effectively never resets
    env_cfg.episode_length_s = 9999.0

    # Spawn robot on top of the platform
    env_cfg.scene.robot.init_state.pos = (0.0, 0.0, PLATFORM_HEIGHT + 0.01)

    # Add tilting platform as a kinematic rigid body with high friction
    env_cfg.scene.platform = RigidObjectCfg(
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

    # Load checkpoint
    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    if not os.path.isdir(log_root_path):
        print(f"[ERROR] No training logs at: {log_root_path}")
        print("  Train first: python scripts/rsl_rl/train.py")
        return

    if args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    env_cfg.log_dir = os.path.dirname(resume_path)

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading checkpoint: {resume_path}")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    platform: RigidObject = env.unwrapped.scene["platform"]
    robot = env.unwrapped.scene["robot"]
    device = env.unwrapped.device
    num_envs = env.unwrapped.num_envs

    # Force-set effort limits at runtime (USD D6 joints may have Max Force = 0)
    num_joints = robot.num_joints
    effort_limits = torch.full((num_envs, num_joints), 50.0, device=device)
    robot.write_joint_effort_limit_to_sim(effort_limits)
    print(f"[INFO]: Set effort limits to 50.0 for {num_joints} joints")

    setup_keyboard()

    tilt_pitch = 0.0
    tilt_roll = 0.0
    tilt_speed = args_cli.tilt_speed
    max_tilt = args_cli.max_tilt
    obs = env.get_observations()
    step_count = 0

    print(f"[INFO]: Ready. Arrow keys to tilt (max +/-{max_tilt} deg), R to reset level.")

    while simulation_app.is_running():
        # Read keyboard
        if key_states["reset"]:
            tilt_pitch = 0.0
            tilt_roll = 0.0
        if key_states["up"]:
            tilt_pitch = min(tilt_pitch + tilt_speed, max_tilt)
        if key_states["down"]:
            tilt_pitch = max(tilt_pitch - tilt_speed, -max_tilt)
        if key_states["right"]:
            tilt_roll = min(tilt_roll + tilt_speed, max_tilt)
        if key_states["left"]:
            tilt_roll = max(tilt_roll - tilt_speed, -max_tilt)

        # Reset robot to upright on platform
        if key_states["reset_robot"]:
            key_states["reset_robot"] = False
            robot = env.unwrapped.scene["robot"]
            # Reset root state: position on platform, identity quaternion, zero velocity
            default_state = robot.data.default_root_state.clone()
            default_state[:, :3] = torch.tensor([[0.0, 0.0, PLATFORM_HEIGHT + 0.01]], device=device)
            robot.write_root_state_to_sim(default_state)
            # Reset joints to default positions and zero velocity
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = torch.zeros_like(joint_pos)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            print("  [Robot reset to upright]                    ")

        # Update platform orientation
        roll_rad = torch.tensor([math.radians(tilt_roll)], device=device).expand(num_envs)
        pitch_rad = torch.tensor([math.radians(tilt_pitch)], device=device).expand(num_envs)
        yaw_rad = torch.zeros(num_envs, device=device)
        quat = quat_from_euler_xyz(roll_rad, pitch_rad, yaw_rad)
        pos = platform.data.default_root_state[:, :3].clone()
        platform.write_root_pose_to_sim(torch.cat([pos, quat], dim=-1))

        # Step policy
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            if version.parse(installed_version) >= version.parse("4.0.0"):
                policy.reset(dones)

        # Debug visualization and console output
        robot = env.unwrapped.scene["robot"]
        grav_b = robot.data.projected_gravity_b[0].cpu()
        root_pos = robot.data.root_pos_w[0].cpu()
        root_quat = robot.data.root_quat_w[0].cpu()
        lin_vel = robot.data.root_lin_vel_w[0].cpu()

        # Draw debug visualization every frame
        from isaacsim.util.debug_draw import _debug_draw
        draw = _debug_draw.acquire_debug_draw_interface()
        draw.clear_lines()
        draw.clear_points()

        bx, by, bz = root_pos[0].item(), root_pos[1].item(), root_pos[2].item()

        # YELLOW vertical line — "ideal upright" reference (from base down to platform)
        draw.draw_lines(
            [(bx, by, bz)], [(bx, by, bz - 0.4)],
            [(1.0, 1.0, 0.0, 0.8)], [4.0],
        )

        # RED line — projected gravity in BODY frame (what the policy actually sees)
        # When upright: points straight down (0,0,-1). When tilted: x/y components appear.
        # We draw it from the base using the robot's own orientation to visualize in world.
        # The DIFFERENCE between red and yellow shows what the policy detects as tilt.
        from isaaclab.utils.math import quat_apply
        # Draw the robot's local "down" direction in world frame
        # This is the robot's Z-axis (body frame down) projected into world
        robot_down_b = torch.tensor([[0.0, 0.0, -1.0]], device=robot.data.root_quat_w.device)
        robot_down_w = quat_apply(robot.data.root_quat_w[0:1], robot_down_b)[0].cpu()
        grav_scale = 0.3
        draw.draw_lines(
            [(bx, by, bz)],
            [(bx + robot_down_w[0].item() * grav_scale, by + robot_down_w[1].item() * grav_scale, bz + robot_down_w[2].item() * grav_scale)],
            [(1.0, 0.1, 0.1, 1.0)], [8.0],
        )

        # BLUE line — velocity vector (shows sliding/falling direction)
        vel_scale = 0.5
        draw.draw_lines(
            [(bx, by, bz)],
            [(bx + lin_vel[0].item() * vel_scale, by + lin_vel[1].item() * vel_scale, bz + lin_vel[2].item() * vel_scale)],
            [(0.2, 0.4, 1.0, 1.0)], [6.0],
        )

        # GREEN dot — robot base position
        draw.draw_points([(bx, by, bz)], [(0.2, 1.0, 0.2, 1.0)], [20.0])

        # WHITE dot — whole-body CoM projected vertically onto platform height
        body_com_w = robot.data.body_com_pos_w  # (1, num_bodies, 3)
        masses = robot.data.default_mass.to(body_com_w.device)  # (1, num_bodies)
        total_mass = masses.sum(dim=-1, keepdim=True)
        whole_com_w = (body_com_w * masses.unsqueeze(-1)).sum(dim=1) / total_mass  # (1, 3)
        cx, cy, cz = whole_com_w[0, 0].item(), whole_com_w[0, 1].item(), whole_com_w[0, 2].item()
        # Draw CoM as a magenta dot
        draw.draw_points([(cx, cy, cz)], [(1.0, 0.3, 1.0, 1.0)], [20.0])
        # Draw vertical projection line from CoM down to platform level
        draw.draw_lines(
            [(cx, cy, cz)], [(cx, cy, PLATFORM_HEIGHT)],
            [(1.0, 0.3, 1.0, 0.5)], [2.0],
        )
        # Draw projected point on platform (WHITE — this must stay inside support polygon)
        draw.draw_points([(cx, cy, PLATFORM_HEIGHT)], [(1.0, 1.0, 1.0, 1.0)], [20.0])

        if step_count % 25 == 0:
            print(
                f"  plat: p={tilt_pitch:+5.1f} r={tilt_roll:+5.1f}"
                f"  | grav_body: [{grav_b[0]:+.3f} {grav_b[1]:+.3f} {grav_b[2]:+.3f}]"
                f"  | robot_down_w: [{robot_down_w[0]:+.3f} {robot_down_w[1]:+.3f} {robot_down_w[2]:+.3f}]",
                end="\r",
            )
        step_count += 1

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()

"""
Observation builder for Adelino's balance policy.

Maps VIMU inference output (joint angles, base orientation, velocities)
to the observation vector format expected by the trained policy.

The policy was trained with these observation terms (balance_env_cfg.py):
  projected_gravity (3D) — base orientation relative to gravity
  base_lin_vel (3D)      — base linear velocity
  base_ang_vel (3D)      — base angular velocity
  com_projection (2D)    — CoM projection offset (approximated)
  joint_pos (5D)         — joint positions relative to default
  joint_vel (5D)         — joint velocities
  actions (5D)           — previous action targets
"""

import math

import numpy as np

from config import (
    NUM_JOINTS,
    OBS_DIM,
    OBS_PROJECTED_GRAVITY,
    OBS_BASE_LIN_VEL,
    OBS_BASE_ANG_VEL,
    OBS_COM_PROJECTION,
    OBS_JOINT_POS,
    OBS_JOINT_VEL,
    OBS_ACTIONS,
    VIMU_JOINT_DIMS,
    VIMU_BASE_ROLL_DIM,
    VIMU_BASE_PITCH_DIM,
)


class ObservationBuilder:
    """Builds observation vectors from VIMU state messages."""

    def __init__(self):
        self.prev_actions = np.zeros(NUM_JOINTS, dtype=np.float32)
        self.obs = np.zeros(OBS_DIM, dtype=np.float32)

    def build(self, vimu_state: dict) -> np.ndarray:
        """
        Build observation vector from a VIMU WebSocket state message.

        Args:
            vimu_state: Parsed JSON from VIMU inference WebSocket, e.g.:
                {
                    "timestamp": 1.23,
                    "fps": 92.3,
                    "dims": [
                        {"name": "joint_1", "position": 0.1, "velocity": 0.01, ...},
                        ...
                        {"name": "base_roll", "position": 0.05, "velocity": 0.001, ...},
                        {"name": "base_pitch", "position": -0.02, "velocity": 0.0, ...},
                    ]
                }

        Returns:
            observation vector of shape (OBS_DIM,)
        """
        dims = {d["name"]: d for d in vimu_state["dims"]}
        obs = self.obs

        # -- projected_gravity (3D) --
        # In Isaac Lab, projected_gravity is the gravity vector in the robot's base frame.
        # With roll and pitch from VIMU, we can reconstruct it.
        roll = dims.get(VIMU_BASE_ROLL_DIM, {}).get("position", 0.0)
        pitch = dims.get(VIMU_BASE_PITCH_DIM, {}).get("position", 0.0)
        obs[OBS_PROJECTED_GRAVITY] = _projected_gravity(roll, pitch)

        # -- base_lin_vel (3D) --
        # VIMU doesn't directly provide base linear velocity (no translational tracking).
        # Set to zero — the policy was trained with noise on this, so it should be robust.
        obs[OBS_BASE_LIN_VEL] = 0.0

        # -- base_ang_vel (3D) --
        # Derive from VIMU's base roll/pitch velocity estimates.
        roll_vel = dims.get(VIMU_BASE_ROLL_DIM, {}).get("velocity", 0.0)
        pitch_vel = dims.get(VIMU_BASE_PITCH_DIM, {}).get("velocity", 0.0)
        obs[OBS_BASE_ANG_VEL] = [roll_vel, pitch_vel, 0.0]

        # -- com_projection (2D) --
        # Approximate: with small joint angles, CoM stays near base center.
        # A proper implementation would need the forward kinematics model.
        # For now, use a simple approximation based on joint positions.
        joint_positions = np.array([
            dims.get(name, {}).get("position", 0.0)
            for name in VIMU_JOINT_DIMS
        ])
        obs[OBS_COM_PROJECTION] = _approximate_com_projection(joint_positions)

        # -- joint_pos (5D) --
        # Joint positions relative to default (which is 0 for all Adelino joints).
        obs[OBS_JOINT_POS] = joint_positions

        # -- joint_vel (5D) --
        obs[OBS_JOINT_VEL] = [
            dims.get(name, {}).get("velocity", 0.0)
            for name in VIMU_JOINT_DIMS
        ]

        # -- actions (5D) --
        obs[OBS_ACTIONS] = self.prev_actions

        return obs.copy()

    def update_actions(self, actions: np.ndarray):
        """Store the latest actions for the next observation."""
        self.prev_actions[:] = actions


def _projected_gravity(roll: float, pitch: float) -> np.ndarray:
    """
    Compute the gravity vector projected into the robot's base frame.

    In Isaac Lab, projected_gravity = R^T @ [0, 0, -1] where R is the
    base-to-world rotation. With small-angle approximation:
      gx ≈ -sin(pitch)
      gy ≈  sin(roll) * cos(pitch)
      gz ≈ -cos(roll) * cos(pitch)

    For a perfectly level robot: [0, 0, -1].
    """
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    return np.array([-sp, sr * cp, -cr * cp], dtype=np.float32)


def _approximate_com_projection(joint_pos: np.ndarray) -> np.ndarray:
    """
    Rough approximation of CoM projection offset from base center.

    This is a simplification — in simulation, this is computed from
    the full articulation's center of mass. On real hardware without
    a full kinematics solver, we approximate based on joint angles.

    For Adelino's serial chain (yaw-pitch-pitch-roll-yaw), the CoM
    shifts primarily with J2 (pitch) and J3 (pitch) motion.
    """
    # Approximate CoM x-offset from J2+J3 pitch (forward/backward lean)
    com_x = 0.03 * math.sin(joint_pos[1]) + 0.02 * math.sin(joint_pos[1] + joint_pos[2])
    # Approximate CoM y-offset from J1 yaw
    com_y = 0.02 * math.sin(joint_pos[0])
    return np.array([com_x, com_y], dtype=np.float32)

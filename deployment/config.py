"""
Adelino deployment configuration.

Defines connection parameters, observation mapping, and joint configuration
for the sim-to-real deployment pipeline.
"""

import math

# ─── Connection endpoints ────────────────────────────────────────────────────

# VIMU inference WebSocket (provides proprioceptive state)
VIMU_WS_URL = "ws://localhost:9001"

# adelino-standalone controller WebSocket (receives motor commands)
CONTROLLER_WS_URL = "ws://localhost:8765"

# ─── Control loop ────────────────────────────────────────────────────────────

CONTROL_RATE_HZ = 30          # Target control loop frequency
ACTION_SCALE = 0.25           # Must match training: balance_env_cfg.py ActionsCfg.scale

# ─── Joint configuration ────────────────────────────────────────────────────

NUM_JOINTS = 5

# Joint names matching Isaac Lab training config
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

# VIMU output dimension names (set during VIMU training for Adelino)
# These must match the --num-joints and output order from VIMU's model
VIMU_JOINT_DIMS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
VIMU_BASE_ROLL_DIM = "base_roll"
VIMU_BASE_PITCH_DIM = "base_pitch"

# Joint limits in radians (matching embodiment and training config)
JOINT_LIMITS = {
    "joint_1": (-math.pi / 2, math.pi / 2),
    "joint_2": (-math.pi / 2, math.pi / 2),
    "joint_3": (-math.pi / 2, math.pi / 2),
    "joint_4": (-math.pi / 2, math.pi / 2),
    "joint_5": (-math.pi / 2, math.pi / 2),
}

# Default joint positions (radians) — the "standing" pose
DEFAULT_JOINT_POS = [0.0, 0.0, 0.0, 0.0, 0.0]

# ─── Observation vector layout ──────────────────────────────────────────────
# Must match balance_env_cfg.py ObservationsCfg.PolicyCfg:
#   projected_gravity (3) + base_lin_vel (3) + base_ang_vel (3)
#   + com_projection (2) + joint_pos (5) + joint_vel (5) + actions (5)
# Total: 26 dimensions

OBS_DIM = 26

# Observation slice indices (for clarity)
OBS_PROJECTED_GRAVITY = slice(0, 3)
OBS_BASE_LIN_VEL = slice(3, 6)
OBS_BASE_ANG_VEL = slice(6, 9)
OBS_COM_PROJECTION = slice(9, 11)
OBS_JOINT_POS = slice(11, 16)
OBS_JOINT_VEL = slice(16, 21)
OBS_ACTIONS = slice(21, 26)

# ─── VIMU data collection config (for training VIMU on Adelino) ─────────────

VIMU_COLLECTION_JOINT_RANGES = [
    (-1.2, 1.2),   # J1 — Base Yaw
    (-1.0, 1.0),   # J2 — Pitch
    (-0.8, 0.8),   # J3 — Pitch
    (-1.0, 1.0),   # J4 — Roll
    (-1.2, 1.2),   # J5 — Head Yaw
]

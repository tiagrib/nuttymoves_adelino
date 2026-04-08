"""Unit tests for deployment/observation_builder.py."""

import math

import numpy as np
import pytest

from config import (
    NUM_JOINTS,
    OBS_ACTIONS,
    OBS_DIM,
    OBS_JOINT_POS,
    VIMU_JOINT_DIMS,
)
from observation_builder import (
    ObservationBuilder,
    _approximate_com_projection,
    _projected_gravity,
)


# ---------------------------------------------------------------------------
# Helper: build a VIMU state message matching the contract
# ---------------------------------------------------------------------------

def _make_vimu_state(
    joint_positions=None,
    joint_velocities=None,
    base_roll=0.0,
    base_pitch=0.0,
    base_roll_vel=0.0,
    base_pitch_vel=0.0,
    timestamp=1.0,
    fps=60.0,
):
    """Create a VIMU WebSocket state dict matching vimu-websocket.md."""
    if joint_positions is None:
        joint_positions = [0.0] * NUM_JOINTS
    if joint_velocities is None:
        joint_velocities = [0.0] * NUM_JOINTS

    dims = []
    for i, name in enumerate(VIMU_JOINT_DIMS):
        dims.append({
            "name": name,
            "raw": joint_positions[i],
            "position": joint_positions[i],
            "velocity": joint_velocities[i],
            "acceleration": 0.0,
        })

    dims.append({
        "name": "base_roll",
        "raw": base_roll,
        "position": base_roll,
        "velocity": base_roll_vel,
        "acceleration": 0.0,
    })
    dims.append({
        "name": "base_pitch",
        "raw": base_pitch,
        "position": base_pitch,
        "velocity": base_pitch_vel,
        "acceleration": 0.0,
    })

    return {"timestamp": timestamp, "fps": fps, "dims": dims}


# ---------------------------------------------------------------------------
# _projected_gravity tests
# ---------------------------------------------------------------------------

class TestProjectedGravity:
    def test_level_robot(self):
        """Level robot (roll=0, pitch=0) should give [0, 0, -1]."""
        result = _projected_gravity(0.0, 0.0)
        np.testing.assert_allclose(result, [0.0, 0.0, -1.0], atol=1e-6)

    def test_45_degree_roll(self):
        """45-degree roll should tilt the gravity vector."""
        result = _projected_gravity(math.pi / 4, 0.0)
        expected = [
            0.0,                             # -sin(0)
            math.sin(math.pi / 4),           # sin(pi/4)*cos(0)
            -math.cos(math.pi / 4),          # -cos(pi/4)*cos(0)
        ]
        np.testing.assert_allclose(result, expected, atol=1e-6)

    def test_45_degree_pitch(self):
        """45-degree pitch should tilt the gravity vector forward."""
        result = _projected_gravity(0.0, math.pi / 4)
        expected = [
            -math.sin(math.pi / 4),          # -sin(pi/4)
            0.0,                              # sin(0)*cos(pi/4)
            -math.cos(math.pi / 4),           # -cos(0)*cos(pi/4)
        ]
        np.testing.assert_allclose(result, expected, atol=1e-6)


# ---------------------------------------------------------------------------
# _approximate_com_projection tests
# ---------------------------------------------------------------------------

class TestApproximateComProjection:
    def test_zero_joint_positions(self):
        """All-zero joint positions should give near-zero CoM projection."""
        result = _approximate_com_projection(np.zeros(NUM_JOINTS))
        np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-6)


# ---------------------------------------------------------------------------
# ObservationBuilder tests
# ---------------------------------------------------------------------------

class TestObservationBuilder:
    def test_build_output_shape(self):
        """build() should return an array of shape (OBS_DIM,) = (26,)."""
        builder = ObservationBuilder()
        state = _make_vimu_state()
        obs = builder.build(state)
        assert obs.shape == (OBS_DIM,)

    def test_build_joint_positions_in_correct_slice(self):
        """Joint positions from the VIMU state should appear in OBS_JOINT_POS."""
        builder = ObservationBuilder()
        joint_pos = [0.1, -0.2, 0.3, -0.4, 0.5]
        state = _make_vimu_state(joint_positions=joint_pos)
        obs = builder.build(state)
        np.testing.assert_allclose(obs[OBS_JOINT_POS], joint_pos, atol=1e-6)

    def test_actions_zero_initially(self):
        """On the first call, prev_actions should be zero."""
        builder = ObservationBuilder()
        state = _make_vimu_state()
        obs = builder.build(state)
        np.testing.assert_allclose(obs[OBS_ACTIONS], np.zeros(NUM_JOINTS), atol=1e-6)

    def test_update_actions_reflected_in_next_build(self):
        """After update_actions(), the next build() should include those actions."""
        builder = ObservationBuilder()
        state = _make_vimu_state()

        # First build — actions should be zero
        obs1 = builder.build(state)
        np.testing.assert_allclose(obs1[OBS_ACTIONS], np.zeros(NUM_JOINTS), atol=1e-6)

        # Update actions
        new_actions = np.array([0.1, -0.2, 0.3, 0.0, -0.5], dtype=np.float32)
        builder.update_actions(new_actions)

        # Second build — actions should reflect the update
        obs2 = builder.build(state)
        np.testing.assert_allclose(obs2[OBS_ACTIONS], new_actions, atol=1e-6)

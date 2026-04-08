"""Unit tests for deployment/config.py."""

from config import (
    DEFAULT_JOINT_POS,
    JOINT_NAMES,
    NUM_JOINTS,
    OBS_ACTIONS,
    OBS_BASE_ANG_VEL,
    OBS_BASE_LIN_VEL,
    OBS_COM_PROJECTION,
    OBS_DIM,
    OBS_JOINT_POS,
    OBS_JOINT_VEL,
    OBS_PROJECTED_GRAVITY,
    VIMU_JOINT_DIMS,
)


def test_obs_dim_equals_sum_of_slices():
    """OBS_DIM (26) must equal the total span of all observation slices."""
    slices = [
        OBS_PROJECTED_GRAVITY,
        OBS_BASE_LIN_VEL,
        OBS_BASE_ANG_VEL,
        OBS_COM_PROJECTION,
        OBS_JOINT_POS,
        OBS_JOINT_VEL,
        OBS_ACTIONS,
    ]
    total = sum(s.stop - s.start for s in slices)
    assert total == OBS_DIM == 26


def test_num_joints_is_five():
    assert NUM_JOINTS == 5


def test_joint_names_length():
    assert len(JOINT_NAMES) == NUM_JOINTS


def test_vimu_joint_dims_length():
    assert len(VIMU_JOINT_DIMS) == NUM_JOINTS


def test_default_joint_pos_length():
    assert len(DEFAULT_JOINT_POS) == NUM_JOINTS


def test_observation_slices_contiguous_and_non_overlapping():
    """All observation slices must be contiguous and cover [0, OBS_DIM)."""
    slices = [
        OBS_PROJECTED_GRAVITY,
        OBS_BASE_LIN_VEL,
        OBS_BASE_ANG_VEL,
        OBS_COM_PROJECTION,
        OBS_JOINT_POS,
        OBS_JOINT_VEL,
        OBS_ACTIONS,
    ]
    # Sort by start index
    sorted_slices = sorted(slices, key=lambda s: s.start)

    # First slice should start at 0
    assert sorted_slices[0].start == 0

    # Each slice should start where the previous one ended
    for i in range(1, len(sorted_slices)):
        assert sorted_slices[i].start == sorted_slices[i - 1].stop, (
            f"Gap or overlap between slice ending at {sorted_slices[i - 1].stop} "
            f"and slice starting at {sorted_slices[i].start}"
        )

    # Last slice should end at OBS_DIM
    assert sorted_slices[-1].stop == OBS_DIM

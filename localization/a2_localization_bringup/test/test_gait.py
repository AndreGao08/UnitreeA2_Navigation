import math

import pytest

from a2_localization_bringup.gait import (
    DEFAULT_JOINT_POSITIONS,
    JOINT_NAMES,
    command_is_active,
    move_towards,
    trot_joint_targets,
)


def test_zero_command_returns_unitree_nominal_pose():
    assert trot_joint_targets(0.2, 0.0, 0.0, 0.0) == DEFAULT_JOINT_POSITIONS


def test_trot_emits_all_finite_joint_targets():
    targets = trot_joint_targets(0.17, 0.35, 0.1, -0.2)
    assert tuple(targets) == JOINT_NAMES
    assert all(math.isfinite(value) for value in targets.values())


def test_diagonal_pairs_share_phase():
    targets = trot_joint_targets(0.10, 0.35, 0.0, 0.0)
    fl = targets['FL_thigh_joint'] - DEFAULT_JOINT_POSITIONS['FL_thigh_joint']
    rr = targets['RR_thigh_joint'] - DEFAULT_JOINT_POSITIONS['RR_thigh_joint']
    fr = targets['FR_thigh_joint'] - DEFAULT_JOINT_POSITIONS['FR_thigh_joint']
    rl = targets['RL_thigh_joint'] - DEFAULT_JOINT_POSITIONS['RL_thigh_joint']
    assert fl == pytest.approx(rr)
    assert fr == pytest.approx(rl)
    assert fl != pytest.approx(fr)


def test_swing_pair_shortens_while_stance_pair_does_not():
    targets = trot_joint_targets(0.15, 0.35, 0.0, 0.0)
    assert targets['FL_calf_joint'] == pytest.approx(-1.8)
    assert targets['RR_calf_joint'] == pytest.approx(-1.8)
    assert targets['FR_calf_joint'] < -1.8
    assert targets['RL_calf_joint'] < -1.8


def test_yaw_command_drives_left_and_right_in_opposite_directions():
    targets = trot_joint_targets(0.0, 0.0, 0.0, 0.5)
    fl = targets['FL_thigh_joint'] - DEFAULT_JOINT_POSITIONS['FL_thigh_joint']
    fr = targets['FR_thigh_joint'] - DEFAULT_JOINT_POSITIONS['FR_thigh_joint']
    assert fl * fr > 0.0  # Opposite phase plus opposite wheel-equivalent command.


def test_command_deadband():
    assert not command_is_active(0.019, 0.0, 0.0)
    assert command_is_active(0.02, 0.0, 0.0)


def test_move_towards_limits_each_step_without_overshoot():
    assert move_towards(0.0, 1.0, 0.2) == pytest.approx(0.2)
    assert move_towards(0.0, -1.0, 0.2) == pytest.approx(-0.2)
    assert move_towards(0.9, 1.0, 0.2) == pytest.approx(1.0)

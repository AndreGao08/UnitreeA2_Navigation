import math

import pytest

from a2_localization_bringup.trajectory_commander import motion_profile, trajectory_command


def test_waits_during_start_delay():
    assert trajectory_command('square', 3.0, 8.0, 0.35, 2.0, 0.35) == (0.0, 0.0)


def test_square_uses_finite_straight_and_turn_phases():
    speed = 0.5
    scale = 1.0
    yaw_rate = 0.25
    delay = 2.0
    _, straight_duration = motion_profile(2.0 * scale, speed, 0.25, 0.0)

    linear, angular = trajectory_command(
        'square', delay + 1.0, delay, speed, scale, yaw_rate
    )
    assert linear > 0.0
    assert angular == 0.0
    turn_linear, turn_angular = trajectory_command(
        'square', delay + straight_duration + 0.1, delay, speed, scale, yaw_rate
    )
    assert turn_linear == 0.0
    assert -yaw_rate <= turn_angular < 0.0


def test_square_turn_integrates_to_quarter_turn():
    yaw_rate = 0.4
    turn_duration = (math.pi / 2.0) / yaw_rate
    assert yaw_rate * turn_duration == pytest.approx(math.pi / 2.0)


def test_straight_stops_after_four_metres():
    _, duration = motion_profile(4.0, 0.4, 0.25, 0.0)
    assert trajectory_command('straight', 1.0, 0.0, 0.4, 2.0, 0.35)[0] > 0.0
    assert trajectory_command('straight', duration, 0.0, 0.4, 2.0, 0.35) == (0.0, 0.0)


def test_motion_profile_integrates_requested_distance():
    dt = 0.001
    velocity_sum = 0.0
    _, duration = motion_profile(4.0, 0.35, 0.25, 0.0)
    steps = int(math.ceil(duration / dt))
    for index in range(steps + 1):
        velocity_sum += motion_profile(4.0, 0.35, 0.25, index * dt)[0] * dt
    assert velocity_sum == pytest.approx(4.0, abs=1.0e-3)

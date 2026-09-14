import math
from types import SimpleNamespace

import pytest

from hesai_jt128_sim.imu_adapter import gravity_in_sensor_frame


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return SimpleNamespace(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def test_level_gravity_is_positive_z_for_any_yaw():
    gravity = gravity_in_sensor_frame(quaternion(0.0, 0.0, 1.2), 9.80665)
    assert gravity == pytest.approx((0.0, 0.0, 9.80665))


def test_gravity_is_rotated_into_sensor_frame():
    gravity = gravity_in_sensor_frame(quaternion(0.0, math.pi / 2.0, 0.0), 9.80665)
    assert gravity == pytest.approx((-9.80665, 0.0, 0.0), abs=1.0e-6)

import math

from a2_localization_bringup.transforms import (
    compose_transform, inverse_transform, relative_transform, rpy_quaternion, yaw_quaternion)


def test_inverse_and_compose_identity():
    t = (1.0, -2.0, 0.3)
    q = yaw_quaternion(0.7)
    ti, qi = inverse_transform(t, q)
    tout, qout = compose_transform(t, q, ti, qi)
    assert all(abs(v) < 1e-9 for v in tout)
    assert abs(qout[0]) < 1e-9
    assert abs(qout[1]) < 1e-9
    assert abs(qout[2]) < 1e-9
    assert abs(qout[3] - 1.0) < 1e-9


def test_relative_transform_removes_origin():
    q0 = yaw_quaternion(math.pi / 2.0)
    t, q = relative_transform((1.0, 2.0, 0.0), q0,
                              (1.0, 3.0, 0.0), q0)
    assert abs(t[0] - 1.0) < 1e-9
    assert abs(t[1]) < 1e-9
    assert abs(q[3] - 1.0) < 1e-9


def test_world_imu_pose_converts_back_to_world_base_pose():
    t_wb = (2.0, -1.0, 0.5)
    q_wb = rpy_quaternion(0.2, -0.1, 0.8)
    t_bi = (0.12, -0.03, 0.24)
    q_bi = rpy_quaternion(0.01, 0.04, -0.02)
    t_wi, q_wi = compose_transform(t_wb, q_wb, t_bi, q_bi)
    t_ib, q_ib = inverse_transform(t_bi, q_bi)
    recovered_t_wb, recovered_q_wb = compose_transform(t_wi, q_wi, t_ib, q_ib)
    assert all(abs(a - b) < 1e-9 for a, b in zip(recovered_t_wb, t_wb))
    assert all(abs(a - b) < 1e-9 for a, b in zip(recovered_q_wb, q_wb))

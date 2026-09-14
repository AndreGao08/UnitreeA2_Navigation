import math


def q_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def q_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return q_normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def q_rotate(q, vector):
    x, y, z = vector
    qx, qy, qz, qw = q_normalize(q)
    # Optimized q * (v, 0) * conjugate(q).
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def inverse_transform(translation, rotation):
    rotation_inv = q_conjugate(q_normalize(rotation))
    translation_inv = q_rotate(rotation_inv, tuple(-value for value in translation))
    return translation_inv, rotation_inv


def compose_transform(ta, qa, tb, qb):
    rotated = q_rotate(qa, tb)
    translation = tuple(ta[i] + rotated[i] for i in range(3))
    return translation, q_multiply(qa, qb)


def relative_transform(t0, q0, t1, q1):
    t0_inv, q0_inv = inverse_transform(t0, q0)
    return compose_transform(t0_inv, q0_inv, t1, q1)


def quaternion_angle(q):
    q = q_normalize(q)
    return 2.0 * math.acos(min(1.0, max(-1.0, abs(q[3]))))


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def rpy_quaternion(roll, pitch, yaw):
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return q_normalize((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))

"""Dependency-free A2 stand and analytic trot target generation."""

import math


JOINT_NAMES = (
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
)

# Unitree unitree_rl_mjlab A2 deployment order and nominal pose.
DEFAULT_JOINT_POSITIONS = {
    'FL_hip_joint': -0.1, 'FL_thigh_joint': 0.9, 'FL_calf_joint': -1.8,
    'FR_hip_joint': 0.1, 'FR_thigh_joint': 0.9, 'FR_calf_joint': -1.8,
    'RL_hip_joint': -0.1, 'RL_thigh_joint': 0.9, 'RL_calf_joint': -1.8,
    'RR_hip_joint': 0.1, 'RR_thigh_joint': 0.9, 'RR_calf_joint': -1.8,
}

# FL + RR and FR + RL are the two diagonal trot pairs.
_PHASE_OFFSETS = {'FL': 0.0, 'FR': 0.5, 'RL': 0.5, 'RR': 0.0}
_SIDE = {'FL': 1.0, 'RL': 1.0, 'FR': -1.0, 'RR': -1.0}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def move_towards(value: float, target: float, max_delta: float) -> float:
    """Move value toward target without exceeding max_delta."""
    return value + clamp(target - value, -abs(max_delta), abs(max_delta))


def _smoothstep(value: float) -> float:
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def command_is_active(vx: float, vy: float, wz: float, deadband: float = 0.02) -> bool:
    return max(abs(vx), abs(vy), abs(wz)) >= deadband


def trot_joint_targets(
    elapsed: float,
    vx: float,
    vy: float,
    wz: float,
    *,
    period: float = 0.6,
    stride_gain: float = 0.80,
    lateral_gain: float = 0.22,
    yaw_radius: float = 0.32,
    max_stride: float = 0.22,
    max_lateral: float = 0.12,
    swing_knee_lift: float = 0.20,
) -> dict[str, float]:
    """Return 12 position targets for a conservative diagonal trot.

    This is a deterministic commissioning gait, not a replacement for the
    learned Unitree policy.  It exercises the exact same joint target contract
    as the A2 mjlab policy so the policy runner can replace it later.
    """
    targets = dict(DEFAULT_JOINT_POSITIONS)
    if not command_is_active(vx, vy, wz):
        return targets

    period = max(period, 0.1)
    cycle = (max(elapsed, 0.0) / period) % 1.0
    activity = clamp(max(abs(vx), abs(vy), abs(wz)) / 0.25, 0.25, 1.0)

    for leg in ('FL', 'FR', 'RL', 'RR'):
        phase = (cycle + _PHASE_OFFSETS[leg]) % 1.0
        side = _SIDE[leg]

        # Positive yaw requires the right feet to travel forward relative to
        # the body and the left feet backward, like a differential drive.
        leg_forward = vx - side * yaw_radius * wz
        stride = clamp(stride_gain * leg_forward, -max_stride, max_stride)
        lateral = clamp(lateral_gain * vy, -max_lateral, max_lateral)

        if phase < 0.5:
            # Stance: move the planted foot from front to rear.  The calf stays
            # near nominal length so this diagonal pair supports the body.
            s = _smoothstep(phase * 2.0)
            fore_aft = -stride + 2.0 * stride * s
            side_to_side = lateral * (1.0 - 2.0 * s)
            knee_lift = 0.0
        else:
            # Swing: return the foot smoothly and shorten the leg for clearance.
            s_raw = (phase - 0.5) * 2.0
            s = _smoothstep(s_raw)
            fore_aft = stride - 2.0 * stride * s
            side_to_side = lateral * (-1.0 + 2.0 * s)
            knee_lift = swing_knee_lift * activity * math.sin(math.pi * s_raw)

        targets[f'{leg}_hip_joint'] += side_to_side
        targets[f'{leg}_thigh_joint'] += fore_aft
        targets[f'{leg}_calf_joint'] -= knee_lift

    return targets

import math

from a2_localization_bringup.odom_guard import pose_fault_reason


IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_accepts_normal_motion_step():
    current = ((0.08, 0.01, 0.0), (0.0, 0.0, math.sin(0.04), math.cos(0.04)))
    assert pose_fault_reason(IDENTITY, current, 0.20, math.radians(20.0)) is None


def test_rejects_translation_jump():
    current = ((0.25, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert 'translation jump' in pose_fault_reason(
        IDENTITY, current, 0.20, math.radians(20.0))


def test_rejects_rotation_jump_and_nonfinite_pose():
    rotated = ((0.0, 0.0, 0.0), (0.0, 0.0, math.sin(0.25), math.cos(0.25)))
    assert 'rotation jump' in pose_fault_reason(
        IDENTITY, rotated, 0.20, math.radians(20.0))
    invalid = ((float('inf'), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert 'non-finite' in pose_fault_reason(
        IDENTITY, invalid, 0.20, math.radians(20.0))

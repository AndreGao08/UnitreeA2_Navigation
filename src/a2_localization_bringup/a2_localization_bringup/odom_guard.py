import math
from typing import Optional, Sequence, Tuple


PoseTuple = Tuple[Sequence[float], Sequence[float]]


def pose_fault_reason(
        previous: Optional[PoseTuple], current: PoseTuple,
        max_translation_step: float, max_rotation_step_rad: float) -> Optional[str]:
    """Return a reason when an odometry pose is non-finite or jumps too far."""
    translation, quaternion = current
    values = tuple(translation) + tuple(quaternion)
    if len(translation) != 3 or len(quaternion) != 4 or not all(map(math.isfinite, values)):
        return 'non-finite odometry pose'

    quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
    if quaternion_norm < 1.0e-6:
        return 'invalid zero-norm odometry quaternion'
    if previous is None:
        return None

    previous_translation, previous_quaternion = previous
    translation_step = math.sqrt(sum(
        (current_value - previous_value) ** 2
        for current_value, previous_value in zip(translation, previous_translation)
    ))
    if translation_step > max_translation_step:
        return (
            f'odometry translation jump {translation_step:.3f} m exceeds '
            f'{max_translation_step:.3f} m'
        )

    previous_norm = math.sqrt(sum(value * value for value in previous_quaternion))
    if previous_norm < 1.0e-6:
        return 'previous odometry quaternion is invalid'
    dot = abs(sum(
        current_value * previous_value
        for current_value, previous_value in zip(quaternion, previous_quaternion)
    ) / (quaternion_norm * previous_norm))
    rotation_step = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
    if rotation_step > max_rotation_step_rad:
        return (
            f'odometry rotation jump {math.degrees(rotation_step):.1f} deg exceeds '
            f'{math.degrees(max_rotation_step_rad):.1f} deg'
        )
    return None

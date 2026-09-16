import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def trajectory_command(
    trajectory: str,
    elapsed: float,
    start_delay: float,
    speed: float,
    scale: float,
    angular_speed: float,
    linear_acceleration: float = 0.25,
    angular_acceleration: float = 0.25,
):
    """Return body-frame forward and yaw velocities for a scripted path."""
    if elapsed < start_delay or trajectory == 'stationary':
        return 0.0, 0.0

    t = elapsed - start_delay
    if trajectory == 'straight':
        linear, _ = motion_profile(4.0, speed, linear_acceleration, t)
        return linear, 0.0

    if trajectory == 'square':
        # Traverse each edge, then turn in place. Unlike SetEntityPose, the
        # finite-duration turn produces angular velocity that the IMU sees.
        _, straight_duration = motion_profile(
            2.0 * scale, speed, linear_acceleration, 0.0
        )
        _, turn_duration = motion_profile(
            math.pi / 2.0, angular_speed, angular_acceleration, 0.0
        )
        phase_duration = straight_duration + turn_duration
        phase_t = t % phase_duration
        if phase_t < straight_duration:
            linear, _ = motion_profile(
                2.0 * scale, speed, linear_acceleration, phase_t
            )
            return linear, 0.0
        angular, _ = motion_profile(
            math.pi / 2.0,
            angular_speed,
            angular_acceleration,
            phase_t - straight_duration,
        )
        # Turn clockwise: the resulting square stays clear of the box cluster
        # in the north-east quadrant of the localization world.
        return 0.0, -angular

    if trajectory == 'figure8':
        # Body velocities for x=a*sin(p), y=a*sin(p)*cos(p), rotated so its
        # initial tangent agrees with the robot's initial heading.
        omega = speed / max(scale, 1.0e-3)
        phase = omega * t
        cos_p = math.cos(phase)
        cos_2p = math.cos(2.0 * phase)
        sin_p = math.sin(phase)
        sin_2p = math.sin(2.0 * phase)
        # Ramp the initial command to avoid an acceleration impulse at startup.
        blend = min(t / max(speed / linear_acceleration, 1.0e-3), 1.0)
        linear = blend * speed * math.hypot(cos_p, cos_2p)
        denominator = cos_p * cos_p + cos_2p * cos_2p
        yaw_rate = blend * omega * (
            -2.0 * cos_p * sin_2p + cos_2p * sin_p
        ) / max(denominator, 1.0e-6)
        return linear, yaw_rate

    raise ValueError(
        f"unknown trajectory '{trajectory}'; expected manual, stationary, "
        'straight, square, or figure8'
    )


def motion_profile(distance: float, max_velocity: float, acceleration: float, t: float):
    """Sample a rest-to-rest trapezoidal velocity profile.

    Returns ``(velocity, total_duration)``. A triangular profile is selected
    automatically when there is not enough distance to reach max_velocity.
    """
    distance = max(distance, 0.0)
    max_velocity = max(max_velocity, 1.0e-6)
    acceleration = max(acceleration, 1.0e-6)
    ramp_duration = max_velocity / acceleration
    ramp_distance = max_velocity * max_velocity / acceleration

    if distance <= ramp_distance:
        peak_velocity = math.sqrt(distance * acceleration)
        ramp_duration = peak_velocity / acceleration
        cruise_duration = 0.0
    else:
        peak_velocity = max_velocity
        cruise_duration = (distance - ramp_distance) / max_velocity

    total_duration = 2.0 * ramp_duration + cruise_duration
    if t <= 0.0 or t >= total_duration:
        return 0.0, total_duration
    if t < ramp_duration:
        return acceleration * t, total_duration
    if t < ramp_duration + cruise_duration:
        return peak_velocity, total_duration
    return acceleration * (total_duration - t), total_duration


class TrajectoryCommander(Node):
    """Publish repeatable, continuous velocity commands for LIO validation."""

    def __init__(self) -> None:
        super().__init__('a2_trajectory_commander')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('trajectory', 'figure8')
        self.declare_parameter('start_delay', 8.0)
        self.declare_parameter('speed', 0.35)
        self.declare_parameter('scale', 2.0)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('linear_acceleration', 0.25)
        self.declare_parameter('angular_acceleration', 0.25)
        self.declare_parameter('update_rate', 50.0)

        self._trajectory = str(self.get_parameter('trajectory').value)
        self._delay = float(self.get_parameter('start_delay').value)
        self._speed = float(self.get_parameter('speed').value)
        self._scale = float(self.get_parameter('scale').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._linear_acceleration = float(
            self.get_parameter('linear_acceleration').value
        )
        self._angular_acceleration = float(
            self.get_parameter('angular_acceleration').value
        )
        rate = float(self.get_parameter('update_rate').value)

        if min(
            self._speed,
            self._scale,
            self._angular_speed,
            self._linear_acceleration,
            self._angular_acceleration,
        ) <= 0.0:
            raise ValueError('speed, scale, and acceleration parameters must be positive')
        if self._trajectory not in {
            'manual', 'stationary', 'straight', 'square', 'figure8'
        }:
            raise ValueError(f"unsupported trajectory '{self._trajectory}'")

        topic = str(self.get_parameter('cmd_vel_topic').value)
        self._publisher = self.create_publisher(Twist, topic, 10)
        self._start = None
        self._manual = self._trajectory == 'manual'
        if self._manual:
            self.get_logger().info(
                f'manual mode: trajectory commander will not publish; send Twist to {topic}'
            )
        else:
            self.get_logger().info(
                f"trajectory={self._trajectory}: publishing continuous Twist on {topic}"
            )
            self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        now = self.get_clock().now()
        if self._start is None:
            self._start = now
        elapsed = (now - self._start).nanoseconds * 1.0e-9
        linear, angular = trajectory_command(
            self._trajectory,
            elapsed,
            self._delay,
            self._speed,
            self._scale,
            self._angular_speed,
            self._linear_acceleration,
            self._angular_acceleration,
        )
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node._manual and rclpy.ok():
            node._publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

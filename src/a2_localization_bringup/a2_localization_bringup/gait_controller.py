"""ROS 2 controller that drives the A2 Gazebo joint target interface."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gait import (
    DEFAULT_JOINT_POSITIONS,
    JOINT_NAMES,
    clamp,
    command_is_active,
    move_towards,
)
from .gait import trot_joint_targets


class A2GaitController(Node):
    """Safe stand plus conservative trot for commissioning A2 dynamics."""

    def __init__(self) -> None:
        super().__init__('a2_gait_controller')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'joint_trajectory_topic', '/model/unitree_a2/joint_trajectory')
        self.declare_parameter('base_motion_assist', False)
        self.declare_parameter(
            'base_motion_assist_topic', '/a2/base_velocity_assist')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('stand_duration', 3.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('gait_period', 0.6)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_lateral_speed', 0.3)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.declare_parameter('max_linear_accel', 0.6)
        self.declare_parameter('max_yaw_accel', 1.2)
        self.declare_parameter('stride_gain', 0.80)
        self.declare_parameter('lateral_gain', 0.22)
        self.declare_parameter('yaw_radius', 0.32)
        self.declare_parameter('max_stride', 0.22)
        self.declare_parameter('max_lateral', 0.12)
        self.declare_parameter('swing_knee_lift', 0.20)

        rate = float(self.get_parameter('control_rate').value)
        if rate <= 0.0:
            raise ValueError('control_rate must be positive')
        # A zero-duration point is reported as complete immediately by the
        # Gazebo Fortress JointTrajectoryController and may never be applied.
        # Keep each target valid for two controller ticks so consecutive
        # commands form a continuous set-point stream.
        self._trajectory_horizon = 2.0 / rate
        self._stand_duration = float(self.get_parameter('stand_duration').value)
        self._timeout = float(self.get_parameter('command_timeout').value)
        self._period = float(self.get_parameter('gait_period').value)
        self._max_vx = float(self.get_parameter('max_linear_speed').value)
        self._max_vy = float(self.get_parameter('max_lateral_speed').value)
        self._max_wz = float(self.get_parameter('max_yaw_rate').value)
        self._linear_step = (
            float(self.get_parameter('max_linear_accel').value) / rate)
        self._yaw_step = float(self.get_parameter('max_yaw_accel').value) / rate
        self._gait_parameters = {
            name: float(self.get_parameter(name).value)
            for name in (
                'stride_gain', 'lateral_gain', 'yaw_radius', 'max_stride',
                'max_lateral', 'swing_knee_lift',
            )
        }

        trajectory_topic = str(
            self.get_parameter('joint_trajectory_topic').value)
        self._joint_publisher = self.create_publisher(
            JointTrajectory, trajectory_topic, 1)
        self._base_motion_assist = bool(
            self.get_parameter('base_motion_assist').value)
        self._assist_publisher = None
        if self._base_motion_assist:
            self._assist_publisher = self.create_publisher(
                Twist,
                str(self.get_parameter('base_motion_assist_topic').value),
                1,
            )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            String, '/a2/locomotion/state', status_qos)
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value),
            self._command_callback, 10)

        self._command = Twist()
        self._last_command_time = None
        self._start_time = None
        self._gait_start_time = None
        self._last_state = None
        self._filtered_vx = 0.0
        self._filtered_vy = 0.0
        self._filtered_wz = 0.0
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'A2 gait controller ready: STAND -> analytic TROT; '
            'joint contract matches Unitree mjlab A2 policy output; '
            f'base_motion_assist={self._base_motion_assist}')

    def _command_callback(self, message: Twist) -> None:
        values = (
            message.linear.x, message.linear.y, message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('discarding non-finite cmd_vel')
            return
        self._command = message
        self._last_command_time = self.get_clock().now()

    def _publish(self, targets: dict[str, float]) -> None:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(targets[name]) for name in JOINT_NAMES]
        point.time_from_start.sec = int(self._trajectory_horizon)
        point.time_from_start.nanosec = int(
            (self._trajectory_horizon - point.time_from_start.sec) * 1.0e9)
        message.points = [point]
        self._joint_publisher.publish(message)

    def _set_state(self, state: str) -> None:
        if state == self._last_state:
            return
        self._last_state = state
        self._status_publisher.publish(String(data=state))
        self.get_logger().info(f'locomotion state: {state}')

    def _publish_assist(self, vx: float, vy: float, wz: float) -> None:
        if self._assist_publisher is None:
            return
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        self._assist_publisher.publish(message)

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self._start_time is None:
            self._start_time = now
        since_start = (now - self._start_time).nanoseconds * 1.0e-9

        if since_start < self._stand_duration:
            self._gait_start_time = None
            self._filtered_vx = 0.0
            self._filtered_vy = 0.0
            self._filtered_wz = 0.0
            self._set_state('STAND_INITIALIZING')
            self._publish(DEFAULT_JOINT_POSITIONS)
            self._publish_assist(0.0, 0.0, 0.0)
            return

        command_fresh = (
            self._last_command_time is not None
            and (now - self._last_command_time).nanoseconds * 1.0e-9 <= self._timeout
        )
        target_vx = clamp(
            self._command.linear.x, -self._max_vx, self._max_vx
        ) if command_fresh else 0.0
        target_vy = clamp(
            self._command.linear.y, -self._max_vy, self._max_vy
        ) if command_fresh else 0.0
        target_wz = clamp(
            self._command.angular.z, -self._max_wz, self._max_wz
        ) if command_fresh else 0.0
        self._filtered_vx = move_towards(
            self._filtered_vx, target_vx, self._linear_step)
        self._filtered_vy = move_towards(
            self._filtered_vy, target_vy, self._linear_step)
        self._filtered_wz = move_towards(
            self._filtered_wz, target_wz, self._yaw_step)

        if not command_is_active(
                self._filtered_vx, self._filtered_vy, self._filtered_wz):
            self._gait_start_time = None
            self._set_state('STAND')
            self._publish(DEFAULT_JOINT_POSITIONS)
            self._publish_assist(0.0, 0.0, 0.0)
            return

        if self._gait_start_time is None:
            self._gait_start_time = now
        gait_elapsed = (now - self._gait_start_time).nanoseconds * 1.0e-9
        targets = trot_joint_targets(
            gait_elapsed,
            self._filtered_vx,
            self._filtered_vy,
            self._filtered_wz,
            period=self._period,
            **self._gait_parameters,
        )
        self._set_state('TROT')
        self._publish(targets)
        self._publish_assist(
            self._filtered_vx, self._filtered_vy, self._filtered_wz)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = A2GaitController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish(DEFAULT_JOINT_POSITIONS)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

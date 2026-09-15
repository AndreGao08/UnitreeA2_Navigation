import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def localization_is_healthy(localization_status: str, odometry_status: str) -> bool:
    return localization_status == 'LOCALIZED' and odometry_status == 'OK'


class LocalizationSafetyMonitor(Node):
    """Gate Nav2 velocity and cancel goals whenever localization is unsafe."""

    def __init__(self) -> None:
        super().__init__('a2_localization_safety_monitor')
        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', '/a2/safe_cmd_vel')
        self.declare_parameter('odometry_topic', '/a2/odometry')
        self.declare_parameter('odometry_timeout', 1.0)
        self.declare_parameter('stop_publish_frequency', 20.0)

        self._localization_status = ''
        self._odometry_status = ''
        self._last_odom_time = None
        self._was_healthy = False
        # Also cancel a goal that might have been sent before localization was ready.
        self._cancel_pending = {
            'navigate_to_pose', 'navigate_through_poses', 'follow_waypoints'
        }
        self._last_cancel_request_time = float('-inf')
        self._odom_timeout = float(self.get_parameter('odometry_timeout').value)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/a2/relocalization/status', self._localization_callback, status_qos)
        self.create_subscription(
            String, '/a2/odometry/status', self._odometry_status_callback, status_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value,
            self._odometry_callback, 20)
        self.create_subscription(
            Twist, self.get_parameter('input_cmd_vel_topic').value,
            self._velocity_callback, 20)
        self._safe_velocity_publisher = self.create_publisher(
            Twist, self.get_parameter('output_cmd_vel_topic').value, 20)

        action_names = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints')
        self._cancel_clients = {
            name: self.create_client(CancelGoal, f'/{name}/_action/cancel_goal')
            for name in action_names
        }
        frequency = max(
            float(self.get_parameter('stop_publish_frequency').value), 1.0)
        self.create_timer(1.0 / frequency, self._timer_callback)
        self.get_logger().info(
            'Localization safety gate is holding zero velocity until both '
            'relocalization and FAST-LIO odometry are healthy')

    def _localization_callback(self, message: String) -> None:
        self._localization_status = message.data
        self._update_state()

    def _odometry_status_callback(self, message: String) -> None:
        self._odometry_status = message.data
        self._update_state()

    def _odometry_callback(self, _: Odometry) -> None:
        self._last_odom_time = time.monotonic()
        self._update_state()

    def _healthy(self) -> bool:
        odometry_fresh = (
            self._last_odom_time is not None
            and time.monotonic() - self._last_odom_time <= self._odom_timeout
        )
        return odometry_fresh and localization_is_healthy(
            self._localization_status, self._odometry_status)

    def _update_state(self) -> None:
        healthy = self._healthy()
        if self._was_healthy and not healthy:
            self.get_logger().error(
                'Localization became unsafe; cancelling Nav2 goals and stopping. '
                f'localization="{self._localization_status}", '
                f'odometry="{self._odometry_status}"')
            self._cancel_pending = set(self._cancel_clients)
            self._publish_stop()
        elif healthy and not self._was_healthy:
            self.get_logger().info('Localization healthy; Nav2 velocity output enabled')
        self._was_healthy = healthy

    def _velocity_callback(self, message: Twist) -> None:
        if self._healthy():
            self._safe_velocity_publisher.publish(message)
            return
        if any(abs(value) > 1.0e-6 for value in (
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z)) and (
                not self._cancel_pending
                and time.monotonic() - self._last_cancel_request_time > 1.0):
            self._cancel_pending = set(self._cancel_clients)
        self._publish_stop()

    def _timer_callback(self) -> None:
        self._update_state()
        if not self._healthy():
            self._publish_stop()
        sent_cancel = False
        for action_name in tuple(self._cancel_pending):
            client = self._cancel_clients[action_name]
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
                self._cancel_pending.remove(action_name)
                sent_cancel = True
                self.get_logger().warn(f'Cancel requested for Nav2 action /{action_name}')
        if sent_cancel:
            self._last_cancel_request_time = time.monotonic()

    def _publish_stop(self) -> None:
        self._safe_velocity_publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationSafetyMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

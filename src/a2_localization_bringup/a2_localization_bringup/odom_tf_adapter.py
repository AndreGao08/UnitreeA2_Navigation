import copy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .odom_guard import pose_fault_reason
from .transforms import compose_transform, inverse_transform, rpy_quaternion


def _pose_tuple(pose):
    return ((pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


def _triple(text, parameter_name):
    values = tuple(float(value) for value in text.replace(',', ' ').split())
    if len(values) != 3:
        raise ValueError(f'{parameter_name} must contain exactly three numbers')
    return values


class OdomTfAdapter(Node):
    """Convert FAST-LIO's IMU pose into a standard odom -> base_link pose."""

    def __init__(self) -> None:
        super().__init__('a2_lio_odom_adapter')
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/a2/odometry')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('base_to_imu_xyz', [0.0, 0.0, 0.23])
        self.declare_parameter('base_to_imu_quat_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('base_to_imu_xyz_text', '')
        self.declare_parameter('base_to_imu_rpy_text', '')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_translation_step', 0.20)
        self.declare_parameter('max_rotation_step_deg', 20.0)

        self._world = self.get_parameter('world_frame').value
        self._base = self.get_parameter('base_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._max_translation_step = float(
            self.get_parameter('max_translation_step').value)
        self._max_rotation_step = math.radians(float(
            self.get_parameter('max_rotation_step_deg').value))
        self._previous_pose = None
        self._fault_latched = False
        xyz_text = self.get_parameter('base_to_imu_xyz_text').value.strip()
        rpy_text = self.get_parameter('base_to_imu_rpy_text').value.strip()
        t_bi = (_triple(xyz_text, 'base_to_imu_xyz_text') if xyz_text else
                tuple(float(v) for v in self.get_parameter('base_to_imu_xyz').value))
        q_bi = (rpy_quaternion(*_triple(rpy_text, 'base_to_imu_rpy_text')) if rpy_text else
                tuple(float(v) for v in self.get_parameter('base_to_imu_quat_xyzw').value))
        self._t_ib, self._q_ib = inverse_transform(t_bi, q_bi)
        self._publisher = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 20)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, '/a2/odometry/status', status_qos)
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self._callback, 20)

    def _callback(self, msg: Odometry) -> None:
        if self._fault_latched:
            return
        t_wi, q_wi = _pose_tuple(msg.pose.pose)
        t_wb, q_wb = compose_transform(t_wi, q_wi, self._t_ib, self._q_ib)
        current_pose = (t_wb, q_wb)
        fault_reason = pose_fault_reason(
            self._previous_pose, current_pose,
            self._max_translation_step, self._max_rotation_step)
        if fault_reason is not None:
            self._fault_latched = True
            self._publish_status(f'LOST: {fault_reason}')
            self.get_logger().error(
                f'FAST-LIO odometry guard latched: {fault_reason}; '
                'suppressing /a2/odometry and odom -> base_link')
            return
        self._previous_pose = current_pose
        self._publish_status('OK')
        output = copy.deepcopy(msg)
        output.header.frame_id = self._world
        output.child_frame_id = self._base
        (output.pose.pose.position.x,
         output.pose.pose.position.y,
         output.pose.pose.position.z) = t_wb
        (output.pose.pose.orientation.x, output.pose.pose.orientation.y,
         output.pose.pose.orientation.z, output.pose.pose.orientation.w) = q_wb
        self._publisher.publish(output)

        if self._publish_tf:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = self._base
            (transform.transform.translation.x,
             transform.transform.translation.y,
             transform.transform.translation.z) = t_wb
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = q_wb
            self._broadcaster.sendTransform(transform)

    def _publish_status(self, status: str) -> None:
        if getattr(self, '_last_status', None) == status:
            return
        self._last_status = status
        message = String()
        message.data = status
        self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

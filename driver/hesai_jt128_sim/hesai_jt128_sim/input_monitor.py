from collections import deque
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool


REQUIRED_FIELDS = {'x', 'y', 'z', 'intensity', 'ring', 'timestamp'}


class InputMonitor(Node):
    def __init__(self) -> None:
        super().__init__('jt128_input_monitor')
        self.declare_parameter('lidar_topic', '/lidar_points')
        self.declare_parameter('imu_topic', '/lidar_imu')
        self.declare_parameter('report_period', 2.0)
        self._lidar_stamps = deque(maxlen=30)
        self._imu_stamps = deque(maxlen=400)
        self._fields = set()
        self._last_acc_norm = float('nan')
        self._ok_pub = self.create_publisher(Bool, '/a2_localization/input_ok', 1)
        self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value,
            self._cloud_cb, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._imu_cb, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter('report_period').value), self._report)

    @staticmethod
    def _stamp(msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def _rate(stamps: deque) -> float:
        if len(stamps) < 2 or stamps[-1] <= stamps[0]:
            return 0.0
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._lidar_stamps.append(self._stamp(msg))
        self._fields = {field.name for field in msg.fields}

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_stamps.append(self._stamp(msg))
        a = msg.linear_acceleration
        self._last_acc_norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)

    def _report(self) -> None:
        missing = REQUIRED_FIELDS - self._fields
        lidar_rate = self._rate(self._lidar_stamps)
        imu_rate = self._rate(self._imu_stamps)
        monotonic = all(a < b for a, b in zip(self._lidar_stamps, list(self._lidar_stamps)[1:]))
        ok = not missing and lidar_rate > 1.0 and imu_rate > 20.0 and monotonic
        self._ok_pub.publish(Bool(data=ok))
        message = (
            f'input_ok={ok} lidar={lidar_rate:.1f}Hz imu={imu_rate:.1f}Hz '
            f'acc_norm={self._last_acc_norm:.3f}m/s^2 missing_fields={sorted(missing)}')
        # Keep each severity on a distinct call site. rclpy Humble rejects a
        # single call site whose severity changes between invocations.
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InputMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

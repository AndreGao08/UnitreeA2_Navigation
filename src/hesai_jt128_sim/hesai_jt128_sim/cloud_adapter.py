import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from .cloud_codec import convert_cloud


class CloudAdapter(Node):
    def __init__(self) -> None:
        super().__init__('jt128_cloud_adapter')
        self.declare_parameter('input_topic', '/a2/jt128/points')
        self.declare_parameter('output_topic', '/lidar_points')
        self.declare_parameter('output_frame', 'hesai_jt128_link')
        self.declare_parameter('scan_rate_hz', 10.0)
        self.declare_parameter('scan_lines', 128)
        self.declare_parameter('vertical_min_rad', -0.4363323129985824)
        self.declare_parameter('vertical_max_rad', 0.2617993877991494)
        self.declare_parameter('rolling_scan_timestamps', False)

        self._output_frame = self.get_parameter('output_frame').value
        self._scan_rate = float(self.get_parameter('scan_rate_hz').value)
        self._scan_lines = int(self.get_parameter('scan_lines').value)
        self._vertical_min = float(self.get_parameter('vertical_min_rad').value)
        self._vertical_max = float(self.get_parameter('vertical_max_rad').value)
        self._rolling_scan_timestamps = bool(
            self.get_parameter('rolling_scan_timestamps').value
        )
        self._seen = False
        self._errors = 0

        self._publisher = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self._callback, qos_profile_sensor_data)

    def _callback(self, msg: PointCloud2) -> None:
        try:
            output = convert_cloud(
                msg,
                output_frame=self._output_frame,
                scan_rate_hz=self._scan_rate,
                scan_lines=self._scan_lines,
                vertical_min_rad=self._vertical_min,
                vertical_max_rad=self._vertical_max,
                rolling_scan_timestamps=self._rolling_scan_timestamps,
            )
        except (ValueError, TypeError, KeyError) as exc:
            self._errors += 1
            if self._errors <= 5 or self._errors % 100 == 0:
                self.get_logger().error(f'cannot convert Gazebo cloud: {exc}')
            return

        if not self._seen:
            self._seen = True
            fields = ', '.join(field.name for field in output.fields)
            self.get_logger().info(
                f'publishing JT128 cloud with {output.width} points and fields [{fields}]')
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

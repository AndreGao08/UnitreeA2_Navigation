import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def gravity_in_sensor_frame(orientation, magnitude):
    """Rotate world +Z specific force into the sensor frame."""
    norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if norm < 1.0e-9:
        return 0.0, 0.0, magnitude
    x = orientation.x / norm
    y = orientation.y / norm
    z = orientation.z / norm
    w = orientation.w / norm
    return (
        magnitude * 2.0 * (x * z - w * y),
        magnitude * 2.0 * (y * z + w * x),
        magnitude * (1.0 - 2.0 * (x * x + y * y)),
    )


class ImuAdapter(Node):
    def __init__(self) -> None:
        super().__init__('jt128_imu_adapter')
        self.declare_parameter('input_topic', '/a2/imu_raw')
        self.declare_parameter('output_topic', '/lidar_imu')
        self.declare_parameter('output_frame', 'hesai_jt128_imu_link')
        self.declare_parameter('angular_velocity_variance', 1.0e-6)
        self.declare_parameter('linear_acceleration_variance', 4.0e-4)
        self.declare_parameter('add_gravity', False)
        self.declare_parameter('gravity_magnitude', 9.80665)

        self._frame = self.get_parameter('output_frame').value
        gyro_var = float(self.get_parameter('angular_velocity_variance').value)
        acc_var = float(self.get_parameter('linear_acceleration_variance').value)
        self._gyro_covariance = [gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var]
        self._acc_covariance = [acc_var, 0.0, 0.0, 0.0, acc_var, 0.0, 0.0, 0.0, acc_var]
        self._add_gravity = bool(self.get_parameter('add_gravity').value)
        self._gravity = float(self.get_parameter('gravity_magnitude').value)
        self._seen = False
        self._publisher = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            Imu, self.get_parameter('input_topic').value,
            self._callback, qos_profile_sensor_data)

    def _callback(self, msg: Imu) -> None:
        output = copy.deepcopy(msg)
        output.header.frame_id = self._frame
        output.angular_velocity_covariance = self._gyro_covariance
        output.linear_acceleration_covariance = self._acc_covariance
        if self._add_gravity:
            gx, gy, gz = gravity_in_sensor_frame(output.orientation, self._gravity)
            output.linear_acceleration.x += gx
            output.linear_acceleration.y += gy
            output.linear_acceleration.z += gz
        if not self._seen:
            self._seen = True
            gravity_note = ' with simulated gravity' if self._add_gravity else ''
            self.get_logger().info(
                f'publishing SI-unit IMU in frame {self._frame}{gravity_note}'
            )
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

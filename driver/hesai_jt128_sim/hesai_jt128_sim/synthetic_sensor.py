"""Deterministic sensor source for testing FAST-LIO without a simulator."""

import array
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField


class SyntheticSensor(Node):
    def __init__(self):
        super().__init__('jt128_synthetic_sensor')
        self.declare_parameter('horizontal_samples', 128)
        columns = int(self.get_parameter('horizontal_samples').value)
        self._cloud = self._make_room_cloud(128, columns)
        self._cloud_pub = self.create_publisher(
            PointCloud2, '/a2/jt128/points', qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(Imu, '/a2/imu_raw', qos_profile_sensor_data)
        self.create_timer(0.1, self._publish_cloud)
        self.create_timer(0.005, self._publish_imu)

    @staticmethod
    def _make_room_cloud(lines, columns):
        elevation = np.linspace(-0.4363323129985824, 0.2617993877991494, lines)
        azimuth = np.linspace(-math.pi, math.pi, columns, endpoint=False)
        el, az = np.meshgrid(elevation, azimuth, indexing='ij')
        dx = np.cos(el) * np.cos(az)
        dy = np.cos(el) * np.sin(az)
        dz = np.sin(el)

        candidates = []
        boundaries = ((dx, 5.0, -5.0), (dy, 4.0, -4.0), (dz, 2.0, -1.0))
        for component, positive, negative in boundaries:
            distance = np.full_like(component, np.inf)
            np.divide(positive, component, out=distance, where=component > 1.0e-6)
            np.divide(negative, component, out=distance, where=component < -1.0e-6)
            candidates.append(np.where(distance > 0.0, distance, np.inf))
        distance = np.minimum.reduce(candidates)

        dtype = np.dtype({
            'names': ['x', 'y', 'z', 'intensity'],
            'formats': ['<f4', '<f4', '<f4', '<f4'],
            'offsets': [0, 4, 8, 12],
            'itemsize': 16,
        })
        packed = np.empty(lines * columns, dtype=dtype)
        packed['x'] = (distance * dx).reshape(-1)
        packed['y'] = (distance * dy).reshape(-1)
        packed['z'] = (distance * dz).reshape(-1)
        packed['intensity'] = 100.0

        msg = PointCloud2()
        msg.header.frame_id = 'hesai_jt128_link'
        msg.height = lines
        msg.width = columns
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = columns * 16
        msg.is_dense = True
        msg.data = array.array('B', packed.tobytes())
        return msg

    def _publish_cloud(self):
        self._cloud.header.stamp = self.get_clock().now().to_msg()
        self._cloud_pub.publish(self._cloud)

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'hesai_jt128_imu_link'
        msg.orientation.w = 1.0
        msg.linear_acceleration.z = 9.80665
        self._imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

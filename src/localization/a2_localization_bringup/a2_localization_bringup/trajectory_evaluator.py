import csv
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .transforms import q_conjugate, q_multiply, quaternion_angle, relative_transform


def _pose_tuple(msg):
    pose = msg.pose.pose
    return ((pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


def _stamp(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class TrajectoryEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('a2_trajectory_evaluator')
        self.declare_parameter('estimate_topic', '/a2/odometry')
        self.declare_parameter('ground_truth_topic', '/a2/ground_truth/odom')
        self.declare_parameter('max_time_delta', 0.05)
        self.declare_parameter('output_directory', '/tmp/a2_localization_eval')
        self._max_dt = float(self.get_parameter('max_time_delta').value)
        self._output = Path(self.get_parameter('output_directory').value).expanduser()
        self._output.mkdir(parents=True, exist_ok=True)
        self._latest_gt = None
        self._estimate_origin = None
        self._gt_origin = None
        self._rows = []
        self._last_report_count = 0
        self.create_subscription(Odometry, self.get_parameter('ground_truth_topic').value, self._gt_cb, 50)
        self.create_subscription(Odometry, self.get_parameter('estimate_topic').value, self._estimate_cb, 50)
        self.create_timer(5.0, self._report)

    def _gt_cb(self, msg):
        self._latest_gt = msg

    def _estimate_cb(self, estimate):
        if self._latest_gt is None:
            return
        dt = abs(_stamp(estimate) - _stamp(self._latest_gt))
        if dt > self._max_dt:
            return
        estimate_pose = _pose_tuple(estimate)
        gt_pose = _pose_tuple(self._latest_gt)
        if self._estimate_origin is None:
            self._estimate_origin = estimate_pose
            self._gt_origin = gt_pose
        est_t, est_q = relative_transform(*self._estimate_origin, *estimate_pose)
        gt_t, gt_q = relative_transform(*self._gt_origin, *gt_pose)
        error = tuple(est_t[i] - gt_t[i] for i in range(3))
        trans_error = math.sqrt(sum(value * value for value in error))
        rot_error = quaternion_angle(q_multiply(q_conjugate(gt_q), est_q))
        self._rows.append((_stamp(estimate), *est_t, *gt_t, trans_error, rot_error, dt))

    def _summary(self):
        if not self._rows:
            return {'samples': 0}
        trans = [row[7] for row in self._rows]
        rot = [row[8] for row in self._rows]
        return {
            'samples': len(self._rows),
            'translation_rmse_m': math.sqrt(sum(v * v for v in trans) / len(trans)),
            'translation_mean_m': sum(trans) / len(trans),
            'translation_max_m': max(trans),
            'rotation_rmse_deg': math.degrees(math.sqrt(sum(v * v for v in rot) / len(rot))),
            'rotation_mean_deg': math.degrees(sum(rot) / len(rot)),
        }

    def _write(self):
        with (self._output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['stamp', 'est_x', 'est_y', 'est_z', 'gt_x', 'gt_y', 'gt_z',
                             'translation_error_m', 'rotation_error_rad', 'time_delta_s'])
            writer.writerows(self._rows)
        with (self._output / 'summary.json').open('w', encoding='utf-8') as stream:
            json.dump(self._summary(), stream, indent=2)

    def _report(self):
        summary = self._summary()
        if summary['samples'] == 0:
            self.get_logger().warn('waiting for synchronized estimate and ground-truth odometry')
            return
        if summary['samples'] != self._last_report_count:
            self._last_report_count = summary['samples']
            self.get_logger().info(
                f"evaluation samples={summary['samples']} translation RMSE="
                f"{summary['translation_rmse_m']:.3f}m rotation RMSE="
                f"{summary['rotation_rmse_deg']:.2f}deg")
            self._write()

    def destroy_node(self):
        self._write()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

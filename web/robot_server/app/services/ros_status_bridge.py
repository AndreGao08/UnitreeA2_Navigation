import threading
import math
import time
import base64
import json
import struct
import os
import re

import yaml

from robot_server.app.services.robot_profile import robot_profile_manager
from robot_server.app.services.status_store import status_store


class RosStatusBridge:
    POINTCLOUD_TOPICS = (
        "/livox/lidar",
        "/lidar_points",
        "/ground_segmentation/ground_points",
        "/ground_segmentation/obstacle_points",
        "/cur_scan_in_map",
        "/cloud_registered",
        "/cloud_registered_body",
        "/cloud_effected",
        "/Laser_map",
        "/cloud_pcd",
        "/submap",
        "/combined_scan",
        "/front_laser_scan",
        "/rear_laser_scan",
    )

    def __init__(self):
        self._thread = None
        self._stop_event = threading.Event()
        self._node = None
        self._tf_buffer = None
        self._rclpy = None
        self._waypoint_sequence_pub = None
        self._waypoint_condition = threading.Condition()
        self._frontend_command_pending = False
        self._route_cancel = threading.Event()
        self._route_thread = None
        self._goal_pub = None
        self._goal_action_client = None
        self._cancel_all_goals_client = None
        self._active_goal_handle = None
        self._motion = None
        self._goal_metrics = {"active": False, "distance_remaining": None, "target": None, "seen": None}
        self._last_goal_target = None
        self._waypoint_navigation_status = {"state": "idle", "busy": False, "seen": None}
        self._initial_pose_pub = None
        self._cmd_vel_pub = None
        self._cmd_vel_nav_pub = None
        self._tcp_outbound_pub = None
        self._velocity_stop_timer = None
        self._pending_initial_pose_message = None
        self._scan_lock = threading.Lock()
        self._scans = {}
        self._pointclouds = {}
        default_cloud = str(robot_profile_manager.runtime_config().get(
            "default_pointcloud_topic", "/cloud_registered"))
        self._pointcloud_topics = {default_cloud}
        self._pointcloud_seen = {}
        self._pointcloud_versions = {}
        self._pointcloud_subscriptions = {}
        self._livox_subscription = None
        self._pointcloud_subscription_update = threading.Event()
        self._cloud_condition = threading.Condition()
        self._cloud_pending = {}
        self._cloud_processed_at = {}
        self._cloud_worker = None
        self._source_rates = {}
        self._paths = {}
        self._costmaps = {}
        self._costmap_versions = {"local": 0, "global": 0}
        self._live_map = None
        self._trajectory = []
        self._trajectory_version = 0
        self._trajectory_frame = None
        self._mapping_pose = None
        self._trajectory_mode = None
        self._trajectory_recording = False
        self._localization_status = None
        self._localization_status_seen = None
        self._map_seen = None
        self._a2_relocalization_text = ""
        self._a2_odometry_text = ""
        self._battery_percent = None
        self._battery_current_amp = None
        self._battery_power_state = None
        self._battery_status_seen = None
        self._last_tf_pose_stamp = None
        self._live_map_version = 0
        self._graph_lock = threading.Lock()
        self._node_names = set()
        self._raw_livox_available = False
        self._livox_transform_mtime = None
        self._livox_transform_path = None
        self._livox_rotation = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        self._livox_translation = (0.0, 0.0, 0.0)
        self._livox_body_frame = "front_lidar"

    def _base_frame(self):
        return robot_profile_manager.frame("base_link", "base_link")

    @staticmethod
    def _rotation_matrix(rpy):
        roll, pitch, yaw = (float(value) for value in rpy)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    @staticmethod
    def _multiply_rotation(left, right):
        return tuple(
            tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
            for row in range(3)
        )

    def _refresh_livox_transform(self):
        """Mirror FAST-LIO's mount correction and LiDAR-to-IMU transform."""
        try:
            navigation_config = robot_profile_manager.navigation_config()
            mtime = os.stat(navigation_config).st_mtime_ns
            if str(navigation_config) == self._livox_transform_path and mtime == self._livox_transform_mtime:
                return
            with open(navigation_config, "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            fast_lio = config.get("fast_lio", {})
            mount = fast_lio.get("sensor_mount", {})
            lidar_to_imu = fast_lio.get("lidar_to_imu", {})
            mount_rotation = self._rotation_matrix(
                mount.get("rpy", [0.0, 0.0, 0.0])
                if mount.get("enabled", False)
                else [0.0, 0.0, 0.0]
            )
            extrinsic_rotation = self._rotation_matrix(
                lidar_to_imu.get("rpy", [0.0, 0.0, 0.0])
            )
            translation = tuple(float(value) for value in lidar_to_imu.get("xyz", [0.0, 0.0, 0.0]))
            if len(translation) != 3:
                raise ValueError("fast_lio.lidar_to_imu.xyz must contain three values")
            self._livox_rotation = self._multiply_rotation(extrinsic_rotation, mount_rotation)
            self._livox_translation = translation
            self._livox_body_frame = str(config.get("frames", {}).get("front_lidar", "front_lidar"))
            self._livox_transform_mtime = mtime
            self._livox_transform_path = str(navigation_config)
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            # Keep the last valid transform while a configuration file is
            # being edited or temporarily unavailable.
            return

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._cloud_worker = threading.Thread(target=self._cloud_worker_loop, daemon=True)
        self._cloud_worker.start()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        with self._cloud_condition:
            self._cloud_condition.notify_all()
        if self._velocity_stop_timer is not None:
            self._velocity_stop_timer.cancel()
        self.publish_velocity()
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass

    def cancel_navigation(self):
        self._route_cancel.set()
        route_result = self.publish_waypoint_sequence([-1])
        with self._waypoint_condition:
            if self._frontend_command_pending:
                self._waypoint_navigation_status = {
                    "state": "cancelled", "busy": False, "seen": time.monotonic()
                }
            self._frontend_command_pending = False
            self._waypoint_condition.notify_all()
        with self._scan_lock:
            self._finish_goal_metrics_locked()
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception as exc:
                return {"ok": False, "message": f"failed to cancel navigation: {exc}"}
        # A waypoint navigator or another ROS client may own the active goal,
        # in which case this bridge has no goal handle. A zero GoalInfo cancel
        # request asks the action server to cancel every active goal.
        if self._cancel_all_goals_client is not None:
            try:
                from action_msgs.srv import CancelGoal
                self._cancel_all_goals_client.call_async(CancelGoal.Request())
            except Exception as exc:
                return {"ok": False, "message": f"failed to cancel all navigation goals: {exc}"}
        return route_result

    def emergency_stop(self, hold_seconds=0.6):
        """Cancel every navigation source and hold both velocity stages at zero."""
        navigation = self.cancel_navigation()
        if self._cmd_vel_pub is None:
            return {"ok": False, "message": "ROS velocity publisher is not ready"}
        try:
            from geometry_msgs.msg import Twist
            zero = Twist()
            deadline = time.monotonic() + max(0.0, float(hold_seconds))
            while True:
                # /cmd_vel_nav clears the velocity smoother input while
                # /cmd_vel immediately reaches the chassis output stage.
                if self._cmd_vel_nav_pub is not None:
                    self._cmd_vel_nav_pub.publish(zero)
                self._cmd_vel_pub.publish(zero)
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.05)
            if self._velocity_stop_timer is not None:
                self._velocity_stop_timer.cancel()
                self._velocity_stop_timer = None
            return {"ok": True, "navigation": navigation, "hold_seconds": hold_seconds}
        except Exception as exc:
            return {"ok": False, "message": f"failed to publish emergency stop: {exc}"}

    def publish_waypoint_route(self, waypoint_indices, loop=False):
        indices = [int(index) for index in waypoint_indices]
        if not indices:
            return {"ok": False, "message": "waypoint route is empty"}
        try:
            from robot_server.app.services.waypoint_store import waypoint_store
            waypoints = waypoint_store.list_waypoints()
        except (OSError, TypeError, ValueError) as exc:
            return {"ok": False, "message": f"failed to load waypoints: {exc}"}
        if any(index < 0 or index >= len(waypoints) for index in indices):
            return {"ok": False, "message": "waypoint index is out of range"}
        if self._goal_action_client is None or not self._goal_action_client.server_is_ready():
            return {"ok": False, "message": "Nav2 NavigateToPose action server is not ready"}
        with self._waypoint_condition:
            if self._frontend_command_pending or (
                self._route_thread is not None and self._route_thread.is_alive()
            ):
                return {"ok": False, "message": "已有前端导航命令正在执行"}
            self._route_cancel.clear()
            self._frontend_command_pending = True
            self._waypoint_navigation_status = {
                "state": "submitted", "busy": True, "seen": time.monotonic()
            }
            route = [dict(waypoints[index]) for index in indices]
            self._route_thread = threading.Thread(
                target=self._run_waypoint_route,
                args=(route, bool(loop)),
                daemon=True,
            )
            self._route_thread.start()
        return {
            "ok": True,
            "message": "waypoint route submitted to Nav2",
            "waypoint_indices": indices,
            "loop": bool(loop),
        }

    def _run_waypoint_route(self, route, loop):
        final_state = "finished"
        try:
            while not self._route_cancel.is_set():
                for waypoint in route:
                    if self._route_cancel.is_set():
                        final_state = "cancelled"
                        break
                    with self._waypoint_condition:
                        self._waypoint_navigation_status = {
                            "state": f"navigating:{waypoint['index']}",
                            "busy": True,
                            "seen": time.monotonic(),
                        }
                        self._waypoint_condition.notify_all()
                    result = self.navigate_and_wait(
                        waypoint["x"], waypoint["y"], waypoint["yaw"],
                        self._route_cancel,
                    )
                    if not result.get("ok"):
                        final_state = (
                            "cancelled" if self._route_cancel.is_set() else "failed"
                        )
                        break
                if final_state != "finished" or not loop:
                    break
        except Exception:
            final_state = "failed"
        finally:
            with self._waypoint_condition:
                self._frontend_command_pending = False
                self._waypoint_navigation_status = {
                    "state": final_state, "busy": False, "seen": time.monotonic()
                }
                self._waypoint_condition.notify_all()

    def publish_waypoint_sequence(self, data):
        if self._waypoint_sequence_pub is None:
            return {
                "ok": False,
                "message": "ROS command publisher is not ready",
            }

        is_cancel = list(data) == [-1]
        with self._waypoint_condition:
            if not is_cancel and self._frontend_command_pending:
                return {"ok": False, "message": "已有前端导航命令正在等待执行"}
            if not is_cancel:
                self._frontend_command_pending = True
                self._waypoint_navigation_status = {
                    "state": "submitted", "busy": True, "seen": time.monotonic()
                }
        try:
            from std_msgs.msg import Int32MultiArray

            self._waypoint_sequence_pub.publish(Int32MultiArray(data=data))
        except Exception as exc:
            with self._waypoint_condition:
                if not is_cancel:
                    self._frontend_command_pending = False
            return {
                "ok": False,
                "message": f"failed to publish waypoint sequence: {exc}",
            }

        return {
            "ok": True,
            "message": "published waypoint sequence",
            "data": data,
        }

    def navigate_waypoint_and_wait(self, waypoint_index, cancel_event, timeout=1800.0):
        try:
            from robot_server.app.services.waypoint_store import waypoint_store
            waypoint = waypoint_store.list_waypoints()[int(waypoint_index)]
        except (IndexError, OSError, TypeError, ValueError) as exc:
            return {"ok": False, "message": f"failed to load waypoint: {exc}"}
        return self.navigate_and_wait(
            waypoint["x"], waypoint["y"], waypoint["yaw"], cancel_event, timeout
        )

    def wait_for_waypoint_subscriber(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (
                self._waypoint_sequence_pub is not None
                and self._waypoint_sequence_pub.get_subscription_count() > 0
            ):
                return True
            time.sleep(0.1)
        return False

    def publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        values = [float(linear_x), float(linear_y), float(angular_z)]
        if self._cmd_vel_pub is None or not all(math.isfinite(value) for value in values):
            return {"ok": False, "message": "ROS velocity publisher is not ready"}
        limits = (0.5, 0.3, 0.6)
        values = [max(-limit, min(limit, value)) for value, limit in zip(values, limits)]
        if any(abs(value) > 1e-6 for value in values) and not self.velocity_ready_snapshot()["ready"]:
            return {"ok": False, "message": "底盘速度接口未连接，请启动底盘后重试"}
        try:
            from geometry_msgs.msg import Twist
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = values
            self._cmd_vel_pub.publish(message)
            if self._velocity_stop_timer is not None:
                self._velocity_stop_timer.cancel()
                self._velocity_stop_timer = None
            if any(abs(value) > 1e-6 for value in values):
                self._velocity_stop_timer = threading.Timer(
                    0.45, self.publish_velocity
                )
                self._velocity_stop_timer.daemon = True
                self._velocity_stop_timer.start()
            return {"ok": True, "linear_x": values[0], "linear_y": values[1], "angular_z": values[2]}
        except Exception as exc:
            return {"ok": False, "message": f"failed to publish velocity: {exc}"}

    def velocity_ready_snapshot(self):
        if self._cmd_vel_pub is None:
            return {"ready": False, "external_subscribers": 0}
        # The bridge also subscribes to /cmd_vel for telemetry; exclude that
        # local subscription when deciding whether a chassis is connected.
        external = max(0, int(self._cmd_vel_pub.get_subscription_count()) - 1)
        return {"ready": external > 0, "external_subscribers": external}

    def publish_tcp_message(self, text):
        message = str(text or "").strip()
        if not message:
            return {"ok": True, "sent": False}
        if self._tcp_outbound_pub is None:
            return {"ok": False, "message": "TCP outbound ROS publisher is unavailable"}
        from std_msgs.msg import String
        self._tcp_outbound_pub.publish(String(data=message))
        return {"ok": True, "sent": True}

    def publish_goal(self, x, y, yaw):
        values = [float(x), float(y), float(yaw)]
        if self._goal_action_client is None or not all(math.isfinite(value) for value in values):
            return {"ok": False, "message": "Nav2 action client is not ready or pose is invalid"}
        if not self._goal_action_client.server_is_ready():
            return {"ok": False, "message": "Nav2 NavigateToPose action server is not ready"}
        try:
            from nav2_msgs.action import NavigateToPose
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = values[:2]
            goal.pose.pose.orientation.z = math.sin(values[2] / 2.0)
            goal.pose.pose.orientation.w = math.cos(values[2] / 2.0)
            with self._scan_lock:
                self._last_goal_target = list(values)
                self._goal_metrics = {"active": True, "distance_remaining": None, "target": list(self._last_goal_target), "seen": time.monotonic()}
            future = self._goal_action_client.send_goal_async(
                goal, feedback_callback=self._goal_feedback_callback
            )
            future.add_done_callback(self._goal_response_callback)
            return {"ok": True, "message": "navigation goal submitted"}
        except Exception as exc:
            return {"ok": False, "message": f"failed to send navigation goal: {exc}"}

    def navigate_and_wait(self, x, y, yaw, cancel_event, timeout=1800.0):
        values = [float(x), float(y), float(yaw)]
        client = self._goal_action_client
        if client is None or not all(math.isfinite(value) for value in values):
            return {"ok": False, "message": "Nav2 action client is unavailable"}
        if not client.wait_for_server(timeout_sec=2.0):
            return {"ok": False, "message": "Nav2 NavigateToPose action server is unavailable"}
        from nav2_msgs.action import NavigateToPose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = values[:2]
        goal.pose.pose.orientation.z = math.sin(values[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(values[2] / 2.0)
        with self._scan_lock:
            self._last_goal_target = list(values)
            self._goal_metrics = {"active": True, "distance_remaining": None, "target": list(self._last_goal_target), "seen": time.monotonic()}
        goal_future = client.send_goal_async(
            goal, feedback_callback=self._goal_feedback_callback
        )
        deadline = time.monotonic() + timeout
        while not goal_future.done():
            if cancel_event.wait(0.05) or time.monotonic() >= deadline:
                return {"ok": False, "message": "navigation cancelled or timed out"}
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            return {"ok": False, "message": "navigation goal was rejected"}
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if cancel_event.wait(0.05):
                goal_handle.cancel_goal_async()
                return {"ok": False, "message": "navigation cancelled"}
            if time.monotonic() >= deadline:
                goal_handle.cancel_goal_async()
                return {"ok": False, "message": "navigation timed out"}
        wrapped_result = result_future.result()
        self._active_goal_handle = None
        succeeded = wrapped_result.status == 4
        with self._scan_lock:
            self._finish_goal_metrics_locked()
        if not succeeded:
            return {"ok": False, "message": f"navigation finished with status {wrapped_result.status}"}
        return {"ok": True, "message": "waypoint reached"}

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self._active_goal_handle = goal_handle
                goal_handle.get_result_async().add_done_callback(self._goal_result_callback)
            else:
                with self._scan_lock:
                    self._finish_goal_metrics_locked()
        except Exception:
            self._active_goal_handle = None

    def _goal_feedback_callback(self, message):
        try:
            distance = float(message.feedback.distance_remaining)
            with self._scan_lock:
                target = self._last_goal_target
                if target is None:
                    return
                self._goal_metrics = {
                    "active": True,
                    "distance_remaining": round(distance, 3) if math.isfinite(distance) else None,
                    "target": list(target) if target is not None else None,
                    "seen": time.monotonic(),
                }
        except Exception:
            pass

    def _finish_goal_metrics_locked(self):
        """Finish a goal while retaining its latest telemetry until reset/replaced."""
        current = self._goal_metrics
        target = current.get("target") or self._last_goal_target
        distance = current.get("distance_remaining")
        self._last_goal_target = None
        self._goal_metrics = {
            "active": False,
            "distance_remaining": distance,
            "target": list(target) if target is not None else None,
            "seen": time.monotonic(),
        }

    def _goal_pose_callback(self, message):
        pose = message.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z),
        )
        target = [float(pose.position.x), float(pose.position.y), float(yaw)]
        with self._scan_lock:
            self._last_goal_target = target
            self._goal_metrics = {
                "active": True,
                "distance_remaining": None,
                "target": list(target),
                "seen": time.monotonic(),
            }

    def _waypoint_status_callback(self, message):
        state = str(message.data).strip().lower()
        with self._waypoint_condition:
            self._waypoint_navigation_status = {
                "state": state,
                "busy": state == "accepted" or state.startswith("navigating:"),
                "seen": time.monotonic(),
            }
            if state in {"finished", "failed", "cancelled"} or state.startswith("rejected:"):
                self._frontend_command_pending = False
            self._waypoint_condition.notify_all()
        with self._scan_lock:
            if state not in {"finished", "failed", "cancelled"}:
                return
            self._finish_goal_metrics_locked()

    def waypoint_navigation_snapshot(self):
        with self._waypoint_condition:
            result = dict(self._waypoint_navigation_status)
        seen = result.pop("seen", None)
        result["recent"] = seen is not None and time.monotonic() - seen <= 3.0
        if not result["recent"]:
            result["busy"] = False
        return result

    def navigation_command_busy(self):
        waypoint = self.waypoint_navigation_snapshot()
        with self._waypoint_condition:
            frontend_pending = self._frontend_command_pending
        with self._scan_lock:
            goal_active = bool(self._goal_metrics.get("active"))
        return bool(frontend_pending or waypoint.get("busy") or goal_active)

    def _goal_result_callback(self, future):
        self._active_goal_handle = None
        try:
            future.result()
        except Exception:
            pass
        with self._scan_lock:
            self._finish_goal_metrics_locked()

    def _goal_status_callback(self, message):
        """Use the Nav2 action status stream as a result-callback fallback."""
        statuses = getattr(message, "status_list", ())
        if not statuses or int(statuses[-1].status) not in {4, 5, 6}:
            return
        self._active_goal_handle = None
        with self._scan_lock:
            self._finish_goal_metrics_locked()

    def motion_snapshot(self):
        with self._scan_lock:
            now = time.monotonic()
            motion = dict(self._motion) if self._motion else None
            if motion is not None:
                motion["recent"] = now - motion["seen"] <= 1.0
            goal = dict(self._goal_metrics)
            if goal.get("active") and goal.get("seen") is not None and now - goal["seen"] > 5.0:
                goal = {"active": False, "distance_remaining": None, "target": None, "seen": goal["seen"]}
            goal.pop("seen", None)
            return {
                "motion": motion,
                "goal": goal,
            }

    def publish_initial_pose(self, x, y, yaw):
        return self._publish_pose(self._initial_pose_pub, x, y, yaw, initial=True)

    def _publish_pose(self, publisher, x, y, yaw, initial=False):
        values = [float(x), float(y), float(yaw)]
        if publisher is None or not all(math.isfinite(value) for value in values):
            return {"ok": False, "message": "ROS publisher is not ready or pose is invalid"}
        try:
            from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
            message = PoseWithCovarianceStamped() if initial else PoseStamped()
            message.header.frame_id = "map"
            message.header.stamp = self._node.get_clock().now().to_msg()
            pose = message.pose.pose if initial else message.pose
            pose.position.x, pose.position.y = values[:2]
            pose.orientation.z = math.sin(values[2] / 2.0)
            pose.orientation.w = math.cos(values[2] / 2.0)
            if initial:
                message.pose.covariance[0] = 0.25
                message.pose.covariance[7] = 0.25
                message.pose.covariance[35] = 0.0685
            publisher.publish(message)
            subscriber_count = publisher.get_subscription_count()
            if initial:
                self._pending_initial_pose_message = message if subscriber_count == 0 else None
            return {
                "ok": True,
                "message": (
                    "pose published"
                    if subscriber_count
                    else "initial pose queued while global localization starts"
                ),
                "subscriber_count": subscriber_count,
            }
        except Exception as exc:
            return {"ok": False, "message": f"failed to publish pose: {exc}"}

    def scan_snapshot(self):
        with self._scan_lock:
            now = time.monotonic()
            return {
                name: {**scan, "recent": now - scan["seen"] <= 2.0,
                       "frequency_hz": self._frequency_hz(f"scan:{name}")}
                for name, scan in self._scans.items()
            }

    def scan_health_snapshot(self):
        with self._scan_lock:
            now = time.monotonic()
            return {
                name: {
                    "recent": now - scan["seen"] <= 2.0,
                    "frequency_hz": self._frequency_hz(f"scan:{name}"),
                    "version": scan.get("version", 0),
                }
                for name, scan in self._scans.items()
            }

    def pointcloud_snapshot(self):
        with self._scan_lock:
            now = time.monotonic()
            return {
                topic: {**cloud, "recent": now - cloud["seen"] <= 2.0,
                        "frequency_hz": self._frequency_hz(f"cloud:{topic}"),
                        "refresh_frequency_hz": self._frequency_hz(f"processed_cloud:{topic}")}
                for topic, cloud in self._pointclouds.items()
                if topic in self._pointcloud_topics
            }

    def pointcloud_topics_snapshot(self):
        with self._scan_lock:
            return {
                "selected": sorted(self._pointcloud_topics),
                "topics": list(self.POINTCLOUD_TOPICS),
            }

    def pointcloud_health_snapshot(self):
        with self._scan_lock:
            now = time.monotonic()
            result = {
                topic: {"recent": now - seen <= 2.0, "age": round(now - seen, 3),
                        "frequency_hz": self._frequency_hz(f"cloud:{topic}")}
                for topic, seen in self._pointcloud_seen.items()
                if topic != "/livox/lidar"
            }
        # Do not subscribe to the large Livox CustomMsg merely for health
        # reporting. rclpy would still deserialize every frame and contend
        # with visualization callbacks. DDS graph discovery is independent
        # from browser layer selection and adds no point-cloud data path.
        with self._graph_lock:
            raw_livox_available = self._raw_livox_available
        result["/livox/lidar"] = {
            "recent": raw_livox_available,
            "age": 0.0 if raw_livox_available else None,
            "frequency_hz": 0.0,
        }
        return result

    def select_pointcloud_topic(self, topic):
        return self.select_pointcloud_topics([topic])

    def select_pointcloud_topics(self, topics):
        selected = {str(topic) for topic in topics}
        unsupported = selected.difference(self.POINTCLOUD_TOPICS)
        if unsupported:
            raise ValueError(f"不支持点云话题: {', '.join(sorted(unsupported))}")
        with self._scan_lock:
            self._pointcloud_topics = selected
            self._pointclouds = {
                topic: cloud for topic, cloud in self._pointclouds.items() if topic in selected
            }
        self._pointcloud_subscription_update.set()
        return self.pointcloud_topics_snapshot()

    def _desired_pointcloud_sources(self):
        """Return physical PointCloud2 subscriptions needed by visible layers."""
        with self._scan_lock:
            selected = set(self._pointcloud_topics)
        sources = selected.difference({"/livox/lidar", "/combined_scan", "/front_laser_scan", "/rear_laser_scan"})
        return sources

    def costmap_snapshot(self, known_versions=None):
        known_versions = known_versions or {}
        with self._scan_lock:
            now = time.monotonic()
            result = {}
            for name, grid in self._costmaps.items():
                item = {**grid, "recent": now - grid["seen"] <= 3.0,
                        "frequency_hz": self._frequency_hz(f"costmap:{name}")}
                if known_versions.get(name) == grid.get("version"):
                    item.pop("data_base64", None)
                    item["unchanged"] = True
                result[name] = item
            return result

    def mapping_snapshot(self, known_live_map_version=None, known_trajectory_version=None):
        with self._scan_lock:
            live_map = None
            if self._live_map is not None:
                live_map = {
                    **self._live_map,
                    "recent": time.monotonic() - self._live_map["seen"] <= 3.0,
                }
                if known_live_map_version == self._live_map.get("version"):
                    live_map.pop("data_base64", None)
                    live_map["unchanged"] = True
            result = {
                "live_map": live_map,
                "trajectory_version": self._trajectory_version,
                "pose": dict(self._mapping_pose) if self._mapping_pose else None,
                "trajectory_frequency_hz": self._frequency_hz("odometry"),
                "trajectory_recording": self._trajectory_recording,
                "trajectory_mode": self._trajectory_mode,
                "localization_status": dict(self._localization_status)
                if self._localization_status else None,
                "localization_status_recent": bool(
                    self._localization_status_seen is not None
                    and time.monotonic() - self._localization_status_seen <= 3.0
                ),
            }
            if known_trajectory_version != self._trajectory_version:
                result["trajectory"] = list(self._trajectory)
            return result

    def pose_snapshot(self):
        with self._scan_lock:
            return dict(self._mapping_pose) if self._mapping_pose else None

    def display_pose_snapshot(self):
        """Choose the pose source for the currently running operating mode."""
        with self._graph_lock:
            node_names = {name.rsplit("/", 1)[-1] for name in self._node_names}
        navigation_active = bool(
            {"global_localization", "controller_server", "bt_navigator"}
            & node_names
        )
        with self._scan_lock:
            mapping_active = self._trajectory_mode == "mapping"
        if mapping_active and not navigation_active:
            # A previous localization session can remain queryable in the TF
            # buffer after its publishers stop.  While mapping, never expose
            # that stale map -> base_link transform as the current pose.
            return self.pose_snapshot()
        return self.tf_pose_snapshot()

    def tf_pose_snapshot(self):
        """Return the same fixed-frame pose RViz uses, without mixing odometry."""
        if self._node is None or self._rclpy is None:
            return None
        source = self._base_frame()
        for target in ("map", "odom"):
            try:
                transform = self._node.tf_buffer.lookup_transform(
                    target, source, self._rclpy.time.Time()
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                )
                stamp = transform.header.stamp
                stamp_key = (int(stamp.sec), int(stamp.nanosec))
                with self._scan_lock:
                    if stamp_key != self._last_tf_pose_stamp:
                        self._record_frequency("tf_pose")
                        self._last_tf_pose_stamp = stamp_key
                    # The continuously changing robot pose is driven by the
                    # directly observed FAST-LIO odometry stream.  HTTP/TF
                    # sampling aliases badly when both run near 10 Hz.
                    frequency_hz = self._frequency_hz("odometry")
                return {
                    "x": round(float(translation.x), 4),
                    "y": round(float(translation.y), 4),
                    "z": round(float(translation.z), 4),
                    "yaw": round(float(yaw), 5),
                    "ok": True,
                    "frame_id": target,
                    "child_frame_id": source,
                    "stamp_ns": stamp_key[0] * 1_000_000_000 + stamp_key[1],
                    "frequency_hz": frequency_hz,
                }
            except Exception:
                continue
        return None

    def tf_tree_snapshot(self):
        """Return the current TF graph with every frame expressed in one root."""
        if self._tf_buffer is None or self._rclpy is None:
            return {"fixed_frame": None, "frames": []}
        try:
            graph = yaml.safe_load(self._tf_buffer.all_frames_as_yaml()) or {}
        except Exception:
            return {"fixed_frame": None, "frames": []}
        if not isinstance(graph, dict):
            return {"fixed_frame": None, "frames": []}

        normalized_graph = {}
        valid_parents = {}
        current_seconds = (
            self._node.get_clock().now().nanoseconds / 1_000_000_000.0
            if self._node is not None else time.time()
        )
        for child, metadata in graph.items():
            child = str(child).lstrip("/")
            if not child:
                continue
            metadata = metadata if isinstance(metadata, dict) else {}
            normalized_graph[child] = metadata
            parent = str(metadata.get("parent") or "").strip().lstrip("/")
            if not parent or parent == "NO_PARENT":
                continue
            most_recent = float(metadata.get("most_recent_transform") or 0.0)
            buffer_length = float(metadata.get("buffer_length") or 0.0)
            is_static = most_recent == 0.0 and buffer_length == 0.0
            age_seconds = current_seconds - most_recent
            # Simulation clocks and TF metadata can use different epochs while
            # Gazebo is starting. Keep canonical navigation edges visible;
            # otherwise the UI falls back to a leg link such as FL_calf.
            if is_static or (-0.5 <= age_seconds <= 2.0) or parent in {"map", "odom"}:
                valid_parents[child] = parent

        # Build the visible graph exclusively from current edges. In
        # particular, an old navigation map->odom edge must not keep the map
        # frame alive in the TF widget after mapping has switched to odom.
        parents = valid_parents
        all_frames = set(parents)
        all_frames.update(parents.values())
        if not all_frames:
            return {"fixed_frame": None, "frames": []}

        roots = sorted(all_frames - set(parents))
        with self._graph_lock:
            node_names = {name.rsplit("/", 1)[-1] for name in self._node_names}
        navigation_active = bool(
            {"global_localization", "controller_server", "bt_navigator"}
            & node_names
        )
        with self._scan_lock:
            mapping_active = self._trajectory_mode == "mapping" and not navigation_active
        preferred_fixed_frames = ("odom", "map") if mapping_active else ("map", "odom")
        fixed_frame = next(
            (name for name in preferred_fixed_frames if name in all_frames),
            roots[0] if roots else sorted(all_frames)[0],
        )
        frames = []
        for name in sorted(all_frames):
            metadata = normalized_graph.get(name, {})
            parent = parents.get(name)
            if name == fixed_frame:
                translation = [0.0, 0.0, 0.0]
                rotation = [0.0, 0.0, 0.0, 1.0]
            else:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        fixed_frame, name, self._rclpy.time.Time()
                    ).transform
                    translation = [
                        round(float(transform.translation.x), 5),
                        round(float(transform.translation.y), 5),
                        round(float(transform.translation.z), 5),
                    ]
                    rotation = [
                        round(float(transform.rotation.x), 6),
                        round(float(transform.rotation.y), 6),
                        round(float(transform.rotation.z), 6),
                        round(float(transform.rotation.w), 6),
                    ]
                except Exception:
                    # A disconnected branch cannot be placed on the selected
                    # canvas root, so retain it in the relation list only.
                    translation = None
                    rotation = None
            relative_translation = None
            relative_rotation = None
            rpy = None
            if parent:
                try:
                    relative = self._tf_buffer.lookup_transform(
                        parent, name, self._rclpy.time.Time()
                    ).transform
                    relative_translation = [
                        round(float(relative.translation.x), 5),
                        round(float(relative.translation.y), 5),
                        round(float(relative.translation.z), 5),
                    ]
                    relative_rotation = [
                        round(float(relative.rotation.x), 6),
                        round(float(relative.rotation.y), 6),
                        round(float(relative.rotation.z), 6),
                        round(float(relative.rotation.w), 6),
                    ]
                    x, y, z, w = relative_rotation
                    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                    pitch_term = max(-1.0, min(1.0, 2 * (w * y - z * x)))
                    pitch = math.asin(pitch_term)
                    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                    rpy = [round(roll, 5), round(pitch, 5), round(yaw, 5)]
                except Exception:
                    pass
            most_recent = float(metadata.get("most_recent_transform") or 0.0)
            buffer_length = float(metadata.get("buffer_length") or 0.0)
            frames.append({
                "name": name,
                "parent": parent,
                "translation": translation,
                "rotation": rotation,
                "relative_translation": relative_translation,
                "relative_rotation": relative_rotation,
                "rpy": rpy,
                "static": bool(parent and most_recent == 0.0 and buffer_length == 0.0),
                "rate_hz": round(float(metadata.get("rate") or 0.0), 2),
            })
        return {"fixed_frame": fixed_frame, "frames": frames}

    def path_snapshot(self):
        with self._scan_lock:
            return {
                topic: {**path, "frequency_hz": self._frequency_hz(f"path:{topic}")}
                for topic, path in self._paths.items()
            }

    def _record_frequency(self, key, now=None):
        """Track callback frequency with a small EMA; caller holds _scan_lock."""
        now = time.monotonic() if now is None else now
        state = self._source_rates.get(key)
        if state is None:
            self._source_rates[key] = {"last": now, "hz": 0.0}
            return
        elapsed = now - state["last"]
        state["last"] = now
        if elapsed <= 0.0 or elapsed > 5.0:
            state["hz"] = 0.0
            return
        sample = min(1000.0, 1.0 / elapsed)
        state["hz"] = sample if state["hz"] <= 0.0 else state["hz"] * 0.8 + sample * 0.2

    def _frequency_hz(self, key):
        state = self._source_rates.get(key)
        if not state or time.monotonic() - state["last"] > 2.0:
            return 0.0
        return round(state["hz"], 1)

    def reset_trajectory(self, mode=None):
        with self._scan_lock:
            self._trajectory = []
            self._trajectory_version += 1
            self._trajectory_frame = None
            self._live_map = None
            self._mapping_pose = None
            self._trajectory_mode = mode
            self._trajectory_recording = False
            self._localization_status = None
            self._localization_status_seen = None
            self._a2_relocalization_text = ""
            self._a2_odometry_text = ""

    def reset_navigation_visuals(self):
        """Drop canvas data owned by the previous Nav2 session."""
        self._route_cancel.set()
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception:
                pass
        with self._waypoint_condition:
            self._frontend_command_pending = False
            self._waypoint_navigation_status = {
                "state": "idle", "busy": False, "seen": None
            }
            self._waypoint_condition.notify_all()
        with self._scan_lock:
            self._localization_status = None
            self._localization_status_seen = None
            self._a2_relocalization_text = ""
            self._a2_odometry_text = ""
            self._paths = {}
            self._costmaps = {}
            self._costmap_versions = {
                "local": self._costmap_versions.get("local", 0) + 1,
                "global": self._costmap_versions.get("global", 0) + 1,
            }
            self._last_goal_target = None
            self._goal_metrics = {
                "active": False, "distance_remaining": None,
                "target": None, "seen": None,
            }

    def _localization_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                return
        except (TypeError, ValueError):
            return
        with self._scan_lock:
            self._localization_status = status
            self._localization_status_seen = time.monotonic()

    def _a2_localization_status_callback(self, source, msg):
        text = str(getattr(msg, "data", "")).strip()
        now = time.monotonic()
        with self._scan_lock:
            if source == "relocalization":
                self._a2_relocalization_text = text
            else:
                self._a2_odometry_text = text
            relocalized = self._a2_relocalization_text.upper().startswith("LOCALIZED")
            odometry_ok = not self._a2_odometry_text or self._a2_odometry_text.upper().startswith("OK")
            self._localization_status = {
                "stable": bool(relocalized and odometry_ok),
                "state": self._a2_relocalization_text or "WAITING",
                "odometry": self._a2_odometry_text or "WAITING",
            }
            self._localization_status_seen = now

    def _map_status_callback(self, _msg):
        with self._scan_lock:
            self._map_seen = time.monotonic()

    def _refresh_runtime_status(self):
        """Synthesize the status document expected by the migrated frontend."""
        now = time.monotonic()
        pose = self.tf_pose_snapshot()
        with self._scan_lock:
            map_received = self._map_seen is not None
            localization = dict(self._localization_status) if self._localization_status else None
            localization_recent = (
                self._localization_status_seen is not None
                and now - self._localization_status_seen <= 3.0
            )
            lidar_recent = any(
                now - seen <= 2.0
                for topic, seen in self._pointcloud_seen.items()
                if topic in {"/lidar_points", "/cloud_registered", "/Laser_map"}
            )
            # The A2 bridge is authoritative when available. A legacy
            # /localization/status publisher may still report LOST while the
            # FAST-LIO relocalization topic already reports LOCALIZED.
            if self._a2_relocalization_text:
                relocalized = self._a2_relocalization_text.upper().startswith("LOCALIZED")
                odometry_ok = (
                    not self._a2_odometry_text
                    or self._a2_odometry_text.upper().startswith("OK")
                )
                localization = {
                    "stable": bool(relocalized and odometry_ok),
                    "state": self._a2_relocalization_text,
                    "odometry": self._a2_odometry_text or "WAITING",
                }
                localization_recent = True
        status_store.update({
            "connected": True,
            "message": "Unitree A2 ROS 2 bridge online",
            "tf": {
                "ok": pose is not None,
                "errors": [] if pose is not None else ["waiting for map/odom -> base_link"],
            },
            # /map is transient-local and may legitimately be published only
            # once. Keep it loaded after the first valid OccupancyGrid arrives.
            "map": {"received": map_received, "recent": map_received},
            "sensors": {
                "front_scan": lidar_recent,
                "rear_scan": False,
                "combined_scan": lidar_recent,
            },
            "localization": {
                "stable": bool(localization and localization.get("stable") and localization_recent),
                "state": localization.get("state") if localization else "WAITING",
            },
            "navigation": {"status_recent": True, "goal_statuses": []},
        })

    def _battery_status_callback(self, msg):
        """Extract SOC and signed current from the battery status string."""
        data = str(getattr(msg, "data", ""))
        soc_match = re.search(
            r"(?:^|[,，])\s*SOC\s*:\s*"
            r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)",
            data,
            flags=re.IGNORECASE,
        )
        if soc_match is None:
            return
        try:
            soc = float(soc_match.group(1))
        except (TypeError, ValueError):
            return
        if not math.isfinite(soc) or not 0.0 <= soc <= 1.0:
            return
        current_match = re.search(
            r"(?:^|[,，])\s*电流\s*:\s*"
            r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*A?",
            data,
            flags=re.IGNORECASE,
        )
        current_amp = None
        if current_match is not None:
            try:
                parsed_current = float(current_match.group(1))
                if math.isfinite(parsed_current):
                    current_amp = round(parsed_current, 2)
            except (TypeError, ValueError):
                pass
        power_state = None
        if current_amp is not None:
            if current_amp > 0.05:
                power_state = "charging"
            elif current_amp < -0.05:
                power_state = "discharging"
            else:
                power_state = "idle"
        with self._scan_lock:
            self._battery_percent = round(soc * 100.0, 1)
            self._battery_current_amp = current_amp
            self._battery_power_state = power_state
            self._battery_status_seen = time.monotonic()

    def battery_snapshot(self):
        with self._scan_lock:
            percent = self._battery_percent
            current_amp = self._battery_current_amp
            power_state = self._battery_power_state
            seen = self._battery_status_seen
        age = None if seen is None else max(0.0, time.monotonic() - seen)
        return {
            "received": seen is not None,
            "recent": age is not None and age <= 15.0,
            "percent": percent,
            "current_amp": current_amp,
            "power_state": power_state,
            "age_seconds": None if age is None else round(age, 1),
        }

    def graph_snapshot(self):
        with self._graph_lock:
            return sorted(self._node_names)

    def _scan_callback(self, name, msg):
        total = len(msg.ranges)
        points = []
        cartesian_points = []
        # LaserScan is already compact enough for the browser. Keep every
        # valid beam so walls do not become a sparse dotted outline.
        for index in range(total):
            distance = float(msg.ranges[index])
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                angle = msg.angle_min + index * msg.angle_increment
                points.append([round(angle, 4), round(distance, 3)])
                cartesian_points.append([
                    math.cos(angle) * distance,
                    math.sin(angle) * distance,
                    0.0,
                ])
        fixed_points, fixed_frame, transformed = self._transform_points(
            cartesian_points, msg.header.frame_id
        )
        if transformed:
            # Apply the visualization lift in the fixed frame.  The rear
            # The rear laser may be mounted with roll=pi, so lifting in its frame before
            # TF conversion would flip the points below the map plane.
            fixed_points = [[x, y, round(z + 0.08, 3)] for x, y, z in fixed_points]
        with self._scan_lock:
            self._record_frequency(f"scan:{name}")
            version = int(self._scans.get(name, {}).get("version", 0)) + 1
            self._scans[name] = {
                "frame_id": msg.header.frame_id,
                "points": points,
                "fixed_points": fixed_points if transformed else [],
                "fixed_frame": fixed_frame,
                "transformed_to_fixed": transformed,
                "seen": time.monotonic(),
                "version": version,
            }

    def _livox_callback(self, msg):
        with self._scan_lock:
            self._record_frequency("cloud:/livox/lidar")
            self._pointcloud_seen["/livox/lidar"] = time.monotonic()
            if "/livox/lidar" not in self._pointcloud_topics:
                return
        total = len(msg.points)
        stride = max(1, math.ceil(total / 5000))
        self._refresh_livox_transform()
        rotation = self._livox_rotation
        tx, ty, tz = self._livox_translation
        points = []
        for point in msg.points[::stride]:
            x, y, z = float(point.x), float(point.y), float(point.z)
            if all(math.isfinite(value) for value in (x, y, z)):
                points.append([
                    rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + tx,
                    rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + ty,
                    rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + tz,
                ])
        points, fixed_frame, transformed = self._transform_points(points, self._livox_body_frame)
        with self._scan_lock:
            if "/livox/lidar" not in self._pointcloud_topics:
                return
            # Livox CustomMsg is converted directly in this callback rather
            # than by the PointCloud2 worker, so this is its processed rate.
            self._record_frequency("processed_cloud:/livox/lidar")
            self._pointclouds["/livox/lidar"] = {
                "topic": "/livox/lidar",
                "frame_id": msg.header.frame_id,
                "fixed_frame": fixed_frame,
                "transformed_to_fixed": transformed,
                "points": points,
                "seen": time.monotonic(),
            }

    def _pointcloud2_callback(self, topic, msg):
        """Record the true source rate and hand only the latest frame to a worker."""
        with self._scan_lock:
            self._record_frequency(f"cloud:{topic}")
            self._pointcloud_seen[topic] = time.monotonic()
            if topic not in self._pointcloud_topics:
                return
        with self._cloud_condition:
            # Replacing an older entry prevents expensive historical frames
            # from building a queue and delaying pose/scan callbacks.
            self._cloud_pending[topic] = msg
            self._cloud_condition.notify()

    def _cloud_worker_loop(self):
        while not self._stop_event.is_set():
            with self._cloud_condition:
                if not self._cloud_pending and not self._stop_event.is_set():
                    self._cloud_condition.wait(timeout=0.5)
                if self._stop_event.is_set():
                    return
                pending = self._cloud_pending
                self._cloud_pending = {}
            for topic, msg in pending.items():
                try:
                    self._process_pointcloud2(topic, msg)
                except Exception:
                    # A malformed frame must not terminate visualization for
                    # subsequent valid frames.
                    continue

    def _process_pointcloud2(self, topic, msg):
        now = time.monotonic()
        # The browser consumes point clouds at 5 Hz.  Processing faster only
        # burns Python/GIL time and cannot create additional visible frames.
        if now - self._cloud_processed_at.get(topic, 0.0) < 0.15:
            return
        self._cloud_processed_at[topic] = now
        with self._scan_lock:
            if topic not in self._pointcloud_topics:
                return
        fields = {field.name: field for field in msg.fields}
        if not all(axis in fields for axis in ("x", "y", "z")):
            return
        if any(fields[axis].datatype != 7 for axis in ("x", "y", "z")):
            return
        total = int(msg.width) * int(msg.height)
        stride = max(1, math.ceil(total / 5000))
        endian = ">" if msg.is_bigendian else "<"
        offsets = [fields[axis].offset for axis in ("x", "y", "z")]
        points = []
        for index in range(0, total, stride):
            base = index * msg.point_step
            try:
                x, y, z = (struct.unpack_from(endian + "f", msg.data, base + offset)[0] for offset in offsets)
            except (struct.error, IndexError):
                break
            if all(math.isfinite(value) for value in (x, y, z)):
                points.append([x, y, z])
        points, fixed_frame, transformed = self._transform_points(points, msg.header.frame_id)
        with self._scan_lock:
            if topic in self._pointcloud_topics:
                self._record_frequency(f"processed_cloud:{topic}")
                version = self._pointcloud_versions.get(topic, 0) + 1
                self._pointcloud_versions[topic] = version
                self._pointclouds[topic] = {
                    "topic": topic,
                    "frame_id": msg.header.frame_id,
                    "fixed_frame": fixed_frame,
                    "transformed_to_fixed": transformed,
                    "points": points,
                    "seen": time.monotonic(),
                    "version": version,
                }

    def _path_callback(self, topic, msg):
        points = [
            [float(pose.pose.position.x), float(pose.pose.position.y), float(pose.pose.position.z)]
            for pose in msg.poses
        ]
        points, fixed_frame, transformed = self._transform_points(points, msg.header.frame_id)
        with self._scan_lock:
            self._record_frequency(f"path:{topic}")
            self._paths[topic] = {
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "fixed_frame": fixed_frame,
                "transformed_to_fixed": transformed,
                "points": points,
                "seen": time.monotonic(),
            }

    def _transform_points(self, points, source_frame):
        source = str(source_frame or "").lstrip("/")
        if not source or not points or self._tf_buffer is None:
            return self._rounded_points(points), source, False
        transform = None
        target = source
        # Mapping owns an odom-fixed map. A long-lived Web backend can retain
        # the previous navigation session's map->odom transform after
        # localization stops. Never apply that stale correction to mapping
        # clouds, otherwise they no longer align with /projected_map.
        candidates = ("odom",) if self._trajectory_mode == "mapping" else ("map", "odom")
        for candidate in candidates:
            if source == candidate:
                return self._rounded_points(points), candidate, True
            try:
                from rclpy.duration import Duration
                from rclpy.time import Time
                transform = self._tf_buffer.lookup_transform(
                    candidate, source, Time(), timeout=Duration(seconds=0.05)
                )
                if candidate == "map" and self._node is not None:
                    stamp = transform.header.stamp
                    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    now_ns = int(self._node.get_clock().now().nanoseconds)
                    age_seconds = (now_ns - stamp_ns) / 1_000_000_000.0
                    if stamp_ns <= 0 or age_seconds > 2.0 or age_seconds < -0.5:
                        transform = None
                        continue
                target = candidate
                break
            except Exception:
                continue
        if transform is None:
            return self._rounded_points(points), source, False
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        # Quaternion rotation matrix, followed by translation.
        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qz * qw)
        r02 = 2 * (qx * qz + qy * qw)
        r10 = 2 * (qx * qy + qz * qw)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r12 = 2 * (qy * qz - qx * qw)
        r20 = 2 * (qx * qz - qy * qw)
        r21 = 2 * (qy * qz + qx * qw)
        r22 = 1 - 2 * (qx * qx + qy * qy)
        converted = [
            [
                r00 * x + r01 * y + r02 * z + translation.x,
                r10 * x + r11 * y + r12 * z + translation.y,
                r20 * x + r21 * y + r22 * z + translation.z,
            ]
            for x, y, z in points
        ]
        return self._rounded_points(converted), target, True

    @staticmethod
    def _rounded_points(points):
        return [[round(x, 3), round(y, 3), round(z, 3)] for x, y, z in points]

    def _costmap_callback(self, name, msg):
        orientation = msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        output_frame = msg.header.frame_id
        transformed_to_map = output_frame == "map"
        if output_frame and output_frame != "map" and self._node is not None:
            try:
                transform = self._node.tf_buffer.lookup_transform(
                    "map", output_frame, self._rclpy.time.Time()
                )
                rotation = transform.transform.rotation
                transform_yaw = math.atan2(
                    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                )
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                origin_x, origin_y = (
                    tx + math.cos(transform_yaw) * origin_x - math.sin(transform_yaw) * origin_y,
                    ty + math.sin(transform_yaw) * origin_x + math.cos(transform_yaw) * origin_y,
                )
                origin_yaw += transform_yaw
                output_frame = "map"
                transformed_to_map = True
            except Exception:
                transformed_to_map = False
        encoded = base64.b64encode(
            bytes((int(value) + 256) % 256 for value in msg.data)
        ).decode("ascii")
        with self._scan_lock:
            self._record_frequency(f"costmap:{name}")
            self._costmap_versions[name] = self._costmap_versions.get(name, 0) + 1
            self._costmaps[name] = {
                "frame_id": msg.header.frame_id,
                "output_frame": output_frame,
                "transformed_to_map": transformed_to_map,
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": float(msg.info.resolution),
                "origin": [
                    origin_x,
                    origin_y,
                    origin_yaw,
                ],
                "data_base64": encoded,
                # Some rolling costmap publishers reuse or zero header stamps.
                # A receive counter guarantees that every ROS frame invalidates
                # the browser texture, including origin-only rolling updates.
                "version": self._costmap_versions[name],
                "seen": time.monotonic(),
            }

    def _live_map_callback(self, msg):
        orientation = msg.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        encoded = base64.b64encode(
            bytes((int(value) + 256) % 256 for value in msg.data)
        ).decode("ascii")
        with self._scan_lock:
            self._live_map_version += 1
            self._live_map = {
                "frame_id": msg.header.frame_id,
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": float(msg.info.resolution),
                "origin": [
                    float(msg.info.origin.position.x),
                    float(msg.info.origin.position.y),
                    yaw,
                ],
                "data_base64": encoded,
                "version": self._live_map_version,
                "seen": time.monotonic(),
            }

    def _odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        x, y = float(position.x), float(position.y)
        source_frame = (msg.header.frame_id or "").lstrip("/")
        target_frame = "odom" if self._trajectory_mode == "mapping" else "map"
        with self._scan_lock:
            if self._live_map and time.monotonic() - self._live_map["seen"] <= 3.0:
                target_frame = (self._live_map.get("frame_id") or "map").lstrip("/")
        if source_frame and source_frame != target_frame and self._node is not None:
            try:
                transform = self._node.tf_buffer.lookup_transform(
                    target_frame, source_frame, self._rclpy.time.Time()
                )
                rotation = transform.transform.rotation
                transform_yaw = math.atan2(
                    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                )
                tx, ty = transform.transform.translation.x, transform.transform.translation.y
                x, y = (
                    tx + math.cos(transform_yaw) * x - math.sin(transform_yaw) * y,
                    ty + math.sin(transform_yaw) * x + math.cos(transform_yaw) * y,
                )
                yaw += transform_yaw
            except Exception:
                return
        # /Odometry describes the FAST-LIO front-lidar origin. Both the
        # displayed robot pose and the movement trail must follow the planar
        # robot center instead.
        if self._node is None or self._rclpy is None:
            return
        try:
            base_frame = self._base_frame()
            base_transform = self._node.tf_buffer.lookup_transform(
                target_frame, base_frame, self._rclpy.time.Time()
            )
            base_translation = base_transform.transform.translation
            base_rotation = base_transform.transform.rotation
            x = float(base_translation.x)
            y = float(base_translation.y)
            yaw = math.atan2(
                2.0 * (
                    base_rotation.w * base_rotation.z
                    + base_rotation.x * base_rotation.y
                ),
                1.0
                - 2.0
                * (
                    base_rotation.y * base_rotation.y
                    + base_rotation.z * base_rotation.z
                ),
            )
        except Exception:
            # Do not silently fall back to the lidar origin, which would make
            # the trajectory jump when the robot-center TF becomes available.
            return
        point = [round(x, 3), round(y, 3), round(yaw, 4)]
        inferred_mode = None
        if self._trajectory_mode is None:
            with self._graph_lock:
                nodes = {name.rsplit("/", 1)[-1] for name in self._node_names}
            inferred_mode = (
                "navigation"
                if {"global_localization", "controller_server", "bt_navigator"}
                & nodes
                else ("mapping" if "laser_mapping" in nodes else None)
            )
        with self._scan_lock:
            self._record_frequency("odometry")
            if self._trajectory_mode is None:
                self._trajectory_mode = inferred_mode
            if self._trajectory_frame != target_frame:
                self._trajectory = []
                self._trajectory_frame = target_frame
            self._mapping_pose = {
                "x": point[0], "y": point[1], "z": 0.0, "yaw": point[2], "ok": True,
                "frame_id": target_frame, "child_frame_id": self._base_frame(),
            }
            now = time.monotonic()
            if self._trajectory_mode == "navigation":
                localization_ready = bool(
                    self._localization_status
                    and self._localization_status.get("stable")
                    and self._localization_status_seen is not None
                    and now - self._localization_status_seen <= 3.0
                )
                if not localization_ready:
                    self._trajectory_recording = False
                    return
                if not self._trajectory_recording:
                    self._trajectory = []
                    self._trajectory_version += 1
                    self._trajectory_recording = True
            elif self._trajectory_mode == "mapping":
                # Mapping owns a fresh FAST-LIO odometry session.  Record from
                # its first sample so movement immediately after startup is
                # never omitted.
                self._trajectory_recording = True
            elif self._trajectory_mode is None:
                return
            if self._trajectory:
                last = self._trajectory[-1]
                if math.hypot(point[0] - last[0], point[1] - last[1]) < 0.05:
                    return
            self._trajectory.append(point)
            self._trajectory_version += 1
            if len(self._trajectory) > 5000:
                self._trajectory = self._trajectory[-5000:]

    def _cmd_vel_callback(self, msg):
        """Track the final planar velocity command sent on /cmd_vel."""
        x = float(msg.linear.x)
        y = float(msg.linear.y)
        z = float(msg.angular.z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            return
        with self._scan_lock:
            self._motion = {
                "linear_x": round(x, 3),
                "linear_y": round(y, 3),
                # For a planar robot the useful Z velocity is yaw rate (wz).
                "linear_z": round(z, 3),
                "linear_speed": round(math.hypot(x, y), 3),
                "angular_z": round(z, 3),
                "source": "cmd_vel",
                "seen": time.monotonic(),
            }

    def _run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Int32MultiArray
            from std_msgs.msg import String
            from sensor_msgs.msg import LaserScan, PointCloud2
            from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
            from nav2_msgs.action import NavigateToPose
            from action_msgs.srv import CancelGoal
            from action_msgs.msg import GoalStatusArray
            from rclpy.action import ActionClient
            from rclpy.qos import (
                DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from nav_msgs.msg import OccupancyGrid, Odometry, Path
            from tf2_ros import Buffer, TransformListener
        except ImportError as exc:
            status_store.update({
                "connected": False,
                "message": f"ROS Python modules unavailable: {exc}",
            })
            return

        try:
            from livox_ros_driver2.msg import CustomMsg
        except ImportError:
            CustomMsg = None

        self._rclpy = rclpy
        rclpy.init(args=None)

        class StatusNode(Node):
            def __init__(self):
                super().__init__("robot_server_status_bridge")
                self._pointcloud_message_type = PointCloud2
                self._livox_message_type = CustomMsg
                self._pointcloud_qos = qos_profile_sensor_data
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.create_subscription(
                    String,
                    "/robot_system/status",
                    lambda msg: status_store.update_from_json(msg.data),
                    10,
                )
                self.create_subscription(
                    String,
                    "/localization/status",
                    self_outer._localization_status_callback,
                    10,
                )
                self.create_subscription(
                    String,
                    "/a2/relocalization/status",
                    lambda msg: self_outer._a2_localization_status_callback(
                        "relocalization", msg),
                    10,
                )
                self.create_subscription(
                    String,
                    "/a2/odometry/status",
                    lambda msg: self_outer._a2_localization_status_callback(
                        "odometry", msg),
                    10,
                )
                self.create_subscription(
                    String,
                    "/battery_status",
                    self_outer._battery_status_callback,
                    10,
                )
                self.create_subscription(
                    OccupancyGrid,
                    "/projected_map",
                    self_outer._live_map_callback,
                    10,
                )
                map_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.create_subscription(
                    OccupancyGrid,
                    "/map",
                    self_outer._map_status_callback,
                    map_qos,
                )
                self.create_subscription(
                    Odometry,
                    "/Odometry",
                    self_outer._odometry_callback,
                    20,
                )
                self.create_subscription(
                    Twist,
                    "/cmd_vel",
                    self_outer._cmd_vel_callback,
                    20,
                )
                self.waypoint_sequence_pub = self.create_publisher(
                    Int32MultiArray,
                    "/waypoint_sequence/frontend",
                    10,
                )
                self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
                self.create_subscription(
                    PoseStamped,
                    "/goal_pose",
                    self_outer._goal_pose_callback,
                    10,
                )
                self.create_subscription(
                    PoseStamped,
                    "/navigation/active_goal",
                    self_outer._goal_pose_callback,
                    10,
                )
                self.create_subscription(
                    String,
                    "/waypoint_navigation/status",
                    self_outer._waypoint_status_callback,
                    10,
                )
                self.goal_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
                self.cancel_all_goals_client = self.create_client(
                    CancelGoal, "/navigate_to_pose/_action/cancel_goal"
                )
                self.create_subscription(
                    NavigateToPose.Impl.FeedbackMessage,
                    "navigate_to_pose/_action/feedback",
                    self_outer._goal_feedback_callback,
                    10,
                )
                self.create_subscription(
                    GoalStatusArray,
                    "/navigate_to_pose/_action/status",
                    self_outer._goal_status_callback,
                    10,
                )
                initial_pose_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.initial_pose_pub = self.create_publisher(
                    PoseWithCovarianceStamped, "/initialpose", initial_pose_qos
                )
                self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
                self.cmd_vel_nav_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
                self.tcp_outbound_pub = self.create_publisher(String, "/voice_tcp/outbound", 10)
                for name, topic in {
                    "front": "/front_laser_scan",
                    "rear": "/rear_laser_scan",
                    "combined": "/combined_scan",
                }.items():
                    self.create_subscription(
                        LaserScan, topic,
                        lambda msg, scan_name=name: self_outer._scan_callback(scan_name, msg),
                        10,
                    )
                self.sync_pointcloud_subscriptions()
                for path_topic in ("/plan", "/plan_smoothed", "/transformed_global_plan", "/path"):
                    self.create_subscription(
                        Path,
                        path_topic,
                        lambda msg, topic=path_topic: self_outer._path_callback(topic, msg),
                        10,
                    )
                self.create_subscription(
                    OccupancyGrid,
                    "/local_costmap/costmap",
                    lambda msg: self_outer._costmap_callback("local", msg),
                    10,
                )
                self.create_subscription(
                    OccupancyGrid,
                    "/global_costmap/costmap",
                    lambda msg: self_outer._costmap_callback("global", msg),
                    10,
                )

            def sync_pointcloud_subscriptions(self):
                desired = self_outer._desired_pointcloud_sources()
                current = set(self_outer._pointcloud_subscriptions)
                for topic in current - desired:
                    subscription = self_outer._pointcloud_subscriptions.pop(topic, None)
                    if subscription is not None:
                        self.destroy_subscription(subscription)
                    with self_outer._cloud_condition:
                        self_outer._cloud_pending.pop(topic, None)
                for topic in desired - current:
                    self_outer._pointcloud_subscriptions[topic] = self.create_subscription(
                        self._pointcloud_message_type,
                        topic,
                        lambda msg, source_topic=topic: self_outer._pointcloud2_callback(source_topic, msg),
                        self._pointcloud_qos,
                    )
                with self_outer._scan_lock:
                    wants_livox = "/livox/lidar" in self_outer._pointcloud_topics
                if (
                    wants_livox
                    and self._livox_message_type is not None
                    and self_outer._livox_subscription is None
                ):
                    self_outer._livox_subscription = self.create_subscription(
                        self._livox_message_type, "/livox/lidar",
                        self_outer._livox_callback, self._pointcloud_qos,
                    )
                elif not wants_livox and self_outer._livox_subscription is not None:
                    self.destroy_subscription(self_outer._livox_subscription)
                    self_outer._livox_subscription = None
                self_outer._pointcloud_subscription_update.clear()

        self_outer = self
        self._node = StatusNode()
        self._tf_buffer = self._node.tf_buffer
        self._waypoint_sequence_pub = self._node.waypoint_sequence_pub
        self._goal_pub = self._node.goal_pub
        self._goal_action_client = self._node.goal_action_client
        self._cancel_all_goals_client = self._node.cancel_all_goals_client
        self._initial_pose_pub = self._node.initial_pose_pub
        self._cmd_vel_pub = self._node.cmd_vel_pub
        self._cmd_vel_nav_pub = self._node.cmd_vel_nav_pub
        self._tcp_outbound_pub = self._node.tcp_outbound_pub
        status_store.update({
            "connected": True,
            "message": "waiting for /robot_system/status",
        })

        next_graph_update = 0.0
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.2)
            except Exception:
                # Uvicorn shutdown calls rclpy.shutdown() from another thread;
                # Humble raises ExternalShutdownException in this spin loop.
                if self._stop_event.is_set() or not rclpy.ok():
                    break
                raise
            if self._pointcloud_subscription_update.is_set():
                self._node.sync_pointcloud_subscriptions()
            if (
                self._pending_initial_pose_message is not None
                and self._initial_pose_pub.get_subscription_count() > 0
            ):
                self._pending_initial_pose_message.header.stamp = (
                    self._node.get_clock().now().to_msg()
                )
                self._initial_pose_pub.publish(self._pending_initial_pose_message)
                self._pending_initial_pose_message = None
            now = time.monotonic()
            if now >= next_graph_update:
                names = {
                    (namespace.rstrip("/") + "/" + name).replace("//", "/")
                    for name, namespace in self._node.get_node_names_and_namespaces()
                }
                raw_livox_available = bool(
                    CustomMsg is not None
                    and self._node.get_publishers_info_by_topic("/livox/lidar")
                )
                with self._graph_lock:
                    self._node_names = names
                    self._raw_livox_available = raw_livox_available
                try:
                    from robot_server.app.services.process_manager import process_manager
                    process_manager.ensure_monitors(names)
                except Exception:
                    pass
                next_graph_update = now + 1.0
            self._refresh_runtime_status()


ros_status_bridge = RosStatusBridge()

import json
import math
import os
import tempfile
import uuid
from threading import Lock

import yaml

from robot_server.app.paths import CONFIG_BACKUPS, NAVIGATION_DATA_FILE
from robot_server.app.services.robot_profile import robot_profile_manager


class WaypointStore:
    """Store initial pose and waypoints in map-scoped JSON sections."""

    def __init__(self, path=None):
        self.path = str(path or NAVIGATION_DATA_FILE)
        self._lock = Lock()

    def current_map_name(self):
        try:
            with open(robot_profile_manager.navigation_config(), "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            name = str(config.get("paths", {}).get("map_name", "")).strip()
        except (OSError, TypeError, yaml.YAMLError):
            name = ""
        if not name:
            raise ValueError("当前没有选中的地图")
        return name

    def list_waypoints(self, map_name=None):
        with self._lock:
            section = self._map_section_unlocked(map_name or self.current_map_name())
            return self._normalized_waypoints(section.get("waypoints", []))

    def initial_pose(self, map_name=None):
        with self._lock:
            section = self._map_section_unlocked(map_name or self.current_map_name())
            pose = section.get("initial_pose", {})
            return {key: self._coordinate(pose.get(key, 0.0), key) for key in ("x", "y", "yaw")}

    def set_initial_pose(self, x, y, yaw, map_name=None):
        with self._lock:
            name = map_name or self.current_map_name()
            data = self._read_unlocked()
            section = data.setdefault("maps", {}).setdefault(name, self._empty_section())
            section["initial_pose"] = {
                "x": self._coordinate(x, "x"), "y": self._coordinate(y, "y"),
                "yaw": self._coordinate(yaw, "yaw"),
            }
            section.setdefault("waypoints", [])
            self._write_unlocked(data)
            return dict(section["initial_pose"])

    def ensure_map(self, map_name):
        with self._lock:
            data = self._read_unlocked()
            maps = data.setdefault("maps", {})
            if map_name not in maps:
                maps[map_name] = self._empty_section()
                self._write_unlocked(data)
            return maps[map_name]

    def archive_map(self, map_name, archive_id):
        """Remove one map section while retaining a restorable JSON backup."""
        with self._lock:
            data = self._read_unlocked()
            maps = data.setdefault("maps", {})
            section = maps.get(map_name)
            if section is None:
                return None
            backup_directory = os.path.join(str(CONFIG_BACKUPS), "waypoints")
            os.makedirs(backup_directory, exist_ok=True)
            backup_path = os.path.join(backup_directory, f"{map_name}_{archive_id}.json")
            descriptor, temporary = tempfile.mkstemp(prefix=".waypoint-archive-", dir=backup_directory)
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                    json.dump(
                        {"version": 1, "maps": {map_name: section}},
                        stream, ensure_ascii=False, indent=2,
                    )
                    stream.write("\n")
                os.replace(temporary, backup_path)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
            del maps[map_name]
            self._write_unlocked(data)
            return backup_path

    def append_waypoint(self, x, y, yaw, note="", reject_last_duplicate=False):
        with self._lock:
            data, section, waypoints = self._current_data_unlocked()
            waypoint = {
                "id": uuid.uuid4().hex, "index": len(waypoints) + 1,
                "x": self._coordinate(x, "x"), "y": self._coordinate(y, "y"),
                "yaw": self._coordinate(yaw, "yaw"), "note": self._clean_note(note),
            }
            if reject_last_duplicate and waypoints:
                last = waypoints[-1]
                tolerances = {"x": 0.005, "y": 0.005, "yaw": 0.00005}
                if all(abs(last[key] - waypoint[key]) < limit for key, limit in tolerances.items()):
                    raise ValueError("当前位置与最后一个点位重复")
            waypoints.append(waypoint)
            section["waypoints"] = waypoints
            self._write_unlocked(data)
            return waypoint

    def delete_waypoint(self, index):
        with self._lock:
            data, section, waypoints = self._current_data_unlocked()
            zero_index = index - 1
            if zero_index < 0 or zero_index >= len(waypoints):
                return None
            removed = waypoints.pop(zero_index)
            section["waypoints"] = self._normalized_waypoints(waypoints)
            self._write_unlocked(data)
            return removed

    def update_waypoint(self, index, x, y, yaw, note=""):
        with self._lock:
            data, section, waypoints = self._current_data_unlocked()
            zero_index = index - 1
            if zero_index < 0 or zero_index >= len(waypoints):
                return None
            waypoints[zero_index].update({
                "x": self._coordinate(x, "x"), "y": self._coordinate(y, "y"),
                "yaw": self._coordinate(yaw, "yaw"), "note": self._clean_note(note),
            })
            section["waypoints"] = waypoints
            self._write_unlocked(data)
            return waypoints[zero_index]

    def reorder_waypoints(self, order):
        with self._lock:
            data, section, waypoints = self._current_data_unlocked()
            if sorted(order) != list(range(1, len(waypoints) + 1)):
                raise ValueError("order must contain every waypoint index exactly once")
            section["waypoints"] = self._normalized_waypoints([waypoints[index - 1] for index in order])
            self._write_unlocked(data)
            return list(section["waypoints"])

    def _current_data_unlocked(self):
        data = self._read_unlocked()
        section = data.setdefault("maps", {}).setdefault(self.current_map_name(), self._empty_section())
        return data, section, self._normalized_waypoints(section.get("waypoints", []))

    def _map_section_unlocked(self, name):
        return self._read_unlocked().get("maps", {}).get(name, self._empty_section())

    def _read_unlocked(self):
        try:
            with open(self.path, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            if isinstance(data, dict) and isinstance(data.get("maps", {}), dict):
                return data
        except (OSError, ValueError, TypeError):
            pass
        return {"version": 1, "maps": {}}

    def _write_unlocked(self, data):
        directory = os.path.dirname(self.path) or "."
        os.makedirs(directory, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(prefix=".navigation-data-", dir=directory)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def _normalized_waypoints(cls, raw):
        result = []
        for item in raw if isinstance(raw, list) else []:
            if not isinstance(item, dict):
                continue
            try:
                result.append({
                    "id": str(item.get("id") or uuid.uuid4().hex), "index": len(result) + 1,
                    "x": cls._coordinate(item.get("x"), "x"),
                    "y": cls._coordinate(item.get("y"), "y"),
                    "yaw": cls._coordinate(item.get("yaw", 0.0), "yaw"),
                    "note": cls._clean_note(item.get("note", "")),
                })
            except (TypeError, ValueError):
                continue
        return result

    @staticmethod
    def _empty_section():
        return {"initial_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}, "waypoints": []}

    @staticmethod
    def _clean_note(value):
        note = str(value or "").strip()
        if len(note) > 200:
            raise ValueError("备注不能超过 200 个字符")
        return note

    @staticmethod
    def _finite_float(value, field_name):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{field_name} must be finite")
        return number

    @classmethod
    def _coordinate(cls, value, field_name):
        return round(cls._finite_float(value, field_name), 4)


waypoint_store = WaypointStore()

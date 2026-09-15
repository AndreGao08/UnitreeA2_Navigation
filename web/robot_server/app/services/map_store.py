import os
import shutil
import tempfile
from datetime import datetime
from threading import Lock

import yaml

from robot_server.app.paths import CONFIG_BACKUPS, MAP_DIR
from robot_server.app.services.robot_profile import robot_profile_manager
from robot_server.app.services.waypoint_store import waypoint_store


class MapStore:
    DEFAULT_MAP_BASENAME = "a2_map"

    def __init__(
        self,
        map_directory=None,
        navigation_config=None,
    ):
        self.map_directory = str(map_directory or MAP_DIR)
        self._navigation_config = str(navigation_config) if navigation_config else None
        self._lock = Lock()

    @property
    def navigation_config(self):
        return self._navigation_config or str(robot_profile_manager.navigation_config())

    def list_maps(self):
        with self._lock:
            return self._list_maps_unlocked()

    def select_map(self, name):
        with self._lock:
            self._validate_map_name(name)
            map_info = self._map_info_unlocked(name)
            yaml_path = map_info["yaml"]["path"]
            pcd_path = map_info["pcd"]["path"]
            missing = [
                path for path in [yaml_path, pcd_path]
                if not os.path.isfile(path)
            ]
            if missing:
                return {
                    "ok": False,
                    "message": "map yaml/pcd files are required",
                    "missing": missing,
                }

            config = self._read_config_unlocked()
            config.setdefault("paths", {})["map_name"] = name
            config.setdefault("paths", {})["map_file_basename"] = map_info["file_basename"]
            backup_path = self._backup_config_unlocked()
            self._write_config_unlocked(config)
            self._sync_shared_map_unlocked(config.get("paths", {}))
            waypoint_store.ensure_map(name)
            result = self._list_maps_unlocked()
            result.update({
                "ok": True,
                "backup_path": backup_path,
                "requires_navigation_restart": True,
            })
            return result

    def set_initial_pose(self, name, x, y, yaw):
        with self._lock:
            self._validate_map_name(name)
            if not os.path.isdir(os.path.join(self.map_directory, name)):
                raise ValueError("map not found")
            return waypoint_store.set_initial_pose(x, y, yaw, name)

    def prepare_map(self, name):
        with self._lock:
            self._validate_map_name(name)
            map_path = os.path.join(self.map_directory, name)
            os.makedirs(map_path, exist_ok=True)
            config = self._read_config_unlocked()
            config.setdefault("paths", {})["map_name"] = name
            config.setdefault("paths", {})["map_file_basename"] = self.DEFAULT_MAP_BASENAME
            backup_path = self._backup_config_unlocked()
            self._write_config_unlocked(config)
            self._sync_shared_map_unlocked(config.get("paths", {}))
            waypoint_store.ensure_map(name)
            return {"ok": True, "name": name, "path": map_path, "backup_path": backup_path}

    def _sync_shared_map_unlocked(self, source_paths):
        shared = {
            key: source_paths[key]
            for key in ("map_directory", "map_name", "map_file_basename")
            if key in source_paths
        }
        for profile in robot_profile_manager.PROFILES.values():
            path = str(profile["navigation_config"])
            if path == self.navigation_config:
                continue
            try:
                with open(path, "r", encoding="utf-8") as stream:
                    config = yaml.safe_load(stream) or {}
                config.setdefault("paths", {}).update(shared)
                directory = os.path.dirname(path)
                descriptor, temporary = tempfile.mkstemp(prefix=".shared-map-", dir=directory)
                try:
                    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
                    os.replace(temporary, path)
                finally:
                    if os.path.exists(temporary):
                        os.unlink(temporary)
            except (OSError, yaml.YAMLError):
                continue

    def delete_map(self, name):
        with self._lock:
            self._validate_map_name(name)
            if name == self.current_map_name():
                raise ValueError("cannot delete the currently selected map")
            source = os.path.join(self.map_directory, name)
            if not os.path.isdir(source):
                return None
            trash_directory = os.path.join(self.map_directory, ".trash")
            os.makedirs(trash_directory, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            destination = os.path.join(trash_directory, f"{name}_{timestamp}")
            shutil.move(source, destination)
            try:
                waypoint_recovery = waypoint_store.archive_map(name, timestamp)
            except Exception:
                shutil.move(destination, source)
                raise
            return {
                "ok": True,
                "name": name,
                "recovery_path": destination,
                "waypoint_recovery_path": waypoint_recovery,
            }

    def pgm_path(self, name):
        with self._lock:
            self._validate_map_name(name)
            path = self._map_info_unlocked(name)["pgm"]["path"]
            return path if os.path.isfile(path) else None

    def replace_pgm(self, name, uploaded_path):
        with self._lock:
            self._validate_map_name(name)
            destination = self._map_info_unlocked(name)["pgm"]["path"]
            if not os.path.isfile(destination):
                raise ValueError("map PGM does not exist")
            try:
                old_width, old_height, _ = self._read_pgm(destination)
                new_width, new_height, _ = self._read_pgm(uploaded_path)
            except (OSError, ValueError) as exc:
                raise ValueError(f"invalid PGM file: {exc}") from exc
            if (new_width, new_height) != (old_width, old_height):
                raise ValueError(
                    f"PGM size must remain {old_width}x{old_height}; uploaded file is {new_width}x{new_height}"
                )
            backup_directory = os.path.join(self.map_directory, ".trash", "pgm_backups")
            os.makedirs(backup_directory, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            backup = os.path.join(backup_directory, f"{name}_{stamp}.pgm")
            shutil.copy2(destination, backup)
            shutil.copy2(uploaded_path, destination)
            return {
                "ok": True, "name": name, "path": destination,
                "backup_path": backup,
                "requires_navigation_restart": name == self.current_map_name(),
            }

    def thumbnail_path(self, name):
        with self._lock:
            self._validate_map_name(name)
            map_info = self._map_info_unlocked(name)
            pgm_path = map_info["pgm"]["path"]
            if not os.path.isfile(pgm_path):
                return None

            thumbnail_directory = os.path.join(
                tempfile.gettempdir(),
                "nav_map_thumbnails",
            )
            os.makedirs(thumbnail_directory, exist_ok=True)
            thumbnail_path = os.path.join(thumbnail_directory, f"{name}.bmp")
            source_mtime = os.path.getmtime(pgm_path)
            if (
                os.path.isfile(thumbnail_path)
                and os.path.getmtime(thumbnail_path) >= source_mtime
            ):
                return thumbnail_path

            self._write_bmp_thumbnail(pgm_path, thumbnail_path)
            return thumbnail_path

    def _list_maps_unlocked(self):
        current = self.current_map_name()
        names = []
        if os.path.isdir(self.map_directory):
            for filename in os.listdir(self.map_directory):
                path = os.path.join(self.map_directory, filename)
                if os.path.isdir(path) and not filename.startswith("."):
                    names.append(filename)

        maps = []
        for name in sorted(names):
            map_info = self._map_info_unlocked(name)
            map_info["selected"] = name == current
            map_info["initial_pose"] = waypoint_store.initial_pose(name)
            map_info["waypoints"] = waypoint_store.list_waypoints(name)
            maps.append(map_info)

        return {
            "map_directory": self.map_directory,
            "current_map": current,
            "maps": maps,
        }

    def current_map_name(self):
        config = self._read_config_unlocked()
        return str(config.get("paths", {}).get("map_name", "")).strip() or None

    def _map_info_unlocked(self, name):
        map_path = os.path.join(self.map_directory, name)
        file_basename = self._detect_file_basename(map_path)
        metadata = self._map_metadata(map_path, file_basename)
        return {
            "name": name,
            "path": map_path,
            "file_basename": file_basename,
            "yaml": self._file_info(name, f"{file_basename}.yaml"),
            "pgm": self._file_info(name, f"{file_basename}.pgm"),
            "pcd": self._file_info(name, f"{file_basename}.pcd"),
            "metadata": metadata,
        }

    @staticmethod
    def _map_metadata(map_path, file_basename):
        yaml_path = os.path.join(map_path, f"{file_basename}.yaml")
        try:
            with open(yaml_path, "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            resolution = float(config.get("resolution", 0.0))
            origin = [float(value) for value in config.get("origin", [])]
            if resolution <= 0 or len(origin) < 3:
                return None
            pgm_path = os.path.join(map_path, f"{file_basename}.pgm")
            width, height, _ = MapStore._read_pgm(pgm_path)
            return {
                "resolution": resolution,
                "origin": origin[:3],
                "width": width,
                "height": height,
            }
        except (FileNotFoundError, OSError, TypeError, ValueError, yaml.YAMLError):
            return None

    @staticmethod
    def _detect_file_basename(map_path):
        a2_yaml = os.path.join(map_path, "a2_map.yaml")
        if os.path.isfile(a2_yaml):
            return "a2_map"
        if not os.path.isdir(map_path):
            return "a2_map"
        yaml_names = sorted(
            os.path.splitext(filename)[0]
            for filename in os.listdir(map_path)
            if filename.endswith(".yaml")
        )
        return yaml_names[0] if yaml_names else "a2_map"

    def _read_config_unlocked(self):
        try:
            with open(self.navigation_config, "r", encoding="utf-8") as config_file:
                return yaml.safe_load(config_file) or {}
        except FileNotFoundError:
            return {}

    def _write_config_unlocked(self, config):
        with open(self.navigation_config, "w", encoding="utf-8") as config_file:
            yaml.safe_dump(config, config_file, allow_unicode=True, sort_keys=False)

    def _backup_config_unlocked(self):
        backup_directory = str(CONFIG_BACKUPS / "robot_server")
        os.makedirs(backup_directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = os.path.join(
            backup_directory,
            f"navigation_frames_{timestamp}.yaml",
        )
        shutil.copy2(self.navigation_config, backup_path)
        return backup_path

    @staticmethod
    def _validate_map_name(name):
        if (
            not name
            or len(name) > 80
            or name in {".", ".."}
            or os.path.basename(name) != name
            or "\x00" in name
            or any(not (character.isalnum() or character in "-_.") for character in name)
        ):
            raise ValueError("map name may only contain letters, numbers, dot, dash and underscore")

    def _file_info(self, map_name, filename):
        path = os.path.join(self.map_directory, map_name, filename)
        exists = os.path.isfile(path)
        return {
            "exists": exists,
            "path": path,
            "size": os.path.getsize(path) if exists else 0,
            "modified_ns": os.stat(path).st_mtime_ns if exists else None,
        }

    @staticmethod
    def _read_pgm(path):
        with open(path, "rb") as image_file:
            magic = image_file.readline().strip()
            if magic not in {b"P2", b"P5"}:
                raise ValueError("unsupported pgm format")

            tokens = []
            while len(tokens) < 3:
                line = image_file.readline()
                if not line:
                    raise ValueError("invalid pgm header")
                line = line.split(b"#", 1)[0]
                tokens.extend(line.split())

            width, height, max_value = [int(token) for token in tokens[:3]]
            if width <= 0 or height <= 0 or max_value <= 0:
                raise ValueError("invalid pgm size")

            if magic == b"P5":
                data = image_file.read(width * height)
                pixels = list(data[:width * height])
            else:
                body = image_file.read().split()
                pixels = [int(value) for value in body[:width * height]]

            if max_value != 255:
                pixels = [max(0, min(255, int(value * 255 / max_value))) for value in pixels]
            return width, height, pixels

    @classmethod
    def _write_bmp_thumbnail(cls, pgm_path, bmp_path, max_side=1024):
        width, height, pixels = cls._read_pgm(pgm_path)
        scale = max(1, int(max(width, height) / max_side))
        thumb_width = max(1, width // scale)
        thumb_height = max(1, height // scale)
        row_padding = (4 - (thumb_width * 3) % 4) % 4
        pixel_data_size = (thumb_width * 3 + row_padding) * thumb_height
        file_size = 54 + pixel_data_size

        with open(bmp_path, "wb") as bmp_file:
            bmp_file.write(b"BM")
            bmp_file.write(file_size.to_bytes(4, "little"))
            bmp_file.write((0).to_bytes(4, "little"))
            bmp_file.write((54).to_bytes(4, "little"))
            bmp_file.write((40).to_bytes(4, "little"))
            bmp_file.write(thumb_width.to_bytes(4, "little"))
            bmp_file.write(thumb_height.to_bytes(4, "little"))
            bmp_file.write((1).to_bytes(2, "little"))
            bmp_file.write((24).to_bytes(2, "little"))
            bmp_file.write((0).to_bytes(4, "little"))
            bmp_file.write(pixel_data_size.to_bytes(4, "little"))
            bmp_file.write((2835).to_bytes(4, "little"))
            bmp_file.write((2835).to_bytes(4, "little"))
            bmp_file.write((0).to_bytes(4, "little"))
            bmp_file.write((0).to_bytes(4, "little"))

            for y in range(thumb_height - 1, -1, -1):
                source_y = min(height - 1, y * scale)
                row = bytearray()
                for x in range(thumb_width):
                    source_x = min(width - 1, x * scale)
                    value = pixels[source_y * width + source_x]
                    row.extend([value, value, value])
                row.extend(b"\0" * row_padding)
                bmp_file.write(row)


map_store = MapStore()

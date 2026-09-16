"""Robot profile selection shared by the web backend and ROS launch commands."""

import os
import ipaddress
import tempfile
from threading import Lock
from pathlib import Path

import yaml

from robot_server.app.paths import ACTIVE_ROBOT_CONFIG, WORKSPACE_ROOT


class RobotProfileManager:
    PRESET_NAMES = {
        "unitree_a2.terrain": "A2 GSeg3D 地形导航",
    }
    BEHAVIOR_TREE_NAMES = {
        "unitree_a2.navigate_to_pose": "A2 标准导航与恢复",
    }
    PROFILES = {
        "unitree_a2": {
            "name": "Unitree A2",
            "description": "Hesai JT128 + FAST-LIO + GSeg3D + Nav2",
            "navigation_config": WORKSPACE_ROOT / "src/navigation/web/config/unitree_a2.yaml",
            "nav2_directory": WORKSPACE_ROOT / "src/navigation/web/config/nav2",
            "default_nav2_preset": "terrain",
            "behavior_tree_directory": WORKSPACE_ROOT / "src/navigation/web/config/behavior_trees",
            "default_behavior_tree": "navigate_to_pose",
        },
    }

    def __init__(self):
        self._config_lock = Lock()

    def active_id(self):
        try:
            with open(ACTIVE_ROBOT_CONFIG, "r", encoding="utf-8") as stream:
                profile_id = str((yaml.safe_load(stream) or {}).get("active_profile", "unitree_a2"))
        except (OSError, yaml.YAMLError):
            profile_id = "unitree_a2"
        return profile_id if profile_id in self.PROFILES else "unitree_a2"

    def runtime_config(self):
        """Return A2 web-process settings from the active profile."""
        try:
            with open(self.navigation_config(), "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError):
            config = {}
        runtime = config.get("web_runtime", {})
        return runtime if isinstance(runtime, dict) else {}

    def frame(self, key, default):
        try:
            with open(self.navigation_config(), "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            return str(config.get("frames", {}).get(key, default)).lstrip("/")
        except (OSError, yaml.YAMLError):
            return default

    def map_paths(self):
        with open(self.navigation_config(), "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        paths = config.get("paths", {})
        directory = Path(os.path.expandvars(str(paths.get(
            "map_directory", WORKSPACE_ROOT / "maps/web"))))
        name = str(paths.get("map_name", "default"))
        basename = str(paths.get("map_file_basename", "a2_map"))
        root = directory / name
        return {
            "map_directory": str(directory),
            "map_name": name,
            "map_basename": basename,
            "map_pcd": str(root / f"{basename}.pcd"),
            "map_yaml": str(root / f"{basename}.yaml"),
            "map_pgm": str(root / f"{basename}.pgm"),
        }

    def _active_config(self):
        try:
            with open(ACTIVE_ROBOT_CONFIG, "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError):
            config = {}
        return config if isinstance(config, dict) else {}

    def navigation_config(self, profile_id=None):
        selected = profile_id or self.active_id()
        if selected not in self.PROFILES:
            raise ValueError("unknown robot profile")
        path = Path(self.PROFILES[selected]["navigation_config"])
        if not path.is_file():
            raise ValueError(f"robot profile configuration does not exist: {path}")
        return path

    def nav2_presets(self, profile_id=None):
        selected_profiles = [profile_id] if profile_id else list(self.PROFILES)
        presets = []
        for selected in selected_profiles:
            if selected not in self.PROFILES:
                raise ValueError("unknown robot profile")
            directory = Path(self.PROFILES[selected]["nav2_directory"])
            if not directory.is_dir():
                continue
            for path in sorted(directory.glob("*.yaml")):
                presets.append({
                    "id": f"{selected}.{path.stem}",
                    "profile": selected,
                    "name": self.PRESET_NAMES.get(
                        f"{selected}.{path.stem}",
                        path.stem.replace("_", " ").replace("-", " ").title(),
                    ),
                    "filename": path.name,
                    "path": path,
                })
        return presets

    def selected_nav2_preset(self, profile_id=None):
        selected = profile_id or self.active_id()
        if selected not in self.PROFILES:
            raise ValueError("unknown robot profile")
        config = self._active_config()
        configured = str((config.get("nav2_presets") or {}).get(selected, "")).strip()
        default = self.PROFILES[selected]["default_nav2_preset"]
        candidate = configured or default
        preset_id = f"{selected}.{candidate}"
        available = {preset["id"] for preset in self.nav2_presets(selected)}
        if preset_id not in available:
            preset_id = f"{selected}.{default}"
        return preset_id

    def nav2_config(self, profile_id=None, preset_id=None):
        selected = profile_id or self.active_id()
        if selected not in self.PROFILES:
            raise ValueError("unknown robot profile")
        resolved_id = preset_id or self.selected_nav2_preset(selected)
        presets = {preset["id"]: preset for preset in self.nav2_presets(selected)}
        if resolved_id not in presets:
            raise ValueError("Nav2 preset does not belong to the selected robot profile")
        path = Path(presets[resolved_id]["path"])
        if not path.is_file():
            raise ValueError(f"Nav2 configuration does not exist: {path}")
        return path

    def behavior_tree_presets(self, profile_id=None):
        selected_profiles = [profile_id] if profile_id else list(self.PROFILES)
        presets = []
        for selected in selected_profiles:
            if selected not in self.PROFILES:
                raise ValueError("unknown robot profile")
            directory = Path(self.PROFILES[selected]["behavior_tree_directory"])
            if not directory.is_dir():
                continue
            for path in sorted(directory.glob("*.xml")):
                preset_id = f"{selected}.{path.stem}"
                presets.append({
                    "id": preset_id,
                    "profile": selected,
                    "name": self.BEHAVIOR_TREE_NAMES.get(
                        preset_id,
                        path.stem.replace("_", " ").replace("-", " ").title(),
                    ),
                    "filename": path.name,
                    "path": path,
                })
        return presets

    def selected_behavior_tree(self, profile_id=None):
        selected = profile_id or self.active_id()
        if selected not in self.PROFILES:
            raise ValueError("unknown robot profile")
        config = self._active_config()
        configured = str((config.get("behavior_tree_presets") or {}).get(selected, "")).strip()
        default = self.PROFILES[selected]["default_behavior_tree"]
        preset_id = f"{selected}.{configured or default}"
        available = {preset["id"] for preset in self.behavior_tree_presets(selected)}
        if preset_id not in available:
            preset_id = f"{selected}.{default}"
        if preset_id not in available:
            raise ValueError(f"behavior tree configuration does not exist for {selected}")
        return preset_id

    def behavior_tree_config(self, profile_id=None, preset_id=None):
        selected = profile_id or self.active_id()
        resolved_id = preset_id or self.selected_behavior_tree(selected)
        presets = {preset["id"]: preset for preset in self.behavior_tree_presets(selected)}
        if resolved_id not in presets:
            raise ValueError("行为树不属于所选机器人类型")
        return Path(presets[resolved_id]["path"])

    def snapshot(self):
        active = self.active_id()
        return {
            "active": active,
            "profiles": [
                {
                    "id": profile_id,
                    "name": item["name"],
                    "description": item["description"],
                    "nav2_preset": self.selected_nav2_preset(profile_id),
                    "nav2_config": self.nav2_config(profile_id).name,
                    "behavior_tree": self.selected_behavior_tree(profile_id),
                }
                for profile_id, item in self.PROFILES.items()
            ],
            "behavior_trees": [
                {
                    "id": preset["id"],
                    "profile": preset["profile"],
                    "name": preset["name"],
                    "filename": preset["filename"],
                }
                for preset in self.behavior_tree_presets()
            ],
        }

    def tcp_config(self):
        path = self.navigation_config()
        with self._config_lock:
            with open(path, "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
        tcp = config.get("tcp_listener", {})
        return {
            "server_host": str(tcp.get("server_host", "127.0.0.1")),
            "server_port": int(tcp.get("server_port", 8050)),
            "profile": self.active_id(),
        }

    def update_tcp_config(self, server_host, server_port):
        host = str(server_host).strip()
        try:
            ipaddress.ip_address(host)
        except ValueError as exc:
            raise ValueError("请输入有效的 IPv4 或 IPv6 地址") from exc
        port = int(server_port)
        if not 1 <= port <= 65535:
            raise ValueError("端口必须在 1 到 65535 之间")

        path = self.navigation_config()
        with self._config_lock:
            with open(path, "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            tcp = config.setdefault("tcp_listener", {})
            tcp["server_host"] = host
            tcp["server_port"] = port
            descriptor, temporary = tempfile.mkstemp(
                prefix=".tcp-config.", suffix=".yaml", dir=str(path.parent)
            )
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                    yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, path)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
        return {**self.tcp_config(), "ok": True, "restart_required": True}

    def select(self, profile_id, nav2_preset=None, behavior_tree=None):
        if profile_id not in self.PROFILES:
            raise ValueError("unknown robot profile")
        self.navigation_config(profile_id)
        selected_preset = nav2_preset or self.selected_nav2_preset(profile_id)
        self.nav2_config(profile_id, selected_preset)
        selected_tree = behavior_tree or self.selected_behavior_tree(profile_id)
        self.behavior_tree_config(profile_id, selected_tree)
        active_config = self._active_config()
        selected_presets = dict(active_config.get("nav2_presets") or {})
        selected_presets[profile_id] = selected_preset.split(".", 1)[1]
        selected_trees = dict(active_config.get("behavior_tree_presets") or {})
        selected_trees[profile_id] = selected_tree.split(".", 1)[1]
        ACTIVE_ROBOT_CONFIG.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix="active_robot.", suffix=".yaml", dir=str(ACTIVE_ROBOT_CONFIG.parent)
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(
                    {
                        "active_profile": profile_id,
                        "nav2_presets": selected_presets,
                        "behavior_tree_presets": selected_trees,
                    },
                    stream,
                    allow_unicode=True,
                    sort_keys=False,
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, ACTIVE_ROBOT_CONFIG)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return self.snapshot()


robot_profile_manager = RobotProfileManager()

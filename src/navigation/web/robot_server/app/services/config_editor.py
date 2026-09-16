import os
import shutil
import tempfile
from datetime import datetime

import yaml

from robot_server.app.paths import CONFIG_BACKUPS
from robot_server.app.services.robot_profile import robot_profile_manager


class ConfigEditor:

    def read(self, name):
        path = self._path(name)
        with open(path, "r", encoding="utf-8") as stream:
            return {"name": name, "path": path, "content": stream.read()}

    def save(self, name, content):
        path = self._path(name)
        try:
            parsed = yaml.safe_load(content)
        except yaml.YAMLError as exc:
            raise ValueError(f"YAML语法错误: {exc}") from exc
        if not isinstance(parsed, dict):
            raise ValueError("配置文件根节点必须是YAML对象")
        backup_directory = str(CONFIG_BACKUPS / "advanced")
        os.makedirs(backup_directory, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        backup = os.path.join(backup_directory, f"{name}_{stamp}.yaml")
        shutil.copy2(path, backup)
        directory = os.path.dirname(path)
        fd, temporary = tempfile.mkstemp(prefix=f".{name}-", dir=directory)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                stream.write(content.rstrip() + "\n")
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return {"ok": True, "name": name, "path": path, "backup_path": backup, "restart_required": True}

    def nav2_presets(self):
        active = robot_profile_manager.active_id()
        selected_by_profile = {
            profile_id: robot_profile_manager.selected_nav2_preset(profile_id)
            for profile_id in robot_profile_manager.PROFILES
        }
        return {
            "active": selected_by_profile[active],
            "active_profile": active,
            "presets": [
                {
                    "id": preset["id"],
                    "profile": preset["profile"],
                    "name": preset["name"],
                    "filename": preset["filename"],
                    "current": preset["id"] == selected_by_profile[preset["profile"]],
                }
                for preset in robot_profile_manager.nav2_presets()
            ],
        }

    def read_nav2_preset(self, preset_id):
        presets = {
            preset["id"]: preset for preset in robot_profile_manager.nav2_presets()
        }
        if preset_id not in presets:
            raise ValueError("不允许访问该参数方案")
        path = presets[preset_id]["path"]
        if not path.is_file():
            raise ValueError("参数方案不存在")
        with open(path, "r", encoding="utf-8") as stream:
            content = stream.read()
        parsed = yaml.safe_load(content)
        if not isinstance(parsed, dict):
            raise ValueError("参数方案内容无效")
        return {"id": preset_id, "path": str(path), "content": content}

    def apply_nav2_preset(self, preset_id):
        preset = self.read_nav2_preset(preset_id)
        return {"ok": True, "preset": preset_id, "changed": False, "path": preset["path"]}

    def _path(self, name):
        if name == "navigation_frames":
            return str(robot_profile_manager.navigation_config())
        if name == "nav2":
            return str(robot_profile_manager.nav2_config())
        raise ValueError("不允许访问该配置文件")


config_editor = ConfigEditor()

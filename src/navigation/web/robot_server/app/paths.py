"""Portable paths for the Unitree A2 web console."""

import os
from pathlib import Path


def _project_root():
    configured = os.getenv("A2_NAVIGATION_WS", "").strip()
    if not configured:
        configured = os.getenv("A2_LOCALIZATION_WS", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    # src/navigation/web/robot_server/app/paths.py -> UnitreeA2_Navigation
    return Path(__file__).resolve().parents[5]


WORKSPACE_ROOT = _project_root()
WEB_ROOT = WORKSPACE_ROOT / "src" / "navigation" / "web"
FRONTEND_DIR = WEB_ROOT / "frontend"
MAP_DIR = WORKSPACE_ROOT / "maps" / "web"
NAVIGATION_DATA_FILE = WEB_ROOT / "config" / "navigation_data.json"
AUTH_CONFIG = WEB_ROOT / "config" / "auth_config.json"
NAV2_CONFIG_ROOT = WEB_ROOT / "config" / "nav2"
CONFIG_BACKUPS = WEB_ROOT / "config_backups"
ROS_ENV_SCRIPT = WORKSPACE_ROOT / "scripts" / "web_ros_env.sh"
ACTIVE_ROBOT_CONFIG = WEB_ROOT / "config" / "active_robot.yaml"

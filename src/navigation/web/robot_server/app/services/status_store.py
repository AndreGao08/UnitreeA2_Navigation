import json
import time
from threading import Lock


class StatusStore:
    def __init__(self):
        self._lock = Lock()
        self._status = {
            "connected": False,
            "message": "ROS status bridge is not running in this process yet",
        }
        self._last_update_monotonic = None

    def update_from_json(self, text):
        try:
            status = json.loads(text)
        except json.JSONDecodeError:
            status = {"connected": False, "raw": text}
        with self._lock:
            self._status = status
            self._last_update_monotonic = time.monotonic()

    def update(self, status):
        with self._lock:
            self._status = status
            self._last_update_monotonic = time.monotonic()

    def reset_navigation_runtime(self):
        """Invalidate ROS navigation state immediately after an explicit stop."""
        with self._lock:
            self._status = {
                "connected": True,
                "message": "navigation system is stopped",
                "tf": {"ok": False, "errors": ["navigation system is stopped"]},
                "map": {"received": False, "recent": False},
                "sensors": {
                    "front_scan": False,
                    "rear_scan": False,
                    "combined_scan": False,
                },
                "navigation": {"status_recent": False, "goal_statuses": []},
            }
            # status_monitor is infrastructure owned by the Web service and
            # remains alive when the navigation session stops. Preserve its
            # heartbeat so clearing navigation state does not flash offline;
            # snapshot() will still report offline naturally after 3 seconds
            # if the monitor actually stops publishing.

    def snapshot(self):
        with self._lock:
            status = dict(self._status)
            age = (
                time.monotonic() - self._last_update_monotonic
                if self._last_update_monotonic is not None
                else None
            )
            status["status_age_seconds"] = round(age, 2) if age is not None else None
            status["connected"] = age is not None and age <= 3.0
            if not status["connected"]:
                status["message"] = "waiting for fresh /robot_system/status"
            return status


status_store = StatusStore()

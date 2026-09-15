import threading
import time

from robot_server.app.services.ros_status_bridge import ros_status_bridge


class InspectionTaskManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread = None
        self._state = {
            "running": False, "status": "idle", "current_step": None,
            "completed_steps": 0, "total_steps": 0, "error": None,
        }

    def start(self, steps, cycles=1):
        with self._lock:
            if self._thread and self._thread.is_alive():
                raise ValueError("an inspection task is already running")
            expanded = steps * cycles
            self._cancel.clear()
            self._state = {
                "running": True, "status": "running", "current_step": 0,
                "completed_steps": 0, "total_steps": len(expanded), "error": None,
            }
            self._thread = threading.Thread(
                target=self._run, args=(expanded,), daemon=True
            )
            self._thread.start()
            return dict(self._state)

    def cancel(self):
        self._cancel.set()
        ros_status_bridge.cancel_navigation()
        return self.snapshot()

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def _run(self, steps):
        try:
            for index, step in enumerate(steps):
                if self._cancel.is_set():
                    self._finish("cancelled")
                    return
                with self._lock:
                    self._state["current_step"] = index + 1
                    self._state["status"] = {
                        "waypoint": "navigating", "text": "speaking", "wait": "waiting",
                    }[step["type"]]
                if step["type"] == "waypoint":
                    result = ros_status_bridge.navigate_waypoint_and_wait(
                        int(step["index"]) - 1, self._cancel
                    )
                    if not result.get("ok"):
                        if self._cancel.is_set():
                            self._finish("cancelled")
                        else:
                            self._finish("failed", result.get("message"))
                        return
                    if step.get("message"):
                        try:
                            result = ros_status_bridge.publish_tcp_message(step["message"])
                            if not result.get("ok"):
                                raise RuntimeError(result.get("message"))
                        except (RuntimeError, ConnectionError) as exc:
                            self._finish("failed", f"到点语音通知失败: {exc}")
                            return
                elif step["type"] == "text":
                    try:
                        result = ros_status_bridge.publish_tcp_message(step["message"])
                        if not result.get("ok"):
                            raise RuntimeError(result.get("message"))
                    except (RuntimeError, ConnectionError) as exc:
                        self._finish("failed", f"语音文本发送失败: {exc}")
                        return
                else:
                    if self._cancel.wait(step["seconds"]):
                        self._finish("cancelled")
                        return
                with self._lock:
                    self._state["completed_steps"] = index + 1
            self._finish("completed")
        except Exception as exc:
            self._finish("failed", str(exc))

    def _finish(self, status, error=None):
        with self._lock:
            self._state["running"] = False
            self._state["status"] = status
            self._state["current_step"] = None
            self._state["error"] = error


inspection_task_manager = InspectionTaskManager()

import os
import signal
import shlex
import subprocess
import time
from threading import Lock
from threading import Thread
from threading import Event
from threading import current_thread

from robot_server.app.paths import ROS_ENV_SCRIPT, WORKSPACE_ROOT
from robot_server.app.services.robot_profile import robot_profile_manager


class ProcessManager:
    STEP_COMMANDS = {
        "nav2": (
            "A2 定位与导航",
            "ros2 launch a2_terrain_nav a2_terrain_navigation.launch.py "
            "map_pcd:={map_pcd} map_yaml:={map_yaml} "
            "auto_initialize:={auto_initialize} locomotion_mode:=gait_demo "
            "rmw_implementation:=rmw_cyclonedds_cpp gui:={gui} rviz:={rviz}",
        ),
        "mapping": (
            "A2 建图",
            "ros2 launch a2_localization_bringup a2_jt128_gazebo.launch.py "
            "operation_mode:=mapping map_path:={map_pcd} trajectory:=manual "
            "locomotion_mode:=gait_demo rmw_implementation:=rmw_cyclonedds_cpp "
            "gui:={gui} rviz:={rviz}",
        ),
        "map_save": (
            "保存 A2 地图",
            "bash {project_root}/scripts/web_finalize_map.sh {map_pcd} {map_yaml}",
        ),
    }

    DEFAULT_STEPS = ["nav2"]
    NAVIGATION_SESSION_STEPS = ("nav2",)
    NAVIGATION_STOP_CONFIRM_TIMEOUT_SECONDS = 30.0
    NAVIGATION_STOP_STABLE_SECONDS = 1.0
    NAVIGATION_STOP_ORDER = ["nav2", "map_save", "mapping"]
    MONITOR_STEPS = set()
    NODE_MARKERS = {
        "nav2": {"map_localizer", "controller_server", "planner_server", "bt_navigator"},
        # laser_mapping is also present in localization mode, so it cannot be
        # used to distinguish an externally started mapping session.
        "mapping": set(),
        "map_save": set(),
    }
    EXTERNAL_COMMAND_MARKERS = {
        "nav2": (
            "a2_terrain_navigation.launch.py",
            "controller_server",
            "planner_server",
            "bt_navigator",
        ),
        "mapping": ("a2_jt128_gazebo.launch.py", "fastlio_mapping"),
        "map_save": ("web_finalize_map.sh", "pcd_to_nav2_map"),
    }

    def __init__(self):
        self._lock = Lock()
        self._processes = {}
        self._sequence_thread = None
        self._stop_sequence = Event()
        self._last_errors = {}
        self._detected_nodes = set()
        self._monitor_starting = set()
        self._system_transition = Event()
        self._transition_lock = Lock()

    @staticmethod
    def _base_node_names(node_names):
        return {name.rsplit("/", 1)[-1] for name in node_names}

    def ensure_monitors(self, node_names):
        base_names = self._base_node_names(node_names)
        with self._lock:
            self._detected_nodes = set(base_names)

    def start_navigation(
        self, ignore_detected_cache=False, allow_system_transition=False
    ):
        if self._system_transition.is_set() and not allow_system_transition:
            return False
        self.stop_step("mapping")
        self.stop_step("map_save")
        with self._lock:
            managed_running = any(
                (process := self._processes.get(step)) is not None and process.poll() is None
                for step in self.NAVIGATION_SESSION_STEPS
            )
            detected_running = any(
                self.NODE_MARKERS.get(step, set()) & self._detected_nodes
                for step in self.NAVIGATION_SESSION_STEPS
            )
            external_running = any(
                self._external_pids(step)
                for step in self.NAVIGATION_SESSION_STEPS
            )
            if (
                self._sequence_thread and self._sequence_thread.is_alive()
            ) or managed_running or external_running or (
                detected_running and not ignore_detected_cache
            ):
                return False
            # A navigation session owns a fresh movement trail. Import here to
            # avoid a module-level service cycle during application startup.
            from robot_server.app.services.ros_status_bridge import ros_status_bridge
            ros_status_bridge.reset_trajectory(mode="navigation")
            self._stop_sequence.clear()
            self._sequence_thread = Thread(
                target=self._start_navigation_sequence,
                args=(ignore_detected_cache,),
                daemon=True,
            )
            self._sequence_thread.start()
            return True

    def _start_navigation_sequence(self, ignore_detected_cache=False):
        delays = {"nav2": 0}
        for step in self.DEFAULT_STEPS:
            if self._stop_sequence.is_set():
                break
            self.start_step(step, ignore_detected_cache=ignore_detected_cache)
            if self._stop_sequence.wait(delays.get(step, 0)):
                break

    def start_step(self, step, ignore_detected_cache=False):
        with self._lock:
            self._validate_step(step)
            process = self._processes.get(step)
            if process and process.poll() is None:
                return
            markers = self.NODE_MARKERS.get(step, set())
            if (
                markers & self._detected_nodes and not ignore_detected_cache
            ) or self._external_pids(step):
                return

            env = os.environ.copy()
            # The Web backend disables the user site so Conda cannot leak
            # packages from ~/.local. ROS Python nodes intentionally use
            # /usr/bin/python3 and keep Open3D and related dependencies there,
            # so they must not inherit the Web-only isolation flag.
            env.pop("PYTHONNOUSERSITE", None)
            _, launch_template = self.STEP_COMMANDS[step]
            navigation_config = shlex.quote(str(robot_profile_manager.navigation_config()))
            map_paths = {
                key: shlex.quote(value)
                for key, value in robot_profile_manager.map_paths().items()
            }
            runtime = robot_profile_manager.runtime_config()
            launch_command = launch_template.format(
                navigation_config=navigation_config,
                nav2_config=shlex.quote(str(robot_profile_manager.nav2_config())),
                project_root=shlex.quote(str(WORKSPACE_ROOT)),
                auto_initialize=str(bool(runtime.get("auto_initialize", True))).lower(),
                gui=str(bool(runtime.get("gui", False))).lower(),
                rviz=str(bool(runtime.get("rviz", False))).lower(),
                **map_paths,
            )
            command = [
                "bash",
                "-lc",
                f"source {shlex.quote(str(ROS_ENV_SCRIPT))} && {launch_command}",
            ]
            self._processes[step] = subprocess.Popen(
                command,
                cwd=str(WORKSPACE_ROOT),
                env=env,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._last_errors.pop(step, None)

    def stop_navigation(self):
        with self._transition_lock:
            self._system_transition.set()
            try:
                return self._stop_navigation_impl()
            finally:
                self._system_transition.clear()

    def _stop_navigation_impl(self):
        self._stop_sequence.set()
        with self._lock:
            sequence_thread = self._sequence_thread
        stop_threads = [
            Thread(target=self.stop_step, args=(step,), daemon=True)
            for step in self.NAVIGATION_STOP_ORDER
        ]
        for thread in stop_threads:
            thread.start()
        for thread in stop_threads:
            thread.join()
        if sequence_thread and sequence_thread is not current_thread():
            sequence_thread.join(timeout=2.0)
        with self._lock:
            if self._sequence_thread is sequence_thread and (
                sequence_thread is None or not sequence_thread.is_alive()
            ):
                self._sequence_thread = None

        # Process termination and DDS graph removal are asynchronous.  Do not
        # report the system as stopped until all navigation modules remain
        # absent for a short stable window.
        if not self._wait_for_navigation_shutdown(
            self.NAVIGATION_STOP_CONFIRM_TIMEOUT_SECONDS
        ):
            if not self._force_cleanup_navigation_residuals():
                return False
            if not self._wait_for_navigation_shutdown(3.0):
                return False
        from robot_server.app.services.ros_status_bridge import ros_status_bridge
        from robot_server.app.services.status_store import status_store
        ros_status_bridge.reset_navigation_visuals()
        status_store.reset_navigation_runtime()
        return True

    def stop_step(self, step):
        with self._lock:
            self._validate_step(step)
            process = self._processes.pop(step, None)
        if process is not None:
            self._terminate_process_group(
                process.pid,
                graceful_timeout=30.0 if step == "mapping" else 5.0,
            )
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
        else:
            self._terminate_external_step(step)
        # ROS graph discovery is asynchronous.  Once the owned/external
        # process is really gone, remove its cached markers so an immediate
        # restart does not incorrectly skip this step.
        if not self._external_pids(step):
            with self._lock:
                self._detected_nodes.difference_update(self.NODE_MARKERS.get(step, set()))

    @staticmethod
    def _process_group_exists(group_id):
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/stat", "r", encoding="utf-8") as stream:
                    fields = stream.read().rsplit(") ", 1)[1].split()
                state, process_group = fields[0], int(fields[2])
            except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError, ValueError):
                continue
            if process_group == group_id and state != "Z":
                return True
        return False

    @staticmethod
    def _process_is_alive(pid):
        try:
            with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as stream:
                state = stream.read().rsplit(") ", 1)[1].split()[0]
            return state != "Z"
        except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError):
            return False

    def _terminate_process_group(self, group_id, graceful_timeout=5.0):
        # Signal only the ros2 launch group leader first.  Sending SIGINT to
        # the whole group makes FAST-LIO receive it once directly and once
        # again when launch forwards shutdown, which can interrupt its final
        # PCD write.  Escalation still targets the complete group.
        try:
            os.kill(group_id, signal.SIGINT)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + graceful_timeout
        while time.monotonic() < deadline:
            if not self._process_group_exists(group_id):
                return
            time.sleep(0.1)

        for sig, timeout in (
            (signal.SIGTERM, 2.0),
            (signal.SIGKILL, 1.0),
        ):
            try:
                os.killpg(group_id, sig)
            except ProcessLookupError:
                return
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if not self._process_group_exists(group_id):
                    return
                time.sleep(0.1)

    def _terminate_external_step(self, step):
        roots = self._external_pids(step)
        if not roots:
            return
        targets = self._with_descendants(roots)
        # As with managed launches, let each launch parent forward one clean
        # SIGINT to its children instead of signalling parent and children at
        # the same time.
        for pid in roots:
            try:
                os.kill(pid, signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        deadline = time.monotonic() + (30.0 if step == "mapping" else 5.0)
        while time.monotonic() < deadline:
            targets = {pid for pid in targets if os.path.exists(f"/proc/{pid}")}
            if not targets:
                return
            time.sleep(0.1)

        for sig, timeout in (
            (signal.SIGTERM, 2.0),
            (signal.SIGKILL, 1.0),
        ):
            for pid in sorted(targets, reverse=True):
                try:
                    os.kill(pid, sig)
                except (ProcessLookupError, PermissionError):
                    pass
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                targets = {pid for pid in targets if os.path.exists(f"/proc/{pid}")}
                if not targets:
                    return
                time.sleep(0.1)

    @staticmethod
    def _with_descendants(roots):
        children = {}
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/status", "r", encoding="utf-8") as stream:
                    parent_line = next(line for line in stream if line.startswith("PPid:"))
                parent = int(parent_line.split()[1])
            except (FileNotFoundError, PermissionError, ProcessLookupError, StopIteration, ValueError):
                continue
            children.setdefault(parent, set()).add(int(entry))
        result = set(roots)
        pending = list(roots)
        while pending:
            for child in children.get(pending.pop(), set()):
                if child not in result:
                    result.add(child)
                    pending.append(child)
        return result

    def restart_navigation(self):
        with self._transition_lock:
            self._system_transition.set()
            try:
                if not self._stop_navigation_impl():
                    return False

                # Every related process and ROS node has now been observed as
                # stopped.  Start the modules in the normal ordered sequence.
                return self.start_navigation(
                    ignore_detected_cache=True,
                    allow_system_transition=True,
                )
            finally:
                self._system_transition.clear()

    def _wait_for_navigation_shutdown(self, timeout):
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            with self._lock:
                managed_running = any(
                    (process := self._processes.get(step)) is not None
                    and process.poll() is None
                    for step in self.NAVIGATION_SESSION_STEPS
                )
                detected_running = any(
                    self.NODE_MARKERS.get(step, set()) & self._detected_nodes
                    for step in self.NAVIGATION_SESSION_STEPS
                )
            external_running = any(
                self._external_pids(step)
                for step in self.NAVIGATION_SESSION_STEPS
            )
            if not managed_running and not detected_running and not external_running:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif (
                    time.monotonic() - stable_since
                    >= self.NAVIGATION_STOP_STABLE_SECONDS
                ):
                    return True
            else:
                stable_since = None
            time.sleep(0.1)
        return False

    def _force_cleanup_navigation_residuals(self):
        residuals = set()
        for step in self.NAVIGATION_STOP_ORDER:
            residuals.update(self._with_descendants(self._external_pids(step)))

        for pid in sorted(residuals, reverse=True):
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

        deadline = time.monotonic() + 2.0
        while residuals and time.monotonic() < deadline:
            residuals = {
                pid for pid in residuals
                if self._process_is_alive(pid)
            }
            if residuals:
                time.sleep(0.1)

        remaining_external = any(
            self._external_pids(step)
            for step in self.NAVIGATION_STOP_ORDER
        )
        if not remaining_external:
            with self._lock:
                for step in self.NAVIGATION_STOP_ORDER:
                    self._detected_nodes.difference_update(
                        self.NODE_MARKERS.get(step, set())
                    )
        return not residuals and not remaining_external

    def restart_step(self, step):
        self._validate_step(step)
        self.stop_step(step)
        self.start_step(step)

    def start_mapping(self):
        self.stop_navigation()
        # Stop the previous odometry publishers before clearing the browser
        # trail.  Clearing first allows localization callbacks received during
        # shutdown to repopulate the new mapping session with the old route.
        from robot_server.app.services.ros_status_bridge import ros_status_bridge
        ros_status_bridge.reset_navigation_visuals()
        ros_status_bridge.reset_trajectory(mode="mapping")
        self.start_step("mapping")

    def stop_mapping(self):
        self.stop_step("mapping")

    def save_mapping(self):
        self.start_step("map_save")

    def finish_mapping(self):
        self.start_step("map_save")
        with self._lock:
            save_process = self._processes.get("map_save")
        if save_process is not None:
            try:
                save_process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                self.stop_step("map_save")
                raise RuntimeError("保存地图超时；建图仍在运行，请确认 /projected_map 后重试")
            if save_process.returncode != 0:
                with self._lock:
                    self._last_errors["map_save"] = (
                        f"map saver exited with code {save_process.returncode}"
                    )
                raise RuntimeError(
                    "二维地图保存失败；建图仍在运行，请确认主画布已有地图后重试"
                )
        self.stop_step("mapping")

    def snapshot(self):
        with self._lock:
            steps = []
            any_running = False
            operational_running = False
            for key, (label, _) in self.STEP_COMMANDS.items():
                process = self._processes.get(key)
                managed_running = bool(process and process.poll() is None)
                markers = self.NODE_MARKERS.get(key, set())
                detected = bool(markers & self._detected_nodes)
                running = managed_running or detected
                externally_controllable = detected and bool(self._external_pids(key))
                error = self._last_errors.get(key)
                if process and not running and error is None:
                    self._last_errors[key] = f"process exited with code {process.returncode}"
                any_running = any_running or running
                if key not in self.MONITOR_STEPS:
                    operational_running = operational_running or running
                steps.append({
                    "key": key,
                    "label": label,
                    "running": running,
                    "pid": process.pid if managed_running else None,
                    "managed": managed_running,
                    "detected_from_ros": detected,
                    "controllable": managed_running or externally_controllable,
                    "error": None if running else self._last_errors.get(key),
                })
            return {
                "navigation_running": operational_running,
                "any_process_running": any_running,
                "startup_in_progress": bool(
                    self._sequence_thread and self._sequence_thread.is_alive()
                ),
                "shutdown_in_progress": self._system_transition.is_set(),
                "navigation_pid": None,
                "steps": steps,
                "robot_profile": robot_profile_manager.snapshot(),
                "tcp_config": robot_profile_manager.tcp_config(),
            }

    def _external_pids(self, step):
        markers = self.EXTERNAL_COMMAND_MARKERS.get(step, ())
        if not markers:
            return []
        launch_pids = []
        node_pids = []
        own_pid = os.getpid()
        for entry in os.listdir("/proc"):
            if not entry.isdigit() or int(entry) == own_pid:
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as stream:
                    command = stream.read().replace(b"\0", b" ").decode("utf-8", "ignore")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if not any(marker in command for marker in markers):
                continue
            target = launch_pids if ".launch.py" in command else node_pids
            target.append(int(entry))
        return launch_pids or node_pids

    def _validate_step(self, step):
        if step not in self.STEP_COMMANDS:
            raise ValueError("unknown system step")


process_manager = ProcessManager()

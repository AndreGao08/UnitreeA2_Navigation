import os
import threading
import time

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from robot_server.app.routes.navigation import router as navigation_router
from robot_server.app.routes.maps import router as maps_router
from robot_server.app.routes.status import router as status_router
from robot_server.app.routes.system import router as system_router
from robot_server.app.routes.waypoints import router as waypoints_router
from robot_server.app.routes.inspection_tasks import router as inspection_tasks_router
from robot_server.app.routes.auth import router as auth_router
from robot_server.app.routes.advanced import router as advanced_router
from robot_server.app.services.auth import auth_manager
from robot_server.app.services.ros_status_bridge import ros_status_bridge
from robot_server.app.services.process_manager import process_manager
from robot_server.app.paths import FRONTEND_DIR


app = FastAPI(title="HYY导航控制软件")
_runtime_autostart_cancel = threading.Event()
_runtime_autostart_thread = None


def _env_enabled(name):
    return os.getenv(name, "0").strip().lower() not in {
        "", "0", "false", "no", "off",
    }


def _auto_start_runtime():
    """Start requested runtime components once after ROS graph discovery."""
    start_navigation = _env_enabled("HYY_AUTO_START_NAVIGATION")
    start_tcp = _env_enabled("HYY_AUTO_START_TCP")
    if not start_navigation and not start_tcp:
        return
    deadline = time.monotonic() + 5.0
    while not _runtime_autostart_cancel.is_set():
        node_names = ros_status_bridge.graph_snapshot()
        if node_names:
            # Adopt nodes left alive across a backend restart before deciding
            # whether any launch process needs to be created.
            process_manager.ensure_monitors(node_names)
            break
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        _runtime_autostart_cancel.wait(min(0.1, remaining))
    if _runtime_autostart_cancel.is_set():
        return
    system = process_manager.snapshot()
    nav2_running = any(
        step.get("key") == "nav2" and step.get("running")
        for step in system.get("steps", [])
    )
    if start_navigation and not nav2_running:
        process_manager.start_navigation()
    elif start_tcp and "waypoint" in process_manager.STEP_COMMANDS:
        # Match the existing "start voice TCP" component behavior when
        # starting voice independently or adopting an existing Nav2 stack.
        process_manager.start_step("waypoint")
    if _runtime_autostart_cancel.is_set():
        return
    if start_tcp and "tcp_listener" in process_manager.STEP_COMMANDS:
        process_manager.start_step("tcp_listener")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def require_login(request: Request, call_next):
    path = request.url.path
    public = path in {"/login", "/api/auth/login", "/static/login.js", "/static/login.css"}
    if public or request.method == "OPTIONS":
        return await call_next(request)
    if not auth_manager.valid_session(request.cookies.get("hyy_session")):
        if path.startswith("/api/"):
            return JSONResponse({"detail": "登录已失效"}, status_code=401)
        return RedirectResponse("/login", status_code=303)
    return await call_next(request)

app.include_router(status_router, prefix="/api")
app.include_router(system_router, prefix="/api")
app.include_router(navigation_router, prefix="/api")
app.include_router(waypoints_router, prefix="/api")
app.include_router(maps_router, prefix="/api")
app.include_router(inspection_tasks_router, prefix="/api")
app.include_router(auth_router, prefix="/api")
app.include_router(advanced_router, prefix="/api")

if FRONTEND_DIR.is_dir():
    app.mount(
        "/static",
        StaticFiles(directory=str(FRONTEND_DIR)),
        name="static",
    )


@app.get("/")
def frontend_index():
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/login")
def login_page():
    return FileResponse(FRONTEND_DIR / "login.html")


@app.on_event("startup")
def startup():
    global _runtime_autostart_thread
    _runtime_autostart_cancel.clear()
    for disabled_step in ("ultrasonic_stop",):
        if disabled_step in process_manager.STEP_COMMANDS:
            process_manager.stop_step(disabled_step)
    ros_status_bridge.start()
    autostart_requested = _env_enabled("HYY_AUTO_START_NAVIGATION") or _env_enabled(
        "HYY_AUTO_START_TCP"
    )
    if autostart_requested and not (
        _runtime_autostart_thread and _runtime_autostart_thread.is_alive()
    ):
        _runtime_autostart_thread = threading.Thread(
            target=_auto_start_runtime,
            name="runtime-autostart",
            daemon=True,
        )
        _runtime_autostart_thread.start()


@app.on_event("shutdown")
def shutdown():
    _runtime_autostart_cancel.set()
    # Navigation stop intentionally keeps infrastructure monitors alive so the
    # web UI can continue reporting an offline system. Once the web backend
    # itself exits, it owns and must clean up those monitor process groups.
    # Stop the bridge first so its graph watcher cannot immediately re-create
    # a monitor while shutdown cleanup is in progress.
    ros_status_bridge.stop()
    for monitor_step in ("pose_monitor", "status_monitor"):
        if monitor_step in process_manager.STEP_COMMANDS:
            process_manager.stop_step(monitor_step)

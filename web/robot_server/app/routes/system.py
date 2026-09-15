from fastapi import APIRouter
from fastapi import HTTPException
from pydantic import BaseModel

from robot_server.app.services.process_manager import process_manager
from robot_server.app.services.ros_status_bridge import ros_status_bridge
from robot_server.app.services.robot_profile import robot_profile_manager


router = APIRouter()


class StepRequest(BaseModel):
    step: str


class RobotProfileRequest(BaseModel):
    profile: str


class RuntimeConfigurationRequest(BaseModel):
    profile: str
    nav2_preset: str
    behavior_tree: str


class TcpConfigurationRequest(BaseModel):
    server_host: str
    server_port: int


@router.get("/robot-profiles")
def robot_profiles():
    return robot_profile_manager.snapshot()


@router.post("/robot-profile/select")
def select_robot_profile(request: RobotProfileRequest):
    try:
        # Validate before stopping anything so a bad request cannot interrupt
        # the currently running robot.
        robot_profile_manager.navigation_config(request.profile)
        system = process_manager.snapshot()
        running_steps = {
            step["key"] for step in system.get("steps", []) if step.get("running")
        }
        operational_steps = running_steps - process_manager.MONITOR_STEPS
        if system.get("shutdown_in_progress") or operational_steps:
            raise HTTPException(
                status_code=409,
                detail="请先停止导航系统和建图，再切换机器人类型",
            )
        result = robot_profile_manager.select(request.profile)
        result["restart_required"] = True
        return result
    except HTTPException:
        raise
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/configuration/apply")
def apply_runtime_configuration(request: RuntimeConfigurationRequest):
    from robot_server.app.services.config_editor import config_editor

    try:
        robot_profile_manager.navigation_config(request.profile)
        if not request.nav2_preset.startswith(f"{request.profile}."):
            raise ValueError("机器人类型与 Nav2 参数文件不匹配")
        config_editor.read_nav2_preset(request.nav2_preset)
        if not request.behavior_tree.startswith(f"{request.profile}."):
            raise ValueError("机器人类型与行为树不匹配")
        robot_profile_manager.behavior_tree_config(request.profile, request.behavior_tree)
        system = process_manager.snapshot()
        running_steps = {
            step["key"] for step in system.get("steps", []) if step.get("running")
        }
        if (
            system.get("shutdown_in_progress")
            or running_steps - process_manager.MONITOR_STEPS
        ):
            raise HTTPException(
                status_code=409,
                detail="请先停止导航系统和建图，再应用运行配置",
            )
        profile = robot_profile_manager.select(
            request.profile,
            request.nav2_preset,
            request.behavior_tree,
        )
        nav2 = config_editor.apply_nav2_preset(request.nav2_preset)
        return {"ok": True, "profile": profile, "nav2": nav2, "restart_required": True}
    except HTTPException:
        raise
    except (ValueError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/system/start")
def start_system():
    if not process_manager.start_navigation():
        raise HTTPException(status_code=409, detail="导航系统已启动或正在启动，请使用停止或重启")
    return process_manager.snapshot()


@router.post("/system/stop")
def stop_system():
    process_manager.stop_navigation()
    return process_manager.snapshot()


@router.post("/system/restart")
def restart_system():
    if not process_manager.restart_navigation():
        raise HTTPException(status_code=409, detail="旧导航进程尚未完全退出，未开始重复启动")
    return process_manager.snapshot()


@router.get("/system")
def system_snapshot():
    process_manager.ensure_monitors(ros_status_bridge.graph_snapshot())
    return process_manager.snapshot()


@router.post("/system/step/start")
def start_step(request: StepRequest):
    try:
        process_manager.start_step(request.step)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return process_manager.snapshot()


@router.post("/system/step/stop")
def stop_step(request: StepRequest):
    try:
        process_manager.stop_step(request.step)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return process_manager.snapshot()


@router.post("/system/step/restart")
def restart_step(request: StepRequest):
    try:
        process_manager.restart_step(request.step)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return process_manager.snapshot()


@router.post("/system/tcp-control/start")
def start_tcp_control():
    raise HTTPException(status_code=409, detail="Unitree A2 配置未启用语音 TCP 控制")


@router.post("/system/tcp-control/stop")
def stop_tcp_control():
    return process_manager.snapshot()


@router.post("/system/tcp-control/restart")
def restart_tcp_control():
    raise HTTPException(status_code=409, detail="Unitree A2 配置未启用语音 TCP 控制")


@router.post("/system/tcp-config")
def update_tcp_config(request: TcpConfigurationRequest):
    try:
        return robot_profile_manager.update_tcp_config(
            request.server_host, request.server_port
        )
    except (ValueError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

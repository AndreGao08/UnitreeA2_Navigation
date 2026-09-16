from typing import Literal

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, conlist

from robot_server.app.services.inspection_task import inspection_task_manager
from robot_server.app.services.process_manager import process_manager
from robot_server.app.services.ros_status_bridge import ros_status_bridge
from robot_server.app.services.status_store import status_store
from robot_server.app.services.waypoint_store import waypoint_store


router = APIRouter()


class InspectionStep(BaseModel):
    type: Literal["waypoint", "text", "wait"]
    waypoint_index: int | None = None
    waypoint_id: str | None = None
    seconds: float | None = None
    message: str = ""


class InspectionStartRequest(BaseModel):
    steps: conlist(InspectionStep, min_items=1, max_items=500)
    cycles: int = 1


@router.get("/inspection-task")
def inspection_status():
    return inspection_task_manager.snapshot()


@router.post("/inspection-task/start")
def start_inspection(request: InspectionStartRequest):
    if ros_status_bridge.navigation_command_busy():
        raise HTTPException(status_code=409, detail="已有导航命令正在执行，请先取消或等待完成")
    if request.cycles < 1 or request.cycles > 100:
        raise HTTPException(status_code=400, detail="cycles must be between 1 and 100")
    system = process_manager.snapshot()
    if not any(step["key"] == "nav2" and step["running"] for step in system["steps"]):
        raise HTTPException(status_code=409, detail="请先在主界面启动导航")
    status = status_store.snapshot()
    if not status.get("connected") or not status.get("map", {}).get("received") or not status.get("tf", {}).get("ok"):
        raise HTTPException(status_code=409, detail="请先完成地图加载和定位")
    waypoints = waypoint_store.list_waypoints()
    resolved = []
    for step in request.steps:
        if step.type == "waypoint":
            point = next(
                (item for item in waypoints if step.waypoint_id and item.get("id") == step.waypoint_id),
                None,
            )
            if point is None and step.waypoint_index is not None and 1 <= step.waypoint_index <= len(waypoints):
                point = waypoints[step.waypoint_index - 1]
            if point is None:
                raise HTTPException(status_code=400, detail="inspection waypoint is out of range")
            message = step.message.strip() or f"到达点位 {point.get('note') or point['index']}"
            if len(message) > 1000:
                raise HTTPException(status_code=400, detail="guide message cannot exceed 1000 characters")
            resolved.append({"type": "waypoint", "index": point["index"], "x": point["x"], "y": point["y"], "yaw": point["yaw"], "message": message})
        elif step.type == "text":
            message = step.message.strip()
            if not message or len(message) > 1000:
                raise HTTPException(status_code=400, detail="text block must contain 1 to 1000 characters")
            resolved.append({"type": "text", "message": message})
        else:
            if step.seconds is None or step.seconds < 0 or step.seconds > 86400:
                raise HTTPException(status_code=400, detail="wait time must be between 0 and 86400 seconds")
            resolved.append({"type": "wait", "seconds": float(step.seconds)})
    try:
        return inspection_task_manager.start(resolved, request.cycles)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@router.post("/inspection-task/cancel")
def cancel_inspection():
    return inspection_task_manager.cancel()

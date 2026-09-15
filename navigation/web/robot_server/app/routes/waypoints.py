from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from robot_server.app.services.waypoint_store import waypoint_store
from robot_server.app.services.status_store import status_store


router = APIRouter()


class WaypointCreateRequest(BaseModel):
    x: float
    y: float
    yaw: float
    note: str = ""


class WaypointCurrentRequest(BaseModel):
    note: str = ""


class WaypointReorderRequest(BaseModel):
    order: list[int]


@router.get("/waypoints")
def list_waypoints():
    return {
        "map_name": waypoint_store.current_map_name(),
        "initial_pose": waypoint_store.initial_pose(),
        "waypoints": waypoint_store.list_waypoints(),
    }


@router.post("/waypoints")
def create_waypoint(request: WaypointCreateRequest):
    try:
        waypoint = waypoint_store.append_waypoint(
            request.x, request.y, request.yaw, request.note
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {
        "waypoint": waypoint,
    }


@router.post("/waypoints/current")
def create_current_waypoint(request: WaypointCurrentRequest = WaypointCurrentRequest()):
    status = status_store.snapshot()
    pose = status.get("tf") or {}
    if not status.get("connected") or not status.get("map", {}).get("received") or not pose.get("ok"):
        raise HTTPException(
            status_code=409,
            detail="系统尚未完成定位，需同时具备地图和 map 到 base_link 的有效 TF",
        )
    try:
        waypoint = waypoint_store.append_waypoint(
            pose["x"], pose["y"], pose.get("yaw", 0.0), request.note,
            reject_last_duplicate=True,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"waypoint": waypoint}


@router.put("/waypoints/{index}")
def update_waypoint(index: int, request: WaypointCreateRequest):
    waypoint = waypoint_store.update_waypoint(
        index, request.x, request.y, request.yaw, request.note
    )
    if waypoint is None:
        raise HTTPException(status_code=404, detail="waypoint not found")
    return {"waypoint": waypoint}


@router.post("/waypoints/reorder")
def reorder_waypoints(request: WaypointReorderRequest):
    try:
        waypoints = waypoint_store.reorder_waypoints(request.order)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"waypoints": waypoints}


@router.delete("/waypoints/{index}")
def delete_waypoint(index: int):
    removed = waypoint_store.delete_waypoint(index)
    if removed is None:
        raise HTTPException(status_code=404, detail="waypoint not found")
    return {
        "removed": removed,
        "waypoints": waypoint_store.list_waypoints(),
    }

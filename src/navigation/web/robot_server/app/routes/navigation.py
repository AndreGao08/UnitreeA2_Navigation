from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, conlist

from robot_server.app.services.ros_status_bridge import ros_status_bridge
from robot_server.app.services.waypoint_store import waypoint_store
from robot_server.app.services.process_manager import process_manager
from robot_server.app.services.status_store import status_store


router = APIRouter()


class RouteStartRequest(BaseModel):
    waypoint_indices: conlist(int, min_items=1)
    loop: bool = False
    one_based_index: bool = True


class PoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class NavigationTestRequest(BaseModel):
    waypoint_indices: conlist(int, min_items=1)
    cycles: int = 1


class VelocityRequest(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


def checked(result):
    if not result.get("ok"):
        raise HTTPException(status_code=503, detail=result.get("message", "ROS command failed"))
    return result


@router.post("/navigation/cancel")
def cancel_navigation():
    return checked(ros_status_bridge.cancel_navigation())


@router.post("/navigation/route/start")
def start_route(request: RouteStartRequest):
    if ros_status_bridge.navigation_command_busy():
        raise HTTPException(status_code=409, detail="已有导航命令正在执行，请先取消或等待完成")
    indices = request.waypoint_indices
    if request.one_based_index:
        indices = [index - 1 for index in indices]

    if any(index < 0 for index in indices):
        raise HTTPException(status_code=400, detail="waypoint indices must be positive")
    waypoint_count = len(waypoint_store.list_waypoints())
    if any(index >= waypoint_count for index in indices):
        raise HTTPException(status_code=400, detail="waypoint index is out of range")

    return checked(ros_status_bridge.publish_waypoint_route(indices, loop=request.loop))


@router.post("/navigation/goal")
def navigation_goal(request: PoseRequest):
    if ros_status_bridge.navigation_command_busy():
        raise HTTPException(status_code=409, detail="已有导航命令正在执行，请先取消或等待完成")
    return checked(ros_status_bridge.publish_goal(request.x, request.y, request.yaw))


@router.post("/navigation/initial-pose")
def navigation_initial_pose(request: PoseRequest):
    return checked(ros_status_bridge.publish_initial_pose(request.x, request.y, request.yaw))


@router.post("/navigation/test/start")
def start_navigation_test(request: NavigationTestRequest):
    if ros_status_bridge.navigation_command_busy():
        raise HTTPException(status_code=409, detail="已有导航命令正在执行，请先取消或等待完成")
    system = process_manager.snapshot()
    nav2_running = any(
        step["key"] == "nav2" and step["running"] for step in system["steps"]
    )
    status = status_store.snapshot()
    if not nav2_running:
        raise HTTPException(status_code=409, detail="请先在主界面启动导航")
    if not status.get("connected") or not status.get("map", {}).get("received") or not status.get("tf", {}).get("ok"):
        raise HTTPException(status_code=409, detail="请先完成地图加载和定位")
    if request.cycles < 1 or request.cycles > 100:
        raise HTTPException(status_code=400, detail="cycles must be between 1 and 100")
    indices = [index - 1 for index in request.waypoint_indices]
    waypoint_count = len(waypoint_store.list_waypoints())
    if any(index < 0 or index >= waypoint_count for index in indices):
        raise HTTPException(status_code=400, detail="waypoint index is out of range")
    round_trip = indices + indices[-2::-1]
    sequence = []
    if len(indices) == 1:
        sequence = indices * request.cycles
    else:
        for cycle in range(request.cycles):
            sequence.extend(round_trip if cycle == 0 else round_trip[1:])
    return checked(ros_status_bridge.publish_waypoint_route(sequence, loop=False))


@router.post("/navigation/velocity")
def navigation_velocity(request: VelocityRequest):
    moving = any(abs(value) > 1e-6 for value in (
        request.linear_x, request.linear_y, request.angular_z
    ))
    if moving and ros_status_bridge.navigation_command_busy():
        raise HTTPException(status_code=409, detail="自动导航执行中，手动遥控已锁定")
    return checked(ros_status_bridge.publish_velocity(
        request.linear_x, request.linear_y, request.angular_z
    ))


@router.post("/navigation/emergency-stop")
def emergency_stop():
    return checked(ros_status_bridge.emergency_stop())

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from robot_server.app.services.status_store import status_store
from robot_server.app.services.ros_status_bridge import ros_status_bridge
from robot_server.app.services.system_resources import system_resource_monitor


router = APIRouter()


class PointcloudTopicRequest(BaseModel):
    topic: str


class PointcloudTopicsRequest(BaseModel):
    topics: list[str]


@router.get("/health")
def health():
    return {"ok": True}


@router.get("/status")
def status():
    result = status_store.snapshot()
    scans = ros_status_bridge.scan_health_snapshot()
    result["scans"] = scans
    result["pointcloud_topics"] = ros_status_bridge.pointcloud_topics_snapshot()
    result["pointcloud_health"] = ros_status_bridge.pointcloud_health_snapshot()
    result["waypoint_navigation"] = ros_status_bridge.waypoint_navigation_snapshot()
    result["status_monitor_connected"] = bool(result.get("connected"))
    sensor_connected = any(scan.get("recent") for scan in scans.values()) or bool(
        any(item.get("recent") for item in result["pointcloud_health"].values())
    )
    result["ros_data_connected"] = bool(result.get("connected")) or sensor_connected
    if sensor_connected and not result.get("connected"):
        result["message"] = "sensor data received; waiting for system status monitor"
    return result


@router.get("/status/resources")
def system_resources():
    result = system_resource_monitor.snapshot()
    result["battery"] = ros_status_bridge.battery_snapshot()
    return result


@router.get("/status/pose")
def pose_status():
    """Lightweight robot pose path, independent from point-cloud payloads."""
    runtime = status_store.snapshot()
    tf_status = runtime.get("tf") or {"ok": False}
    return {
        "pose": ros_status_bridge.display_pose_snapshot(),
        "tf_tree": ros_status_bridge.tf_tree_snapshot(),
        "velocity": ros_status_bridge.velocity_ready_snapshot(),
        "tf": tf_status,
        "tf_ok": bool(runtime.get("connected") and tf_status.get("ok")),
    }


@router.get("/status/visualization")
def visualization_status(
    v: str = "",
    local_costmap_version: int = -1,
    global_costmap_version: int = -1,
    live_map_version: int = -1,
    trajectory_version: int = -1,
):
    """Small, high-frequency payload used only by the WebGL canvas."""
    if v:
        try:
            versions = [int(value) for value in v.split(",")]
            if len(versions) != 4:
                raise ValueError
            (
                local_costmap_version,
                global_costmap_version,
                live_map_version,
                trajectory_version,
            ) = versions
        except ValueError as exc:
            raise HTTPException(status_code=400, detail="v 必须包含四个整数版本号") from exc
    snapshot = status_store.snapshot()
    motion = ros_status_bridge.motion_snapshot()
    return {
        "tf": snapshot.get("tf"),
        "tf_tree": ros_status_bridge.tf_tree_snapshot(),
        "pose": ros_status_bridge.display_pose_snapshot(),
        "scans": ros_status_bridge.scan_snapshot(),
        "pointclouds": ros_status_bridge.pointcloud_snapshot(),
        "pointcloud_topics": ros_status_bridge.pointcloud_topics_snapshot(),
        "pointcloud_health": ros_status_bridge.pointcloud_health_snapshot(),
        "paths": ros_status_bridge.path_snapshot(),
        "costmaps": ros_status_bridge.costmap_snapshot({
            "local": local_costmap_version,
            "global": global_costmap_version,
        }),
        # Keep the live projected map and mapping trajectory on the same
        # high-frequency path as the canvas.  The regular /status refresh is
        # intentionally slower and cannot drive a live mapping view reliably.
        "mapping": ros_status_bridge.mapping_snapshot(live_map_version, trajectory_version),
        "motion": motion["motion"],
        "goal_metrics": motion["goal"],
    }


@router.post("/status/pointcloud-topic")
def select_pointcloud_topic(request: PointcloudTopicRequest):
    try:
        return ros_status_bridge.select_pointcloud_topic(request.topic)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/status/pointcloud-topics")
def select_pointcloud_topics(request: PointcloudTopicsRequest):
    try:
        return ros_status_bridge.select_pointcloud_topics(request.topics)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

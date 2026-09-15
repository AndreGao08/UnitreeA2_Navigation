import os
import tempfile

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel

from robot_server.app.services.map_store import map_store
from robot_server.app.services.process_manager import process_manager


router = APIRouter()


class MapSelectRequest(BaseModel):
    name: str


class MappingRequest(BaseModel):
    name: str


class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


@router.get("/maps")
def list_maps():
    return map_store.list_maps()


@router.post("/maps/select")
def select_map(request: MapSelectRequest):
    try:
        system = process_manager.snapshot()
        running_steps = {
            step["key"] for step in system.get("steps", []) if step.get("running")
        }
        protected_steps = {"driver", "localization", "nav2", "waypoint", "mapping", "map_save"}
        if (
            system.get("startup_in_progress")
            or system.get("shutdown_in_progress")
            or running_steps & protected_steps
        ):
            raise HTTPException(
                status_code=409,
                detail="导航系统或建图正在运行，请先停止后再切换地图",
            )
        result = map_store.select_map(request.name)
    except HTTPException:
        raise
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    if not result.get("ok"):
        raise HTTPException(status_code=404, detail=result)
    return result


@router.put("/maps/{name}/initial-pose")
def update_map_initial_pose(name: str, request: InitialPoseRequest):
    try:
        pose = map_store.set_initial_pose(name, request.x, request.y, request.yaw)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "name": name, "initial_pose": pose}


@router.delete("/maps/{name}")
def delete_map(name: str):
    try:
        result = map_store.delete_map(name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    if result is None:
        raise HTTPException(status_code=404, detail="map not found")
    return result


@router.get("/maps/{name}/download")
def download_map(name: str):
    try:
        pgm_path = map_store.pgm_path(name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    if not pgm_path:
        raise HTTPException(status_code=404, detail="map PGM not found")
    return FileResponse(
        pgm_path, media_type="image/x-portable-graymap", filename=f"{name}.pgm"
    )


@router.post("/maps/upload")
async def upload_map(request: Request, name: str):
    fd, pgm_path = tempfile.mkstemp(prefix="map-upload-", suffix=".pgm")
    size = 0
    try:
        with os.fdopen(fd, "wb") as pgm_file:
            async for chunk in request.stream():
                size += len(chunk)
                if size > 256 * 1024 * 1024:
                    raise HTTPException(status_code=413, detail="upload exceeds 256 MB")
                pgm_file.write(chunk)
        if size == 0:
            raise HTTPException(status_code=400, detail="empty upload")
        try:
            return map_store.replace_pgm(name, pgm_path)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
    finally:
        if os.path.exists(pgm_path):
            os.unlink(pgm_path)


@router.post("/mapping/start")
def start_mapping(request: MappingRequest):
    try:
        system = process_manager.snapshot()
        running_steps = {
            step["key"] for step in system.get("steps", []) if step.get("running")
        }
        navigation_steps = {
            "driver", "localization", "nav2", "waypoint", "mission_service",
        }
        if (
            system.get("startup_in_progress")
            or system.get("shutdown_in_progress")
            or running_steps & navigation_steps
        ):
            raise HTTPException(
                status_code=409,
                detail="导航系统尚未完全关闭，不能开始建图",
            )
        if running_steps & {"mapping", "map_save"}:
            raise HTTPException(status_code=409, detail="建图或存图任务已在运行")
        result = map_store.prepare_map(request.name)
        process_manager.start_mapping()
    except HTTPException:
        raise
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    result["system"] = process_manager.snapshot()
    return result


@router.post("/mapping/stop")
def stop_mapping():
    process_manager.stop_mapping()
    return process_manager.snapshot()


@router.post("/mapping/save")
def save_mapping():
    process_manager.save_mapping()
    return process_manager.snapshot()


@router.post("/mapping/finish")
def finish_mapping():
    try:
        process_manager.finish_mapping()
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    return process_manager.snapshot()


@router.get("/maps/{name}/thumbnail.bmp")
def map_thumbnail(name: str):
    try:
        thumbnail_path = map_store.thumbnail_path(name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    if not thumbnail_path:
        raise HTTPException(status_code=404, detail="map thumbnail not found")
    return FileResponse(thumbnail_path, media_type="image/bmp")

from fastapi import APIRouter, HTTPException

from robot_server.app.services.config_editor import config_editor


router = APIRouter()


@router.get("/advanced/nav2-presets")
def list_nav2_presets():
    return config_editor.nav2_presets()


@router.get("/advanced/nav2-presets/{preset_id}")
def read_nav2_preset(preset_id: str):
    try:
        return config_editor.read_nav2_preset(preset_id)
    except (ValueError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

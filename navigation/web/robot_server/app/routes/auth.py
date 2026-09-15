from fastapi import APIRouter, HTTPException, Response
from pydantic import BaseModel

from robot_server.app.services.auth import auth_manager


router = APIRouter()


class LoginRequest(BaseModel):
    username: str
    password: str


@router.post("/auth/login")
def login(request: LoginRequest, response: Response):
    if not auth_manager.verify_login(request.username, request.password):
        raise HTTPException(status_code=401, detail="账户或密码错误")
    response.set_cookie(
        "hyy_session", auth_manager.create_session(request.username),
        httponly=True, samesite="strict", max_age=8 * 3600, path="/",
    )
    return {"ok": True}


@router.post("/auth/logout")
def logout(response: Response):
    response.delete_cookie("hyy_session", path="/")
    return {"ok": True}


@router.get("/auth/session")
def session_status():
    return {"authenticated": True}

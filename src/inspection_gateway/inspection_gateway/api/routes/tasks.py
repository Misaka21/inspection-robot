"""Task control endpoints: start / pause / resume / stop / status."""

from __future__ import annotations

import uuid

from fastapi import APIRouter, Depends

from ..deps import get_bridge, get_runtime
from ..models import (
    ControlTaskRequest,
    ControlTaskResponse,
    ErrorCode,
    GetTaskStatusResponse,
    Result,
    StartInspectionRequest,
    StartInspectionResponse,
)
from ...domain.converters import GatewayTaskContext, ros_system_state_to_task_status
from ...domain.runtime import GatewayRuntime
from ...ros.bridge import RosBridge

router = APIRouter(tags=["tasks"])


def _ctx(rt: GatewayRuntime) -> GatewayTaskContext:
    return GatewayTaskContext(task_id=rt.task_id, plan_id=rt.plan_id, task_name=rt.task_name)


@router.post("/tasks", response_model=StartInspectionResponse)
def start_inspection(
    body: StartInspectionRequest,
    bridge: RosBridge = Depends(get_bridge),
    runtime: GatewayRuntime = Depends(get_runtime),
):
    try:
        resp = bridge.start_inspection(plan_id=body.plan_id, dry_run=body.dry_run, timeout_s=3.0)
    except TimeoutError as ex:
        return StartInspectionResponse(result=Result(code=ErrorCode.TIMEOUT, message=str(ex)))
    except Exception as ex:
        return StartInspectionResponse(result=Result(code=ErrorCode.UNAVAILABLE, message=str(ex)))

    if not resp.success:
        return StartInspectionResponse(result=Result(code=ErrorCode.INTERNAL, message=str(resp.message)))

    runtime.task_id = str(uuid.uuid4())
    runtime.plan_id = body.plan_id
    runtime.task_name = ""
    return StartInspectionResponse(result=Result(code=ErrorCode.OK, message="ok"), task_id=runtime.task_id)


@router.post("/tasks/{task_id}/pause", response_model=ControlTaskResponse)
def pause_inspection(
    task_id: str,
    body: ControlTaskRequest = ControlTaskRequest(),
    bridge: RosBridge = Depends(get_bridge),
):
    try:
        resp = bridge.pause_inspection(task_id, body.reason, timeout_s=3.0)
    except TimeoutError as ex:
        return ControlTaskResponse(result=Result(code=ErrorCode.TIMEOUT, message=str(ex)))
    except Exception as ex:
        return ControlTaskResponse(result=Result(code=ErrorCode.UNAVAILABLE, message=str(ex)))

    code = ErrorCode.OK if resp.success else ErrorCode.INTERNAL
    return ControlTaskResponse(result=Result(code=code, message=str(resp.message)))


@router.post("/tasks/{task_id}/resume", response_model=ControlTaskResponse)
def resume_inspection(
    task_id: str,
    body: ControlTaskRequest = ControlTaskRequest(),
    bridge: RosBridge = Depends(get_bridge),
):
    try:
        resp = bridge.resume_inspection(task_id, body.reason, timeout_s=3.0)
    except TimeoutError as ex:
        return ControlTaskResponse(result=Result(code=ErrorCode.TIMEOUT, message=str(ex)))
    except Exception as ex:
        return ControlTaskResponse(result=Result(code=ErrorCode.UNAVAILABLE, message=str(ex)))

    code = ErrorCode.OK if resp.success else ErrorCode.INTERNAL
    return ControlTaskResponse(result=Result(code=code, message=str(resp.message)))


@router.post("/tasks/{task_id}/stop", response_model=ControlTaskResponse)
def stop_inspection(
    task_id: str,
    body: ControlTaskRequest = ControlTaskRequest(),
    bridge: RosBridge = Depends(get_bridge),
):
    try:
        resp = bridge.stop_inspection(task_id, body.reason, timeout_s=3.0)
    except TimeoutError as ex:
        return ControlTaskResponse(result=Result(code=ErrorCode.TIMEOUT, message=str(ex)))
    except Exception as ex:
        return ControlTaskResponse(result=Result(code=ErrorCode.UNAVAILABLE, message=str(ex)))

    code = ErrorCode.OK if resp.success else ErrorCode.INTERNAL
    return ControlTaskResponse(result=Result(code=code, message=str(resp.message)))


@router.get("/tasks/{task_id}/status", response_model=GetTaskStatusResponse)
def get_task_status(
    task_id: str,
    bridge: RosBridge = Depends(get_bridge),
    runtime: GatewayRuntime = Depends(get_runtime),
):
    try:
        resp = bridge.get_status(task_id, timeout_s=3.0)
    except TimeoutError as ex:
        return GetTaskStatusResponse(result=Result(code=ErrorCode.TIMEOUT, message=str(ex)))
    except Exception as ex:
        return GetTaskStatusResponse(result=Result(code=ErrorCode.UNAVAILABLE, message=str(ex)))

    if not resp.success:
        return GetTaskStatusResponse(result=Result(code=ErrorCode.INTERNAL, message=str(resp.message)))

    status = ros_system_state_to_task_status(resp.state, _ctx(runtime))
    if not status.task_id:
        status.task_id = task_id or runtime.task_id
    return GetTaskStatusResponse(result=Result(code=ErrorCode.OK, message="ok"), status=status)

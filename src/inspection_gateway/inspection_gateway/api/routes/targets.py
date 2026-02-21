"""Inspection targets endpoint."""

from __future__ import annotations

from fastapi import APIRouter, Depends

from ..deps import get_runtime
from ..models import ErrorCode, Result, SetTargetsRequest, SetTargetsResponse
from ...domain.runtime import GatewayRuntime

router = APIRouter(tags=["targets"])


@router.post("/targets", response_model=SetTargetsResponse)
def set_inspection_targets(
    body: SetTargetsRequest,
    runtime: GatewayRuntime = Depends(get_runtime),
):
    # Persist targets in runtime for later plan generation
    runtime.targets_by_model[body.model_id] = [t.model_dump() for t in body.targets]

    total = len(body.targets)
    return SetTargetsResponse(
        result=Result(code=ErrorCode.OK, message="ok"),
        total_targets=total,
    )

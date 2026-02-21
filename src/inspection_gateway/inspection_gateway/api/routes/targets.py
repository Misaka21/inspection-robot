"""Inspection targets endpoint."""

from __future__ import annotations

from fastapi import APIRouter

from ..models import ErrorCode, Result, SetTargetsRequest, SetTargetsResponse

router = APIRouter(tags=["targets"])


@router.post("/targets", response_model=SetTargetsResponse)
def set_inspection_targets(body: SetTargetsRequest):
    total = len(body.targets)
    return SetTargetsResponse(
        result=Result(code=ErrorCode.OK, message="ok"),
        total_targets=total,
    )

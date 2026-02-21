"""Capture listing endpoint."""

from __future__ import annotations

from fastapi import APIRouter, Query

from ..models import ErrorCode, ListCapturesResponse, Result

router = APIRouter(tags=["captures"])


@router.get("/tasks/{task_id}/captures", response_model=ListCapturesResponse)
def list_captures(
    task_id: str,
    point_id: int = Query(0, description="Filter by point_id (0 = all)"),
    include_thumbnails: bool = Query(False),
):
    return ListCapturesResponse(
        result=Result(code=ErrorCode.UNAVAILABLE, message="ListCaptures not implemented"),
    )

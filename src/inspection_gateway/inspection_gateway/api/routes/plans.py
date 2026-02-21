"""Inspection plan endpoints."""

from __future__ import annotations

from fastapi import APIRouter

from ..models import (
    ErrorCode,
    GetPlanResponse,
    PlanInspectionRequest,
    PlanInspectionResponse,
    Result,
)

router = APIRouter(tags=["plans"])


@router.post("/plans", response_model=PlanInspectionResponse)
def plan_inspection(body: PlanInspectionRequest):
    return PlanInspectionResponse(
        result=Result(code=ErrorCode.UNAVAILABLE, message="PlanInspection not implemented"),
    )


@router.get("/plans/{plan_id}", response_model=GetPlanResponse)
def get_plan(plan_id: str):
    return GetPlanResponse(
        result=Result(code=ErrorCode.NOT_FOUND, message="plan not found"),
    )

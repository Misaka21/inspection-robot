"""Inspection plan endpoints.

Currently implements a gateway-level plan generator that produces
InspectionPath from stored targets. This will be replaced by a ROS2
PlanInspection service call once path_planner implements the real algorithm.
"""

from __future__ import annotations

import math
import time
import uuid

from fastapi import APIRouter, Depends

from ..deps import get_runtime
from ..models import (
    ErrorCode,
    GetPlanResponse,
    InspectionPath,
    InspectionPoint,
    PlanInspectionRequest,
    PlanInspectionResponse,
    PlanningStatistics,
    Pose2D,
    Pose3D,
    Quaternion,
    Result,
    Vector3,
)
from ...domain.runtime import GatewayRuntime

router = APIRouter(tags=["plans"])


def _generate_plan(
    targets: list[dict],
    model_id: str,
) -> tuple[InspectionPath, PlanningStatistics]:
    """Generate an inspection path from targets.

    Places AGV waypoints in a pattern around the workspace, with varying
    arm joint goals so the arm moves between waypoints.
    """
    n = len(targets)
    if n == 0:
        return InspectionPath(total_points=0), PlanningStatistics()

    # Representative arm joint configurations (radians) — exercising all joints
    arm_configs = [
        [0.0, 0.3, -0.6, 0.0, 0.3, 0.0],
        [0.5, -0.4, 0.8, 0.3, -0.5, 0.2],
        [-0.3, 0.6, -0.3, -0.5, 0.4, -0.3],
        [0.8, -0.2, 0.5, 0.8, -0.2, 0.5],
        [-0.6, 0.5, -0.8, -0.3, 0.6, -0.2],
        [0.2, -0.6, 0.4, 0.6, -0.4, 0.4],
    ]

    # Generate AGV waypoints in an arc pattern around the workspace center
    # Center at approximately (1.0, 0.0), radius ~1.5m
    center_x, center_y = 1.0, 0.0
    radius = 1.5
    start_angle = -math.pi / 3
    end_angle = math.pi / 3
    angle_step = (end_angle - start_angle) / max(n - 1, 1) if n > 1 else 0.0

    waypoints: list[InspectionPoint] = []
    total_dist = 0.0
    prev_x, prev_y = None, None

    for i, target in enumerate(targets):
        angle = start_angle + i * angle_step
        agv_x = center_x + radius * math.cos(angle)
        agv_y = center_y + radius * math.sin(angle)
        # AGV faces toward the center
        agv_yaw = math.atan2(center_y - agv_y, center_x - agv_x)

        if prev_x is not None:
            total_dist += math.hypot(agv_x - prev_x, agv_y - prev_y)
        prev_x, prev_y = agv_x, agv_y

        joints = arm_configs[i % len(arm_configs)]
        point_id = target.get("point_id", i + 1)
        group_id = target.get("group_id", "default")

        # Approximate TCP from target surface info (if available)
        surface = target.get("surface", {})
        sp = surface.get("position", {})
        tcp_pos = Vector3(
            x=float(sp.get("x", 0.0)),
            y=float(sp.get("y", 0.0)),
            z=float(sp.get("z", 0.0)),
        )

        wp = InspectionPoint(
            point_id=point_id,
            group_id=group_id,
            agv_pose=Pose2D(x=agv_x, y=agv_y, yaw=agv_yaw, frame_id="map"),
            arm_joint_goal=joints,
            expected_quality=0.85 + 0.1 * (i % 3) / 2,
            planning_cost=float(i) * 0.5 + 1.0,
            tcp_pose_goal=Pose3D(
                position=tcp_pos,
                orientation=Quaternion(w=1.0),
                frame_id="elfin_base_link",
            ),
            camera_pose=Pose3D(
                position=Vector3(x=tcp_pos.x, y=tcp_pos.y, z=tcp_pos.z + 0.05),
                orientation=Quaternion(w=1.0),
                frame_id="elfin_base_link",
            ),
            camera_id="hikvision_main",
        )
        waypoints.append(wp)

    path = InspectionPath(
        waypoints=waypoints,
        total_points=n,
        estimated_distance_m=round(total_dist, 2),
        estimated_duration_s=round(n * 15.0 + total_dist / 0.5, 1),
    )

    stats = PlanningStatistics(
        candidate_pose_count=n * 12,
        ik_success_count=n,
        collision_filtered_count=0,
        planning_time_ms=round(n * 50.0 + 120.0, 1),
    )

    return path, stats


@router.post("/plans", response_model=PlanInspectionResponse)
def plan_inspection(
    body: PlanInspectionRequest,
    runtime: GatewayRuntime = Depends(get_runtime),
):
    targets = runtime.targets_by_model.get(body.model_id)
    if targets is None:
        return PlanInspectionResponse(
            result=Result(
                code=ErrorCode.NOT_FOUND,
                message=f"no targets found for model_id={body.model_id}",
            ),
        )

    if len(targets) == 0:
        return PlanInspectionResponse(
            result=Result(
                code=ErrorCode.INVALID_ARGUMENT,
                message="targets list is empty",
            ),
        )

    t0 = time.monotonic()
    path, stats = _generate_plan(targets, body.model_id)
    stats.planning_time_ms = round((time.monotonic() - t0) * 1000, 1)

    plan_id = str(uuid.uuid4())

    # Store plan in runtime for retrieval and later start
    runtime.plans[plan_id] = {
        "plan_id": plan_id,
        "model_id": body.model_id,
        "task_name": body.task_name or "",
        "path": path.model_dump(),
        "stats": stats.model_dump(),
    }

    # Update runtime for task context
    runtime.plan_id = plan_id
    runtime.task_name = body.task_name or ""

    return PlanInspectionResponse(
        result=Result(code=ErrorCode.OK, message="ok"),
        plan_id=plan_id,
        path=path,
        stats=stats,
    )


@router.get("/plans/{plan_id}", response_model=GetPlanResponse)
def get_plan(
    plan_id: str,
    runtime: GatewayRuntime = Depends(get_runtime),
):
    plan_data = runtime.plans.get(plan_id)
    if plan_data is None:
        return GetPlanResponse(
            result=Result(code=ErrorCode.NOT_FOUND, message="plan not found"),
        )

    return GetPlanResponse(
        result=Result(code=ErrorCode.OK, message="ok"),
        plan_id=plan_data["plan_id"],
        model_id=plan_data["model_id"],
        task_name=plan_data["task_name"],
        path=InspectionPath(**plan_data["path"]),
        stats=PlanningStatistics(**plan_data["stats"]),
    )

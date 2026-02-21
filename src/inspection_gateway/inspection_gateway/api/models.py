"""Pydantic v2 data models — single source of truth for the REST / WebSocket API contract.

These models replace the protobuf message definitions from inspection-api/proto/inspection_gateway.proto.
FastAPI uses them for automatic request validation, response serialization, and OpenAPI doc generation.
"""

from __future__ import annotations

from datetime import datetime
from enum import IntEnum
from typing import Optional

from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------

class ErrorCode(IntEnum):
    UNSPECIFIED = 0
    OK = 1
    INVALID_ARGUMENT = 2
    NOT_FOUND = 3
    TIMEOUT = 4
    BUSY = 5
    INTERNAL = 6
    UNAVAILABLE = 7
    CONFLICT = 8


class Result(BaseModel):
    code: ErrorCode = ErrorCode.OK
    message: str = ""


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

class Pose2D(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    frame_id: str = "map"


class Vector3(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Quaternion(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


class Pose3D(BaseModel):
    position: Vector3 = Field(default_factory=Vector3)
    orientation: Quaternion = Field(default_factory=Quaternion)
    frame_id: str = ""


class SurfacePoint(BaseModel):
    position: Vector3 = Field(default_factory=Vector3)
    normal: Vector3 = Field(default_factory=Vector3)
    frame_id: str = ""
    face_index: int = 0


class ViewHint(BaseModel):
    view_direction: Vector3 = Field(default_factory=Vector3)
    roll_deg: float = 0.0


# ---------------------------------------------------------------------------
# Media references
# ---------------------------------------------------------------------------

class MediaRef(BaseModel):
    media_id: str = ""
    mime_type: str = ""
    size_bytes: int = 0
    sha256: str = ""
    url: str = ""


class ImageRef(BaseModel):
    media: MediaRef = Field(default_factory=MediaRef)
    width: int = 0
    height: int = 0
    thumbnail_jpeg: Optional[str] = None  # base64 encoded


# ---------------------------------------------------------------------------
# Defect / detection
# ---------------------------------------------------------------------------

class BoundingBox2D(BaseModel):
    x: int = 0
    y: int = 0
    w: int = 0
    h: int = 0


class DefectResult(BaseModel):
    has_defect: bool = False
    defect_type: str = ""
    confidence: float = 0.0
    bbox: BoundingBox2D = Field(default_factory=BoundingBox2D)


# ---------------------------------------------------------------------------
# Capture configuration
# ---------------------------------------------------------------------------

class CaptureConfig(BaseModel):
    camera_id: str = ""
    focus_distance_m: float = 0.0
    fov_h_deg: float = 0.0
    fov_v_deg: float = 0.0
    max_tilt_from_normal_deg: float = 0.0


# ---------------------------------------------------------------------------
# Inspection target / plan
# ---------------------------------------------------------------------------

class InspectionTarget(BaseModel):
    point_id: int = 0
    group_id: str = ""
    surface: SurfacePoint = Field(default_factory=SurfacePoint)
    view: ViewHint = Field(default_factory=ViewHint)


class InspectionPoint(BaseModel):
    point_id: int = 0
    group_id: str = ""
    agv_pose: Pose2D = Field(default_factory=Pose2D)
    arm_pose: Pose3D = Field(default_factory=Pose3D)
    arm_joint_goal: list[float] = Field(default_factory=list)
    expected_quality: float = 0.0
    planning_cost: float = 0.0
    tcp_pose_goal: Pose3D = Field(default_factory=Pose3D)
    camera_pose: Pose3D = Field(default_factory=Pose3D)
    camera_id: str = ""


class InspectionPath(BaseModel):
    waypoints: list[InspectionPoint] = Field(default_factory=list)
    total_points: int = 0
    estimated_distance_m: float = 0.0
    estimated_duration_s: float = 0.0


class PlanningWeights(BaseModel):
    w_agv_distance: float = 0.0
    w_joint_delta: float = 0.0
    w_manipulability: float = 0.0
    w_view_error: float = 0.0
    w_joint_limit: float = 0.0


class PlanOptions(BaseModel):
    candidate_radius_m: float = 0.0
    candidate_yaw_step_deg: float = 0.0
    enable_collision_check: bool = True
    enable_tsp_optimization: bool = True
    ik_solver: str = ""
    weights: PlanningWeights = Field(default_factory=PlanningWeights)


class PlanningStatistics(BaseModel):
    candidate_pose_count: int = 0
    ik_success_count: int = 0
    collision_filtered_count: int = 0
    planning_time_ms: float = 0.0


# ---------------------------------------------------------------------------
# Task status
# ---------------------------------------------------------------------------

class TaskPhase(IntEnum):
    UNSPECIFIED = 0
    IDLE = 1
    LOCALIZING = 2
    PLANNING = 3
    EXECUTING = 4
    PAUSED = 5
    COMPLETED = 6
    FAILED = 7
    STOPPED = 8


class AgvStatus(BaseModel):
    connected: bool = False
    arrived: bool = False
    moving: bool = False
    stopped: bool = False
    current_pose: Pose2D = Field(default_factory=Pose2D)
    battery_percent: float = 0.0
    error_code: str = ""
    linear_velocity_mps: float = 0.0
    angular_velocity_rps: float = 0.0
    goal_pose: Optional[Pose2D] = None
    map_id: str = ""
    localization_quality: float = 0.0


class ArmStatus(BaseModel):
    connected: bool = False
    arrived: bool = False
    moving: bool = False
    current_joints: list[float] = Field(default_factory=list)
    manipulability: float = 0.0
    error_code: str = ""
    servo_enabled: bool = False
    tcp_pose: Optional[Pose3D] = None
    base_pose: Optional[Pose3D] = None


class TaskStatus(BaseModel):
    task_id: str = ""
    phase: TaskPhase = TaskPhase.UNSPECIFIED
    progress_percent: float = 0.0
    current_action: str = ""
    error_message: str = ""
    agv: AgvStatus = Field(default_factory=AgvStatus)
    arm: ArmStatus = Field(default_factory=ArmStatus)
    updated_at: Optional[datetime] = None
    plan_id: str = ""
    task_name: str = ""
    current_waypoint_index: int = 0
    current_point_id: int = 0
    total_waypoints: int = 0
    interlock_ok: bool = False
    interlock_message: str = ""
    remaining_time_est_s: float = 0.0
    started_at: Optional[datetime] = None
    finished_at: Optional[datetime] = None


# ---------------------------------------------------------------------------
# Navigation map
# ---------------------------------------------------------------------------

class NavMapInfo(BaseModel):
    map_id: str = ""
    name: str = ""
    resolution_m_per_pixel: float = 0.0
    width: int = 0
    height: int = 0
    origin: Pose2D = Field(default_factory=Pose2D)
    image: ImageRef = Field(default_factory=ImageRef)
    updated_at: Optional[datetime] = None


# ---------------------------------------------------------------------------
# Events
# ---------------------------------------------------------------------------

class InspectionEventType(IntEnum):
    UNSPECIFIED = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    CAPTURED = 4
    DEFECT_FOUND = 5


class InspectionEvent(BaseModel):
    task_id: str = ""
    point_id: int = 0
    type: InspectionEventType = InspectionEventType.UNSPECIFIED
    message: str = ""
    defect: Optional[DefectResult] = None
    timestamp: Optional[datetime] = None
    capture_id: str = ""
    camera_id: str = ""
    image: Optional[ImageRef] = None
    defects: list[DefectResult] = Field(default_factory=list)
    camera_pose: Optional[Pose3D] = None


# ---------------------------------------------------------------------------
# Capture records
# ---------------------------------------------------------------------------

class CaptureRecord(BaseModel):
    task_id: str = ""
    point_id: int = 0
    capture_id: str = ""
    camera_id: str = ""
    image: ImageRef = Field(default_factory=ImageRef)
    defects: list[DefectResult] = Field(default_factory=list)
    captured_at: Optional[datetime] = None


# ---------------------------------------------------------------------------
# Request / Response models for REST endpoints
# ---------------------------------------------------------------------------

class UploadCadResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    model_id: str = ""
    total_bytes: int = 0


class SetTargetsRequest(BaseModel):
    model_id: str
    targets: list[InspectionTarget]
    capture: CaptureConfig = Field(default_factory=CaptureConfig)
    operator_id: str = ""


class SetTargetsResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    total_targets: int = 0


class PlanInspectionRequest(BaseModel):
    model_id: str
    task_name: str = ""
    options: PlanOptions = Field(default_factory=PlanOptions)


class PlanInspectionResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    plan_id: str = ""
    path: Optional[InspectionPath] = None
    stats: Optional[PlanningStatistics] = None


class GetPlanResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    plan_id: str = ""
    model_id: str = ""
    task_name: str = ""
    options: Optional[PlanOptions] = None
    path: Optional[InspectionPath] = None
    stats: Optional[PlanningStatistics] = None
    created_at: Optional[datetime] = None


class StartInspectionRequest(BaseModel):
    plan_id: str
    dry_run: bool = False


class StartInspectionResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    task_id: str = ""


class ControlTaskRequest(BaseModel):
    reason: str = ""


class ControlTaskResponse(BaseModel):
    result: Result = Field(default_factory=Result)


class GetTaskStatusResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    status: Optional[TaskStatus] = None


class GetNavMapResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    map: Optional[NavMapInfo] = None


class ListCapturesResponse(BaseModel):
    result: Result = Field(default_factory=Result)
    captures: list[CaptureRecord] = Field(default_factory=list)

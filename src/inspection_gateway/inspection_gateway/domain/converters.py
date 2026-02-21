"""Pure conversion functions: ROS2 messages -> Pydantic models.

No ROS2 or gRPC imports at module level.  ROS message objects are accessed
through duck-typing (getattr) so this module stays unit-testable without a
running ROS2 environment.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timezone

from ..api.models import (
    AgvStatus,
    ArmStatus,
    ImageRef,
    MediaRef,
    NavMapInfo,
    Pose2D,
    TaskPhase,
    TaskStatus,
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


@dataclass(frozen=True)
class GatewayTaskContext:
    task_id: str = ""
    plan_id: str = ""
    task_name: str = ""


def ros_agv_status_to_model(ros_msg) -> AgvStatus:
    pose = Pose2D(
        x=float(ros_msg.current_pose.position.x),
        y=float(ros_msg.current_pose.position.y),
        yaw=float(
            _yaw_from_quaternion(
                ros_msg.current_pose.orientation.x,
                ros_msg.current_pose.orientation.y,
                ros_msg.current_pose.orientation.z,
                ros_msg.current_pose.orientation.w,
            )
        ),
        frame_id=str(ros_msg.frame_id or "map"),
    )
    return AgvStatus(
        connected=bool(ros_msg.connected),
        arrived=bool(ros_msg.arrived),
        moving=bool(ros_msg.moving),
        stopped=bool(ros_msg.stopped),
        current_pose=pose,
        battery_percent=float(ros_msg.battery_percent),
        error_code=str(ros_msg.error_code),
        linear_velocity_mps=float(ros_msg.linear_speed),
        angular_velocity_rps=float(ros_msg.angular_speed),
    )


def ros_arm_status_to_model(ros_msg) -> ArmStatus:
    return ArmStatus(
        connected=bool(ros_msg.connected),
        arrived=bool(ros_msg.arrived),
        moving=bool(ros_msg.moving),
        current_joints=[float(x) for x in ros_msg.current_joints],
        manipulability=float(ros_msg.manipulability),
        error_code=str(ros_msg.error_code),
        servo_enabled=False,
    )


def ros_system_state_to_task_status(ros_state, ctx: GatewayTaskContext) -> TaskStatus:
    task_id = str(ros_state.task_id) if getattr(ros_state, "task_id", "") else ctx.task_id

    interlock_ok = bool(
        ros_state.agv.connected
        and ros_state.agv.arrived
        and ros_state.agv.stopped
        and (ros_state.agv.error_code == "OK")
    )
    interlock_message = "OK" if interlock_ok else "WAIT_AGV_READY"

    phase_val = int(ros_state.phase)
    phase = TaskPhase(phase_val) if 0 <= phase_val <= 8 else TaskPhase.UNSPECIFIED

    return TaskStatus(
        task_id=task_id,
        phase=phase,
        progress_percent=float(ros_state.progress_percent),
        current_action=str(ros_state.current_action),
        error_message=str(ros_state.error_message),
        agv=ros_agv_status_to_model(ros_state.agv),
        arm=ros_arm_status_to_model(ros_state.arm),
        updated_at=_now_utc(),
        plan_id=ctx.plan_id,
        task_name=ctx.task_name,
        current_waypoint_index=0,
        current_point_id=0,
        total_waypoints=0,
        interlock_ok=interlock_ok,
        interlock_message=interlock_message,
        remaining_time_est_s=0.0,
    )


def ros_nav_map_to_model(map_resp, media_ref: MediaRef | None) -> NavMapInfo:
    yaw = _yaw_from_quaternion(
        map_resp.origin.orientation.x,
        map_resp.origin.orientation.y,
        map_resp.origin.orientation.z,
        map_resp.origin.orientation.w,
    )
    origin = Pose2D(
        x=float(map_resp.origin.position.x),
        y=float(map_resp.origin.position.y),
        yaw=float(yaw),
        frame_id=str(map_resp.frame_id or "map"),
    )

    image = ImageRef()
    if media_ref is not None:
        image.media = media_ref
        image.width = int(map_resp.width_px)
        image.height = int(map_resp.height_px)

    import base64
    if getattr(map_resp, "thumbnail_jpeg", None):
        image.thumbnail_jpeg = base64.b64encode(bytes(map_resp.thumbnail_jpeg)).decode("ascii")

    map_id = f"agv:{map_resp.map_name}:{map_resp.map_md5}" if map_resp.map_name else ""

    return NavMapInfo(
        map_id=map_id,
        name=str(map_resp.map_name),
        resolution_m_per_pixel=float(map_resp.resolution_m_per_pixel),
        width=int(map_resp.width_px),
        height=int(map_resp.height_px),
        origin=origin,
        image=image,
        updated_at=_now_utc(),
    )

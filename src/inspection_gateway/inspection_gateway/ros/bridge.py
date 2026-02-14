import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from inspection_interface.msg import SystemState
from inspection_interface.srv import (
    GetInspectionStatus,
    GetNavMap,
    PauseInspection,
    ResumeInspection,
    StartInspection,
    StopInspection,
)

from .state_hub import StateHub


@dataclass(frozen=True)
class RosNames:
    root_ns: str = "/inspection"

    @property
    def start_srv(self) -> str:
        return f"{self.root_ns}/start"

    @property
    def pause_srv(self) -> str:
        return f"{self.root_ns}/pause"

    @property
    def resume_srv(self) -> str:
        return f"{self.root_ns}/resume"

    @property
    def stop_srv(self) -> str:
        return f"{self.root_ns}/stop"

    @property
    def get_status_srv(self) -> str:
        return f"{self.root_ns}/get_status"

    @property
    def system_state_topic(self) -> str:
        return f"{self.root_ns}/state"

    @property
    def get_nav_map_srv(self) -> str:
        return f"{self.root_ns}/agv/get_nav_map"


class RosBridge(Node):
    def __init__(self, names: RosNames, state_hub: StateHub):
        super().__init__("inspection_gateway_ros")
        self._names = names
        self._state_hub = state_hub

        self._state_sub = self.create_subscription(
            SystemState,
            self._names.system_state_topic,
            self._on_system_state,
            10,
        )

        self._start_cli = self.create_client(StartInspection, self._names.start_srv)
        self._pause_cli = self.create_client(PauseInspection, self._names.pause_srv)
        self._resume_cli = self.create_client(ResumeInspection, self._names.resume_srv)
        self._stop_cli = self.create_client(StopInspection, self._names.stop_srv)
        self._get_status_cli = self.create_client(GetInspectionStatus, self._names.get_status_srv)
        self._get_nav_map_cli = self.create_client(GetNavMap, self._names.get_nav_map_srv)

    def _on_system_state(self, msg: SystemState) -> None:
        self._state_hub.publish(msg)

    def _call(self, client, req, service_name: str, timeout_s: float):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"service not available: {service_name}")

        event = threading.Event()
        out = {"resp": None, "exc": None}

        future = client.call_async(req)

        def _done(_fut):
            try:
                out["resp"] = _fut.result()
            except Exception as ex:  # pragma: no cover
                out["exc"] = ex
            finally:
                event.set()

        future.add_done_callback(_done)

        if not event.wait(timeout_s):
            raise TimeoutError("service call timeout")
        if out["exc"] is not None:
            raise out["exc"]
        return out["resp"]

    def start_inspection(self, plan_id: str, dry_run: bool, timeout_s: float):
        req = StartInspection.Request()
        req.legacy_task_id = 0
        req.plan_id = plan_id
        req.inspection_type = ""
        req.dry_run = bool(dry_run)
        return self._call(self._start_cli, req, self._names.start_srv, timeout_s)

    def pause_inspection(self, task_id: str, reason: str, timeout_s: float):
        req = PauseInspection.Request()
        req.task_id = task_id
        req.reason = reason
        return self._call(self._pause_cli, req, self._names.pause_srv, timeout_s)

    def resume_inspection(self, task_id: str, reason: str, timeout_s: float):
        req = ResumeInspection.Request()
        req.task_id = task_id
        req.reason = reason
        return self._call(self._resume_cli, req, self._names.resume_srv, timeout_s)

    def stop_inspection(self, task_id: str, reason: str, timeout_s: float):
        req = StopInspection.Request()
        req.legacy_task_id = 0
        req.task_id = task_id
        req.reason = reason
        return self._call(self._stop_cli, req, self._names.stop_srv, timeout_s)

    def get_status(self, task_id: str, timeout_s: float):
        req = GetInspectionStatus.Request()
        req.legacy_task_id = 0
        req.task_id = task_id
        return self._call(self._get_status_cli, req, self._names.get_status_srv, timeout_s)

    def get_nav_map(self, map_name: str, include_thumbnail: bool, timeout_s: float):
        req = GetNavMap.Request()
        req.map_name = map_name
        req.include_image_data = True
        req.include_thumbnail = bool(include_thumbnail)
        req.include_smap_json = False
        return self._call(self._get_nav_map_cli, req, self._names.get_nav_map_srv, timeout_s)

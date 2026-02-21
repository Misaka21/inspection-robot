"""Mutable runtime context for the gateway process.

Holds task_id, plan_id, task_name that are assigned when StartInspection
succeeds and read by state converters and WebSocket handlers.

Also stores targets and generated plans for the gateway-level planner.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class GatewayRuntime:
    task_id: str = ""
    plan_id: str = ""
    task_name: str = ""

    # model_id -> list of target dicts (from SetTargetsRequest)
    targets_by_model: dict[str, list[Any]] = field(default_factory=dict)

    # plan_id -> plan dict (PlanInspectionResponse payload)
    plans: dict[str, dict[str, Any]] = field(default_factory=dict)

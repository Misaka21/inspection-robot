"""Mutable runtime context for the gateway process.

Holds task_id, task_name that are assigned when StartInspection
succeeds and read by state converters and WebSocket handlers.
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

"""Mutable runtime context for the gateway process.

Holds task_id, plan_id, task_name that are assigned when StartInspection
succeeds and read by state converters and WebSocket handlers.
"""

from dataclasses import dataclass


@dataclass
class GatewayRuntime:
    task_id: str = ""
    plan_id: str = ""
    task_name: str = ""

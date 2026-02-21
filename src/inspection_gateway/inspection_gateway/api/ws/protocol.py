"""WebSocket message envelope definition.

All messages over the ``/ws`` endpoint use a uniform envelope:

.. code-block:: json

    {
        "type": "system_state" | "inspection_event" | "error" | "ping" | "pong",
        "seq": 42,
        "timestamp": "2026-02-21T10:30:00.123Z",
        "payload": { ... }
    }
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from pydantic import BaseModel, Field


class WsEnvelope(BaseModel):
    type: str
    seq: int = 0
    timestamp: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    payload: Any = None

    def to_json_bytes(self) -> bytes:
        return self.model_dump_json(by_alias=True).encode("utf-8")

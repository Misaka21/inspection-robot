"""WebSocket endpoint ``/ws``.

Subscribes to :class:`StateHub` and pushes ``system_state`` messages to all
connected clients.  Handles ``ping`` / ``pong`` keep-alive.
"""

from __future__ import annotations

import asyncio
import logging
import queue
from datetime import datetime, timezone
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from .protocol import WsEnvelope

log = logging.getLogger("inspection_gateway.ws")

ws_router = APIRouter()


def _build_envelope(msg_type: str, payload: Any, seq: int) -> str:
    env = WsEnvelope(
        type=msg_type,
        seq=seq,
        timestamp=datetime.now(timezone.utc),
        payload=payload,
    )
    return env.model_dump_json()


@ws_router.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()

    state_hub = ws.app.state.state_hub
    runtime = ws.app.state.runtime

    sub_id, q = state_hub.subscribe()
    seq = 0
    log.info("WebSocket client connected (sub_id=%d)", sub_id)

    try:
        # Send initial snapshot if available
        snap = state_hub.get_last()
        if snap is not None:
            from ...domain.converters import GatewayTaskContext, ros_system_state_to_task_status

            ctx = GatewayTaskContext(task_id=runtime.task_id, plan_id=runtime.plan_id, task_name=runtime.task_name)
            status = ros_system_state_to_task_status(snap.ros_state, ctx)
            seq += 1
            await ws.send_text(_build_envelope("system_state", status.model_dump(mode="json"), seq))

        while True:
            # Non-blocking poll of the StateHub queue via asyncio
            try:
                ros_state = await asyncio.wait_for(
                    asyncio.get_event_loop().run_in_executor(None, lambda: q.get(0.5)),
                    timeout=1.0,
                )
            except (asyncio.TimeoutError, queue.Empty):
                # Check if client sent anything (ping or close)
                try:
                    msg = await asyncio.wait_for(ws.receive_text(), timeout=0.01)
                    if msg:
                        import json

                        try:
                            data = json.loads(msg)
                            if data.get("type") == "ping":
                                seq += 1
                                await ws.send_text(_build_envelope("pong", None, seq))
                        except (json.JSONDecodeError, TypeError):
                            pass
                except asyncio.TimeoutError:
                    pass
                continue

            from ...domain.converters import GatewayTaskContext, ros_system_state_to_task_status

            ctx = GatewayTaskContext(task_id=runtime.task_id, plan_id=runtime.plan_id, task_name=runtime.task_name)
            status = ros_system_state_to_task_status(ros_state, ctx)
            seq += 1
            await ws.send_text(_build_envelope("system_state", status.model_dump(mode="json"), seq))

    except WebSocketDisconnect:
        log.info("WebSocket client disconnected (sub_id=%d)", sub_id)
    except Exception:
        log.exception("WebSocket error (sub_id=%d)", sub_id)
    finally:
        state_hub.unsubscribe(sub_id)

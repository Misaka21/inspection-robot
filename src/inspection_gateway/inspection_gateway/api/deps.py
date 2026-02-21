"""Dependency injection for FastAPI route handlers.

All shared objects (RosBridge, StateHub, stores, runtime) are stored in
``app.state`` by :func:`create_app` and retrieved here via ``request.app.state``.
"""

from __future__ import annotations

from fastapi import Request

from ..domain.runtime import GatewayRuntime
from ..ros.bridge import RosBridge
from ..ros.state_hub import StateHub
from ..store.cad_store import CadStore
from ..store.media_store import MediaStore


def get_bridge(request: Request) -> RosBridge:
    return request.app.state.bridge


def get_state_hub(request: Request) -> StateHub:
    return request.app.state.state_hub


def get_media_store(request: Request) -> MediaStore:
    return request.app.state.media_store


def get_cad_store(request: Request) -> CadStore:
    return request.app.state.cad_store


def get_runtime(request: Request) -> GatewayRuntime:
    return request.app.state.runtime

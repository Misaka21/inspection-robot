"""FastAPI application factory.

``create_app()`` wires up all routes, the WebSocket endpoint, CORS middleware,
and (optionally) static file serving for the built React frontend.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from ..domain.runtime import GatewayRuntime
from ..ros.bridge import RosBridge
from ..ros.state_hub import StateHub
from ..store.cad_store import CadStore
from ..store.media_store import MediaStore
from .routes import cad, captures, media, nav, plans, targets, tasks
from .ws.handler import ws_router


def create_app(
    bridge: RosBridge,
    state_hub: StateHub,
    media_store: MediaStore,
    cad_store: CadStore,
    runtime: GatewayRuntime,
    frontend_dir: Optional[Path] = None,
) -> FastAPI:
    app = FastAPI(
        title="Inspection Gateway",
        version="1.0.0",
        description="REST + WebSocket API bridging the Web HMI to ROS2",
    )

    # -- Store shared objects on app.state for dependency injection ----------
    app.state.bridge = bridge
    app.state.state_hub = state_hub
    app.state.media_store = media_store
    app.state.cad_store = cad_store
    app.state.runtime = runtime

    # -- CORS (allow Vite dev server on :5173) -------------------------------
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # -- REST routes ---------------------------------------------------------
    prefix = "/api/v1"
    app.include_router(tasks.router, prefix=prefix)
    app.include_router(nav.router, prefix=prefix)
    app.include_router(media.router, prefix=prefix)
    app.include_router(cad.router, prefix=prefix)
    app.include_router(targets.router, prefix=prefix)
    app.include_router(plans.router, prefix=prefix)
    app.include_router(captures.router, prefix=prefix)

    # -- WebSocket -----------------------------------------------------------
    app.include_router(ws_router)

    # -- Static files (built React frontend) ---------------------------------
    if frontend_dir and frontend_dir.is_dir():
        # Serve index.html for SPA client-side routing
        app.mount("/", StaticFiles(directory=str(frontend_dir), html=True), name="frontend")

    return app

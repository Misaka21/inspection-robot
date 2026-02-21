"""Navigation map endpoint."""

from __future__ import annotations

from fastapi import APIRouter, Depends, Query

from ..deps import get_bridge, get_media_store
from ..models import ErrorCode, GetNavMapResponse, MediaRef, Result
from ...domain.converters import ros_nav_map_to_model
from ...ros.bridge import RosBridge
from ...store.media_store import MediaStore

router = APIRouter(tags=["nav"])


@router.get("/nav/map", response_model=GetNavMapResponse)
def get_nav_map(
    map_id: str = Query("", description="Map ID (empty = current map)"),
    include_image_thumbnail: bool = Query(False),
    bridge: RosBridge = Depends(get_bridge),
    media_store: MediaStore = Depends(get_media_store),
):
    map_name = ""
    if map_id:
        if map_id.startswith("agv:"):
            parts = map_id.split(":")
            map_name = parts[1] if len(parts) >= 2 else ""
        else:
            map_name = map_id

    try:
        resp = bridge.get_nav_map(map_name, include_thumbnail=include_image_thumbnail, timeout_s=3.0)
    except TimeoutError as ex:
        return GetNavMapResponse(result=Result(code=ErrorCode.TIMEOUT, message=str(ex)))
    except Exception as ex:
        return GetNavMapResponse(result=Result(code=ErrorCode.UNAVAILABLE, message=str(ex)))

    if not resp.success:
        return GetNavMapResponse(result=Result(code=ErrorCode.INTERNAL, message=str(resp.message)))

    media_ref = None
    if resp.image_data:
        info = media_store.put_bytes(bytes(resp.image_data), mime_type=str(resp.image_mime_type or "image/png"))
        media_ref = MediaRef(
            media_id=info.media_id,
            mime_type=info.mime_type,
            size_bytes=int(info.size_bytes),
            sha256=info.sha256,
        )

    nav_map = ros_nav_map_to_model(resp, media_ref=media_ref)
    return GetNavMapResponse(result=Result(code=ErrorCode.OK, message="ok"), map=nav_map)

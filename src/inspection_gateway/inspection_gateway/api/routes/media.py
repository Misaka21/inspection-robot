"""Media download endpoint (StreamingResponse for binary blobs)."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse

from ..deps import get_media_store
from ...store.media_store import MediaStore

router = APIRouter(tags=["media"])


def _iter_file(path, chunk_size: int = 256 * 1024):
    with path.open("rb") as f:
        while True:
            buf = f.read(chunk_size)
            if not buf:
                break
            yield buf


@router.get("/media/{media_id}")
def download_media(
    media_id: str,
    media_store: MediaStore = Depends(get_media_store),
):
    info = media_store.get(media_id)
    if info is None:
        raise HTTPException(status_code=404, detail="media not found")

    return StreamingResponse(
        _iter_file(info.path),
        media_type=info.mime_type,
        headers={
            "Content-Length": str(info.size_bytes),
            "X-Media-SHA256": info.sha256,
        },
    )

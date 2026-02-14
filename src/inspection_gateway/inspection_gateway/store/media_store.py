import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class MediaInfo:
    media_id: str
    mime_type: str
    size_bytes: int
    sha256: str
    path: Path


class MediaStore:
    def __init__(self, root_dir: Path):
        self._root_dir = root_dir
        self._media_dir = self._root_dir / "media"
        self._meta_dir = self._root_dir / "media_meta"
        self._media_dir.mkdir(parents=True, exist_ok=True)
        self._meta_dir.mkdir(parents=True, exist_ok=True)

    def put_bytes(self, data: bytes, mime_type: str) -> MediaInfo:
        sha256 = hashlib.sha256(data).hexdigest()
        media_id = sha256

        path = self._media_dir / media_id
        meta_path = self._meta_dir / f"{media_id}.json"

        if not path.exists():
            path.write_bytes(data)
            meta_path.write_text(
                json.dumps(
                    {
                        "media_id": media_id,
                        "mime_type": mime_type,
                        "size_bytes": len(data),
                        "sha256": sha256,
                    },
                    ensure_ascii=True,
                    indent=2,
                ),
                encoding="utf-8",
            )

        return MediaInfo(
            media_id=media_id,
            mime_type=mime_type,
            size_bytes=len(data),
            sha256=sha256,
            path=path,
        )

    def get(self, media_id: str) -> Optional[MediaInfo]:
        path = self._media_dir / media_id
        meta_path = self._meta_dir / f"{media_id}.json"
        if not path.is_file():
            return None
        mime_type = "application/octet-stream"
        size_bytes = path.stat().st_size
        sha256 = media_id
        if meta_path.is_file():
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
                mime_type = str(meta.get("mime_type") or mime_type)
                size_bytes = int(meta.get("size_bytes") or size_bytes)
                sha256 = str(meta.get("sha256") or sha256)
            except Exception:
                pass
        return MediaInfo(
            media_id=media_id,
            mime_type=mime_type,
            size_bytes=size_bytes,
            sha256=sha256,
            path=path,
        )


import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Optional


@dataclass(frozen=True)
class CadInfo:
    model_id: str
    filename: str
    size_bytes: int
    sha256: str
    path: Path


class CadStore:
    def __init__(self, root_dir: Path):
        self._root_dir = root_dir
        self._cad_dir = self._root_dir / "cad"
        self._meta_dir = self._root_dir / "cad_meta"
        self._cad_dir.mkdir(parents=True, exist_ok=True)
        self._meta_dir.mkdir(parents=True, exist_ok=True)

    def open_upload(self, upload_id: str) -> BinaryIO:
        tmp_path = self._cad_dir / f"{upload_id}.upload"
        return tmp_path.open("wb")

    def finalize_upload(self, upload_id: str, filename: str, sha256_hex: str) -> CadInfo:
        tmp_path = self._cad_dir / f"{upload_id}.upload"
        if not tmp_path.is_file():
            raise FileNotFoundError(f"upload tmp not found: {tmp_path}")

        model_id = sha256_hex
        out_path = self._cad_dir / model_id
        if not out_path.exists():
            tmp_path.rename(out_path)
        else:
            tmp_path.unlink(missing_ok=True)

        size_bytes = out_path.stat().st_size
        meta_path = self._meta_dir / f"{model_id}.json"
        if not meta_path.exists():
            meta_path.write_text(
                json.dumps(
                    {
                        "model_id": model_id,
                        "filename": filename,
                        "size_bytes": size_bytes,
                        "sha256": sha256_hex,
                    },
                    ensure_ascii=True,
                    indent=2,
                ),
                encoding="utf-8",
            )

        return CadInfo(
            model_id=model_id,
            filename=filename,
            size_bytes=size_bytes,
            sha256=sha256_hex,
            path=out_path,
        )


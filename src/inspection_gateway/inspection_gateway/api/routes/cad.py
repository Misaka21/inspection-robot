"""CAD upload endpoint (multipart/form-data)."""

from __future__ import annotations

import hashlib
import uuid

from fastapi import APIRouter, Depends, UploadFile

from ..deps import get_cad_store
from ..models import ErrorCode, Result, UploadCadResponse
from ...store.cad_store import CadStore

router = APIRouter(tags=["cad"])


@router.post("/cad/upload", response_model=UploadCadResponse)
async def upload_cad(
    file: UploadFile,
    cad_store: CadStore = Depends(get_cad_store),
):
    upload_id = str(uuid.uuid4())
    filename = file.filename or "cad.bin"
    hasher = hashlib.sha256()
    total = 0

    try:
        f = cad_store.open_upload(upload_id)
        while True:
            chunk = await file.read(256 * 1024)
            if not chunk:
                break
            f.write(chunk)
            hasher.update(chunk)
            total += len(chunk)
        f.close()
    except Exception as ex:
        return UploadCadResponse(result=Result(code=ErrorCode.INTERNAL, message=str(ex)))

    sha256_hex = hasher.hexdigest()
    info = cad_store.finalize_upload(upload_id, filename, sha256_hex)

    return UploadCadResponse(
        result=Result(code=ErrorCode.OK, message="ok"),
        model_id=info.model_id,
        total_bytes=int(info.size_bytes),
    )

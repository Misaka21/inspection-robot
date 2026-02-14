import queue
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import grpc

from ..ros.bridge import RosBridge
from ..ros.state_hub import StateHub
from ..store.cad_store import CadStore
from ..store.media_store import MediaStore
from .converters import GatewayTaskContext, ros_nav_map_to_pb, ros_system_state_to_task_status


@dataclass
class GatewayRuntime:
    task_id: str = ""
    plan_id: str = ""
    task_name: str = ""


class InspectionGatewayServicer:
    """
    gRPC servicer implementing inspection-api/proto/inspection_gateway.proto.

    NOTE: This class is instantiated with generated pb2/pb2_grpc modules, so it does not
    import protobuf modules at import-time.
    """

    def __init__(
        self,
        pb2,
        ros: RosBridge,
        state_hub: StateHub,
        media_store: MediaStore,
        cad_store: CadStore,
        runtime: GatewayRuntime,
        cache_dir: Path,
    ):
        self._pb2 = pb2
        self._ros = ros
        self._state_hub = state_hub
        self._media = media_store
        self._cad = cad_store
        self._rt = runtime
        self._cache_dir = cache_dir

    def _ctx(self) -> GatewayTaskContext:
        return GatewayTaskContext(task_id=self._rt.task_id, plan_id=self._rt.plan_id, task_name=self._rt.task_name)

    def _result(self, code_name: str, message: str) -> object:
        code = self._pb2.ErrorCode.Value(code_name)
        return self._pb2.Result(code=code, message=message)

    # --- RPCs ---

    def UploadCad(self, request_iterator, context) -> object:
        # Streamed upload to local disk. model_id = sha256 of file bytes.
        import hashlib

        upload_id = None
        filename = "cad.bin"
        hasher = hashlib.sha256()
        total = 0
        f = None

        try:
            for chunk in request_iterator:
                if upload_id is None:
                    upload_id = chunk.upload_id or str(uuid.uuid4())
                    filename = chunk.filename or filename
                    f = self._cad.open_upload(upload_id)

                data = bytes(chunk.data)
                if data:
                    f.write(data)
                    hasher.update(data)
                    total += len(data)

                if chunk.eof:
                    break

            if upload_id is None or f is None:
                return self._pb2.UploadCadResponse(result=self._result("INVALID_ARGUMENT", "empty upload"))

            f.close()
            sha256_hex = hasher.hexdigest()

            # Optional checksum validation (client may provide sha256 in the final chunk).
            if getattr(chunk, "sha256", ""):
                if str(chunk.sha256) != sha256_hex:
                    return self._pb2.UploadCadResponse(
                        result=self._result("INVALID_ARGUMENT", "sha256 mismatch"),
                    )

            info = self._cad.finalize_upload(upload_id, filename, sha256_hex)
            return self._pb2.UploadCadResponse(
                result=self._result("OK", "ok"),
                model_id=info.model_id,
                total_bytes=int(info.size_bytes),
            )
        except Exception as ex:
            if f is not None:
                try:
                    f.close()
                except Exception:
                    pass
            return self._pb2.UploadCadResponse(result=self._result("INTERNAL", str(ex)))

    def SetInspectionTargets(self, request, context) -> object:
        # V1: store-only stub. Planner integration will be added later.
        total = len(getattr(request, "targets", []))
        return self._pb2.SetInspectionTargetsResponse(
            result=self._result("OK", "ok"),
            total_targets=int(total),
        )

    def PlanInspection(self, request, context) -> object:
        return self._pb2.PlanInspectionResponse(result=self._result("UNAVAILABLE", "PlanInspection not implemented"))

    def GetPlan(self, request, context) -> object:
        return self._pb2.GetPlanResponse(result=self._result("NOT_FOUND", "plan not found"))

    def StartInspection(self, request, context) -> object:
        try:
            resp = self._ros.start_inspection(plan_id=str(request.plan_id), dry_run=bool(request.dry_run), timeout_s=3.0)
        except TimeoutError as ex:
            return self._pb2.StartInspectionResponse(result=self._result("TIMEOUT", str(ex)))
        except Exception as ex:
            return self._pb2.StartInspectionResponse(result=self._result("UNAVAILABLE", str(ex)))

        if not resp.success:
            return self._pb2.StartInspectionResponse(result=self._result("INTERNAL", str(resp.message)))

        # Coordinator currently does not provide a stable task_id; gateway owns task_id for now.
        self._rt.task_id = str(uuid.uuid4())
        self._rt.plan_id = str(request.plan_id)
        self._rt.task_name = ""
        return self._pb2.StartInspectionResponse(result=self._result("OK", "ok"), task_id=self._rt.task_id)

    def PauseInspection(self, request, context) -> object:
        try:
            resp = self._ros.pause_inspection(str(request.task_id), str(request.reason), timeout_s=3.0)
        except TimeoutError as ex:
            return self._pb2.ControlTaskResponse(result=self._result("TIMEOUT", str(ex)))
        except Exception as ex:
            return self._pb2.ControlTaskResponse(result=self._result("UNAVAILABLE", str(ex)))

        return self._pb2.ControlTaskResponse(
            result=self._result("OK", "ok") if resp.success else self._result("INTERNAL", str(resp.message))
        )

    def ResumeInspection(self, request, context) -> object:
        try:
            resp = self._ros.resume_inspection(str(request.task_id), str(request.reason), timeout_s=3.0)
        except TimeoutError as ex:
            return self._pb2.ControlTaskResponse(result=self._result("TIMEOUT", str(ex)))
        except Exception as ex:
            return self._pb2.ControlTaskResponse(result=self._result("UNAVAILABLE", str(ex)))

        return self._pb2.ControlTaskResponse(
            result=self._result("OK", "ok") if resp.success else self._result("INTERNAL", str(resp.message))
        )

    def StopInspection(self, request, context) -> object:
        try:
            resp = self._ros.stop_inspection(str(request.task_id), str(request.reason), timeout_s=3.0)
        except TimeoutError as ex:
            return self._pb2.ControlTaskResponse(result=self._result("TIMEOUT", str(ex)))
        except Exception as ex:
            return self._pb2.ControlTaskResponse(result=self._result("UNAVAILABLE", str(ex)))

        return self._pb2.ControlTaskResponse(
            result=self._result("OK", "ok") if resp.success else self._result("INTERNAL", str(resp.message))
        )

    def GetTaskStatus(self, request, context) -> object:
        try:
            resp = self._ros.get_status(str(request.task_id), timeout_s=3.0)
        except TimeoutError as ex:
            return self._pb2.GetTaskStatusResponse(result=self._result("TIMEOUT", str(ex)))
        except Exception as ex:
            return self._pb2.GetTaskStatusResponse(result=self._result("UNAVAILABLE", str(ex)))

        if not resp.success:
            return self._pb2.GetTaskStatusResponse(result=self._result("INTERNAL", str(resp.message)))

        status = ros_system_state_to_task_status(self._pb2, resp.state, self._ctx())
        if not status.task_id:
            status.task_id = str(request.task_id or self._rt.task_id)
        return self._pb2.GetTaskStatusResponse(result=self._result("OK", "ok"), status=status)

    def SubscribeSystemState(self, request, context) -> Iterable[object]:
        sub_id, q = self._state_hub.subscribe()
        try:
            if request.include_snapshot:
                snap = self._state_hub.get_last()
                if snap is not None:
                    status = ros_system_state_to_task_status(self._pb2, snap.ros_state, self._ctx())
                    yield self._pb2.SystemStateEvent(status=status)

            while context.is_active():
                try:
                    ros_state = q.get(0.5)
                except queue.Empty:
                    continue

                # Best-effort filtering by task_id.
                if request.task_id:
                    sid = str(getattr(ros_state, "task_id", "") or self._rt.task_id)
                    if sid and sid != str(request.task_id):
                        continue

                status = ros_system_state_to_task_status(self._pb2, ros_state, self._ctx())
                if not status.task_id:
                    status.task_id = self._rt.task_id
                yield self._pb2.SystemStateEvent(status=status)
        finally:
            self._state_hub.unsubscribe(sub_id)

    def SubscribeInspectionEvents(self, request, context) -> Iterable[object]:
        # Planned: bridge from /inspection/events
        return iter(())

    def GetNavMap(self, request, context) -> object:
        map_id = str(request.map_id)
        map_name = ""
        if map_id:
            if map_id.startswith("agv:"):
                parts = map_id.split(":")
                if len(parts) >= 3:
                    map_name = parts[1]
                elif len(parts) == 2:
                    map_name = parts[1]
            else:
                map_name = map_id

        try:
            resp = self._ros.get_nav_map(map_name, include_thumbnail=bool(request.include_image_thumbnail), timeout_s=3.0)
        except TimeoutError as ex:
            return self._pb2.GetNavMapResponse(result=self._result("TIMEOUT", str(ex)))
        except Exception as ex:
            return self._pb2.GetNavMapResponse(result=self._result("UNAVAILABLE", str(ex)))

        if not resp.success:
            return self._pb2.GetNavMapResponse(result=self._result("INTERNAL", str(resp.message)))

        media_ref = None
        if resp.image_data:
            info = self._media.put_bytes(bytes(resp.image_data), mime_type=str(resp.image_mime_type or "image/png"))
            media_ref = self._pb2.MediaRef(
                media_id=info.media_id,
                mime_type=info.mime_type,
                size_bytes=int(info.size_bytes),
                sha256=info.sha256,
            )

        nav_map = ros_nav_map_to_pb(self._pb2, resp, media_ref=media_ref)
        return self._pb2.GetNavMapResponse(result=self._result("OK", "ok"), map=nav_map)

    def ListCaptures(self, request, context) -> object:
        return self._pb2.ListCapturesResponse(result=self._result("UNAVAILABLE", "ListCaptures not implemented"))

    def DownloadMedia(self, request, context) -> Iterable[object]:
        media_id = str(request.media_id)
        info = self._media.get(media_id)
        if info is None:
            context.abort(grpc.StatusCode.NOT_FOUND, "media not found")

        chunk_size = 1024 * 256
        total = int(info.size_bytes)
        sent = 0
        idx = 0

        with info.path.open("rb") as f:
            while context.is_active():
                buf = f.read(chunk_size)
                if not buf:
                    break
                sent += len(buf)
                eof = sent >= total
                chunk = self._pb2.MediaChunk(
                    media_id=media_id,
                    data=buf,
                    chunk_index=idx,
                    eof=eof,
                )
                if eof:
                    chunk.sha256 = info.sha256
                    chunk.total_bytes = total
                idx += 1
                yield chunk

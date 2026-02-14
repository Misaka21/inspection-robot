import importlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class ProtoModules:
    pb2: object
    pb2_grpc: object


def _find_repo_proto(start_dir: Path) -> Optional[Path]:
    # Search for ".../inspection-api/proto/inspection_gateway.proto" in parent dirs.
    cur = start_dir.resolve()
    for _ in range(8):
        candidate = cur / "inspection-api" / "proto" / "inspection_gateway.proto"
        if candidate.is_file():
            return candidate
        if cur.parent == cur:
            break
        cur = cur.parent
    return None


def _find_proto_file() -> Path:
    # 1) Explicit env var
    env_path = os.environ.get("INSPECTION_GATEWAY_PROTO")
    if env_path:
        p = Path(env_path).expanduser()
        if not p.is_file():
            raise FileNotFoundError(f"INSPECTION_GATEWAY_PROTO not found: {p}")
        return p

    # 2) Relative to a workspace that also contains inspection-api
    repo_proto = _find_repo_proto(Path(__file__).parent)
    if repo_proto is not None:
        return repo_proto

    raise FileNotFoundError(
        "inspection_gateway.proto not found. Set INSPECTION_GATEWAY_PROTO or keep inspection-api repo nearby."
    )


def _compile_proto(proto_file: Path, out_dir: Path) -> None:
    try:
        from grpc_tools import protoc  # type: ignore
        import pkg_resources  # type: ignore
    except Exception as ex:  # pragma: no cover
        raise RuntimeError(
            "grpc_tools/protoc not available. Install grpcio-tools (python3-grpcio-tools)."
        ) from ex

    proto_dir = proto_file.parent
    grpc_tools_include = Path(pkg_resources.resource_filename("grpc_tools", "_proto"))

    out_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        "grpc_tools.protoc",
        f"-I{proto_dir}",
        f"-I{grpc_tools_include}",
        f"--python_out={out_dir}",
        f"--grpc_python_out={out_dir}",
        str(proto_file),
    ]
    rc = protoc.main(cmd)
    if rc != 0:
        raise RuntimeError(f"protoc failed (rc={rc}): {' '.join(cmd)}")


def load_proto_modules(cache_dir: Path) -> ProtoModules:
    """
    Load generated modules for inspection-api/proto/inspection_gateway.proto.

    We compile the proto into cache_dir and import:
      - inspection_gateway_pb2
      - inspection_gateway_pb2_grpc
    """
    proto_file = _find_proto_file()

    gen_dir = cache_dir / "proto_gen"
    pb2_file = gen_dir / "inspection_gateway_pb2.py"
    pb2_grpc_file = gen_dir / "inspection_gateway_pb2_grpc.py"

    needs_compile = True
    if pb2_file.is_file() and pb2_grpc_file.is_file():
        try:
            needs_compile = pb2_file.stat().st_mtime < proto_file.stat().st_mtime
        except OSError:
            needs_compile = True

    if needs_compile:
        _compile_proto(proto_file, gen_dir)

    if str(gen_dir) not in sys.path:
        sys.path.insert(0, str(gen_dir))

    pb2 = importlib.import_module("inspection_gateway_pb2")
    pb2_grpc = importlib.import_module("inspection_gateway_pb2_grpc")
    return ProtoModules(pb2=pb2, pb2_grpc=pb2_grpc)


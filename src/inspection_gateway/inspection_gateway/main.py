import argparse
import logging
from pathlib import Path
import threading
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .ros.bridge import RosBridge, RosNames
from .ros.state_hub import StateHub
from .rpc.proto_loader import load_proto_modules
from .rpc.server import GrpcServerConfig, start_server
from .rpc.servicer import GatewayRuntime, InspectionGatewayServicer
from .store.cad_store import CadStore
from .store.media_store import MediaStore


def _parse_args(argv: Optional[list[str]] = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(prog="inspection_gateway")
    parser.add_argument("--grpc-port", type=int, default=50051)
    parser.add_argument("--grpc-max-workers", type=int, default=16)
    parser.add_argument("--data-dir", type=str, default="~/.ros/inspection_gateway")
    parser.add_argument("--cache-dir", type=str, default="~/.cache/inspection_gateway")
    parser.add_argument("--ros-root-ns", type=str, default="/inspection")
    args, unknown = parser.parse_known_args(argv)
    return args, unknown


def _normalize_root_ns(ns: str) -> str:
    ns = (ns or "").strip()
    if not ns:
        return "/inspection"
    if not ns.startswith("/"):
        ns = "/" + ns
    if ns != "/" and ns.endswith("/"):
        ns = ns.rstrip("/")
    return ns


def main(argv: Optional[list[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("inspection_gateway")

    args, ros_argv = _parse_args(argv)

    data_dir = Path(args.data_dir).expanduser().resolve()
    cache_dir = Path(args.cache_dir).expanduser().resolve()
    data_dir.mkdir(parents=True, exist_ok=True)
    cache_dir.mkdir(parents=True, exist_ok=True)

    root_ns = _normalize_root_ns(str(args.ros_root_ns))

    proto = load_proto_modules(cache_dir=cache_dir)

    state_hub = StateHub()
    names = RosNames(root_ns=root_ns)

    rclpy.init(args=ros_argv)
    ros = RosBridge(names=names, state_hub=state_hub)
    executor = SingleThreadedExecutor()
    executor.add_node(ros)

    spin_thread = threading.Thread(target=executor.spin, name="ros_executor", daemon=True)
    spin_thread.start()

    media_store = MediaStore(root_dir=data_dir)
    cad_store = CadStore(root_dir=data_dir)
    runtime = GatewayRuntime()

    servicer = InspectionGatewayServicer(
        pb2=proto.pb2,
        ros=ros,
        state_hub=state_hub,
        media_store=media_store,
        cad_store=cad_store,
        runtime=runtime,
        cache_dir=cache_dir,
    )

    cfg = GrpcServerConfig(port=int(args.grpc_port), max_workers=int(args.grpc_max_workers))
    server = start_server(proto.pb2_grpc, servicer, cfg)
    log.info("inspection_gateway started (grpc_port=%d, ros_root_ns=%s, data_dir=%s)", cfg.port, root_ns, data_dir)

    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            server.stop(grace=1).wait(timeout=2)
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            ros.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        spin_thread.join(timeout=2)


if __name__ == "__main__":
    main()

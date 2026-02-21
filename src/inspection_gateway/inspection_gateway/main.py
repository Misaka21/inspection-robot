import argparse
import logging
from pathlib import Path
import threading
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .api.app import create_app
from .domain.runtime import GatewayRuntime
from .ros.bridge import RosBridge, RosNames
from .ros.state_hub import StateHub
from .store.cad_store import CadStore
from .store.media_store import MediaStore


def _parse_args(argv: Optional[list[str]] = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(prog="inspection_gateway")
    parser.add_argument("--port", type=int, default=8080)
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

    # Resolve frontend dist directory (built React app)
    pkg_dir = Path(__file__).resolve().parent
    frontend_dir = pkg_dir / "frontend" / "dist"
    if not frontend_dir.is_dir():
        frontend_dir = None

    app = create_app(
        bridge=ros,
        state_hub=state_hub,
        media_store=media_store,
        cad_store=cad_store,
        runtime=runtime,
        frontend_dir=frontend_dir,
    )

    import uvicorn

    port = int(args.port)
    log.info("inspection_gateway starting (port=%d, ros_root_ns=%s, data_dir=%s)", port, root_ns, data_dir)

    try:
        uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
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

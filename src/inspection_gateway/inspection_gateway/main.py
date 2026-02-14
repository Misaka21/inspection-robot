import argparse
import threading
from typing import Optional

import rclpy
from rclpy.node import Node


class GatewayNode(Node):
    def __init__(self, grpc_port: int):
        super().__init__("inspection_gateway")
        self._grpc_port = grpc_port

    def start(self) -> None:
        # NOTE: gRPC service implementation is planned; the package skeleton exists to
        # enforce architecture + placement in the ROS2 workspace.
        self.get_logger().warn(
            "inspection_gateway package skeleton is installed, but gRPC server is not implemented yet."
        )


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="inspection_gateway")
    parser.add_argument("--grpc-port", type=int, default=50051)
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = _parse_args(argv)

    rclpy.init(args=None)
    node = GatewayNode(grpc_port=args.grpc_port)

    try:
        node.start()

        # Keep spinning so future ROS subscriptions/clients can be added without changing the entrypoint.
        exec_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        exec_thread.start()
        exec_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


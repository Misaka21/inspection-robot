"""DIO driver ROS2 node — publishes DI/DO status, provides set_output service."""

import rclpy
from rclpy.node import Node
from inspection_interface.msg import DioStatus
from inspection_interface.srv import SetDioOutput


class DioDriverNode(Node):
    def __init__(self):
        super().__init__("dio_driver")

        # Parameters
        self.declare_parameter("chip_path", "/dev/gpiochip3")
        self.declare_parameter("do_lines", [0, 1, 2, 3])
        self.declare_parameter("di_lines", [8, 9, 10, 11])
        self.declare_parameter("poll_interval_ms", 50)
        self.declare_parameter("use_fake", False)

        _chip_path = self.get_parameter("chip_path").value
        _do_lines = list(self.get_parameter("do_lines").value)
        _di_lines = list(self.get_parameter("di_lines").value)
        _poll_ms = self.get_parameter("poll_interval_ms").value
        _use_fake = self.get_parameter("use_fake").value

        # Adapter layer (no gpiod import in this file)
        if _use_fake:
            from ..adapter.gpio_adapter import FakeGpioAdapter

            self._adapter = FakeGpioAdapter(_do_lines, _di_lines)
            self.get_logger().info("DIO using FakeGpioAdapter (simulation mode)")
        else:
            from ..adapter.gpio_adapter import GpioAdapter

            self._adapter = GpioAdapter(_chip_path, _do_lines, _di_lines)
            if self._adapter.connected:
                self.get_logger().info(
                    f"DIO connected: chip={_chip_path} "
                    f"DO={_do_lines} DI={_di_lines}"
                )
            else:
                self.get_logger().error(f"DIO init failed: {self._adapter.error}")

        # Publisher
        self._status_pub = self.create_publisher(DioStatus, "status", 10)

        # Service
        self._set_do_srv = self.create_service(
            SetDioOutput, "set_output", self._on_set_output
        )

        # Poll timer
        self._timer = self.create_timer(_poll_ms / 1000.0, self._on_poll)

        self.get_logger().info(
            f"dio_driver started (poll={_poll_ms}ms, fake={_use_fake})"
        )

    def _on_poll(self):
        msg = DioStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self._adapter.connected
        if msg.connected:
            msg.di_values = self._adapter.read_inputs()
            msg.do_values = self._adapter.read_outputs()
            msg.error_code = "OK"
        else:
            msg.di_values = []
            msg.do_values = []
            msg.error_code = self._adapter.error or "DISCONNECTED"
        self._status_pub.publish(msg)

    def _on_set_output(self, request, response):
        try:
            self._adapter.set_output(request.channel, request.value)
            response.success = True
            response.message = f"DO{request.channel} = {'HIGH' if request.value else 'LOW'}"
            self.get_logger().info(response.message)
        except (ValueError, OSError) as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"set_output failed: {e}")
        return response

    def destroy_node(self):
        self._adapter.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DioDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

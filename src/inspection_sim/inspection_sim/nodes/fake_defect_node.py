import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class FakeDefectNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_defect")
        self._srv = self.create_service(Trigger, "detect_defect", self._on_detect_defect)

    def _on_detect_defect(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        (req,) = (req,)
        res.success = True
        res.message = "ok"
        return res


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = FakeDefectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


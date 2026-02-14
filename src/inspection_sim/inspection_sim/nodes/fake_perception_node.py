import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

from ..core.angles import quat_from_yaw


class FakePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_perception")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("pose_x", 0.0)
        self.declare_parameter("pose_y", 0.0)
        self.declare_parameter("pose_z", 0.0)
        self.declare_parameter("pose_yaw_deg", 0.0)

        self._pub = self.create_publisher(PoseStamped, "detected_pose", 10)
        self._srv = self.create_service(Trigger, "detect", self._on_detect)

    def _on_detect(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        (req,) = (req,)

        frame_id = str(self.get_parameter("frame_id").value)
        x = float(self.get_parameter("pose_x").value)
        y = float(self.get_parameter("pose_y").value)
        z = float(self.get_parameter("pose_z").value)
        yaw_deg = float(self.get_parameter("pose_yaw_deg").value)
        yaw = math.radians(yaw_deg)

        qx, qy, qz, qw = quat_from_yaw(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pub.publish(msg)

        res.success = True
        res.message = "ok"
        return res


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = FakePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


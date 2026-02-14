import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_srvs.srv import Trigger

from ..core.angles import quat_from_yaw


class FakePlanningNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_planning")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("radius_m", 0.6)
        self.declare_parameter("waypoint_count", 3)

        self._path_pub = self.create_publisher(PoseArray, "path", 10)
        self._srv = self.create_service(Trigger, "optimize", self._on_optimize)

    def _on_optimize(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        (req,) = (req,)

        frame_id = str(self.get_parameter("frame_id").value)
        cx = float(self.get_parameter("center_x").value)
        cy = float(self.get_parameter("center_y").value)
        r = float(self.get_parameter("radius_m").value)
        n = int(self.get_parameter("waypoint_count").value)
        n = max(1, n)

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        for i in range(n):
            a = (2.0 * math.pi * i) / float(n)
            x = cx + r * math.cos(a)
            y = cy + r * math.sin(a)
            yaw = a + math.pi  # face towards center
            qx, qy, qz, qw = quat_from_yaw(yaw)

            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.0
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            msg.poses.append(p)

        self._path_pub.publish(msg)

        res.success = True
        res.message = f"ok (waypoints={n})"
        return res


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = FakePlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

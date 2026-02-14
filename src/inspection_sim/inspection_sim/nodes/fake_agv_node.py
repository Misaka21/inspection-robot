import hashlib
import math
from array import array
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from inspection_interface.msg import AgvStatus
from inspection_interface.srv import GetNavMap

from ..core.angles import normalize_angle_rad, quat_from_yaw
from ..core.png import make_grid_png


@dataclass
class _Goal:
    x: float
    y: float
    yaw: float


class FakeAgvNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_agv")

        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.declare_parameter("linear_speed_mps", 0.4)
        self.declare_parameter("angular_speed_rps", 1.0)
        self.declare_parameter("pos_tolerance_m", 0.05)
        self.declare_parameter("yaw_tolerance_deg", 5.0)

        # Nav map stub config.
        self.declare_parameter("map_name", "sim_map")
        self.declare_parameter("map_resolution_m_per_pixel", 0.02)
        self.declare_parameter("map_width_px", 800)
        self.declare_parameter("map_height_px", 800)
        self.declare_parameter("map_grid_px", 40)

        self._map_frame_id = str(self.get_parameter("map_frame_id").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._x = float(self.get_parameter("initial_x").value)
        self._y = float(self.get_parameter("initial_y").value)
        self._yaw = float(self.get_parameter("initial_yaw").value)

        self._linear_speed_mps = float(self.get_parameter("linear_speed_mps").value)
        self._angular_speed_rps = float(self.get_parameter("angular_speed_rps").value)
        self._pos_tol_m = float(self.get_parameter("pos_tolerance_m").value)
        self._yaw_tol_rad = math.radians(float(self.get_parameter("yaw_tolerance_deg").value))

        self._goal: Optional[_Goal] = None
        self._moving = False
        self._arrived = True
        self._stopped = True

        self._status_pub = self.create_publisher(AgvStatus, "status", 10)
        self._pose_pub = self.create_publisher(PoseStamped, "current_pose", 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)

        self._goal_sub = self.create_subscription(PoseStamped, "goal_pose", self._on_goal_pose, 10)
        self._cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        self._tf = TransformBroadcaster(self) if self._publish_tf else None

        self._nav_map_srv = self.create_service(GetNavMap, "get_nav_map", self._on_get_nav_map)
        self._nav_map_cache = self._build_nav_map_cache()

        self._last_t = self.get_clock().now()
        self._timer = self.create_timer(0.05, self._on_timer)  # 20 Hz

    def _build_nav_map_cache(self) -> dict:
        name = str(self.get_parameter("map_name").value)
        res = float(self.get_parameter("map_resolution_m_per_pixel").value)
        w = int(self.get_parameter("map_width_px").value)
        h = int(self.get_parameter("map_height_px").value)
        grid_px = int(self.get_parameter("map_grid_px").value)

        png = make_grid_png(w, h, grid_px=grid_px)
        md5 = hashlib.md5(png).hexdigest()

        # origin is pose of pixel (u=0,v=0) in map frame.
        # Using a centered map: (0,0) is map center => (u,v)=(W/2,H/2).
        origin_x = -0.5 * w * res
        origin_y = 0.5 * h * res

        return {
            "name": name,
            "md5": md5,
            "resolution": res,
            "width": w,
            "height": h,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "png": png,
        }

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self._map_frame_id:
            self.get_logger().warn(
                f"reject goal_pose: frame_id={msg.header.frame_id} expected={self._map_frame_id}"
            )
            return

        q = msg.pose.orientation
        # yaw from quaternion (x=y=0 in typical planar case; still handle generic).
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self._goal = _Goal(x=float(msg.pose.position.x), y=float(msg.pose.position.y), yaw=float(yaw))
        self._moving = True
        self._arrived = False
        self._stopped = False

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Optional: open-loop debug. For now, ignore to keep simulation deterministic.
        (msg,) = (msg,)

    def _on_get_nav_map(self, req: GetNavMap.Request, res: GetNavMap.Response) -> GetNavMap.Response:
        (req,) = (req,)
        info = self._nav_map_cache

        res.success = True
        res.message = "ok"
        res.map_name = info["name"]
        res.map_md5 = info["md5"]

        res.resolution_m_per_pixel = float(info["resolution"])

        # Map extents in map frame.
        w_m = float(info["width"]) * float(info["resolution"])
        h_m = float(info["height"]) * float(info["resolution"])
        res.min_pos = Point(x=-0.5 * w_m, y=-0.5 * h_m, z=0.0)
        res.max_pos = Point(x=0.5 * w_m, y=0.5 * h_m, z=0.0)

        res.width_px = int(info["width"])
        res.height_px = int(info["height"])
        res.frame_id = self._map_frame_id

        res.origin = Pose()
        res.origin.position.x = float(info["origin_x"])
        res.origin.position.y = float(info["origin_y"])
        res.origin.position.z = 0.0
        res.origin.orientation.w = 1.0

        res.image_mime_type = "image/png"

        if bool(req.include_image_data):
            res.image_data = array("B", info["png"])
        else:
            res.image_data = array("B")

        # We don't generate JPEG thumbnail (no Pillow dependency). Keep empty.
        res.thumbnail_jpeg = array("B")
        res.smap_json = ""
        return res

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05
        self._last_t = now

        lin_v = 0.0
        ang_v = 0.0

        if self._goal is not None and self._moving:
            dx = self._goal.x - self._x
            dy = self._goal.y - self._y
            dist = math.hypot(dx, dy)

            if dist > self._pos_tol_m:
                step = min(self._linear_speed_mps * dt, dist)
                self._x += step * (dx / dist)
                self._y += step * (dy / dist)
                lin_v = self._linear_speed_mps

            yaw_err = normalize_angle_rad(self._goal.yaw - self._yaw)
            if abs(yaw_err) > self._yaw_tol_rad:
                yaw_step = min(self._angular_speed_rps * dt, abs(yaw_err))
                self._yaw += math.copysign(yaw_step, yaw_err)
                ang_v = math.copysign(self._angular_speed_rps, yaw_err)

            dx2 = self._goal.x - self._x
            dy2 = self._goal.y - self._y
            dist2 = math.hypot(dx2, dy2)
            yaw_err2 = normalize_angle_rad(self._goal.yaw - self._yaw)
            if dist2 <= self._pos_tol_m and abs(yaw_err2) <= self._yaw_tol_rad:
                self._moving = False
                self._arrived = True
                self._stopped = True
                lin_v = 0.0
                ang_v = 0.0

        qx, qy, qz, qw = quat_from_yaw(self._yaw)
        quat = Quaternion(x=qx, y=qy, z=qz, w=qw)

        pose = Pose()
        pose.position.x = float(self._x)
        pose.position.y = float(self._y)
        pose.position.z = 0.0
        pose.orientation = quat

        # Publish current pose
        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = self._map_frame_id
        ps.pose = pose
        self._pose_pub.publish(ps)

        # Publish odom (minimal)
        odom = Odometry()
        odom.header.stamp = ps.header.stamp
        odom.header.frame_id = self._map_frame_id
        odom.child_frame_id = self._base_frame_id
        odom.pose.pose = pose
        odom.twist.twist.linear.x = float(lin_v)
        odom.twist.twist.angular.z = float(ang_v)
        self._odom_pub.publish(odom)

        # Publish TF (map -> base_link)
        if self._tf is not None:
            t = TransformStamped()
            t.header.stamp = ps.header.stamp
            t.header.frame_id = self._map_frame_id
            t.child_frame_id = self._base_frame_id
            t.transform.translation.x = float(self._x)
            t.transform.translation.y = float(self._y)
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self._tf.sendTransform(t)

        # Publish status
        st = AgvStatus()
        st.connected = True
        st.arrived = bool(self._arrived)
        st.moving = bool(self._moving)
        st.stopped = bool(self._stopped)
        st.current_pose = pose
        st.linear_speed = float(lin_v)
        st.angular_speed = float(ang_v)
        st.frame_id = self._map_frame_id
        st.battery_percent = 100.0
        st.error_code = "OK"
        self._status_pub.publish(st)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = FakeAgvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


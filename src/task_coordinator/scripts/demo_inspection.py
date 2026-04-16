#!/usr/bin/env python3
"""
展示脚本：AGV 静止不动，机械臂依次走到预设拍照点位，
每到达一个点位后调用相机软触发拍照。

用法：
  ros2 run task_coordinator demo_inspection

修改点位：直接改下面的 STATIONS 列表。
  - agv: None                           禁用 AGV 移动
  - arm_joints: [j1..j6]                关节角（角度制，代码内自动转弧度）
  - dwell: 停留秒数（等待拍照完成）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from inspection_interface.msg import AgvStatus, ArmStatus
from inspection_interface.srv import MoveToJoints, MoveToPose, SetDioOutput
from std_srvs.srv import Trigger
import math
import time


def deg2rad(joints_deg):
    """将角度列表转为弧度列表"""
    return [math.radians(d) for d in joints_deg]


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  拍照点位（角度制，代码内自动转弧度）
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
STATIONS = [
    {
        "name": "Point_top",
        "agv": None,
        "arm_joints": deg2rad([-94.553, 20.932, 96.416, -0.000, 104.517, -34.554]),
        "dwell": 3.0,
    },
    {
        "name": "Point_leftback",
        "agv": None,
        "arm_joints": deg2rad([-9.630, -26.345, 95.961, -21.888, 118.083, 45.046]),
        "dwell": 3.0,
    },
    {
        "name": "Point_left",
        "agv": None,
        "arm_joints": deg2rad([-24.924, -34.632, 87.300, -25.541, 115.987, 43.571]),
        "dwell": 3.0,
    },
    {
        "name": "Point_leftfront",
        "agv": None,
        "arm_joints": deg2rad([-44.288, -56.501, 49.081, -32.257, 140.881, -21.667]),
        "dwell": 3.0,
    },
    {
        "name": "Point_front",
        "agv": None,
        "arm_joints": deg2rad([-83.802, -41.614, 50.439, -14.330, 135.078, -26.880]),
        "dwell": 3.0,
    },
    {
        "name": "Point_rightfront",
        "agv": None,
        "arm_joints": deg2rad([-136.763, -60.490, 54.722, 25.612, 129.906, -96.151]),
        "dwell": 3.0,
    },
    {
        "name": "Point_right",
        "agv": None,
        "arm_joints": deg2rad([-157.184, -49.899, 78.107, 24.876, 119.658, -99.837]),
        "dwell": 3.0,
    },
    {
        "name": "Point_rightback",
        "agv": None,
        "arm_joints": deg2rad([-175.157, -43.514, 90.955, 26.160, 114.912, -100.856]),
        "dwell": 3.0,
    },
]

# 机械臂收回零位（每个站点拍完照后收回）
ARM_HOME = [1.57, -0.523, 2.094, 0.0, 0.524, 0.0]

# MoveIt 速度缩放因子 (0~1)，值越小越慢越平稳
ARM_SPEED = 0.4


class DemoNode(Node):
    def __init__(self):
        super().__init__("demo_inspection")

        # Publishers
        self._agv_goal_pub = self.create_publisher(
            PoseStamped, "/inspection/agv/goal_pose", 10)
        self._agv_goal_station_pub = self.create_publisher(
            String, "/inspection/agv/goal_station", 10)

        # Status
        self._agv_status = None
        self._arm_status = None
        self.create_subscription(
            AgvStatus, "/inspection/agv/status", self._on_agv_status, 10)
        self.create_subscription(
            ArmStatus, "/inspection/arm/status", self._on_arm_status, 10)

        # Service clients
        self._enable_client = self.create_client(
            Trigger, "/inspection/arm/enable")
        self._move_joints_client = self.create_client(
            MoveToJoints, "/inspection/arm_control/move_to_joints")
        self._move_pose_client = self.create_client(
            MoveToPose, "/inspection/arm_control/move_to_pose")
        self._set_dio_client = self.create_client(
            SetDioOutput, "/inspection/dio/set_output")
        self._trigger_capture_client = self.create_client(
            Trigger, "/inspection/hikvision/trigger_capture")

        self.get_logger().info("Demo inspection node started")

    def _on_agv_status(self, msg):
        self._agv_status = msg

    def _on_arm_status(self, msg):
        self._arm_status = msg

    def enable_arm(self):
        """使能机械臂"""
        if not self._enable_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arm enable service not available")
            return False
        future = self._enable_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info("Arm enabled")
            return True
        self.get_logger().error("Arm enable failed")
        return False

    def send_agv_goal(self, x, y, yaw):
        """发送 AGV 导航目标"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._agv_goal_pub.publish(msg)
        # 清掉旧 status，防止读到 goal 发出前的残留 arrived=True
        self._agv_status = None
        self.get_logger().info(f"AGV goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def send_agv_goal_by_name(self, station_name):
        """发送 AGV 站点名称导航目标（走 RoboShop 预设路径）"""
        msg = String()
        msg.data = station_name
        self._agv_goal_station_pub.publish(msg)
        self._agv_status = None
        self.get_logger().info(f"AGV goal_station: {station_name}")

    def send_arm_joints(self, positions, speed=ARM_SPEED):
        """通过 MoveToJoints service 发送关节目标（走 MoveIt 规划）"""
        self.get_logger().info(
            f"Arm goal: [{', '.join(f'{p:.2f}' for p in positions)}] speed={speed}")

        if not self._move_joints_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("move_to_joints service not available")
            return False

        req = MoveToJoints.Request()
        req.target_joints = [float(p) for p in positions]
        req.speed = float(speed)

        future = self._move_joints_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        result = future.result()
        if result is None:
            self.get_logger().error("move_to_joints service call timeout")
            return False
        if not result.success:
            self.get_logger().error(f"move_to_joints failed: {result.message}")
            return False

        self.get_logger().info("Arm move completed")
        return True

    def send_arm_pose(self, pose_dict, speed=ARM_SPEED):
        """通过 MoveToPose service 发送世界坐标位姿目标（走 MoveIt 规划）
        pose_dict: {x, y, z, roll, pitch, yaw} 单位：米、弧度
        """
        from geometry_msgs.msg import Pose
        x = pose_dict["x"]
        y = pose_dict["y"]
        z = pose_dict["z"]
        roll = pose_dict.get("roll", 0.0)
        pitch = pose_dict.get("pitch", 0.0)
        yaw = pose_dict.get("yaw", 0.0)

        self.get_logger().info(
            f"Arm pose goal: x={x:.3f} y={y:.3f} z={z:.3f} "
            f"roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f} speed={speed}")

        if not self._move_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("move_to_pose service not available")
            return False

        # RPY -> 四元数
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)

        req = MoveToPose.Request()
        req.target_pose = Pose()
        req.target_pose.position.x = float(x)
        req.target_pose.position.y = float(y)
        req.target_pose.position.z = float(z)
        req.target_pose.orientation.x = sr * cp * cy - cr * sp * sy
        req.target_pose.orientation.y = cr * sp * cy + sr * cp * sy
        req.target_pose.orientation.z = cr * cp * sy - sr * sp * cy
        req.target_pose.orientation.w = cr * cp * cy + sr * sp * sy
        req.speed = float(speed)

        future = self._move_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        result = future.result()
        if result is None:
            self.get_logger().error("move_to_pose service call timeout")
            return False
        if not result.success:
            self.get_logger().error(f"move_to_pose failed: {result.message}")
            return False

        self.get_logger().info("Arm pose move completed")
        return True

    def set_dio_output(self, channel, value):
        """设置单路 DO 输出（DIO 服务不可用时跳过不阻塞）"""
        if not self._set_dio_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("DIO set_output service not available, skipping")
            return False
        req = SetDioOutput.Request()
        req.channel = int(channel)
        req.value = bool(value)
        future = self._set_dio_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            self.get_logger().error("DIO set_output timeout")
            return False
        if not result.success:
            self.get_logger().error(f"DIO set_output failed: {result.message}")
            return False
        self.get_logger().info(f"DIO: DO{channel} = {'HIGH' if value else 'LOW'}")
        return True

    def trigger_camera(self):
        """调用海康相机软触发服务拍照"""
        if not self._trigger_capture_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Camera trigger_capture service not available, skipping")
            return False

        future = self._trigger_capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            self.get_logger().error("Camera trigger_capture timeout")
            return False
        if not result.success:
            self.get_logger().warn(f"Camera trigger_capture failed: {result.message}")
            return False

        self.get_logger().info("Camera triggered successfully")
        return True

    def wait_agv_arrived(self, timeout=60.0):
        """等待 AGV 到达"""
        self.get_logger().info("Waiting for AGV to arrive...")
        start = time.time()
        last_log_time = 0.0
        while rclpy.ok() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self._agv_status
            if s and s.connected and s.arrived and not s.moving:
                self.get_logger().info("AGV arrived!")
                return True
            # 每 5 秒输出一次当前状态，方便排查
            elapsed = time.time() - start
            if elapsed - last_log_time >= 5.0:
                last_log_time = elapsed
                if s is None:
                    self.get_logger().warn(
                        f"  [{elapsed:.0f}s] no AgvStatus received yet")
                else:
                    self.get_logger().info(
                        f"  [{elapsed:.0f}s] connected={s.connected} "
                        f"arrived={s.arrived} moving={s.moving} "
                        f"error={s.error_code}")
        s = self._agv_status
        if s is None:
            self.get_logger().error(
                "AGV arrival timeout: never received AgvStatus — "
                "check agv_driver is running and topic remapping")
        else:
            self.get_logger().error(
                f"AGV arrival timeout: connected={s.connected} "
                f"arrived={s.arrived} moving={s.moving} "
                f"error={s.error_code}")
        return False

    def run(self):
        """执行展示流程"""
        self.get_logger().info("=" * 40)
        self.get_logger().info("  Demo inspection started")
        self.get_logger().info("=" * 40)

        # 等一下让 status 订阅生效
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        # 使能机械臂
        if not self.enable_arm():
            return

        for i, station in enumerate(STATIONS):
            self.get_logger().info(f"\n--- {station['name']} ({i+1}/{len(STATIONS)}) ---")

            # 1. AGV 去站点（支持站点名称、坐标、或 None 跳过）
            agv_target = station.get("agv")
            if agv_target is None:
                self.get_logger().info("AGV skipped (no agv target)")
            elif isinstance(agv_target, str):
                self.send_agv_goal_by_name(agv_target)
                if not self.wait_agv_arrived():
                    self.get_logger().error("AGV failed, aborting")
                    break
            else:
                x, y, yaw = agv_target
                self.send_agv_goal(x, y, yaw)
                if not self.wait_agv_arrived():
                    self.get_logger().error("AGV failed, aborting")
                    break

            # 3. 机械臂摆拍照位姿（关节角或世界坐标，通过 MoveIt 规划执行）
            arm_ok = False
            if "arm_pose" in station:
                arm_ok = self.send_arm_pose(station["arm_pose"])
            elif "arm_joints" in station:
                arm_ok = self.send_arm_joints(station["arm_joints"])
            else:
                self.get_logger().warn("No arm target specified, skipping")

            # 4. 到位后调用相机软触发拍照
            if arm_ok:
                self.trigger_camera()

            # 5. DIO 控制（可选）
            dio_actions = station.get("dio")
            if dio_actions:
                for action in dio_actions:
                    self.set_dio_output(action["channel"], action["value"])

            # 6. 停留（等待拍照/曝光完成）
            dwell = station.get("dwell", 3.0)
            self.get_logger().info(f"Dwelling {dwell}s (waiting for capture)...")
            time.sleep(dwell)

        self.get_logger().info("=" * 40)
        self.get_logger().info("  Demo complete!")
        self.get_logger().info("=" * 40)


def main():
    rclpy.init()
    node = DemoNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Demo interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

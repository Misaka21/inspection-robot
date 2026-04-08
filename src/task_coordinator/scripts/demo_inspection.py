#!/usr/bin/env python3
"""
展示脚本：AGV 走预设站点，每个站点机械臂摆不同拍照位姿。

用法：
  ros2 run task_coordinator demo_inspection

修改点位：直接改下面的 STATIONS 列表。
  - agv: [x, y, yaw]          单位：米、弧度，地图坐标系
  - arm_joints: [j1..j6]      单位：弧度
  - dwell: 停留秒数（拍照时间）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from inspection_interface.msg import AgvStatus, ArmStatus
from std_srvs.srv import Trigger
import math
import time

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  在这里改点位！
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
STATIONS = [
    {
        "name": "站点1",
        "agv": [1.0, 0.0, 0.0],            # [x, y, yaw]
        "arm_joints": [0.0, 0.0, 1.57, 0.0, 1.57, 0.0],  # 拍照位姿1
        "dwell": 3.0,                        # 停留秒数
    },
    {
        "name": "站点2",
        "agv": [2.0, 1.0, 1.57],
        "arm_joints": [0.5, -0.3, 1.2, 0.0, 1.0, 0.0],   # 拍照位姿2
        "dwell": 3.0,
    },
    {
        "name": "站点3",
        "agv": [0.0, 0.0, 0.0],            # 回到起点
        "arm_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # 收回零位
        "dwell": 1.0,
    },
]

# 机械臂收回零位（每个站点拍完照后收回，AGV 再走）
ARM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]


class DemoNode(Node):
    def __init__(self):
        super().__init__("demo_inspection")

        # Publishers
        self._agv_goal_pub = self.create_publisher(
            PoseStamped, "/inspection/agv/goal_pose", 10)
        self._arm_joint_pub = self.create_publisher(
            JointState, "/inspection/arm_control/joint_goal", 10)

        # Status
        self._agv_status = None
        self._arm_status = None
        self.create_subscription(
            AgvStatus, "/inspection/agv/status", self._on_agv_status, 10)
        self.create_subscription(
            ArmStatus, "/inspection/arm/status", self._on_arm_status, 10)

        # Arm enable client
        self._enable_client = self.create_client(
            Trigger, "/inspection/arm/enable")

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
        self.get_logger().info(f"AGV goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def send_arm_joints(self, positions):
        """发送关节目标"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(p) for p in positions]
        self._arm_joint_pub.publish(msg)
        self.get_logger().info(
            f"Arm goal: [{', '.join(f'{p:.2f}' for p in positions)}]")

    def wait_agv_arrived(self, timeout=60.0):
        """等待 AGV 到达"""
        self.get_logger().info("Waiting for AGV to arrive...")
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self._agv_status
            if s and s.connected and s.arrived and not s.moving:
                self.get_logger().info("AGV arrived!")
                return True
        self.get_logger().warn("AGV arrival timeout")
        return False

    def wait_arm_arrived(self, timeout=15.0):
        """等待机械臂到位"""
        self.get_logger().info("Waiting for arm to arrive...")
        # 先等一小段让机械臂开始动
        time.sleep(0.5)
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self._arm_status
            if s and s.arrived and not s.moving:
                self.get_logger().info("Arm arrived!")
                return True
        self.get_logger().warn("Arm arrival timeout")
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

            # 1. 机械臂先收回零位再走（第一个站除外）
            if i > 0:
                self.get_logger().info("Arm -> home before moving AGV")
                self.send_arm_joints(ARM_HOME)
                self.wait_arm_arrived()
                time.sleep(0.5)

            # 2. AGV 去站点
            x, y, yaw = station["agv"]
            self.send_agv_goal(x, y, yaw)
            if not self.wait_agv_arrived():
                self.get_logger().error("AGV failed, aborting")
                break

            # 3. 机械臂摆拍照位姿
            self.send_arm_joints(station["arm_joints"])
            if not self.wait_arm_arrived():
                self.get_logger().warn("Arm timeout, continuing anyway")

            # 4. 停留（模拟拍照）
            dwell = station.get("dwell", 3.0)
            self.get_logger().info(f"Dwelling {dwell}s (simulating capture)...")
            time.sleep(dwell)

        # 最后收回
        self.get_logger().info("\nFinal: arm -> home")
        self.send_arm_joints(ARM_HOME)
        self.wait_arm_arrived()

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

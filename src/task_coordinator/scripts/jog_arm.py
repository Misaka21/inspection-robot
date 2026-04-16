#!/usr/bin/env python3
"""
极简机械臂关节调试脚本：直接发 joint_cmd，绕过 MoveIt。
只需要启动 arm_driver，不需要 arm_controller。

用法：
  ros2 run task_coordinator jog_arm

操作：
  - 直接输入 6 个角度（空格分隔，单位：度），如：
      350 -26 95 -21 118 45
  - 微调节：输入 "关节号 偏移量"，如：
      1 +10    （J1 加 10 度）
      3 -5     （J3 减 5 度）
  - h        回到 home 位
  - e        使能机械臂
  - q        退出
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import math
import sys

# 与 arm_driver.yaml 里的 command_joint_names 保持一致
JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

ARM_HOME_DEG = [90.0, -30.0, 120.0, 0.0, 30.0, 0.0]


def deg2rad(deg_list):
    return [math.radians(d) for d in deg_list]


class JogArmNode(Node):
    def __init__(self):
        super().__init__("jog_arm")
        self._pub = self.create_publisher(JointState, "/inspection/arm/joint_cmd", 10)
        self._enable_client = self.create_client(Trigger, "/inspection/arm/enable")
        self._current_joints = deg2rad(ARM_HOME_DEG)
        self.get_logger().info("Jog arm node started")

    def enable_arm(self):
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

    def send_joints(self, joints_rad):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(j) for j in joints_rad]
        self._pub.publish(msg)
        deg_str = ", ".join(f"{math.degrees(j):.2f}" for j in joints_rad)
        self.get_logger().info(f"Sent joints (deg): [{deg_str}]")
        self._current_joints = list(joints_rad)


def main():
    rclpy.init()
    node = JogArmNode()

    print("\n" + "=" * 50)
    print("  机械臂关节调试工具")
    print("=" * 50)
    print("输入示例：")
    print("  350 -26 95 -21 118 45   → 发送 6 个关节角度（度）")
    print("  1 +10                   → J1 加 10 度")
    print("  h                       → 回 home 位")
    print("  e                       → 使能机械臂")
    print("  q                       → 退出")
    print("=" * 50 + "\n")

    # 等一下让 publisher 就绪
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)

    while rclpy.ok():
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not line:
            continue

        if line.lower() == "q":
            break

        if line.lower() == "e":
            node.enable_arm()
            continue

        if line.lower() == "h":
            node.send_joints(deg2rad(ARM_HOME_DEG))
            continue

        parts = line.split()

        # 微调节模式：2 个参数，第一个是 1~6 的关节号
        if len(parts) == 2:
            try:
                idx = int(parts[0])
                delta = float(parts[1])
                if idx < 1 or idx > 6:
                    print("关节号必须是 1~6")
                    continue
                new_joints = list(node._current_joints)
                new_joints[idx - 1] += math.radians(delta)
                node.send_joints(new_joints)
                continue
            except ValueError:
                pass

        # 直接发送 6 个角度
        if len(parts) == 6:
            try:
                joints_deg = [float(p) for p in parts]
                node.send_joints(deg2rad(joints_deg))
                continue
            except ValueError:
                pass

        print("输入格式不对，请参考上面的示例。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

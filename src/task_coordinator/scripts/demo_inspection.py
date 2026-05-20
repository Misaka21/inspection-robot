#!/usr/bin/env python3
"""
巡检 Demo 脚本：
  AGV 到站位(AP1/AP2/AP3) -> 臂 HOME->wait点 -> 停留 -> wait点->photo点 -> 拍照 -> 臂回HOME

用法：
  ros2 run task_coordinator demo_inspection

配置：直接改下面的 HOME_JOINTS 和 STATIONS 列表（关节角度填角度制）。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from inspection_interface.msg import AgvStatus, ArmStatus
from inspection_interface.srv import MoveToJoints
from std_srvs.srv import Trigger
import math
import time


def deg2rad(joints_deg):
    return [math.radians(d) for d in joints_deg]


# ============================================================
#  HOME 位姿 — 所有站位从这里出发，拍照后回到这里（角度制）
# ============================================================
HOME_JOINTS = deg2rad([90.0, 30.0, 120.0, 0.0, 90.0, -30.0])

# 所有站位的 wait / photo 关节角（角度制）
WAIT_JOINTS  = deg2rad([83.308, -66.203, 105.656, -90.426, 91.595, 147.600])
PHOTO_JOINTS = deg2rad([78.058, -66.347, 104.714, -95.635, 90.774, 146.845])

# ============================================================
#  巡检点位
#    wait_joints:    臂先到这个位置停留
#    wait_duration_sec: 停留秒数
#    photo_joints:   停留后移到这个位置拍照
#    dwell:          拍照后停留秒数
# ============================================================
STATIONS = [
    {
        "name": "AP1",
        "wait_joints": WAIT_JOINTS,
        "wait_duration_sec": 2.0,
        "photo_joints": PHOTO_JOINTS,
        "dwell": 3.0,
    },
    {
        "name": "AP2",
        "wait_joints": WAIT_JOINTS,
        "wait_duration_sec": 2.0,
        "photo_joints": PHOTO_JOINTS,
        "dwell": 3.0,
    },
    {
        "name": "AP3",
        "wait_joints": WAIT_JOINTS,
        "wait_duration_sec": 2.0,
        "photo_joints": PHOTO_JOINTS,
        "dwell": 3.0,
    },
    {
        "name": "AP4",
        "wait_joints": WAIT_JOINTS,
        "wait_duration_sec": 2.0,
        "photo_joints": PHOTO_JOINTS,
        "dwell": 3.0,
    },
    {
        "name": "AP5",
        "wait_joints": WAIT_JOINTS,
        "wait_duration_sec": 2.0,
        "photo_joints": PHOTO_JOINTS,
        "dwell": 3.0,
    },
]

ARM_SPEED = 0.4


class DemoNode(Node):
    def __init__(self):
        super().__init__("demo_inspection")

        # Publishers
        self._agv_goal_station_pub = self.create_publisher(
            String, "/inspection/agv/goal_station", 10)

        # AGV status
        self._agv_status = None
        self.create_subscription(
            AgvStatus, "/inspection/agv/status", self._on_agv_status, 10)
        self.create_subscription(
            ArmStatus, "/inspection/arm/status", self._on_arm_status, 10)

        # Service clients
        self._enable_client = self.create_client(
            Trigger, "/inspection/arm/enable")
        self._move_joints_client = self.create_client(
            MoveToJoints, "/inspection/arm_control/move_to_joints")
        self._trigger_capture_client = self.create_client(
            Trigger, "/inspection/hikvision/trigger_capture")

        self.get_logger().info("Demo inspection node started")

    def _on_agv_status(self, msg):
        self._agv_status = msg

    def _on_arm_status(self, msg):
        self._arm_status = msg

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

    def send_agv_goal_by_name(self, station_name):
        """通过站点名称导航 AGV（如 AP1 / AP2 / AP3）"""
        msg = String()
        msg.data = station_name
        self._agv_goal_station_pub.publish(msg)
        self._agv_status = None
        self.get_logger().info(f"AGV goal_station: {station_name}")

    def send_arm_joints(self, positions, speed=ARM_SPEED):
        self.get_logger().info(
            f"Arm -> [{', '.join(f'{math.degrees(p):.1f}' for p in positions)}] deg  speed={speed}")

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
            self.get_logger().error("move_to_joints timeout")
            return False
        if not result.success:
            self.get_logger().error(f"move_to_joints failed: {result.message}")
            return False

        self.get_logger().info("Arm arrived")
        return True

    def trigger_camera(self):
        if not self._trigger_capture_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Camera trigger not available, skip")
            return False

        future = self._trigger_capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            self.get_logger().error("Camera trigger timeout")
            return False
        if not result.success:
            self.get_logger().warn(f"Camera trigger failed: {result.message}")
            return False

        self.get_logger().info("Camera triggered")
        return True

    def wait_agv_arrived(self, timeout=60.0):
        self.get_logger().info("Waiting for AGV...")
        start = time.time()
        last_log = 0.0
        while rclpy.ok() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self._agv_status
            if s and s.connected and s.arrived and not s.moving:
                self.get_logger().info("AGV arrived!")
                return True
            elapsed = time.time() - start
            if elapsed - last_log >= 5.0:
                last_log = elapsed
                if s is None:
                    self.get_logger().warn(f"  [{elapsed:.0f}s] no AGV status yet")
                else:
                    self.get_logger().info(
                        f"  [{elapsed:.0f}s] connected={s.connected} "
                        f"arrived={s.arrived} err={s.error_code}")
        self.get_logger().error("AGV arrival timeout")
        return False

    def run(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("  Inspection Demo")
        self.get_logger().info("  AGV -> HOME->wait -> wait->photo -> capture -> HOME")
        self.get_logger().info("=" * 50)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.enable_arm():
            return

        # 先回到 HOME
        self.get_logger().info("Initial: moving arm to HOME")
        if not self.send_arm_joints(HOME_JOINTS):
            return

        while rclpy.ok():
            for i, st in enumerate(STATIONS):
                name = st["name"]
                self.get_logger().info(f"\n--- {name} ({i+1}/{len(STATIONS)}) ---")

                # 1. AGV 去站位 AP1/AP2/AP3
                self.get_logger().info(f"  [1/6] AGV -> {name}")
                self.send_agv_goal_by_name(name)
                if not self.wait_agv_arrived():
                    return

                # 2. 臂 HOME -> 等待点
                self.get_logger().info(f"  [2/6] HOME -> wait point")
                if not self.send_arm_joints(st["wait_joints"]):
                    return

                # 3. 在等待点停留
                wait_sec = st.get("wait_duration_sec", 2.0)
                self.get_logger().info(f"  [3/6] wait {wait_sec:.1f}s")
                time.sleep(wait_sec)

                # 4. 臂 等待点 -> 拍照点
                self.get_logger().info(f"  [4/6] wait point -> photo point")
                if not self.send_arm_joints(st.get("photo_joints", st["wait_joints"])):
                    return

                # 5. 拍照
                self.get_logger().info(f"  [5/6] capture")
                self.trigger_camera()

                # 6. 拍照后停留 -> 回 HOME
                dwell = st.get("dwell", 3.0)
                self.get_logger().info(f"  [6/6] dwell {dwell:.1f}s -> HOME")
                time.sleep(dwell)
                if not self.send_arm_joints(HOME_JOINTS):
                    return

            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info("  All stations complete, next round...")
            self.get_logger().info(f"{'='*50}")


def main():
    rclpy.init()
    node = DemoNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

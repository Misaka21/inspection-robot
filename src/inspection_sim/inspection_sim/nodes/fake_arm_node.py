import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from inspection_interface.msg import ArmStatus


class FakeArmNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_arm")

        self.declare_parameter(
            "joint_names",
            ["elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5", "elfin_joint6"],
        )
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("max_joint_speed_rps", 1.0)
        self.declare_parameter("joint_tolerance_rad", 0.01)

        self._joint_names = [str(x) for x in self.get_parameter("joint_names").value]
        self._rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._max_speed = float(self.get_parameter("max_joint_speed_rps").value)
        self._tol = float(self.get_parameter("joint_tolerance_rad").value)

        n = len(self._joint_names)
        self._cur = [0.0] * n
        self._tgt = [0.0] * n
        self._moving = False

        self._lock = threading.Lock()
        self._last_t = self.get_clock().now()

        # Match arm_driver semantics: /joint_states is a global topic.
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._status_pub = self.create_publisher(ArmStatus, "status", 10)

        self._cmd_sub = self.create_subscription(JointState, "joint_cmd", self._on_joint_cmd, 10)

        period = 1.0 / max(1.0, self._rate_hz)
        self._timer = self.create_timer(period, self._on_timer)

    def _on_joint_cmd(self, msg: JointState) -> None:
        with self._lock:
            if msg.name:
                name_to_idx = {n: i for i, n in enumerate(self._joint_names)}
                for n, p in zip(msg.name, msg.position):
                    if n in name_to_idx:
                        self._tgt[name_to_idx[n]] = float(p)
            else:
                for i, p in enumerate(msg.position[: len(self._tgt)]):
                    self._tgt[i] = float(p)
            self._moving = True

    def _step_towards(self, cur: float, tgt: float, max_step: float) -> float:
        err = tgt - cur
        if abs(err) <= max_step:
            return tgt
        return cur + math.copysign(max_step, err)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(1.0, self._rate_hz)
        self._last_t = now

        with self._lock:
            max_step = self._max_speed * dt
            if self._moving:
                for i in range(len(self._cur)):
                    self._cur[i] = self._step_towards(self._cur[i], self._tgt[i], max_step)
                done = all(abs(self._tgt[i] - self._cur[i]) <= self._tol for i in range(len(self._cur)))
                if done:
                    self._moving = False

            cur = list(self._cur)
            moving = bool(self._moving)
            arrived = not moving

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(self._joint_names)
        js.position = cur
        self._joint_pub.publish(js)

        st = ArmStatus()
        st.connected = True
        st.arrived = bool(arrived)
        st.moving = bool(moving)
        st.current_joints = cur
        st.manipulability = 1.0
        st.error_code = "OK"
        self._status_pub.publish(st)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = FakeArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


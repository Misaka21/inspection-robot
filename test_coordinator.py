#!/usr/bin/env python3
"""
测试 task_coordinator 状态机流转（无需实物）
自动 mock pose_detector/agv/arm，验证完整链路
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from inspection_interface.msg import AgvStatus, ArmStatus, SystemState, InspectionPath
from std_srvs.srv import Trigger
from inspection_interface.srv import StartInspection
import time
import threading


class MockPoseDetector(Node):
    """Mock pose_detector: 响应 detect 服务，发布 detected_pose"""
    def __init__(self):
        super().__init__('mock_pose_detector')
        self.srv = self.create_service(Trigger, '/inspection/perception/detect', self.handle_detect)
        self.pub = self.create_publisher(PoseStamped, '/inspection/perception/detected_pose', 10)
        self.detected_pose = PoseStamped()
        self.detected_pose.header.frame_id = 'map'
        self.detected_pose.pose.position.x = 1.0
        self.detected_pose.pose.position.y = 0.5
        self.detected_pose.pose.position.z = 0.5
        self.detected_pose.pose.orientation.w = 1.0
        self.get_logger().info('MockPoseDetector started')

    def handle_detect(self, request, response):
        self.detected_pose.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.detected_pose)
        response.success = True
        response.message = 'Pose detected'
        self.get_logger().info(f'Service: detect -> {response.message}')
        return response


class MockAgvDriver(Node):
    """Mock AGV: 响应 goal_pose，反馈 status"""
    def __init__(self):
        super().__init__('mock_agv_driver')
        self.status_pub = self.create_publisher(AgvStatus, '/inspection/agv/status', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/inspection/agv/current_pose', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/inspection/agv/goal_pose', self.on_goal, 10)
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.pose.orientation.w = 1.0
        self.target_pose = None
        self.arrived = False
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info('MockAgvDriver started')

    def on_goal(self, msg):
        self.target_pose = msg
        self.arrived = False
        self.get_logger().info(f'Received goal: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]')

    def update(self):
        now = self.get_clock().now().to_msg()

        # 模拟 AGV 向目标移动
        if self.target_pose and not self.arrived:
            dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
            dist = (dx**2 + dy**2) ** 0.5

            if dist < 0.05:  # 到达阈值
                self.arrived = True
                self.current_pose.pose.position = self.target_pose.pose.position
                self.get_logger().info('AGV arrived at goal')
            else:
                # 向目标移动一小步
                speed = 0.1
                self.current_pose.pose.position.x += dx / dist * speed * 0.1
                self.current_pose.pose.position.y += dy / dist * speed * 0.1

        self.current_pose.header.stamp = now
        self.current_pose_pub.publish(self.current_pose)

        status = AgvStatus()
        status.connected = True
        status.arrived = self.arrived
        status.stopped = self.arrived
        status.error_code = 'OK'
        self.status_pub.publish(status)


class MockArmController(Node):
    """Mock 机械臂: 响应 joint_goal，反馈 status"""
    def __init__(self):
        super().__init__('mock_arm_controller')
        self.status_pub = self.create_publisher(ArmStatus, '/inspection/arm/status', 10)
        self.joint_sub = self.create_subscription(JointState, '/inspection/arm_control/joint_goal', self.on_joint_goal, 10)
        self.arrived = True
        self.timer = self.create_timer(0.1, self.publish_status)
        self.pending_goal = None
        self.goal_time = None
        self.get_logger().info('MockArmController started')

    def on_joint_goal(self, msg):
        self.pending_goal = msg
        self.arrived = False
        self.goal_time = time.time()
        self.get_logger().info(f'Received joint goal: {list(msg.position)}')

    def publish_status(self):
        now = self.get_clock().now().to_msg()

        # 模拟 1 秒后到达
        if not self.arrived and self.goal_time:
            if time.time() - self.goal_time > 1.0:
                self.arrived = True
                self.get_logger().info('Arm arrived at goal')

        status = ArmStatus()
        status.connected = True
        status.arrived = self.arrived
        status.error_code = 'OK'
        self.status_pub.publish(status)


class MockDefectDetector(Node):
    """Mock 缺陷检测: 响应 detect_defect 服务"""
    def __init__(self):
        super().__init__('mock_defect_detector')
        self.srv = self.create_service(Trigger, '/inspection/perception/detect_defect', self.handle_detect)
        self.get_logger().info('MockDefectDetector started')

    def handle_detect(self, request, response):
        response.success = True
        response.message = 'Defect detection completed'
        self.get_logger().info(f'Service: detect_defect -> {response.message}')
        return response


class TestOrchestrator(Node):
    """测试编排: 启动任务，验证状态流转"""
    def __init__(self):
        super().__init__('test_orchestrator')

        # QoS for state topic
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.state_sub = self.create_subscription(SystemState, '/inspection/state', self.on_state, qos)
        self.path_sub = self.create_subscription(InspectionPath, '/inspection/planning/path_detail', self.on_path, 10)

        self.start_client = self.create_client(StartInspection, '/inspection/start')

        self.phase_names = {
            0: 'IDLE',
            1: 'LOCALIZING',
            2: 'PLANNING',
            3: 'EXECUTING',
            4: 'PAUSED',
            5: 'COMPLETED',
            6: 'FAILED',
            7: 'STOPPED'
        }

        self.phase_history = []
        self.current_phase = 0
        self.path_received = False

        self.timer = self.create_timer(15.0, self.timeout_check)
        self.test_passed = False

    def on_state(self, msg):
        phase_name = self.phase_names.get(msg.phase, f'UNKNOWN({msg.phase})')

        if msg.phase != self.current_phase:
            self.get_logger().info(f'Phase transition: {self.phase_names.get(self.current_phase)} -> {phase_name}')
            self.get_logger().info(f'  Progress: {msg.progress_percent:.1f}%, Action: {msg.current_action}')
            self.phase_history.append((msg.phase, msg.current_action))
            self.current_phase = msg.phase

            if msg.phase == SystemState.PHASE_COMPLETED:
                self.test_passed = True
                self.print_report()

    def on_path(self, msg):
        if not self.path_received:
            self.path_received = True
            self.get_logger().info(f'Received InspectionPath: {len(msg.waypoints)} waypoints')
            for i, wp in enumerate(msg.waypoints):
                self.get_logger().info(f'  WP{i}: agv=[{wp.agv_pose.position.x:.2f}, {wp.agv_pose.position.y:.2f}], joints={len(wp.joint_angles)}')

    def timeout_check(self):
        if not self.test_passed:
            self.get_logger().error('Test TIMEOUT')
            self.print_report()
            rclpy.shutdown()

    def print_report(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('TEST REPORT')
        self.get_logger().info('=' * 50)
        for phase, action in self.phase_history:
            self.get_logger().info(f'  {self.phase_names.get(phase)}: {action}')
        self.get_logger().info(f'Path received: {self.path_received}')
        self.get_logger().info(f'Test passed: {self.test_passed}')

    def start_test(self):
        self.get_logger().info('Waiting for start service...')
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = StartInspection.Request()
        future = self.start_client.call_async(request)
        self.get_logger().info('Start inspection called')


def main():
    rclpy.init()

    # 创建 mock 节点
    mock_pose = MockPoseDetector()
    mock_agv = MockAgvDriver()
    mock_arm = MockArmController()
    mock_defect = MockDefectDetector()
    orchestrator = TestOrchestrator()

    # 启动 executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mock_pose)
    executor.add_node(mock_agv)
    executor.add_node(mock_arm)
    executor.add_node(mock_defect)
    executor.add_node(orchestrator)

    # 在后台运行
    thread = threading.Thread(target=executor.spin)
    thread.start()

    # 等待系统稳定
    time.sleep(2.0)

    # 启动测试
    orchestrator.start_test()

    # 等待测试完成或超时
    thread.join(timeout=20.0)

    if orchestrator.test_passed:
        print('\n✅ TEST PASSED')
    else:
        print('\n❌ TEST FAILED')

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/bin/bash
# 手动测试脚本 - 逐步验证每个环节

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== 启动系统（后台）==="
ros2 launch inspection_bringup test_coordinator.launch.py &
LAUNCH_PID=$!
sleep 5

echo ""
echo "=== 查看节点 ==="
ros2 node list

echo ""
echo "=== 查看 topic ==="
ros2 topic list | grep -E "(inspection|planning|perception)"

echo ""
echo "=== 手动发送检测点 ==="
ros2 topic pub /inspection/planning/detection_points geometry_msgs/PoseArray '{header: {frame_id: "map"}, poses: [{position: {x: 1.0, y: 0.5, z: 0.5}, orientation: {w: 1.0}}]}' --once

echo ""
echo "=== 手动发送 AGV 当前位姿 ==="
ros2 topic pub /inspection/agv/current_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1.0}}}' --once

echo ""
echo "=== 调用规划服务 ==="
ros2 service call /inspection/planning/optimize std_srvs/srv/Trigger

echo ""
echo "=== 查看规划结果 ==="
sleep 1
ros2 topic echo /inspection/planning/path --once 2>/dev/null || echo "No path published"
ros2 topic echo /inspection/planning/path_detail --once 2>/dev/null || echo "No path_detail published"

echo ""
echo "=== 清理 ==="
kill $LAUNCH_PID 2>/dev/null

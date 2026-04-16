#!/bin/bash
# 一键启动 Demo 所需的 4 个终端
# 核心修复：所有终端必须显式使用同一套 FastDDS 配置，否则 sudo 与普通用户进程互相发现/通信会失败。

PASSWORD="mic-733ao"
WORK_DIR="$HOME/huo_ws/inspection-robot"
FASTDDS_XML="$WORK_DIR/src/inspection_bringup/config/fastdds_no_shm.xml"

# 当前脚本也需要 ROS 环境才能做 topic 探测
source /opt/ros/humble/setup.bash
source "$WORK_DIR/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML"

cmds=(
    "echo '$PASSWORD' | sudo -E -S bash -c 'export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch arm_driver arm_driver.launch.py'"
    "export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch arm_controller arm_controller.launch.py rviz:=true"
    "export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch hikvision_driver hikvision_driver.launch.py use_trigger_mode:=true"
    "export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run task_coordinator demo_inspection"
)

titles=(
    "Arm Driver"
    "Arm Controller"
    "Hikvision Driver"
    "Demo Inspection"
)

# 第一个终端（sudo arm_driver）
echo "Starting [${titles[0]}]..."
gnome-terminal --title="${titles[0]}" --working-directory="$WORK_DIR" \
    -- bash -c "${cmds[0]}; exec bash"

# 循环检测 /joint_states 是否已发布（说明 EtherCAT 初始化成功）
echo "Waiting for arm_driver to initialize and publish /joint_states..."
MAX_WAIT=120
WAITED=0
while [ $WAITED -lt $MAX_WAIT ]; do
    if ros2 topic list 2>/dev/null | grep -q "^/joint_states$"; then
        echo "/joint_states detected after ${WAITED}s. Proceeding..."
        break
    fi
    sleep 2
    WAITED=$((WAITED + 2))
done

if [ $WAITED -ge $MAX_WAIT ]; then
    echo "Warning: timed out waiting for /joint_states. arm_controller/RViz may not see the arm pose."
fi

# 后续终端
for ((i=1; i<${#cmds[@]}; i++)); do
    echo "Starting [${titles[i]}]..."
    gnome-terminal --title="${titles[i]}" --working-directory="$WORK_DIR" \
        -- bash -c "${cmds[i]}; exec bash"
    if (( i < ${#cmds[@]} - 1 )); then
        echo "Wait 10s..."
        sleep 10
    fi
done

echo "All terminals started."

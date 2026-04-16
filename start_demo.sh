#!/bin/bash
# 一键启动 Demo 所需的 4 个终端
# 终端 1: arm_driver (sudo)，启动后等待 15s
# 终端 2-4: 各等待 3s 后顺序启动

PASSWORD="mic-733ao"
WORK_DIR="$HOME/huo_ws/inspection-robot"

cmds=(
    "echo '$PASSWORD' | sudo -E -S bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch arm_driver arm_driver.launch.py'"
    "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch arm_controller arm_controller.launch.py rviz:=true"
    "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch hikvision_driver hikvision_driver.launch.py use_trigger_mode:=true"
    "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run task_coordinator demo_inspection"
)

titles=(
    "Arm Driver"
    "Arm Controller"
    "Hikvision Driver"
    "Demo Inspection"
)

# 第一个终端
echo "Starting [${titles[0]}]..."
gnome-terminal --title="${titles[0]}" --working-directory="$WORK_DIR" \
    -- bash -c "${cmds[0]}; exec bash"
echo "Wait 15s for arm_driver to initialize..."
sleep 15

# 后续终端
for ((i=1; i<${#cmds[@]}; i++)); do
    echo "Starting [${titles[i]}]..."
    gnome-terminal --title="${titles[i]}" --working-directory="$WORK_DIR" \
        -- bash -c "${cmds[i]}; exec bash"
    if (( i < ${#cmds[@]} - 1 )); then
        echo "Wait 3s..."
        sleep 3
    fi
done

echo "All terminals started."

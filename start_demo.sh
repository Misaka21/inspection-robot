#!/bin/bash
# 一键启动 Demo 所需的 4 个终端
# 终端 1: arm_driver (sudo)
# 终端 2: arm_controller + RViz
# 终端 3: hikvision_driver (触发模式)
# 终端 4: demo_inspection

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

# 创建主终端窗口
gnome-terminal --window --title="${titles[0]}" --working-directory="$WORK_DIR" \
    -- bash -c "${cmds[0]}; exec bash"

# 添加新标签页
for ((i=1; i<${#cmds[@]}; i++)); do
    gnome-terminal --tab --title="${titles[i]}" --working-directory="$WORK_DIR" \
        -- bash -c "${cmds[i]}; exec bash"
done

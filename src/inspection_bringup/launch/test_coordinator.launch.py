"""测试 task_coordinator 的 launch（无硬件依赖）"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 基础参数
    params = {'namespace': '/inspection'}

    nodes = [
        # task_coordinator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('task_coordinator'), 'launch', 'task_coordinator.launch.py')
            ),
            launch_arguments=params.items(),
        ),
        # path_planner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('path_planner'), 'launch', 'path_planner.launch.py')
            ),
            launch_arguments={'namespace': '/inspection/planning'}.items(),
        ),
        # pose_detector (骨架)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pose_detector'), 'launch', 'pose_detector.launch.py')
            ),
            launch_arguments={'namespace': '/inspection/perception'}.items(),
        ),
        # defect_detector (骨架)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('defect_detector'), 'launch', 'defect_detector.launch.py')
            ),
            launch_arguments={'namespace': '/inspection/perception'}.items(),
        ),
        # agv_driver (骨架)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('agv_driver'), 'launch', 'agv_driver.launch.py')
            ),
            launch_arguments={'namespace': '/inspection/agv'}.items(),
        ),
        # arm_driver (骨架)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('arm_driver'), 'launch', 'arm_driver.launch.py')
            ),
            launch_arguments={'namespace': '/inspection/arm'}.items(),
        ),
        # arm_controller (骨架)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('arm_controller'), 'launch', 'arm_controller.launch.py')
            ),
            launch_arguments={'namespace': '/inspection/arm_control'}.items(),
        ),
    ]

    return LaunchDescription(nodes)

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """启动完整的检测系统"""
    
    # 驱动层
    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('inspection_bringup'), 'launch'),
            '/drivers.launch.py'
        ])
    )
    
    # AGV / Arm
    agv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agv_driver'), 'launch', 'agv_driver.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/agv',
        }.items(),
    )
    arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_driver'), 'launch', 'arm_driver.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/arm',
        }.items(),
    )
    arm_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_controller'), 'launch', 'arm_controller.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/arm_control',
        }.items(),
    )

    # Perception / planning / coordination (工程骨架)
    pose_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_detector'), 'launch', 'pose_detector.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/perception',
        }.items(),
    )
    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('path_planner'), 'launch', 'path_planner.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/planning',
        }.items(),
    )
    defect_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('defect_detector'), 'launch', 'defect_detector.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/perception',
        }.items(),
    )
    task_coordinator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('task_coordinator'), 'launch', 'task_coordinator.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection',
        }.items(),
    )
    
    return LaunchDescription([
        drivers_launch,
        agv_launch,
        arm_driver_launch,
        arm_controller_launch,
        pose_detector_launch,
        path_planner_launch,
        defect_detector_launch,
        task_coordinator_launch,
        
        # Foxglove Bridge（用于可视化调试）
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        ),
    ])

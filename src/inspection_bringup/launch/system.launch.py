from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """启动完整的检测系统"""

    bringup_config = os.path.join(get_package_share_directory('inspection_bringup'), 'config')

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
            'params_file': os.path.join(bringup_config, 'agv_driver.yaml'),
        }.items(),
    )
    arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_driver'), 'launch', 'arm_driver.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/arm',
            'params_file': os.path.join(bringup_config, 'arm_driver.yaml'),
        }.items(),
    )
    arm_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_controller'), 'launch', 'arm_controller.launch.py')
        ),
        launch_arguments={
            'namespace': '/inspection/arm_control',
            'params_file': os.path.join(bringup_config, 'arm_controller.yaml'),
        }.items(),
    )

    # Perception / coordination
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
            'stations_file': os.path.join(bringup_config, 'inspection_stations.yaml'),
        }.items(),
    )

    # HMI <-> ROS2 gateway (FastAPI REST/WS)
    inspection_gateway_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('inspection_gateway'), 'launch', 'inspection_gateway.launch.py')
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
        defect_detector_launch,
        task_coordinator_launch,
        inspection_gateway_launch,
        
        # Foxglove Bridge（用于可视化调试）
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        ),
    ])

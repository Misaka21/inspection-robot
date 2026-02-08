from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import Shutdown, IncludeLaunchDescription
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
    
    # TODO: 添加其他节点
    # - agv_driver
    # - arm_driver  
    # - workpiece_detector
    # - path_optimizer
    # - defect_detector
    # - task_coordinator
    
    return LaunchDescription([
        drivers_launch,
        
        # Foxglove Bridge（用于可视化调试）
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        ),
    ])

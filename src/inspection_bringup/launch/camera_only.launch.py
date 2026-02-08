"""
仅启动海康工业相机（调试用）
用法:
  ros2 launch inspection_bringup camera_only.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import Shutdown
from ament_index_python.packages import get_package_share_directory

import os

node_params = os.path.join(
    get_package_share_directory('inspection_bringup'),
    'config',
    'inspection.yaml'
)


def generate_launch_description():
    return LaunchDescription([
        # 相机驱动容器
        ComposableNodeContainer(
            name='hikvision_container',
            namespace='inspection',
            package='rclcpp_components',
            executable='component_container_isolated',
            arguments=['--use_multi_threaded_executor'],
            composable_node_descriptions=[
                ComposableNode(
                    package='hikvision_driver',
                    plugin='hikvision_driver::HikvisionDriverNode',
                    name='hikvision_driver',
                    namespace='inspection/hikvision',
                    parameters=[node_params, {
                        'use_trigger_mode': False,  # 调试用连续模式
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        ),
        
        # 相机可视化
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        ),
    ])

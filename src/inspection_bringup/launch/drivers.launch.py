from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import Shutdown
from ament_index_python.packages import get_package_share_directory

import os

# 参数文件路径
node_params = os.path.join(
    get_package_share_directory('inspection_bringup'),
    'config',
    'inspection.yaml'
)


def get_camera_container():
    """创建相机驱动容器（组件模式）"""
    return ComposableNodeContainer(
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
                parameters=[node_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

def get_realsense_launch():
    """启动 RealSense 官方驱动，参数由 realsense_driver 配置包提供"""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_namespace': 'inspection/realsense',
            'camera_name': 'd435',
            'config_file': os.path.join(
                get_package_share_directory('realsense_driver'),
                'config',
                'realsense.yaml'
            ),
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription([
        # 相机驱动容器
        get_camera_container(),
        get_realsense_launch(),
        
        # TF 静态变换：机械臂末端到工业相机
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tcp_to_hikvision',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.05',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                '--frame-id', 'tool0',
                '--child-frame-id', 'hikvision_frame'
            ],
        ),
    ])

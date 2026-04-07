"""
RealSense 相机启动文件

使用系统安装的 ros-humble-realsense2-camera 驱动
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 系统包的路径
    realsense2_camera_share = get_package_share_directory('realsense2_camera')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('inspection_bringup'),
            'config',
            'realsense.yaml',
        ),
        description='Path to RealSense config file',
    )

    return LaunchDescription([
        config_file_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense2_camera_share, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'camera_namespace': 'inspection/realsense',
                'camera_name': 'd435',
                'config_file': LaunchConfiguration('config_file'),
            }.items(),
        )
    ])

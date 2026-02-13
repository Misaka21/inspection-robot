"""
RealSense 相机启动文件

使用系统安装的 ros-humble-realsense2-camera 驱动
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 系统包的路径
    realsense2_camera_share = get_package_share_directory('realsense2_camera')

    # 本地配置文件
    config_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config',
        'realsense.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense2_camera_share, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'camera_namespace': 'inspection/realsense',
                'camera_name': 'd435',
                'config_file': config_file,
            }.items(),
        )
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("dio_driver")

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="/inspection/dio"
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "config", "dio_driver.yaml"),
    )

    dio_node = Node(
        package="dio_driver",
        executable="dio_driver",
        namespace=LaunchConfiguration("namespace"),
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([namespace_arg, params_file_arg, dio_node])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/planning",
        description="Namespace for path_planner node",
    )

    # 默认参数文件路径
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("path_planner"),
            "config",
            "path_planner.yaml",
        ),
        description="Path to ROS2 parameters file",
    )

    node = Node(
        package="path_planner",
        executable="path_planner_node",
        name="path_planner",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        node,
    ])

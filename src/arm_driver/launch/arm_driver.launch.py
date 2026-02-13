from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/arm",
        description="Namespace for arm_driver node",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("arm_driver"),
            "config",
            "arm_driver.yaml",
        ),
        description="Path to ROS2 parameters file",
    )

    arm_driver_node = Node(
        package="arm_driver",
        executable="arm_driver_node",
        name="arm_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        arm_driver_node,
    ])

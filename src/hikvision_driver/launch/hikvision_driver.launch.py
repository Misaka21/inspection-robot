from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    params_file_default = os.path.join(
        get_package_share_directory("hikvision_driver"),
        "config",
        "hikvision_driver.yaml",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/hikvision",
        description="Namespace for hikvision_driver node",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file_default,
        description="Path to ROS2 parameters file",
    )
    use_trigger_mode_arg = DeclareLaunchArgument(
        "use_trigger_mode",
        default_value="false",
        description="Whether to enable trigger mode",
    )

    node = Node(
        package="hikvision_driver",
        executable="hikvision_driver_node",
        name="hikvision_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_trigger_mode": LaunchConfiguration("use_trigger_mode")},
        ],
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        use_trigger_mode_arg,
        node,
    ])

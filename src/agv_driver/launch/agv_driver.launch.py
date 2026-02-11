from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/agv",
        description="Namespace for agv_driver_node",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("agv_driver"), "config", "agv_driver.yaml"
        ),
        description="Path to ROS2 parameters file",
    )

    node = Node(
        package="agv_driver",
        executable="agv_driver_node",
        name="agv_driver_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        node,
    ])

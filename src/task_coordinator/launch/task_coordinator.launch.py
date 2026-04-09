from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection",
        description="Namespace for task_coordinator node",
    )

    stations_file_arg = DeclareLaunchArgument(
        "stations_file",
        default_value="",
        description="Path to inspection_stations.yaml",
    )

    node = Node(
        package="task_coordinator",
        executable="task_coordinator_node",
        name="task_coordinator",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "stations_file": LaunchConfiguration("stations_file"),
        }],
    )

    return LaunchDescription([
        namespace_arg,
        stations_file_arg,
        node,
    ])

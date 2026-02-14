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

    node = Node(
        package="task_coordinator",
        executable="task_coordinator_node",
        name="task_coordinator",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )

    return LaunchDescription([
        namespace_arg,
        node,
    ])


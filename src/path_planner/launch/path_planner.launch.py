from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/planning",
        description="Namespace for path_planner node",
    )

    node = Node(
        package="path_planner",
        executable="path_planner_node",
        name="path_planner",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )

    return LaunchDescription([
        namespace_arg,
        node,
    ])


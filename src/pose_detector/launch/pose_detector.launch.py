from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/perception",
        description="Namespace for pose_detector node",
    )

    node = Node(
        package="pose_detector",
        executable="pose_detector_node",
        name="pose_detector",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )

    return LaunchDescription([
        namespace_arg,
        node,
    ])


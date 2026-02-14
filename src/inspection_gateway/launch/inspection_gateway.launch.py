from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection",
        description="Namespace for inspection_gateway process",
    )
    grpc_port_arg = DeclareLaunchArgument(
        "grpc_port",
        default_value="50051",
        description="gRPC listen port",
    )

    node = Node(
        package="inspection_gateway",
        executable="inspection_gateway",
        name="inspection_gateway",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        arguments=[
            "--grpc-port",
            LaunchConfiguration("grpc_port"),
        ],
    )

    return LaunchDescription([
        namespace_arg,
        grpc_port_arg,
        node,
    ])


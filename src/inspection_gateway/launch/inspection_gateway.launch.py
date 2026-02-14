from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection",
        description="Namespace for inspection system (also passed to inspection_gateway --ros-root-ns)",
    )
    grpc_port_arg = DeclareLaunchArgument(
        "grpc_port",
        default_value="50051",
        description="gRPC listen port",
    )
    grpc_workers_arg = DeclareLaunchArgument(
        "grpc_max_workers",
        default_value="16",
        description="gRPC thread pool size",
    )
    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value="~/.ros/inspection_gateway",
        description="Local data dir for CAD/media cache",
    )
    cache_dir_arg = DeclareLaunchArgument(
        "cache_dir",
        default_value="~/.cache/inspection_gateway",
        description="Local cache dir for generated protobuf modules",
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
            "--grpc-max-workers",
            LaunchConfiguration("grpc_max_workers"),
            "--data-dir",
            LaunchConfiguration("data_dir"),
            "--cache-dir",
            LaunchConfiguration("cache_dir"),
            "--ros-root-ns",
            LaunchConfiguration("namespace"),
        ],
    )

    return LaunchDescription([
        namespace_arg,
        grpc_port_arg,
        grpc_workers_arg,
        data_dir_arg,
        cache_dir_arg,
        node,
    ])

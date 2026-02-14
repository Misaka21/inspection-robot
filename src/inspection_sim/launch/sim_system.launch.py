from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    with_gateway_arg = DeclareLaunchArgument(
        "with_gateway",
        default_value="false",
        description="Whether to start inspection_gateway (requires grpcio/grpcio-tools installed)",
    )
    grpc_port_arg = DeclareLaunchArgument(
        "grpc_port",
        default_value="50051",
        description="inspection_gateway gRPC listen port",
    )

    fake_agv = Node(
        package="inspection_sim",
        executable="fake_agv",
        name="fake_agv",
        namespace="/inspection/agv",
        output="screen",
    )

    fake_arm = Node(
        package="inspection_sim",
        executable="fake_arm",
        name="fake_arm",
        namespace="/inspection/arm",
        output="screen",
    )

    fake_perception = Node(
        package="inspection_sim",
        executable="fake_perception",
        name="fake_perception",
        namespace="/inspection/perception",
        output="screen",
    )

    fake_planning = Node(
        package="inspection_sim",
        executable="fake_planning",
        name="fake_planning",
        namespace="/inspection/planning",
        output="screen",
    )

    fake_defect = Node(
        package="inspection_sim",
        executable="fake_defect",
        name="fake_defect",
        namespace="/inspection/perception",
        output="screen",
    )

    task_coordinator = Node(
        package="task_coordinator",
        executable="task_coordinator_node",
        name="task_coordinator",
        namespace="/inspection",
        output="screen",
    )

    gateway = Node(
        package="inspection_gateway",
        executable="inspection_gateway",
        name="inspection_gateway",
        namespace="/inspection",
        output="screen",
        arguments=[
            "--grpc-port",
            LaunchConfiguration("grpc_port"),
            "--ros-root-ns",
            "/inspection",
        ],
        condition=IfCondition(LaunchConfiguration("with_gateway")),
    )

    return LaunchDescription([
        with_gateway_arg,
        grpc_port_arg,
        fake_agv,
        fake_arm,
        fake_perception,
        fake_planning,
        fake_defect,
        task_coordinator,
        gateway,
    ])


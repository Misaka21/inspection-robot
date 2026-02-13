from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable

import os
import yaml


def load_file(package_name: str, relative_path: str) -> str:
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, relative_path)
    with open(abs_path, "r", encoding="utf-8") as f:
        return f.read()


def load_yaml(package_name: str, relative_path: str):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, relative_path)
    with open(abs_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="/inspection/arm_control",
        description="Namespace for arm_controller node",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("arm_controller"),
            "config",
            "arm_controller.yaml",
        ),
        description="Path to ROS2 parameters file",
    )

    xacro_file = os.path.join(
        get_package_share_directory("elfin_description"),
        "urdf",
        "elfin5.urdf.xacro",
    )

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])
    }

    robot_description_semantic = {
        "robot_description_semantic": load_file("elfin5_ros2_moveit2", "config/elfin5.srdf")
    }

    kinematics_yaml = load_yaml("elfin5_ros2_moveit2", "config/kinematics.yaml")

    arm_controller_node = Node(
        package="arm_controller",
        executable="arm_controller_node",
        name="arm_controller",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    # Optional: publish TF tree from /joint_states + robot_description.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="arm_robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        robot_state_publisher_node,
        arm_controller_node,
    ])

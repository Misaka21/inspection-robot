from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            get_package_share_directory("inspection_bringup"),
            "config",
            "arm_controller.yaml",
        ),
        description="Path to ROS2 parameters file",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz with MoveIt MotionPlanning plugin",
    )

    # ── Robot description ──
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

    # Joint limits (MoveIt 时间参数化依赖；URDF 只提供 velocity，没有 acceleration，
    # 不加这段的话 MoveIt 加速度默认 1.0 rad/s²，再怎么调 acceleration_scaling 也慢)
    # 固件上限：速度 180/200 deg/s (3.1416/3.4907 rad/s)，加速度 360 deg/s² (6.28 rad/s²)
    joint_limits = {
        "robot_description_planning": {
            "joint_limits": {
                "elfin_joint1": {"has_velocity_limits": True, "max_velocity": 3.1416,
                                 "has_acceleration_limits": True, "max_acceleration": 5.0},
                "elfin_joint2": {"has_velocity_limits": True, "max_velocity": 3.1416,
                                 "has_acceleration_limits": True, "max_acceleration": 5.0},
                "elfin_joint3": {"has_velocity_limits": True, "max_velocity": 3.1416,
                                 "has_acceleration_limits": True, "max_acceleration": 5.0},
                "elfin_joint4": {"has_velocity_limits": True, "max_velocity": 3.1416,
                                 "has_acceleration_limits": True, "max_acceleration": 5.0},
                "elfin_joint5": {"has_velocity_limits": True, "max_velocity": 3.4907,
                                 "has_acceleration_limits": True, "max_acceleration": 5.0},
                "elfin_joint6": {"has_velocity_limits": True, "max_velocity": 3.4907,
                                 "has_acceleration_limits": True, "max_acceleration": 5.0},
            }
        }
    }

    kinematics_yaml = load_yaml("elfin5_ros2_moveit2", "config/kinematics.yaml")

    # ── OMPL planning pipeline (for move_group) ──
    ompl_planning_yaml = load_yaml("elfin5_ros2_moveit2", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ]),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # ── Trajectory execution (planning-only mode, no ros2_control) ──
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ── Nodes ──
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="arm_robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

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

    # ── RViz (optional) ──
    rviz_config = os.path.join(
        get_package_share_directory("elfin5_ros2_moveit2"),
        "launch",
        "elfin5_moveit2.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        rviz_arg,
        move_group_node,
        robot_state_publisher_node,
        arm_controller_node,
        rviz_node,
    ])

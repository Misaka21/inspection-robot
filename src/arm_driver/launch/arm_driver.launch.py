from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("arm_driver"),
        "config",
        "arm_driver.yaml",
    )

    arm_driver_node = Node(
        package="arm_driver",
        executable="arm_driver_node",
        name="arm_driver",
        namespace="inspection/arm",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([arm_driver_node])

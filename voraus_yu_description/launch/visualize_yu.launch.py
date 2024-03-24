import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simple_collision_model_parameter_name = "simple_collision_model"
    simple_collision_model = LaunchConfiguration(
        simple_collision_model_parameter_name, default=True
    )

    robot_xacro_file = os.path.join(
        get_package_share_directory("voraus_yu_description"),
        "urdf",
        "yu+5-100.urdf.xacro",
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_xacro_file,
            " simple_collision_model:=",
            simple_collision_model,
        ]
    )

    rviz_file = os.path.join(
        get_package_share_directory("voraus_yu_description"),
        "rviz",
        "visualize_yu.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                simple_collision_model_parameter_name,
                default_value="false",
                description="Use simple collision model or meshes",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
            ),
        ]
    )

import os


from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
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
            " simple_collision_model:=true",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="voraus_ros_bridge",
                executable="voraus_ros_bridge",
                name="voraus_ros_bridge",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    (
                        "/joint_states",
                        "/voraus_ros_bridge/joint_states",
                    ),
                ],
            )
        ]
    )

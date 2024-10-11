import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="voraus-ros-bridge",
                namespace=os.environ.get("ROS_NAMESPACE", "robot1"),
                executable="voraus-ros-bridge",
                name="voraus_ros_bridge",
            )
        ]
    )

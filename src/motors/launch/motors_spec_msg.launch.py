##launch file
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="motors",
                executable="motors_node",
                name="motors_node",
                output="screen",
            ),
        ]
    )

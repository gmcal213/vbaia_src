from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="oak_d_poe",
            executable="implement_detector",
            name="detector"
        )
    ])

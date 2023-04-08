from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="lidar_processor",
            executable="lidarvis_node",
            name="lidarvis1",
            parameters=[
                {"topic_name": "filtered/segmented"}
            ]
        ),
        Node(
            package="oak_d_poe",
            executable="rgb_subscriber",
            name="oakvis"
        )
    ])
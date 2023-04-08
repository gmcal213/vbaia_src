from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lidar_processor",
            executable="lidarconcat_node",
            name="concatenator",
            parameters=[
                {"num_point_clouds": 4}
            ]
        ),
        Node(
            package="lidar_processor",
            executable="lidarfil_node",
            name="filterer",
            parameters=[
                {"x_min": 1.0},
                {"x_max": 3.0}
            ]
        ),
        Node(
            package="lidar_processor",
            executable="lidarseg_node",
            name="segmenter"
        )
    ])
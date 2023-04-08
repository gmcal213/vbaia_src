import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    livox_mid40 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('livox_ros2_driver'), 'launch'),
            '/livox_lidar_launch.py'
        ])
    )
    livox_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lidar_processor'), 'launch'),
            '/lidar_processor_launch.py'
        ])
    )
    oak_d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('oak_d_poe')),
            '/oak_d_launch.py'
        ])
    )


    return LaunchDescription([
        livox_mid40,
        livox_processor,
        oak_d
    ])
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# this is the function launch  system will look for
def generate_launch_description():

    launch_slam = os.path.join(
        get_package_share_directory('lidarslam'), 
        'launch',
        'lidarslam.launch.py'
    )

    launch_localiz = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'launch',
        'lidar_localization.launch.py'
    )

    # Robot State Publisher

    start_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam)
    )

    start_localisation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_localiz)
    )

    
    # create and return launch description object
    return LaunchDescription([

        start_slam,   
        start_localisation,
        ])

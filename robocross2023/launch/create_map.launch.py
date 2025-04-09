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
        get_package_share_directory('lidarslam'),  # Укажите имя вашего пакета
        'launch',
        'lidarslam.launch.py'
    )

    # Robot State Publisher

    start_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam)
    )

    create_2d_map_node = Node(
        package='robocross2023',
        executable='create_2d_map',
        name='create_2d_map',
    )
    
    # create and return launch description object
    return LaunchDescription([

        start_slam,   
        create_2d_map_node,


        ])

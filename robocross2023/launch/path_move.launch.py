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

    hybrid_a_star_node = Node(
        package='robocross2023',
        executable='hybrid_a_star.py',
        name='hybrid_a_star',
    )

    pure_pursuit_node = Node(
        package='robocross2023',
        executable='pure_pursuit',
        name='pure_pursuit',
    )

    
    # create and return launch description object
    return LaunchDescription([

        hybrid_a_star_node,   
        pure_pursuit_node,

        ])

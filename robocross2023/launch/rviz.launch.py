import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# this is the function launch  system will look for
def generate_launch_description():


    bringup_launch = os.path.join(
        get_package_share_directory('robocross2023'),
        'launch',
        'bringup.launch.py'
    )

    localiz_launch = os.path.join(
        get_package_share_directory('robocross2023'),
        'launch',
        'localization.launch.py'
    )

    start_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch)
    )

    start_localiz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localiz_launch)
    )

    rviz = os.path.join(get_package_share_directory('robocross2023'), 'rviz', 'robocross.rviz')


    

    tfs = Node(
        package='robocross2023',
        executable='transforms.py',
        name='transforms',
    )




    create_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    mapping = Node(
        package='robocross2023',
        executable='mapping',
        name='mapping',
    )

    planner = Node(
        package='robocross2023',
        executable='hybrid_a_star.py',
        name='hybrid_a_star',
    )

    pure_node = Node(
        package='robocross2023',
        executable='pure_pursuit',
        name='pure_pursuit',
    )

    mission_node = Node(
        package='robocross2023',
        executable='mission_controll.py',
        name='mission_controll',
    )


    
    
    # create and return launch description object
    return LaunchDescription([  
    Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output='screen'
        )
        ])

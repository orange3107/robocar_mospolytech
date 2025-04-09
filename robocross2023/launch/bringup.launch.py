import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'model.sdf'
    #xacro_file = "box_bot.xacro"
    package_description = "robocross2023"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), 'models/autocar_real', urdf_file)

    launch_velodyne = os.path.join(
        get_package_share_directory('velodyne'),
        'launch',
        'velodyne-all-nodes-VLP32C-launch.py'
    )

    # Robot State Publisher

    velodyne_start_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_velodyne)
    )

    serial_move_node = Node(
        package='robocross2023',
        executable='ser_move.py',
        name='ser_move',
    )

    serial_odom_node = Node(
        package='robocross2023',
        executable='serial_odom.py',
        name='serial_odom',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    calc_odom_node = Node(
        package='robocross2023',
        executable='calc_odom.py',
        name='calc_odom',
    )

    car_in_global_map_node = Node(
        package='robocross2023',
        executable='car_in_global_map.py',
        name='car_in_global_map',
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '-1.57', '0', '0', 'map', 'odom']
    )
    
    # create and return launch description object
    return LaunchDescription([   

            serial_move_node,
            robot_state_publisher_node,
            serial_odom_node,
            calc_odom_node,
            car_in_global_map_node,
            velodyne_start_launch,
            

        ])

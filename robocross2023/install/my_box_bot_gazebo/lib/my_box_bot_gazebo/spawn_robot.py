#!/usr/bin/python3
# -*- coding: utf-8 -*-

import argparse
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy
def main():
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo')
    parser.add_argument('-sdf', '--robot_sdf', type=str, default='/home/parallels/projects/ar_ht/src/pr_sim_ros2/ar_ht_gazebo/models/ar_ht_robot/model.sdf',
                        help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_name', type=str, default='ar_ht_robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-namespace', '--namespace', type=bool, default=True,
                        help='Whether to enable namespacing')
    parser.add_argument('-x','--x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y','--y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z','--z', type=float, default=0,
                        help='the z component of the initial position [meters]')

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    sdf_file_path = args.robot_sdf
    # sdf_file_path = "/home/rodion/PROJECTS/quadro_sim/install/ran_sim/share/ran_sim/models/dif_drive/model.sdf"
    content = ''
    with open(sdf_file_path, 'r') as content_file:
        content = content_file.read()

    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    # root = root[0]
    imu_plugin = None
    diff_drive_plugin = None 
    # for child in root:
    #     for plugin in child.iter('plugin'):
    #         if 'ar_ht_diff_drive' in plugin.attrib.values():
    #             diff_drive_plugin = plugin
    #         elif 'box_bot_imu_plugin' in plugin.attrib.values():
    #             imu_plugin = plugin

    # tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
    # tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
    # tag_diff_drive_ns.text = '/' + args.robot_namespace
    # ros_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
    # ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'
 
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')
    # request.xml = content
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)
    # print(request.xml)
    if args.namespace is True:
        node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.robot_name, args.robot_namespace, args.x, args.y, args.z))

        request.robot_namespace = args.robot_namespace
        print(args.robot_namespace)

    else:
        node.get_logger().info('spawning `{}` at {}, {}, {}'.format(
            args.robot_name, args.x, args.y, args.z))

    node.get_logger().info('Spawning Robot using service: `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
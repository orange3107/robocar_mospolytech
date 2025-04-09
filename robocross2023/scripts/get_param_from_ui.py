#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String
import os
from ament_index_python.packages import get_package_share_directory

class GetParamFromUI(Node):

    def __init__(self):
        super().__init__('get_param_from_ui')

        #подписки
        self.car_parameter_sub = self.create_subscription(Float32MultiArray, '/car_parameter', self.get_car_parameter, 1)
        self.state_button_sub = self.create_subscription(String, "/state_button", self.get_state_button, 10)
        self.steer_angle_parameter_sub = self.create_subscription(Float32MultiArray, "/steer_angle_parameter", self.get_steer_angle_parameter, 1)

        self.points_name_sub = self.create_subscription(String, "/points_name", self.get_points_name, 1)
        self.map_name_sub = self.create_subscription(String, "/map_name", self.get_map_name, 1)
        

    def get_car_parameter(self, data):
        self.modify_one_block_of_yaml('length', data.data[0], 'car_params.yaml')
        self.modify_one_block_of_yaml('width', data.data[1], 'car_params.yaml')
        self.modify_one_block_of_yaml('wheelBase', data.data[2], 'car_params.yaml')
        self.modify_one_block_of_yaml('wheeltrack', data.data[3], 'car_params.yaml')
        self.modify_one_block_of_yaml('distLidar', data.data[4], 'car_params.yaml')
        self.modify_one_block_of_yaml('heightLidar', data.data[5], 'car_params.yaml')

    def get_state_button(self, data):
        None

    def get_steer_angle_parameter(self, data):
        print(data.data)
        self.modify_one_block_of_yaml('wheelAngles', [[data.data[0], data.data[1], data.data[2], data.data[3], data.data[4]],[data.data[5], data.data[6], data.data[7], data.data[8], data.data[9]]], 'car_params.yaml')

    def get_points_name(self, data):
        self.modify_one_block_of_yaml('points_name', data, 'nav_params.yaml')

    def get_map_name(self, data):
        self.modify_one_block_of_yaml('map_name', data, 'nav_params.yaml')


    def modify_one_block_of_yaml(self, key, value, yaml_name):

        #dir_path = os.path.join(get_package_share_directory('robocross2023'), 'params', yaml_name)
        dir_path = "/home/ilya22/ros2_humble/src/robocross2023/params/" + yaml_name
        with open(dir_path, 'r') as f:
            data = yaml.safe_load(f)
            data[f'{key}'] = f'{value}' 
            print(data)

        with open(dir_path, 'w') as f:
            yaml.dump(data, f) 
        #print('done!')



def main(args=None):

    rclpy.init(args=args)
    get_param_from_ui = GetParamFromUI()
    rclpy.spin(get_param_from_ui)
    get_param_from_ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
    -[90, 60, 30, 15, 0] -[1.0, 0.8, 0.5, 0.4, 0.0]
"""
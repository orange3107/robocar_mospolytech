#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import serial
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Twist
import ros2_numpy
import struct
import ctypes
import sensor_msgs_py.point_cloud2 
import math
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Int8MultiArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from random import randint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
from time import sleep
from std_msgs.msg import UInt32MultiArray, Int32,Float32, Bool, String
from ament_index_python.packages import get_package_share_directory
import os
import heapq
import time



class MissionControl(Node):

  def __init__(self):

    super().__init__('missiob_control_node')

    self.subscription_butt = self.create_subscription(
      String, 
      '/buttons_status', 
      self.buttons_status_callback, 
      10)

    self.subscription_light = self.create_subscription(
      Int32, 
      '/traffic_light_color', 
      self.traffic_light_callback, 
      10)

    
    self.subscription_twist = self.create_subscription(
      Twist, 
      '/pure_cmd_vel', 
      self.pure_twist_callback, 
      10)

    self.pub_twist = self.create_publisher(Twist, '/autocar/cmd_vel', 1)

    self.mission_status = "Pause"
    self.light_status = 3 #red
    self.pure_twist = Twist()

    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.main_timer)

  def buttons_status_callback(self, msg):
    if msg.data != "None":
      self.mission_status = msg.data
    
  def traffic_light_callback(self, msg):
    if msg.data == 1 and self.mission_status == "Start": # green
      self.light_status = msg.data

  def pure_twist_callback(self, msg):
      self.pure_twist = msg

  def main_timer(self):


    twist_control = Twist()

    twist_control.linear.x = 0.0
    twist_control.linear.y = 0.0
    twist_control.linear.z = 0.0

    twist_control.angular.x = 0.0
    twist_control.angular.y = 0.0
    twist_control.angular.z = self.pure_twist.angular.z

    if self.mission_status == "Start":
      print(self.mission_status)
      twist_control = self.pure_twist

    self.pub_twist.publish(twist_control)

  

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  missiob_control_node = MissionControl()
  
  # Spin the node so the callback function is called.
  rclpy.spin(missiob_control_node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  missiob_control_node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
  

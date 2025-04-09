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
from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy
import struct
import ctypes
import sensor_msgs_py.point_cloud2 
import math
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Int8MultiArray
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from random import randint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Header
from rclpy.clock import Clock
from std_msgs.msg import UInt32MultiArray, Float32, Bool, String


import csv
import os

class myPoint:
    def __init__(self, X, Y, A):
        self.x = X
        self.y = Y
        self.a = A


class GoGlobalPath(Node):

    def __init__(self):
    
        super().__init__('go_global_path')

        self.odomAuto = myPoint(0.0, 0.0, 0.0)
        self.searchStatus = "None"
        
        self.subscription = self.create_subscription(
        Marker, 
        '/odomAuto', 
        self.odomAuto_collback, 
        10)

        self.subscription_search = self.create_subscription(
        String, 
        '/nav_status', 
        self.status_collback, 
        10)
        
        self.inc = 0

        self.publisher_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
        self.pathArr = np.empty((0,4))
        self.indexPoint = 0

        dir_path = os.path.join(get_package_share_directory('robocross2023'), 'paths', 'path2.csv')

        with open(dir_path, 'r') as file:
            reader = csv.reader(file)
            #print(reader)
            msg = Path()
            msg.header.frame_id = "map"
            for index, line in enumerate(reader):
                if(index > 0):
                    x = float(line[0])
                    y = float(line[1])
                    a = float(line[2])
                    r = float(line[3])
                    self.pathArr = np.append(self.pathArr, [[x, y, a, r]], axis=0)

        self.publish_pose(self.pathArr[0])
        self.publish_pose(self.pathArr[0])
        self.publish_pose(self.pathArr[0])

    def status_collback(self, msg):

            if msg.data != "Go":
                self.searchStatus = msg.data
                print("status", msg.data)


    def timer_callback(self):

        if self.indexPoint == 0:
            self.publish_pose(self.pathArr[self.indexPoint])
            self.indexPoint += 1

        print(self.indexPoint)

        if self.IsPointInCircle(self.pathArr[self.indexPoint][0], self.pathArr[self.indexPoint][1], self.odomAuto.x, self.odomAuto.y, 1) and self.are_angles_equivalent(self.pathArr[self.indexPoint][2], self.odomAuto.a, 0.1):
            self.indexPoint += 1
            self.publish_pose(self.pathArr[self.indexPoint])

    def publish_pose(self, pose):
        print(pose)
        q = self.quaternion_from_euler(0.0, 0.0, pose[2]+math.pi/2)
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        print(msg.pose.orientation.x)

        self.publisher_goal_pose.publish(msg)

    def angle_between_lines(self, p1, p2, p3):
        #print(p1, p2, p3)
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        mult = np.dot(v1, v2)
        vl1 = math.sqrt(v1[0]**2 + v1[1]**2)
        vl2 = math.sqrt(v2[0]**2 + v2[1]**2)
        angle = math.acos(mult/(vl1 * vl2))
        return angle

    def are_angles_equivalent(self, angle1, angle2, tol=1e-9):

      # Приводим углы к диапазону от 0 до 2*pi
      angle1 = angle1 % (2 * math.pi)
      angle2 = angle2 % (2 * math.pi)
    
      # Проверяем, равны ли углы с учетом заданной точности
      return math.isclose(angle1, angle2, abs_tol=tol)
   

    def odomAuto_collback(self, data):
        euler = self.euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)      
        self.odomAuto.a = euler[2]
        self.odomAuto.x = data.pose.position.x
        self.odomAuto.y = data.pose.position.y


    def IsPointInCircle(self, x, y, xc, yc, r):
        return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r
    
  
    def euler_from_quaternion(self, x, y, z, w):
        euler = np.empty((3, ))
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        euler[0] = roll_x
        euler[1] = pitch_y
        euler[2] = yaw_z

        return euler

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q


  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  go_global = GoGlobalPath()
  
  # Spin the node so the callback function is called.
  rclpy.spin(go_global)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  go_global.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
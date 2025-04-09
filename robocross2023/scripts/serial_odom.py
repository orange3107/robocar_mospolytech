#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from serial import Serial
from time import sleep, time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion, Pose, Vector3
from std_msgs.msg import UInt32MultiArray, Float32, Bool, String
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from math import cos, sin, pi
import serial.tools.list_ports
from rclpy.clock import Clock
import os
import sys
import math
import numpy as np
import re
import threading
from sensor_msgs.msg import JointState


class SerialControl(Node):
	
	def __init__(self):
		
		super().__init__('serial_odom')		
		self.tiks = [0,0]
		self.sp = [0,0]
		self.x = 0.0
		self.y = 0.0
		self.a = 0.0
		self.bs = ''
		
		self.publisher_ = self.create_publisher(JointState, 'joint_states', 1)

		self.publisher_vels_ = self.create_publisher(JointState, 'vels', 1)

		self.joint_state = JointState()
		self.joint_state.velocity = [0.0, 0.0]

		self.vels = JointState()
		self.vels.velocity = [0.0, 0.0]
		
		self.previous_cmd_time = Clock().now()
		self.previous_cmd_time = time()
		self.previous_odom_time = time()

		self.theta = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.wz = 0.0
		self.nav_state = 'not' #stay, move, finished
		self.start_msg = False

		self.R = 0.203  # радиус колеса в метрах
		self.D = 1.55   # расстояние между колёсами в метрах
		self.T = 80   # количество тиков за один оборот колеса
		self.delta_t = 0.1  # период опроса в секундах

		connected = False
		while connected == False:
			try:
				self.serOdom = Serial('/dev/ttyACM0', 115200 ,timeout = 0.05)
				connected = True
				sleep(3)
			except Exception as e:
				print(e)
				sleep(0.5)
		self.time = self.get_clock().now()	
		while True: self.read()

	def calculate_wheel_speed(self, ticks, R, T, delta_t):
		sp = (1.96 * ticks) / (T * delta_t)
		return sp 

	def read(self):
		try:

			self.bs = self.serOdom.readline()
			self.bs = self.bs.decode()
			
			if self.bs != '':
				self.bs = self.bs.split(";")
								
				for i in range(len(self.bs)):
					self.tiks[i] = int(self.bs[i])
					#print(tiks)
					rp = (self.tiks[i]*10)/80
				
					#self.sp[i] = rp/(2 * math.pi * 0.2159)
					self.sp[i] = self.calculate_wheel_speed(self.tiks[i], self.R, self.T, self.delta_t)
					#print(self.sp[i])
				
				#Vx_real = (self.sp[0]+self.sp[1])/2
				#Wz_real = (self.sp[1] - self.sp[0])/self.D
				#self.bs = ''


				
				self.vels.velocity = [float(self.sp[0]), float(self.sp[1])]
				#print(self.vels)
				self.publisher_vels_.publish(self.vels)
				#print("pub")

				self.joint_state.velocity = [float(self.tiks[0]), float(self.tiks[1])]
				self.publisher_.publish(self.joint_state)

		except Exception as e:
			None
			print(e)
			

def main(args=None):

    rclpy.init(args=args)
    controller = SerialControl()
    rclpy.spin(controller)
    controller.destroy_node()	
    rclpy.shutdown()
    
    
if __name__ == '__main__':
  main()
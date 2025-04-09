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
		
		
		super().__init__('serial_control')		
		self.tiks = [0,0]
		self.sp = [0,0]
		self.x = 0.0
		self.y = 0.0
		self.a = 0.0
		self.bs = ''

		self.basePos = [0.0,0.0]
		self.quaternion = Quaternion()
		
		self.previous_cmd_time = Clock().now()
		self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)
		self.previous_cmd_time = time()
		self.previous_odom_time = time()

		self.timer_odom = self.create_timer(0.01, self.timer_odom_callback)

		self.theta = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.wz = 0.0
		self.nav_state = 'not' #stay, move, finished
		self.start_msg = False

		self.vel = JointState()
		self.vel.velocity = [0.0, 0.0]

		self.R = 0.203  # радиус колеса в метрах
		self.D = 1.55   # расстояние между колёсами в метрах
		self.T = 80   # количество тиков за один оборот колеса
		self.LW = 1.95  # период опроса в секундах

		self.sub = self.create_subscription(
            JointState, 'joint_states', self.calc_odom, 10)


	def timer_odom_callback(self):

		#print(self.x, self.y, self.a)

		odom_quat = self.quaternion_from_euler(0,0,self.a - math.pi/2)

		self.quaternion.x = odom_quat[0]
		self.quaternion.y = odom_quat[1]
		self.quaternion.z = odom_quat[2]
		self.quaternion.w = odom_quat[3]

		transform_stamped_msg = TransformStamped()
		transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()# - rclpy.time.Duration(seconds=0.1)
		transform_stamped_msg.header.frame_id = 'odom'
		transform_stamped_msg.child_frame_id = 'base_link'
		transform_stamped_msg.transform.translation.x = self.basePos[0]
		transform_stamped_msg.transform.translation.y = self.basePos[1]
		transform_stamped_msg.transform.translation.z = 0.0
		transform_stamped_msg.transform.rotation.x = self.quaternion.x
		transform_stamped_msg.transform.rotation.y = self.quaternion.y
		transform_stamped_msg.transform.rotation.z = self.quaternion.z
		transform_stamped_msg.transform.rotation.w = self.quaternion.w

		odom = Odometry()
		#odom.header.stamp = Clock().now()
		odom.header.frame_id = "odom"
		#odom.pose.pose = Pose(Point(self.x,self.y,0.0), Quaternion(*odom_quat))
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation = self.quaternion
		odom.child_frame_id = "base_link"
		print(self.vel.velocity[1])

		odom.twist.twist.linear.x = self.vel.velocity[0]
		odom.twist.twist.linear.y = 0.0
		odom.twist.twist.linear.z = self.vel.velocity[1]
		odom.twist.covariance[0] = 0.05
		odom.twist.covariance[7] = 0.05
		odom.twist.covariance[35] = 0.01
		self.odom_pub.publish(odom)

		#odom.twist.twist = Twist(Vector3(velocity[0],velocity[1],0.0),Vector3(0,0,velocity[2]))

		self.broadcaster.sendTransform(transform_stamped_msg)

	def calc_odom(self,data):
		self.vel.velocity = data.velocity

		self.x, self.y, self.a = self.update_odometry(data.velocity[0], data.velocity[1], self.LW, self.D, self.T, self.x, self.y, self.a)

		
		point_robot = np.array([1.33, 0])
		self.basePos = self.transform_point_to_global(self.x, self.y, self.a, point_robot)

		
		#print(basePos)

	def update_odometry(self, ticks_left, ticks_right, LW, D, T, x, y, theta):
		d_left = self.calculate_wheel_distance(ticks_left, LW, T)
		d_right = self.calculate_wheel_distance(ticks_right, LW, T)
		
		# Вычисляем изменение позиции и ориентации
		delta_x = (d_left + d_right) / 2 * math.cos(theta)
		delta_y = (d_left + d_right) / 2 * math.sin(theta)
		delta_theta = (d_right - d_left) / D
		
		# Обновляем текущую позицию и ориентацию
		x += delta_x
		y += delta_y
		theta += delta_theta
		
		return x, y, theta

	def calculate_wheel_distance(self, ticks, LW, T):
		return (LW * ticks) / T


	def transform_point_to_global(self, x_robot, y_robot, theta_robot, point_robot):

		# Матрица поворота для преобразования из системы координат робота в глобальную систему координат
		R = np.array([
			[np.cos(theta_robot), -np.sin(theta_robot)],
			[np.sin(theta_robot), np.cos(theta_robot)]
		])
		
		# Преобразование координат точки в глобальную систему координат
		point_global = R @ point_robot + np.array([x_robot, y_robot])
		
		return point_global


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

    rclpy.init(args=args)
    controller = SerialControl()
    rclpy.spin(controller)
    controller.destroy_node()	
    rclpy.shutdown()
    
    
if __name__ == '__main__':
  main()
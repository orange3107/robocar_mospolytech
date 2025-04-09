import array
import sys
import time
import tkinter.filedialog

import threading
import random
import re
import datetime
import os

from PyQt5.QtWidgets import QApplication, QWidget

#from PyQt5.QtGui import QImage, QPixmap
from datetime import datetime

import numpy as np
#import pyqtgraph as pg
#from pyqtgraph import PlotWidget, plot

#import pyqtgraph.examples
import numpy as np
from array import *

from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


import math
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage, JointState
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge
import cv2








class MainWindowLocal(QWidget):
    def __init__(self):
        super(MainWindowLocal, self).__init__()
        uic.loadUi('/home/traiblazer/lab1_ros_workspace/src/mycode_v2/scripts/form/mainform.ui', self)



        # самое важное
        rospy.init_node('main')



        # настройки скопипастеные с телеоп кей. start
        # self.BURGER_MAX_LIN_VEL = 0.22
        # self.BURGER_MAX_ANG_VEL = 2.84
        self.BURGER_MAX_LIN_VEL = 0.11
        self.BURGER_MAX_ANG_VEL = 0.5

        self.WAFFLE_MAX_LIN_VEL = 0.26
        self.WAFFLE_MAX_ANG_VEL = 1.82

        self.LIN_VEL_STEP_SIZE = 0.01
        self.ANG_VEL_STEP_SIZE = 0.1

        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

        self.turtlebot3_model = rospy.get_param("model", "burger")

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        # настройки скопипастеные с телеоп кей. end



        # все подписки. start
        self.sub_image = rospy.Subscriber('/camera/image', Image, self.whenWeGetCameraImage, queue_size=1)
        self.sub_lidar = rospy.Subscriber('/scan', LaserScan, self.getLidarData, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.getOdomertyData, queue_size=1)
        self.sub_encoders = rospy.Subscriber('/joint_states', JointState, self.getEncoderData, queue_size=1)
        # все подписки. end



        # все паблишеры. start
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_coords = rospy.Publisher('main_coords', String, queue_size=1)
        # все паблишеры. end



        # отложенные методы. start

        # отложенные методы. end



        # псевдоглобальные-рабочие переменные. start
        self.cvBridge = CvBridge()

        self.isAutoWork = False
        self.isLineMove = False
        self.allowGetSign = True

        self.oldThereBarrier = -1
        self.trafficLight = -1  # -1 - не обнаружен светофор, 0 - красный, 1 - желтый, 2 - зеленый
        self.sign = -1  # -1 - не обнаружен знак, 0 - знак парковка, 1 - знак стоп
        self.lidar_forward_val = -1
        self.LineMonitorDisplayMode = 3

        self.rightencoder = 0
        self.leftencoder = 0



        self.currX = 121.447 / 2.0 + 121.447 * 1.0 / 8.0
        self.currY = 121.447 - 10
        self.currAngle = 0.0
        self.oldencright = 0
        self.oldencleft = 0
        # псевдоглобальные-рабочие переменные. end



        # события формы. start
        self.btMoveForward.clicked.connect(self.btMoveForward_Click)
        self.btMoveStop.clicked.connect(self.btMoveStop_Click)
        self.btMoveBackward.clicked.connect(self.btMoveBackward_Click)
        self.btRotationLeft.clicked.connect(self.btRotationLeft_Click)
        self.btRotationStop.clicked.connect(self.btRotationStop_Click)
        self.btRotationRight.clicked.connect(self.btRotationRight_Click)
        self.btStartStopMission.clicked.connect(self.btStartStopMission_Click)





        self.btDebug.clicked.connect(self.btDebug_Click)
        self.btDebug_2.clicked.connect(self.btDebug_2_Click)
        self.btDebug_3.clicked.connect(self.btDebug_3_Click)
        self.btDebug_4.clicked.connect(self.btDebug_4_Click)
        # события формы. end



        # начальные настройки. start

        # начальные настройки. end





    # телеуправление. start
    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)
    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input
    def checkLinearLimitVelocity(self, vel):
        if self.turtlebot3_model == "burger":
            vel = self.constrain(vel, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(vel, -self.WAFFLE_MAX_LIN_VEL, self.WAFFLE_MAX_LIN_VEL)
        else:
            vel = self.constrain(vel, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)

        return vel
    def checkAngularLimitVelocity(self, vel):
        if self.turtlebot3_model == "burger":
            vel = self.constrain(vel, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(vel, -self.WAFFLE_MAX_ANG_VEL, self.WAFFLE_MAX_ANG_VEL)
        else:
            vel = self.constrain(vel, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)

        return vel
    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input

        return output
    # телеуправление. end






    # Методы для подписок. start
    def getLidarData(self, data):
        # scan = rospy.wait_for_message('scan', LaserScan)
        scan = data
        scan_filter = []

        samples = len(scan.ranges)  # The number of samples is defined in
        # turtlebot3_<model>.gazebo.xacro file,
        # the default is 360.
        samples_view = 360  # 1 <= samples_view <= samples

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view // 2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view // 2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0


        self.lidar_arr = scan_filter
        self.DrawLidarData(scan_filter)



        # чисто чтобы понять че там впереди робота ---------------------------------
        scan = data
        scan_filter = []

        samples = len(scan.ranges)  # The number of samples is defined in
        # turtlebot3_<model>.gazebo.xacro file,
        # the default is 360.
        samples_view = 1  # 1 <= samples_view <= samples

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view // 2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view // 2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0


        self.lidar_forward_val = min(scan_filter)
        # чисто чтобы понять че там впереди робота ---------------------------------







        # print(scan_filter)
        return scan_filter

    def whenWeGetCameraImage(self, data):
        cv_image_original = self.cvBridge.imgmsg_to_cv2(data, "bgr8")

        h, w, ch = cv_image_original.shape
        img = self.convert_cv_qt(cv_image_original, w, h)
        self.imgFromCamera.setPixmap(img)

        height, width, cha = cv_image_original.shape

    def getStatusOfEndLab(self, data):
        print(data.data)

    def getOdomertyData(self, odom):
        self.lbOdomData.setText("Orientation: w = " + str(odom.pose.pose.orientation.w) +
                                " x = " + str(odom.pose.pose.orientation.x) +
                                " y = " + str(odom.pose.pose.orientation.y) +
                                " z = " + str(odom.pose.pose.orientation.z))
        return

    def getEncoderData(self, enc):
        self.lbEncodeData.setText("RightWheel = " + str(enc.position[0]) + " LeftWheel = " + str(enc.position[1]))
        self.rightencoder = enc.position[0]
        self.leftencoder = enc.position[1]


        deltaright = self.rightencoder - self.oldencright
        deltaleft = self.leftencoder - self.oldencleft
        self.oldencright = self.rightencoder
        self.oldencleft = self.leftencoder

        # print("deltaright=" + str(deltaright) + "   deltaleft="+ str(deltaleft))

        if deltaright > 0 and deltaleft < 0:
            self.currAngle += 90.0 * deltaright / 3.9
        elif deltaright < 0 and deltaleft > 0:
            self.currAngle -= 90.0 * deltaleft / 3.9
        else:
            self.currX += deltaright * math.cos(self.currAngle * (math.pi / 180.0))
            self.currY += -deltaright * math.cos((90.0 - self.currAngle) * (math.pi / 180.0))

        if self.currAngle > 360:
            self.currAngle = 0
        elif self.currAngle < 0:
            self.currAngle = 360


        self.lbCurrXY.setText("X = " + str(self.currX) + "  Y = " + str(self.currY) + "   ANGLE = " + str(self.currAngle))


        # print("X=" + str(self.currX) + " Y=" + str(self.currY) + " ANGLE=" + str(self.currAngle))

        self.update()

        self.pub_coords.publish(str(self.currX) + ";" + str(self.currY) + ";" + str(self.currAngle))
    # Методы для подписок. end






    # методы для рисования/автопроезда и дополнительное... start
    def GoToPoint(self, x0, y0, x1, y1, currAngle):
        # х0, у0 - точка, где сейчас робот
        # х1, у1 - целевая точка, где должен оказаться робот
        # currAngle - текущий угол поворота робота

        # шаг 1. Определяем угол поворота, которые нужен чтобы доехать до точки
        angle = math.atan2(y0 - y1, x0 - x1) / math.pi * 180 - 180
        angle = -angle

        print("\n\nangle = " + str(angle))
        print("currAngle = " + str(currAngle))

        self.isLeftMove = True # нужно двигаться влево...
        neededAngle = angle - currAngle # угол, на который надо бы повернуться...

        if neededAngle >= 180:
            neededAngle = 360 - neededAngle
            self.isLeftMove = False
        elif neededAngle <= -180:
            neededAngle = 360 + neededAngle
            self.isLeftMove = True
        elif neededAngle < 0:
            neededAngle = -neededAngle
            self.isLeftMove = False



        print("\nneededAngle = " + str(neededAngle))
        if self.isLeftMove:
            print("едем влево...")
        else:
            print("едем вправо...")


        self.neededEncodeToTurn = 3.9 / 90 * neededAngle
        if neededAngle < 0.5:
            self.neededEncodeToTurn = 0

        print("\nself.neededEncodeToTurn = " + str(self.neededEncodeToTurn))

        if self.isLeftMove:
            self.oldencforturn = self.rightencoder
        else:
            self.oldencforturn = self.leftencoder

        print("\nself.oldencforturn = " + str(self.oldencforturn))

        # шаг 2. Поворачиваем робота на угол, который посчитали
        self.ableToGo = False
        timerTurn = threading.Timer(0.01, self.TurnBody)
        timerTurn.start()

        # шаг 3. Определяем сколько по энкодеру ехать до точки
        self.alreadyNeedToGo = False
        deltax = x1 - x0
        deltay = y1 - y0
        self.distance = math.sqrt(deltax * deltax + deltay * deltay)

        # шаг 4. Собсна едем по дочки
        self.oldenc = self.rightencoder + self.leftencoder
        self.newenc = self.rightencoder + self.leftencoder

        self.alreadyNeedToGo = True
        if self.ableToGo and self.alreadyNeedToGo:
            timerMove = threading.Timer(0.01, self.MoveForwardBody)
            timerMove.start()

    def MoveForwardBody(self):
        self.newenc = self.rightencoder + self.leftencoder
        if self.newenc - self.oldenc <= self.distance * 2.0:
            self.btMoveForward_Click()

            timerMove = threading.Timer(0.01, self.MoveForwardBody)
            timerMove.start()
        else:
            self.btMoveStop_Click()
            print("доехали наконец")
    def TurnBody(self):
        if self.isLeftMove:
            self.newenc = self.rightencoder
        else:
            self.newenc = self.leftencoder

        # print("self.newenc - self.oldenc  =  " + str(self.newenc - self.oldenc))
        delta = self.newenc - self.oldencforturn
        print("delta = " + str(delta))
        if delta <= self.neededEncodeToTurn:
            if self.isLeftMove:
                self.btRotationLeft_Click()
            else:
                self.btRotationRight_Click()



            timerMove = threading.Timer(0.01, self.TurnBody)
            timerMove.start()
        else:
            self.btRotationStop_Click()
            print("\n\nстоп поворот")

            self.ableToGo = True
            if self.ableToGo and self.alreadyNeedToGo:
                timerMove = threading.Timer(0.01, self.MoveForwardBody)
                timerMove.start()



    def convert_cv_qt(self, cv_img, width, height):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        # p = convert_to_Qt_format.scaled(self.imgFromCamera.width(), self.imgFromCamera.height())
        p = convert_to_Qt_format.scaled(width, height)
        return QtGui.QPixmap.fromImage(p)

    def DrawLidarData(self, arr):
        canvas = QtGui.QPixmap(240, 240)
        painter = QtGui.QPainter(canvas)


        pen = QtGui.QPen()
        pen.setWidth(3)
        pen.setColor(QtGui.QColor("#C0C0C0"))
        painter.setPen(pen)

        brush = QtGui.QBrush()
        brush.setColor(QtGui.QColor("#C0C0C0"))
        brush.setStyle(Qt.SolidPattern)
        painter.setBrush(brush)

        painter.drawRect(0, 0, 240, 240)

        pen = QtGui.QPen()
        pen.setWidth(1)
        pen.setColor(QtGui.QColor("#000000"))
        painter.setPen(pen)

        brush = QtGui.QBrush()
        brush.setColor(QtGui.QColor("#000000"))
        brush.setStyle(Qt.SolidPattern)
        painter.setBrush(brush)



        r = 1
        strt_point = QtCore.QPoint(int(240 / 2), int(240 / 2))

        # painter.Antialiasing = True
        painter.translate(strt_point)
        # painter.scale(1, -1)
        # painter.rotate(90)
        for i in range(360):
            length = arr[i] * 134
            x = int(length * math.cos(math.radians(i)))
            y = int(length * math.sin(math.radians(i)))
            point = QtCore.QPoint(x, y)
            # painter.drawPoint(point)
            painter.drawEllipse(x - r, y - r, r * 2, r * 2)







        painter.end()

        self.imgLidarView.setPixmap(canvas)

    def pub_velocity(self, x, z, time):
        pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        vel = Twist()
        for i in range(0, int(time * 10)):
            vel.linear.x = x
            vel.angular.z = z
            pub_vel.publish(vel)
            rospy.sleep(0.1)
    def do_parking(self):
        # self.pub_velocity(0, 0, 0.5)
        # self.pub_velocity(0, -0.4, 4)
        # self.pub_velocity(0, 0, 0.3)

        self.pub_velocity(0.13, 0, 2.4)
        self.pub_velocity(0, 0, 0.5)
        self.pub_velocity(0, 0.4, 4)

        self.pub_velocity(0, 0, 3)

        self.pub_velocity(0, -0.4, 4)
        self.pub_velocity(0, 0, 0.3)
        self.pub_velocity(-0.13, 0, 2.4)
        self.pub_velocity(0, 0, 0.3)
        self.pub_velocity(0, 0.4, 4)
        self.pub_velocity(0, 0, 0.5)

    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def Send(self):
        twist = Twist()

        self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (self.LIN_VEL_STEP_SIZE / 2.0))
        twist.linear.x = self.control_linear_vel;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0

        self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (self.ANG_VEL_STEP_SIZE / 2.0))
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = self.control_angular_vel

        self.pub.publish(twist)
    # методы для визуализации/автопроезда и дополнительное... end






    # методы отложенных методов. start
    def turnOnFindBarrier(self):
        self.pub_barrier_detect.publish(str(True))
    def turnOffLineTrackingAfterBarrier(self):
        # print("едем по стенке")
        self.isLineMove = False
        self.pub_line_tracking.publish(str(self.isLineMove))

        self.pub_mov_by_wall.publish(str(True))

    def timerWhenWeGettingSign_ToAllowGetSign_Tick(self):
        self.allowGetSign = True

        self.timerWhenWeGettingSign_ToAllowGetSign = threading.Timer(3.0, self.timerWhenWeGettingSign_ToAllowGetSign_Tick)
    def timerWhenParkingSign_Tick(self):
        # print("опа, событие для проезда кое-куда")
        self.isLineMove = False
        self.pub_line_tracking.publish(str(self.isLineMove))

        self.pub_velocity(0, 0, 0.5)
        self.pub_velocity(0, -0.4, 3.35)
        self.pub_velocity(0, 0, 0.3)
        # print(self.lidar_forward_val)
        if self.lidar_forward_val < 1.0:
            self.pub_velocity(0, 0.4, 3.35)
            self.pub_velocity(0, 0, 0.3)

            self.isLineMove = True
            self.pub_line_tracking.publish(str(self.isLineMove))
            self.timerWhenParkingSign = threading.Timer(4.5, self.timerWhenParkingSign_Tick)
            self.timerWhenParkingSign.start()
        else:
            self.do_parking()

            self.isLineMove = True
            self.pub_line_tracking.publish(str(self.isLineMove))

            self.timerWhenParkingSign = threading.Timer(7.0, self.timerWhenParkingSign_Tick)
    # методы отложенных методов. end






    # методы для формы. start
    def btMoveForward_Click(self):
        for i in range(50):
            self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel + self.LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            # self.textEdit.setText(self.vels(self.target_linear_vel, self.target_angular_vel))
            self.Send()
    def btMoveStop_Click(self):
        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0
        # print(self.vels(self.target_linear_vel, self.target_angular_vel))
        self.Send()
    def btMoveBackward_Click(self):
        for i in range(50):
            self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel - self.LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            # self.textEdit.setText(self.vels(self.target_linear_vel, self.target_angular_vel))
            self.Send()

    def btRotationLeft_Click(self):
        for i in range(50):
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + self.ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            # self.textEdit.setText(self.vels(self.target_linear_vel, self.target_angular_vel))
            self.Send()
    def btRotationStop_Click(self):
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0
        # self.textEdit.setText(self.vels(self.target_linear_vel, self.target_angular_vel))
        self.Send()
    def btRotationRight_Click(self):
        for i in range(50):
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - self.ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            # self.textEdit.setText(self.vels(self.target_linear_vel, self.target_angular_vel))
            self.Send()
    def btStartStopMission_Click(self):
        self.isAutoWork = not self.isAutoWork

        if self.isAutoWork:
            self.btStartStopMission.setText("Остановить выполнение миссии")

            self.isLineMove = self.isAutoWork
            self.pub_line_tracking.publish(str(self.isLineMove))

            self.gbTeleop.setEnabled(False)
        else:
            for i in range(50):
                self.target_linear_vel = 0
                self.target_angular_vel = 0
                self.Send()

            self.isLineMove = self.isAutoWork
            self.pub_line_tracking.publish(str(self.isLineMove))

            self.btStartStopMission.setText("Начать выполнение миссии")

            self.gbTeleop.setEnabled(True)

    def cbLineMonitorDisplayMode_IndChanged(self):
        self.LineMonitorDisplayMode = self.cbLineMonitorDisplayMode.currentIndex()

        self.pub_disp_mode_line_track.publish(str(self.LineMonitorDisplayMode))

    def chbNeedPerspectiveTransform_StateChanged(self):
        rab = False
        if self.chbNeedPerspectiveTransform.checkState() == 2:
            rab = True
        self.pub_need_persp_transf_line_track.publish(str(rab))






    def btDebug_Click(self):
        self.pub_velocity(0, 0, 0.5)
        self.pub_velocity(0, 0.4, 4)
        self.pub_velocity(0, 0, 0.3)
    def btDebug_2_Click(self):
        self.pub_velocity(0, 0, 1)
        self.pub_velocity(-0.13, 0, 1.5)
        self.pub_velocity(0, 0, 1)
    def btDebug_3_Click(self):
        self.pub_velocity(0, 0, 0.5)
        self.pub_velocity(0.13, 0, 1)
        self.pub_velocity(0, 0, 0.5)
    def btDebug_4_Click(self):
        self.GoToPoint(self.currX, self.currY, self.currX + 121.447/4.0, self.currY, self.currAngle)
        # self.GoToPoint(
        #     float(self.textEdit.toPlainText()),
        #     float(self.textEdit_2.toPlainText()),
        #     float(self.textEdit_3.toPlainText()),
        #     float(self.textEdit_4.toPlainText()),
        #     self.currAngle
        # )
    # методы для формы. end








if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow1 = MainWindowLocal()
    mainWindow1.show()
    sys.exit(app.exec_())
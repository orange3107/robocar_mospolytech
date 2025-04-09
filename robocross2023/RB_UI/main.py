import array
import sys
import time
import tkinter.filedialog

import threading
import random
import re
import datetime
import os

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

from PyQt5 import uic, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QFileDialog
from PyQt5.QtCore import Qt, QTimer, QPoint

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

from sensor_msgs.msg import Image # Image is the message type

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from visualization_msgs.msg import Marker
import numpy as np
import math
from std_msgs.msg import Float32MultiArray, Int32, String


class MouseScroll(QtCore.QObject):
    
    mouseScroll = QtCore.pyqtSignal(QtCore.QPoint)


    def __init__(self, widget):
        super().__init__(widget)
        self._widget = widget
        self.widget.setMouseTracking(True)
        self.widget.installEventFilter(self)

    @property
    def widget(self):
        return self._widget

    def eventFilter(self, o, e):
        sc = False
        if o is self.widget and e.type() == QtCore.QEvent.Wheel:

            self.mouseScroll.emit(e.angleDelta())

        return super().eventFilter(o, e)


class MainWindowLocal(QWidget):
    def __init__(self, parent=None):

        super(MainWindowLocal, self).__init__(parent)
        rclpy.init(args=None)
        super(MainWindowLocal, self).__init__()
        uic.loadUi('mainform.ui', self)

        #self.frontImg = QtGui.QPixmap("front.png")
        self.imgCarFront.setStyleSheet("background-image : url(front.png)")

        self.imgCarBottom.setStyleSheet("background-image : url(bottom.png)")

        #переменные для топиков
        self.rosId = '26'
        self.topicMap = '/image_map'
        self.topicPose = '/odomAuto'
        self.topicGPath = '/globalPath'
        self.topicLPath = '/localPath'

        #массив с углами
        self.steer_angle_parameter = [0.0, 15.0, 30.0, 60.0, 90.0, 0.0, 0.2, 0.5, 0.8, 1.0] 

        #Кнопки
        self.setCarParam.clicked.connect(self.setCarParam_Click)
        self.setRosBt.clicked.connect(self.setRosBt_Click)
        self.sendMissionBt.clicked.connect(self.sendMissionBt_Click)
        self.resetBtn.clicked.connect(self.resetBtn_Click)
        self.stopBtn.clicked.connect(self.stopBtn_Click)
        self.startBtn.clicked.connect(self.startBtn_Click)
        self.pauseBtn.clicked.connect(self.pauseBtn_Click)

        #параметры для машины
        self.lCar = 4.0
        self.wCar = 1.7
        self.wheelBase = 2.5
        self.distLidar = 2.0
        self.wheeltrack = 1.2
        self.heightLidar = 1.8

        #Имена точек и карты
        self.pointsName = "path.csv"
        self.mapName = "my_map.pgm"

        #Текст Боксы
        self.rosIdTb.setPlainText(self.rosId)
        self.topicMapTb.setPlainText(self.topicMap)
        self.topicPoseTb.setPlainText(self.topicPose)
        self.topicGPathTb.setPlainText(self.topicGPath)
        self.topicLPathTb.setPlainText(self.topicLPath)

        self.angle0Tb.setPlainText(str(self.steer_angle_parameter[0]))
        self.angle1Tb.setPlainText(str(self.steer_angle_parameter[1]))
        self.angle2Tb.setPlainText(str(self.steer_angle_parameter[2]))
        self.angle3Tb.setPlainText(str(self.steer_angle_parameter[3]))
        self.angle4Tb.setPlainText(str(self.steer_angle_parameter[4]))

        self.value0Tb.setPlainText(str(self.steer_angle_parameter[5]))
        self.value1Tb.setPlainText(str(self.steer_angle_parameter[6]))
        self.value2Tb.setPlainText(str(self.steer_angle_parameter[7]))
        self.value3Tb.setPlainText(str(self.steer_angle_parameter[8]))
        self.value4Tb.setPlainText(str(self.steer_angle_parameter[9]))

        self.heightLidarTextBox.setPlainText(str(self.heightLidar))
        self.distLidarTextBox.setPlainText(str(self.distLidar))
        self.wheelBaseTextBox.setPlainText(str(self.wheelBase))
        self.lenCarTextBox.setPlainText(str(self.lCar))
        self.wheeltrackTextBox.setPlainText(str(self.wheeltrack))
        self.widthCarTextBox.setPlainText(str(self.wCar))

        self.namePointsTb.setPlainText(self.pointsName)
        self.pathMapTb.setPlainText(self.mapName)


        #self.setCarParam.clicked.connect(self.setCarParam_Click)

        #переменная для пути
        #self.controlPoints = pd.read_csv("/home/ilya22/ros2_humble/src/robocross2023/paths/global_path.csv")

        #переменная для карты
        self.map = None

        self.update_ros()
        self.scale = 1



        tracker = MouseScroll(self.imgMap)
        tracker.mouseScroll.connect(self.on_positionChanged)
        

    def connect_ros(self):
        #self.node = Node('Qt_view_node')
        #self.node.destroy_node()
        self.node = Node('Qt_view_node')
        #подписки
        self.node.sub_map = self.node.create_subscription(Image,self.topicMap, self.getMap, 1)
        self.subscription = self.node.create_subscription(Marker, self.topicPose, self.odomAuto_collback, 10)

        #паблишеры
        self.car_parameter_publisher = self.node.create_publisher(Float32MultiArray, '/car_parameter',10)
        self.state_button_publisher = self.node.create_publisher(String, '/buttons_status',10)

        self.steer_angle_parameter_publisher = self.node.create_publisher(Float32MultiArray, '/steer_angle_parameter',10)

        self.points_name_publisher = self.node.create_publisher(String, '/points_name',10)
        self.map_name_publisher = self.node.create_publisher(String, '/map_name',10)

        

    def on_positionChanged(self, pos):
        if pos.y() > 0:
            self.scale += 0.05
        else:
            self.scale -= 0.05


    def getMap(self, data):

        bridge = CvBridge()
        cv_image_original = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.drawAuto(cv_image_original)
        #Разные отрисовки будут
        cv_image_original = cv2.flip(cv_image_original, 0) 

        h, w = cv_image_original.shape

        img = self.convert_cv_qt(cv_image_original, w, h)
        self.imgMap.setPixmap(img)

    def drawAuto(self, img):
        img = cv2.circle(img,(int(self.odomX), int(self.odomY)), 5, (255,255,255), -1)

        car = self.getLocalCar((self.odomX, self.odomY), self.eulerOdom, self.lCar, self.wCar, self.wheelBase)

        img = cv2.line(img,(int(car[0][0]),int(car[0][1])),(int(car[1][0]),int(car[1][1])),(255,255,255),3)
        img = cv2.line(img,(int(car[1][0]),int(car[1][1])),(int(car[2][0]),int(car[2][1])),(255,255,255),3)
        img = cv2.line(img,(int(car[2][0]),int(car[2][1])),(int(car[3][0]),int(car[3][1])),(255,255,255),3)
        img = cv2.line(img,(int(car[3][0]),int(car[3][1])),(int(car[0][0]),int(car[0][1])),(255,255,255),3)

    def getLocalCar(self, p, angle, width, height, wBase):

      inc = (wBase/2)*20
      whCar = (height*20, width*20)


      car = [[p[0] - whCar[0]/2, p[1] + whCar[1]/2 + inc],
             [p[0] + whCar[0]/2, p[1] + whCar[1]/2 + inc],
             [p[0] + whCar[0]/2, p[1] - whCar[1]/2 + inc], 
             [p[0] - whCar[0]/2, p[1] - whCar[1]/2 + inc]]

      for i in range(len(car)):
         car[i][0] -= p[0]
         car[i][1] -= p[1]
         x = car[i][0]
         y = car[i][1]
         car[i][0] = x*math.cos(angle) - y*math.sin(angle) + p[0]
         car[i][1] = x*math.sin(angle) + y*math.cos(angle) + p[1]
      return car


    def odomAuto_collback(self, data):
      euler = self.euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)      
      self.eulerOdom = euler[2]
      self.odomX = data.pose.position.x * 20
      self.odomY = data.pose.position.y * 20
      #print(self.odomX, self.odomY)

    def update_ros(self):
            # create timer
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.timer_float_update)
            self.timer.start(1)  

    def convert_cv_qt(self, cv_img, width, height):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        #p = convert_to_Qt_format.scaled(self.imgFromCamera.width(), self.imgFromCamera.height())
        convert_to_Qt_format = convert_to_Qt_format.scaled(int(width * self.scale), int(height*self.scale))
        return QtGui.QPixmap.fromImage(convert_to_Qt_format)

    def timer_float_update(self):
        try:
            rclpy.spin_once(self.node)

        except:
            None

        #self.update_float_data_label()
        #self.show()
        #self.timer.start(10)

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

    #Методы кнопок
    def setCarParam_Click(self):
        self.lCar = float(self.lenCarTextBox.toPlainText())
        self.wCar = float(self.widthCarTextBox.toPlainText())
        self.wheelBase = float(self.wheelBaseTextBox.toPlainText())

        self.heightLidar = float(self.heightLidarTextBox.toPlainText())
        self.wheeltrack = float(self.wheeltrackTextBox.toPlainText())
        self.distLidar = float(self.distLidarTextBox.toPlainText())

        msg = Float32MultiArray() 
        msg.data = [self.lCar, self.wCar, self.wheelBase, self.wheeltrack, self.heightLidar, self.distLidar]
        self.car_parameter_publisher.publish(msg)

        self.steer_angle_parameter[0] = float(self.angle0Tb.toPlainText())
        self.steer_angle_parameter[1] = float(self.angle1Tb.toPlainText())
        self.steer_angle_parameter[2] = float(self.angle2Tb.toPlainText())
        self.steer_angle_parameter[3] = float(self.angle3Tb.toPlainText())
        self.steer_angle_parameter[4] = float(self.angle4Tb.toPlainText())

        self.steer_angle_parameter[5] = float(self.value0Tb.toPlainText())
        self.steer_angle_parameter[6] = float(self.value1Tb.toPlainText())
        self.steer_angle_parameter[7] = float(self.value2Tb.toPlainText())
        self.steer_angle_parameter[8] = float(self.value3Tb.toPlainText())
        self.steer_angle_parameter[9] = float(self.value4Tb.toPlainText())

        msg = Float32MultiArray() 
        msg.data = self.steer_angle_parameter
        self.steer_angle_parameter_publisher.publish(msg)

    def setRosBt_Click(self):

        self.rosId = "export ROS_DOMAIN_ID=" + (self.rosIdTb.toPlainText())
        self.topicMap = self.topicMapTb.toPlainText()
        self.topicPose = self.topicPoseTb.toPlainText()
        self.topicGPath = self.topicGPathTb.toPlainText()
        self.topicLPath = self.topicLPathTb.toPlainText()
        os.system(self.rosId)

        self.connect_ros()

    def sendMissionBt_Click(self):

        self.pointsName = self.namePointsTb.toPlainText()
        self.mapName = self.pathMapTb.toPlainText()

        msg = String()
        msg.data = self.pointsName
        self.points_name_publisher.publish(msg)

        msg.data = self.mapName
        self.map_name_publisher.publish(msg)

    def stopBtn_Click(self):
        msg = String()
        msg.data = "Stop"
        self.state_button_publisher.publish(msg)
    
    def startBtn_Click(self):
        msg = String()
        msg.data = "Start"
        self.state_button_publisher.publish(msg)

    def pauseBtn_Click(self):
        msg = String()
        msg.data = "Pause"
        self.state_button_publisher.publish(msg)

    def resetBtn_Click(self):
        msg = String()
        msg.data = "Reset"
        self.state_button_publisher.publish(msg)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow1 = MainWindowLocal()
    mainWindow1.show()
    sys.exit(app.exec_())
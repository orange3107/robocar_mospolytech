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
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from random import randint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
from time import sleep
from std_msgs.msg import UInt32MultiArray, Float32, Bool, String
from ament_index_python.packages import get_package_share_directory
import os
import heapq
import time

class Points:
    def __init__(self, X, Y, A):
        self.x = X
        self.y = Y
        self.children = 0
        self.parent = -1
        self.lenghtPath = 0
        self.a = A

class Cost:
    reverse = 10
    directionChange = 150
    steerAngle = 5
    steerAngleChange = 10
    hybridCost = 150


class Nodes:
   def __init__(self, X, Y, A, L, C, P, I):
        self.x = X
        self.y = Y
        self.l = L
        self.c = C
        self.i = I
        self.children = 0
        self.parent = P
        self.lenghtPath = 0
        self.disc = [X, Y]
        self.a = A
        self.direction = 1
        self.steeringAngle = 0 

   def __lt__(self, other):
      return self.c < other.c

   

class HybridAStar:
   def __init__(self, start, goals, iter, gridArr, stepSize, neighborhood, eulerAuto, posX, posY, waypoint_publisher, RCarF, RCarB, origin_map):
        self.start = Nodes(start[0], start[1], start[2], 0, 0, -1, 0)
        #print("ddd")
        #self.start1 = Nodes(start[0], start[1], start[2], 0, 0)
        self.start.parent = -1
        self.goals = goals
        self.mearestNode = None
        self.iterations = min(iter, 10000)
        self.eulerAuto = eulerAuto
        self.gridArray = gridArr
        self.stp = stepSize
        self.nhood = neighborhood
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
        self.count = -1
        self.posX = posX
        self.posY = posY
        self.RCarF = RCarF
        self.RCarB = RCarB
        self.waypoint_publisher = waypoint_publisher
        self.height = 0
        self.width = 0
        self.origin_map = origin_map


   def planning(self):

      if len(self.goals) < 1:
         return None, None  

      size = 20

      width= len(self.gridArray)
      height= len(self.gridArray[0])

      # for i in self.gridArray:
      #    print(i)

      #print(height, width)
      #sleep(5)


      self.height = height
      self.width = width

      state_node_map = np.empty([int(height/size), int(width/size)], dtype = Nodes)
      img = np.zeros((height,width,3), np.uint8)
      minPanh = math.inf
      stSize = 3
      vec1X = math.cos(math.pi/2)
      vec1Y = math.sin(math.pi/2)
      correctGoal = None
      img = cv2.rectangle(img, (0,height), (width, 0), (255,255,255), 4)
      
      

      car = self.getLocalCar((self.start.x, self.start.y), self.start.a, width, height, img)
      img = cv2.circle(img,(int(self.start.x), int(self.start.y)), 2, (255,255,255), -1)

      for j in range(len(car)): 
         None
         img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)   

                   
      #print(self.goals)

      car = self.getLocalCar((self.goals[len(self.goals)-1][0], self.goals[len(self.goals)-1][1]), self.goals[0][2], width, height, img)

      img = cv2.circle(img, (int(self.goals[len(self.goals)-1][0]), int(self.goals[len(self.goals)-1][1])), 10, (0,255,255), -1)

      rotatedX = 20*math.cos(self.goals[len(self.goals)-1][2] + math.pi/2) + self.goals[len(self.goals)-1][0]
      rotatedY = 20*math.sin(self.goals[len(self.goals)-1][2] + math.pi/2) + self.goals[len(self.goals)-1][1]

      img = cv2.line(img,(int(self.goals[len(self.goals)-1][0]),int(self.goals[len(self.goals)-1][1])),(int(rotatedX),int(rotatedY)),(255,255,0),3)


      rotatedX = 20*math.cos(self.start.a + math.pi/2) + self.start.x
      rotatedY = 20*math.sin(self.start.a + math.pi/2) + self.start.y

      img = cv2.line(img,(int(self.start.x),int(self.start.y)),(int(rotatedX),int(rotatedY)),(255,255,0),3)

      for j in range(len(car)): 
                     None        
                     img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)

      openArr = []
      closeArr = []
      heapq.heappush(openArr, self.start)

      state_node_map[int(self.start.disc[1]/size)][int(self.start.disc[0]/size)] = self.start
      
      cv2.imshow("Image", img)
      cv2.imshow("Image", img)
      self.countGlobal = self.start.i

      while True:

         if len(openArr) < 1:
            return np.empty((0,4)), None

         currNode = openArr[0]

         heapq.heappop(openArr) 

         img = cv2.circle(img,(int(currNode.x), int(currNode.y)), 6, (0,255,255), -1)

         node = self.getLocalNodes(currNode, currNode.i, img)

         cv2.imshow("Image", img)
         cv2.waitKey(1)

         closeArr = np.append(closeArr, currNode)      
         
         for n in node:
            if int(n.disc[0]/size) < int(width /size) and int(n.disc[0]/size) > 0:
               if int(n.disc[1]/size) < int(height/size) and  int(n.disc[1]/size) > 0:
                  if state_node_map[int(n.disc[1]/size)][int(n.disc[0]/size)] == None:

                     nodes_map =[]
                     np.append(nodes_map, n)

                     state_node_map[int(n.disc[1]/size)][int(n.disc[0]/size)] = nodes_map

                     img = cv2.circle(img,(int(n.disc[0]), int(n.disc[1])), 2, (0,255, 0), -1)

                     heapq.heappush(openArr, n)
                     state_node_map[int(n.disc[1]/size)][int(n.disc[0]/size)] = n
                     #print(n.disc[0], n.disc[1])

                  else:

                     if state_node_map[int(n.disc[1]/size)][int(n.disc[0]/size)].c < n.c:
                        state_node_map[int(n.disc[1]/size)][int(n.disc[0]/size)] = n
                        heapq.heappush(openArr, n)



                     
                     
                     
                     # nodes_map = state_node_map[int(n.disc[1]/size)][int(n.disc[0]/size)]

                     # if type(nodes_map) != Node:
                     #    for nc in nodes_map:
                           # if self.are_angles_equivalent(nc.a, n.a, 0.24) and n.c > nc.c:
                           #    nc = n
                           #    state_node_map[int(n.disc[1]/size)][int(n.disc[0]/size)] = nodes_map
                           #    heapq.heappush(openArr, n)
                           #    break
                              





         cv2.imshow("Image", img)
         cv2.imshow("Image", img)
               
         #sleep(1)

         for i in range(len(self.goals)):

            img = cv2.circle(img,(int(self.goals[i][0]), int(self.goals[i][1])), 6, (0,0,255), -1)

            goal = Points(self.goals[i][0], self.goals[i][1], self.goals[i][2])
            check = self.checkR((closeArr[len(closeArr)-1].x, closeArr[len(closeArr)-1].y, closeArr[len(closeArr)-1].a), (goal.x, goal.y, goal.a))
                     
            if check[0]:
               c = True
               if self.are_angles_equivalent(check[4] + closeArr[len(closeArr)-1].a, self.goals[i][2], 0.24):

                  arc = self.getArc(check[2], check[1], 20, check[3], closeArr[len(closeArr)-1], img, check[1])
                  ch = self.checkArc(arc, check, closeArr[len(closeArr)-1], img, width, height)
                           
                  if ch :
                     self.count = closeArr[len(closeArr)-1].i
                     correctGoal = self.goals[i]
                     break
         if len(closeArr) > 1000:
            return None, None

         if np.any(correctGoal) != None :
            break


      msg = Path()
      msg.header.frame_id = "map"
      localPath = np.empty((0,4))
      while(True):
         if(self.count > -1):

            node = None
            for j in range(len(closeArr)):
               if closeArr[j].i == self.count:
                  self.count = closeArr[j].parent
                  node = closeArr[j]
                  break

            x = node.x
            y = node.y
            a = node.a
            r = -1

            font = cv2.FONT_HERSHEY_SIMPLEX 
            color = (255, 255, 255) 
            fontScale = 1
            thickness = 2
            img = cv2.circle(img,(int(x), int(y)), 6, (0,0,255), -1)

            cv2.imshow("Image", img)
            cv2.waitKey(1)

            if(a != -999):
               a1 = self.eulerAuto + a
            else:
               a1 = a

            localPath = np.append(localPath, [[x, y, a, r]], axis=0)
         else:
            break
          

      localPath = np.flip(localPath, axis = 0)
      lenPath = len(localPath)

      i = 0
      for j in range(len(localPath)):
         img = cv2.circle(img,(int(localPath[j][0]), int(localPath[j][1])), 6, (255,255,255), -1)
         cv2.imshow("Image", img)
         cv2.waitKey(1)

      while i < lenPath - 1:
         
         p = Points(localPath[i][0], localPath[i][1], localPath[i][2])

         car = self.getLocalCar((p.x,p.y), p.a, width, height, img)
         for j in range(len(car)):
            img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)

         cv2.imshow("Image", img)
         cv2.waitKey(1)
         infoPoint = self.checkR([p.x, p.y, p.a], [localPath[i+1][0], localPath[i+1][1], localPath[i+1][2]])

         localArc = self.getArc(infoPoint[2], infoPoint[1], stSize, infoPoint[3], p, img, infoPoint[1])

         localPath = np.insert(localPath, i+1, localArc, axis = 0)
         lenPath = len(localPath)

         if len(np.shape(localArc)) > 1:
            i += len(localArc) + 1

         else: 
            i += 2

      lenPath = len(localPath)
      
      if lenPath > 0:
         p = Points(localPath[lenPath - 1][0], localPath[lenPath - 1][1], localPath[lenPath - 1][2])
         infoPoint = self.checkR([p.x, p.y, p.a], [correctGoal[0], correctGoal[1], correctGoal[2]])
         #print(infoPoint[0])
         localArc = self.getArc(infoPoint[2], infoPoint[1], stSize, infoPoint[3], p, img, infoPoint[1])
         localPath = np.append(localPath, localArc, axis=0)
         lenPath = len(localPath)
         
         #print(localPath)
         
         for i in range(lenPath):

            localPath[i][0] = localPath[i][0]/20
            localPath[i][1] = localPath[i][1]/20

            x = localPath[i][0]
            y = localPath[i][1]
            a = localPath[i][2]
            r = localPath[i][3]

            localPath[i][0] = x + self.origin_map[0]/20
            localPath[i][1] = y + self.origin_map[1]/20
            localPath[i][2] = a
            localPath[i][3] = r
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            msg.poses.append(pose)

         self.waypoint_publisher.publish(msg)
      
      return localPath, correctGoal

   def IsPointInCircle(self, x, y, xc, yc, r):
      return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r


   def getLocalNodes(self, car, parNum,img):


      x = None
      y = None
      t = None
   
      L = [self.stp]
      nodes = np.empty((0,1))

      params = [[math.inf, self.stp, 1, 0],
               [-self.RCarF, self.stp, 1, 50],
               [self.RCarF, self.stp, 1, 50],

            
               [math.inf, -self.stp, -1, 50],
               [self.RCarB, -self.stp, -1, 50],
               [-self.RCarB, -self.stp, -1, 50],    

               [-self.RCarF * 2, self.stp, 1, 50],
               [self.RCarF * 2, self.stp, 1, 50],

            
               [self.RCarB * 2, -self.stp, -1, 50],
               [-self.RCarB * 2, -self.stp, -1, 50],

               [self.RCarB * 4, -self.stp, -1, 50],
               [-self.RCarB * 4, -self.stp, -1, 50],           

               ]

      for i in range(len(params)):

            a = 2 * math.asin(params[i][1] / (2 * params[i][0]))

            if abs(a) > 0.05:

               cx = car.x - math.sin(car.a + math.pi/2)*params[i][0]
               cy = car.y + math.cos(car.a + math.pi/2)*params[i][0]

               #print(cx, cy)

               x = cx + math.sin(car.a + math.pi/2 + a)*params[i][0]
               y = cy - math.cos(car.a + math.pi/2 + a)*params[i][0]
               #print(x, y)

               t = (car.a + a)
               L = abs(math.pi*params[i][0]*(math.degrees(a)))/180# + car.l

            else:
               x = car.x + params[i][1] * math.cos(car.a + math.pi/2)
               y = car.y + params[i][1] * math.sin(car.a + math.pi/2)
               t = (car.a)
               L = abs(params[i][1])# + car.l

            carLocal = self.getLocalCar((x,y), t, self.width, self.height, img)

            ch = self.checkTouch(carLocal, self.width, self.height)

            if ch:

               dist = self.distance(x,y,self.goals[len(self.goals)-1][0], self.goals[len(self.goals)-1][1])

               costNode = self.getCostV2(car, params[i], self.stp, a) + dist

               self.countGlobal += + i + 1
               

               node = Nodes(x,y,t,L,costNode, parNum, self.countGlobal)

               node.steeringAngle = a
               node.direction = params[i][2]

               node.parent = parNum
               nodes = np.append(nodes, node)
               font = cv2.FONT_HERSHEY_SIMPLEX 
               color = (255, 255, 255) 
               fontScale = 0.3
               thickness = 1

               #img = cv2.putText(img, str(int(costNode)), (int(x),int(y)), font,  fontScale, color, thickness, cv2.LINE_AA) 
 
               # rotatedX = 20*math.cos(t+ math.pi/2) + x
               # rotatedY = 20*math.sin(t+ math.pi/2) + y
               img = cv2.circle(img,(int(x),int(y)), 2, (255,255,255), -1)
               # img = cv2.line(img,(int(x),int(y)),(int(rotatedX),int(rotatedY)),(255,0,0),1)

      return nodes
         


   def are_angles_equivalent(self, angle1, angle2, tol=1e-9):

      # Приводим углы к диапазону от 0 до 2*pi
      angle1 = angle1 % (2 * math.pi)
      angle2 = angle2 % (2 * math.pi)
    
      # Проверяем, равны ли углы с учетом заданной точности
      return math.isclose(angle1, angle2, abs_tol=tol)

   

   def getCostV2(self, parent, motionCommand, simulationLength, t):

      cost = 0

      if motionCommand[2] == 1:
         if t != parent.steeringAngle:
            if t == 0:
               cost = simulationLength * Cost.steerAngleChange
            else:
               cost = simulationLength * Cost.steerAngleChange * Cost.steerAngle

         else:

            if t == 0:
               cost = simulationLength
            else:
               cost = simulationLength * Cost.steerAngle
      else:

         if t != parent.steeringAngle:
            if t == 0:
               cost = simulationLength * Cost.steerAngleChange * Cost.reverse
            else:
               cost = simulationLength * Cost.steerAngleChange * Cost.steerAngle * Cost.reverse

         else:

            if t == 0:
               cost = simulationLength * Cost.reverse
            else:
               cost = simulationLength * Cost.steerAngle * Cost.reverse

      cost += parent.c

      
      return cost


   def getInterCarV2(self, car, st = 10):
      interCar = np.empty((0,2))
      i = 0
      
      while i < len(car):

         if i < len(car)-1:

            line = self.interpolate_line(car[i][0], car[i][1], car[i+1][0], car[i+1][1], st)
            interCar = np.append(interCar, line, axis = 0) 

         else:

            line = self.interpolate_line(car[i][0], car[i][1], car[0][0], car[0][1], st)
            interCar = np.append(interCar, line, axis = 0) 


            line = self.interpolate_line(car[0][0], car[0][1], car[2][0], car[2][1], st)
            interCar = np.append(interCar, line, axis = 0) 

            line = self.interpolate_line(car[1][0], car[1][1], car[3][0], car[3][1], st)
            interCar = np.append(interCar, line, axis = 0) 


         i += 1
      return interCar 


   def interpolate_line(self, x1, y1, x2, y2, step):
    import numpy as np
    
    # Вычисляем длину прямой
    distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Вычисляем количество шагов
    num_steps = int(distance / step)
    
    # Создаем массив точек
    x_points = np.linspace(x1, x2, num_steps + 1)
    y_points = np.linspace(y1, y2, num_steps + 1)
    
    # Объединяем координаты в массив точек
    points = np.array(list(zip(x_points, y_points)))
    
    return points

   def getLocalCar(self, p, angle, width, height, img):

      h = 2.4
      w = 4.9
      wb = 0.98

      inc = wb*20

      whCar = (h*20,w*20)


      car = [[p[0] - whCar[0]/2, p[1] + whCar[1]/2 + inc],
             [p[0] + whCar[0]/2, p[1] + whCar[1]/2 + inc],
             [p[0] + whCar[0]/2, p[1] - whCar[1]/2 + inc], 
             [p[0] - whCar[0]/2, p[1] - whCar[1]/2 + inc]]
      
      #seg = 8
      #car = self.getInterCar(car, seg)

      for i in range(len(car)):
         car[i][0] -= p[0]
         car[i][1] -= p[1]
         x = car[i][0]
         y = car[i][1]
         car[i][0] = x*math.cos(angle) - y*math.sin(angle) + p[0]
         car[i][1] = x*math.sin(angle) + y*math.cos(angle) + p[1]
      
      car = self.getInterCarV2(car)
      
      return car
   
   def checkArc(self, arc, check, Waypoints, img, width, height):
      ch = False
      if len(np.shape(arc)) > 1:

         ch = True
         #arc = self.getArc(check[2], check[1], 2, check[3], Waypoints, img, check[1])
         for j in range(len(arc)):
            car = self.getLocalCar((arc[j][0], arc[j][1]), arc[j][2], width, height, img)

            for i in range(len(car)):
               None     
               #img = cv2.circle(img,(int(car[i][0]), int(car[i][1])), 2, (0,0,255), -1)

            chCar = self.checkTouch(car, width, height)          
            if chCar == False:
               ch = False
               break
      #print(print(int(arc[j][0]), int(arc[j][1])), ch)      
      return ch
   

   def checkTouch(self, car, width, height):
      ch = True

      for j in range(len(car)):

         #print(int(car[j][0]),int(car[j][1]))          
         if int(car[j][0]) < width and int(car[j][1]) < height and int(car[j][0]) > 0 and int(car[j][1]) > 0 and np.any(self.gridArray[int(car[j][0])][int(car[j][1])]) != 0 :
            ch = False
            break
               
      return ch


   def getArc(self, mode, maxc, step_size, angleAcr, sP, img, r):

      d = True
      if mode == '-R' or mode == '-L':
         d = False

      font = cv2.FONT_HERSHEY_SIMPLEX 
      color = (255, 255, 255) 
      fontScale = 0.5
      thickness = 2

      L = abs(math.pi*maxc*(math.degrees(angleAcr)))/180
      #print("L", L)
      point_num = int(L / step_size)+1 # колличество точек для отрисовки локального пути
      if(point_num < 2):
         return [sP.x, sP.y, sP.a, -1]
      #print("arc",mode, angleAcr, sP.a)
      #print(point_num)

      localArc = np.empty((0,4))

      for i in range(point_num):
         localArc = np.append(localArc, [[0,0,0,-1]], axis=0)
      
      localArc[0][2] = sP.a
      an = 0
      if mode == 'R' or mode == '-R':
        an = sP.a
        #print("a", sP.a)
        localArc[0][0] -= maxc
        r = -r
        #localArc[point_num].a = math.pi/2 + angleAcr

      if mode == 'L' or mode == '-L':
        an = 0
        localArc[0][0] += maxc

      angleStep = angleAcr/point_num
      angleStep1 = angleAcr/(point_num)
      
      if d:
         ll = angleStep
      else:
         ll = angleStep

      x = localArc[0][0]
      y = localArc[0][1]
      #img = cv2.circle(img,(int(x+sP.x),int(y + sP.y)), int(maxc), (0,255,0), 2)
      parent = len(self.Waypoints)
      a = 0
      
      for i in range(1, point_num):
        
         if mode == 'R' or mode == '-R':
            ll -= angleStep
            localArc[i][0] = x*math.cos(ll)+y*math.sin(ll)+ maxc
            localArc[i][1] = y*math.cos(ll)-x*math.sin(ll)
            
            a = an
            if d:   
               an -= abs(angleStep1)
            else:
               an += abs(angleStep1)

         elif mode == 'L' or mode == '-L':
            #print("an", an)
            ll -= angleStep
            localArc[i][0] = x*math.cos(ll)+y*math.sin(ll)- maxc
            localArc[i][1] = y*math.cos(ll)-x*math.sin(ll)
            


            a = an + sP.a 
            if d:   
               an += abs(angleStep1)
            else:
               an -= abs(angleStep1)
            #print("a", a)

         #localArc[i].parent = parent
         parent += 1
         #len_buff += len_step
         #len_arr[i] = len_buff
         #print(a)
         localArc[i][2] = a
         
         #print(a)

      localArc[point_num-1][2] = angleAcr + sP.a
      localArc[0][1] = sP.y
      localArc[0][0] = sP.x
      #print("new", localArc[point_num-1][2])

      angle = sP.a 
      for i in range(1, point_num):
         x = localArc[i][0]
         y = localArc[i][1]

         localArc[i][0] = x*math.cos(angle)-y*math.sin(angle) + sP.x
         localArc[i][1] = y*math.cos(angle)+x*math.sin(angle) + sP.y
         localArc[i][3] = r

      return localArc       

   def checkR(self,pointP, pointC):
      mode = ''
      boolCheck = False
      #print(math.degrees(pointP[2]))
      vec1X = math.cos(pointP[2] + math.pi/2)
      vec1Y = math.sin(pointP[2] + math.pi/2)

      vec2X = pointC[0]- pointP[0]
      vec2Y = pointC[1]- pointP[1]

      anglePC = self.angleV1V2(vec1X, vec1Y, vec2X, vec2Y)
      
      if anglePC <= 0:
         mode = 'R'
      else:
         mode = 'L'

      dist = self.distance(pointP[0], pointP[1], pointC[0], pointC[1])
      r = dist/(2*math.cos((math.pi/2)-abs(anglePC)))  

      if abs(anglePC) > math.pi/2:
         mode = '-' + mode   
         angleAcr = 2*anglePC
         if(angleAcr > 0):
            angleAcr = -(2*math.pi - angleAcr)

         else:       
            angleAcr = -(-2*math.pi - angleAcr)
         if(r >= self.RCarB):
               boolCheck = True
            
   
      else:
         angleAcr = 2*anglePC
         if(r >= self.RCarF):
               boolCheck = True
         

      angleNew = angleAcr
      L = abs(math.pi*r*math.degrees(angleAcr))/180
      info = [boolCheck, r, mode, angleAcr, angleNew, L]
      #print(boolCheck, pointC)
      return info
       

   def distance(self, x1, y1, x2, y2):
      c = math.sqrt((x2-x1)**2 + (y2-y1)**2)
      return c

   def angleV1V2(self, vec1X, vec1Y, vec2X, vec2Y):
      c = math.atan2(vec1X*vec2Y - vec1Y*vec2X, vec1X*vec2X + vec1Y*vec2Y)
      return c      
    
   def IsPointInCircle(self, x, y, xc, yc, r):
      return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r
    
       
    
class CreatePath(Node):

   def __init__(self):
    
      super().__init__('local_map')

      #print("hello")

      #timer_period = 0.5  # seconds
      #self.timer = self.create_timer(timer_period, self.timer_callback)

      self.br = CvBridge()
      width = 1
      height = 1
      self.image_map = np.zeros((width,height,3), np.uint8)
      self.goalX = 0
      self.goalY = 0
      self.c = 0
      self.carX = 0
      self.carY = 0
      self.eulerAuto = 0

      self.corrStartI = None
      self.corrEndI = None

      self.posX = 0
      self.posY = 0
      self.angleAutoPoint = 0
      self.current_map = 0
      self.height = 0
      self.width = 0
      self.mapArr = np.empty((1,1))

      self.localMapArr = np.empty((1,1))
      self.cPoint = 0

      self.odomX = 0
      self.odomY = 0
      self.eulerOdom = 0

      self.origin_map = [0.0, 0.0]

      self.pose_init = False

      self.timer = self.create_timer(0.2, self.timer_path_callback)
      self.timer1 = self.create_timer(0.1, self.timer_path_out)

      
      self.subscription = self.create_subscription(
         PoseStamped, 
         '/goal_pose', 
         self.goal_point, 
         10)

      self.subscription_done_pose = self.create_subscription(
         PoseStamped, 
         '/initial_pose', 
         self.initial_pose_callback, 
         10)

      
      
      self.subscription = self.create_subscription(
         Marker, 
         '/poseAuto', 
         self.poseAuto_collback, 
         10)
      
      self.subscription = self.create_subscription(
         Marker, 
         '/odomAuto', 
         self.odomAuto_collback, 
         10)

      self.subscription = self.create_subscription(
         OccupancyGrid,
         '/local_map1', 
         self.local_map_collback, 
         1)
      self.nav_status_pub = self.create_publisher(String, '/nav_status', 10)
      #print(self.current_map)

      self.start = np.empty((0,3))
      self.waypoint_publisher = self.create_publisher(
               Path, '/pathWay', 1)
      
      self.global_path_publisher = self.create_publisher(Path, '/global_path', 10)
      
      self.pathArr = np.empty((0,4))

      dir_path = os.path.join(get_package_share_directory('robocross2023'), 'paths', 'path1.csv')

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
               #self.pathArr = np.append(self.pathArr, [[x, y, a, r]], axis=0)
               #print(x, y, a)

      #self.mapArr = cv2.imread('/home/ilya22/ros2_humble/src/robocross2023/maps/local_map.pgm')
      #cv2.imshow("local_map", self.mapArr)
      #cv2.waitKey(1)

   def initial_pose_callback(self, msg):
      self.pose_init = True

   def angle_between_lines(self, p1, p2, p3):
    #print(p1, p2, p3)
    v1 = (p1[0] - p2[0], p1[1] - p2[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    mult = np.dot(v1, v2)
    vl1 = math.sqrt(v1[0]**2 + v1[1]**2)
    vl2 = math.sqrt(v2[0]**2 + v2[1]**2)
    angle = math.acos(mult/(vl1 * vl2))
    #angle = math.degrees(angle)
    #angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
    #angle = abs(180-math.degrees(angle))
    # если нужен острый угол
    #return min(180 - angle, angle)
    return angle
   
   def goal_point(self, data):
      if self.pose_init:
         print("goal")
         euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
         euler = euler[2] - math.pi/2
         gX = data.pose.position.x * 20 - self.origin_map[0]
         gY = data.pose.position.y * 20 - self.origin_map[1]

         #print("angles:", euler[2] - math.pi/2, self.eulerAuto)
         startPoint = [self.odomX * 20 - self.origin_map[0], self.odomY * 20 - self.origin_map[1], self.eulerOdom]

         imageMapArr = self.mapArr
         imageMapArr = np.rot90(imageMapArr)
         imageMapArr = np.flip(imageMapArr, axis = 0)
         print("GO_GOAL")
         msg = String()
         msg.data = "search"
         self.nav_status_pub.publish(msg)
         hybA = HybridAStar((startPoint), [[gX, gY, euler]], 35, imageMapArr, 30, 100, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher,  100, 115, self.origin_map)
         path, correctGoal = hybA.planning()

         #print(np.any(correctGoal))
         if np.any(correctGoal) != None:
            self.pathArr = path
            msg = String()
            msg.data = "done"
            self.nav_status_pub.publish(msg)

            dir_path = os.path.join(get_package_share_directory('robocross2023'), 'paths', 'global_path.csv')

            #dir_path = "/home/polytech/ros2_humble/src/robocross2023/paths/path1.csv"
            print(dir_path)

            with open(dir_path, 'w', newline='') as file:
               writer = csv.writer(file)
               writer.writerow(["X", "Y", "A", "R"])

            for i in range(len(path)):
               with open(dir_path, 'a', newline='') as file:
                  writer = csv.writer(file)
                  writer.writerow([path[i][0], path[i][1], path[i][2], path[i][3]])
         #self.eulerAuto = euler[2]
         #print(self.eulerAuto)
         #self.posX = data.pose.position.x
         #self.posY = data.pose.position.y
         None

   def timer_path_out(self):

      print("out")

      try:
      
         #if self.IsPointInCircle(self.odomX, self.odomY, self.pathArr[0][0], self.pathArr[0][1], 0.2):
            #self.pathArr = np.delete(self.pathArr, 0, axis = 0)
         
         lenArr = len(self.pathArr)
         msg = Path()
         msg.header.frame_id = "map"


         pose = PoseStamped()
         pose.pose.position.x = self.odomX
         pose.pose.position.y = self.odomY
         pose.pose.position.z = 0.0

         pose.pose.orientation.x = self.eulerOdom
         pose.pose.orientation.y = 0.0
         pose.pose.orientation.z = 0.0
         pose.pose.orientation.w = 0.0
         #msg.poses.append(pose)

         for i in range(len(self.pathArr)):

            x = self.pathArr[i][0]
            y = self.pathArr[i][1]
            r = self.pathArr[i][3]
               #print(r)

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = r
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            msg.poses.append(pose)

         #print("out")
         self.global_path_publisher.publish(msg)
      
      except:
         print()
    
   def dist(self, x1, y1, x2, y2):
     dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
     return dist

     
   def timer_path_callback(self):

      msg = String()
      msg.data = "Go"
      self.nav_status_pub.publish(msg)
      
      imageMapArr = self.mapArr
      imageMapArr = np.rot90(imageMapArr)
      imageMapArr = np.flip(imageMapArr, axis = 0)

      prMar = imageMapArr

      self.localMapArr = imageMapArr


      #self.mapArr = imageMapArr
      width = len(self.mapArr)
      height = len(self.mapArr[0])
      c = 0
      posCarX = -int(20*self.carX)
      posCarY = width+int(20*self.carY)  
      self.cPoint = 0
      #print(self.posX,self.posY, posCarX, posCarY, "ff")
      img = np.zeros((width,height,3), np.uint8)

      includePoints = np.empty((0,4))

      for i in range(len(self.pathArr)-1):

         px = self.pathArr[i][0] * 20 - self.origin_map[0]
         py = self.pathArr[i][1] * 20 - self.origin_map[1]
         pa = self.pathArr[i][2]

         includePoints = np.append(includePoints, [[px, py, pa, int(i)]], axis=0)

         #if self.IsPointInCircle(px, py, self.pathArr[0][0]*20, self.pathArr[0][1]*20, 200):
            #includePoints = np.append(includePoints, [[px, py, pa, int(i)]], axis=0)
            #print(i)
         #else:
            #break
            
      startPoint = (0,0,0)
      p = self.global_point_in_local_map(self.posX,self.posY, posCarX, posCarY, [self.odomX, self.odomY, self.eulerOdom])
      startPoint = [self.odomX * 20 - self.origin_map[0], self.odomY * 20 - self.origin_map[1], self.eulerOdom]
      self.start = startPoint
      endPoints = np.empty((0,4))
      start = False
      end = False
      backPoint = None
      seg = 2
      segi = 0
      flag = False
      s = False
      e = False
      out = False
      
      for i in range(len(includePoints)-1):
         
         p1 = includePoints[i]
         p2 = includePoints[i + 1]
         #print(i)
         
         #print(p1, " ", p2)
         segment = seg
         #print(p1, p2)


         while(self.distance(p1, p2) > segment):
            pr = self.steerPoint(p1, p2, segment)

            
            if pr[0] > width-1 or pr[1] > width-1: 
               break
               
            segment += seg
               
               #print(imageMapArr[pr[1], pr[0]])
            if self.IsPointInCircle(pr[0], pr[1], startPoint[0], startPoint[1], 500):

               car = self.getLocalCar((pr[0], pr[1]), p1[2],width, height, img)

               if (self.checkTouch(car, width, height) == False):
                  if s == False and e == False:
                     s = True

               else:
                  if s == True and e == False:
                     #print("e")
                     e = True   

               if s and e:
                  endPoints = np.append(endPoints, [[p2[0], p2[1], p2[2], p2[3]]], axis = 0) 
                  #print(endPoints)
            
            
            
            else:
               out = True
               #print("gg")
               #if s and e:
               #    print(s, e)
               #break
         
         if out:
            break
            #print(s, e)

            #    #prMar = cv2.circle(img,(int(pr[0]), int(pr[1])), 10, (0,0,255), -1)

            #    ch = self.checkTouch(car, width, height)
            #    if(ch == False):
            #       #print("in", start, end)
            #       if start and end:
            #          break
            #          #start = False
            #          #end = False
            #       #print(backPoint) 
            #       #print("препядствие: ", pr)
            #       #print(imageMapArr[pr[1], pr[0]])
            #       if start == False:
            #          self.corrStartI = includePoints[i][3]
            #          #print("нашли старт: ", backPoint)
            #          #print('i = ', i)
            #          start = True
                           
            #    else:
            #       if end == False and start == True:
            #          self.corrEndI = includePoints[i][3]
            #          endPoints = includePoints[int(self.corrEndI)+1: int(len(includePoints))]
            #          end = True
            #       elif end == False and start == False:
            #          backPoint = pr 

            # else:
            #    if start and end != True:
            #       ch = self.checkTouch(car, width, height)
            #       if ch == False:
            #          endPoints = includePoints[int(self.corrEndI)+1: int(len(includePoints))]


            #print(pr)

      #print("HGFCVGBHNJMK")
      if(s and e and startPoint[0] != 0 and startPoint[1] != 0):
         msg = String()
         msg.data = "search"
         self.nav_status_pub.publish(msg)
             
         #msg = String()
         #msg.data = "Stop"
         #self.nav_status_pub.publish(msg)
             
         img = cv2.flip(img, 0)
         cv2.imshow("Image1", img)
         cv2.waitKey(1)
             
         start = False
         end = False
         if np.any(startPoint) != None and self.pose_init:
            print("GO")
                      
            rrtStar = HybridAStar((startPoint), endPoints, 35, imageMapArr, 30, 100, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher, 100, 115, self.origin_map)
            #print("GO")
            path, correctGoal = rrtStar.planning()

            #print("goal", correctGoal)
            x = startPoint[0]
            y = startPoint[1]
            #print(np.any(correctGoal))
            if np.any(correctGoal) != None:
               self.pathArr = np.delete(self.pathArr, np.s_[0:int(correctGoal[3]+2)], axis = 0)
               self.pathArr = np.insert(self.pathArr, 0, path, axis = 0)
               msg = String()
               msg.data = "done"
               self.nav_status_pub.publish(msg)

               dir_path = os.path.join(get_package_share_directory('robocross2023'), 'paths', 'global_path.csv')

               with open(dir_path, 'w', newline='') as file:
                  writer = csv.writer(file)
                  writer.writerow(["X", "Y", "A", "R"])

               for i in range(len(path)):
                  with open(dir_path, 'a', newline='') as file:
                     writer = csv.writer(file)
                     writer.writerow([path[i][0], path[i][1], path[i][2], path[i][3]])

            else:
               msg = String()
               msg.data = "fail"
               self.nav_status_pub.publish(msg)

      length = 20*math.sqrt((self.goalX - self.posX)**2 + (self.goalY - self.posY)**2)
      posGaolX = -int(length*math.sin(self.angleAutoPoint)) + posCarX
      posGaolY = int(length*math.cos(self.angleAutoPoint)) + posCarY
      
      ##rrtStar = RrtStar((posCarX, posCarY), (posGaolX, posGaolY), 1000, imageMapArr, 10, 35, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher)
      ##rrtStar.planning()

   def global_point_in_local_map(self, posX, posY, carXLocal, carYLocal, oldPose):
      #print("auto", math.degrees(self.eulerAuto))  
      length = 20*math.sqrt((oldPose[0] - self.posX)**2 + (oldPose[1] - self.posY)**2)
      VecGoalX = oldPose[0] - posX
      VecGoalY = oldPose[1] - posY

      VecAutoX = math.cos(self.eulerAuto + math.pi/2) #вектор авто
      VecAutoY = math.sin(self.eulerAuto + math.pi/2)


      engleGoaltoAuto = math.atan2(VecAutoX*VecGoalY - VecAutoY*VecGoalX, VecAutoX*VecGoalX + VecAutoY*VecGoalY)

      x = -int(length*math.sin(engleGoaltoAuto)) + carXLocal
      y = (int(length*math.cos(engleGoaltoAuto)) + carYLocal)


      VecPointX = math.cos(oldPose[2] + math.pi/2)
      VecPointY = math.sin(oldPose[2] + math.pi/2)

      engleGoalLocal = math.atan2(VecAutoX*VecPointY - VecAutoY*VecPointX, VecAutoX*VecPointX + VecAutoY*VecPointY)


      VecPointX = math.cos(engleGoalLocal + self.eulerAuto) - self.origin_map[0]
      VecPointY = math.sin(engleGoalLocal + self.eulerAuto) - self.origin_map[1]

      return [x, y, engleGoalLocal]
     

   def getLocalCar(self, p, angle, width, height, img):

      #whCar = (20,60)
      """
      car = [[p[0] - whCar[0]/2 + abs(width/2 - self.start[0]), p[1] + whCar[1]/2 + abs(height/2 - self.start[1])],
             [p[0] + whCar[0]/2 + abs(width/2 - self.start[0]), p[1] + whCar[1]/2 + abs(height/2 - self.start[1])],
             [p[0] + whCar[0]/2 + abs(width/2 - self.start[0]), p[1] - whCar[1]/2 + abs(height/2 - self.start[1])], 
             [p[0] - whCar[0]/2 + abs(width/2 - self.start[0]), p[1] - whCar[1]/2 + abs(height/2 - self.start[1])]]
      """

      
      h = 2.0
      w = 4.5
      wb = 0.95

      inc = wb*20

      whCar = (h*20,w*20)


      car = [[p[0] - whCar[0]/2, p[1] + whCar[1]/2 + inc],
             [p[0] + whCar[0]/2, p[1] + whCar[1]/2 + inc],
             [p[0] + whCar[0]/2, p[1] - whCar[1]/2 + inc], 
             [p[0] - whCar[0]/2, p[1] - whCar[1]/2 + inc]]
      
      seg = 5
      car = self.getInterCar(car, seg)

      for i in range(len(car)):
         car[i][0] -= p[0]
         car[i][1] -= p[1]
         x = car[i][0]
         y = car[i][1]
         car[i][0] = x*math.cos(angle) - y*math.sin(angle) + p[0]
         car[i][1] = x*math.sin(angle) + y*math.cos(angle) + p[1]
      
      for j in range(len(car)):
         None
         img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)

      
      return car
   

   def getInterCar(self, car, seg):
      interCar = np.empty((0,2))
      i = 0
      
      while i < len(car):

         if i < len(car)-1:
            dist = self.distance((car[i][0], car[i][1]), (car[i+1][0], car[i+1][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[i][0], car[i][1], 0), car[i+1], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg
         else:
            dist = self.distance((car[i][0], car[i][1]), (car[0][0], car[0][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[i][0], car[i][1], 0), car[0], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg

            dist = self.distance((car[0][0], car[0][1]), (car[2][0], car[2][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[0][0], car[0][1], 0), car[2], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg

            dist = self.distance((car[1][0], car[1][1]), (car[3][0], car[3][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[1][0], car[1][1], 0), car[3], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg
         i += 1
      return interCar


   
   def checkArc(self, arc, check, Waypoints, img, width, height):
      ch = False
      if len(np.shape(arc)) > 1:
         ch = True
         #arc = self.getArc(check[2], check[1], 2, check[3], Waypoints, img, check[1])
         for j in range(len(arc)):
            car = self.getLocalCar((arc[j][0], arc[j][1]), arc[j][2], width, height, img)
            chCar = self.checkTouch(car, width, height)          
            if chCar == False:
               ch = False
               break
      #print(print(int(arc[j][0]), int(arc[j][1])), ch)      
      return ch
   

   def checkTouch(self, car, height, width):
      ch = True
      for j in range(len(car)):          
         if int(car[j][0]) < width and int(car[j][1]) < height and int(car[j][0]) > 0 and int(car[j][1]) > 0 and np.any(self.localMapArr[int(car[j][0])-1][int(car[j][1])-1]) != 0 :
            ch = False
            break   
      return ch

   def distance(self, p1, p2):
      c = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
      return c

   def steerPoint(self, locStart, locEnd, stp):
       vec = np.array([locEnd[0] - locStart[0], locEnd[1] - locStart[1]])
       if(vec[0]+vec[1] == 0):
          offset = (0,0)
       else:
        offset = stp*self.UnVector(vec)
       point = np.array([int(offset[0] + locStart[0]), int(offset[1] + locStart[1])])
       return point

   def UnVector(self, vec):
       u_hat = vec/np.linalg.norm(vec)
       return u_hat
   
   def poseAuto_collback(self, data):
      euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
      self.eulerAuto = euler[2]
      #print(self.eulerAuto)
      self.posX = data.pose.position.x
      self.posY = data.pose.position.y
      #print("pose", self.posX, self.posY)

   def odomAuto_collback(self, data):
      euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)      
      self.eulerOdom = euler[2]
      #print(self.eulerAuto)
      self.odomX = data.pose.position.x
      self.odomY = data.pose.position.y
      #print("pose", self.posX, self.posY)

   def local_map_collback(self, data):
      self.height = data.info.height
      self.width = data.info.width
      self.mapArr = np.reshape(data.data, (self.height, self.width))
      self.origin_map = [data.info.origin.position.x * 20, data.info.origin.position.y * 20] 

      #print(self.mapArr)
      #self.carX = data.info.origin.position.x
      #self.carY = data.info.origin.position.y
      #print(data.info.origin.position.x, data.info.origin.position.y)

   def IsPointInCircle(self, x, y, xc, yc, r):
      return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r
    
  
def euler_from_quaternion(x, y, z, w):
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

def checkPoint(radius, x, y):
 
    polarradius = math.sqrt(x * x + y * y)
    # Check whether polarradius is less
    # then radius of circle or not and
    # Angle is between startAngle and
    # endAngle or not
    if (polarradius < radius):
        return True
    else:
        return False
 
# Driver code
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  local_rrt = CreatePath()
  
  # Spin the node so the callback function is called.
  rclpy.spin(local_rrt)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  local_rrt.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.task import Future
import time

DIST = 0.4
DIST_L = 0.2
DIST_R = 0.2

class Very_talkative_Node(Node):
    def __init__(self):
        self.f = Future()
        super().__init__("Talkative_Node")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
    def laser_callback(self, msg: LaserScan):
        self.get_logger().info("I am very talkative node!") 
        self.f.set_result(1)

class LaserAngleAndDistance: #класс угол-расстояние
    
    def __init__(self, angle, range):
        self.angle = angle
        self.range = range

    def toStr(self):
        s = "Angle: " + str(self.angle) + " Distance: " + str(self.range)
        return s
    
    def get_angle(self):
        return self.angle
    
    def get_range(self):
        return self.range

class LaserScanSubscriberNode(Node):

    def __init__(self):
        super().__init__("Laser_subscriber")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)



    def laser_callback(self, msg: LaserScan):
        self.angle_min = msg.angle_min #задаем поля для углов
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        
        ranges = msg.ranges #список расстояний
        angle_list = [] #список углов
        temp = self.angle_min
        while temp <= self.angle_max: #заполняем спсиок промежуточных значений углов 
            angle_list.append(temp)
            temp += self.angle_increment

        self.AngRangeList = [] #спсиок объектов класса угол-расстояние
        
        for i in range(0, len(angle_list)): #заполянем
            self.AngRangeList.append(LaserAngleAndDistance(angle_list[i], ranges[i]))
        
        '''
        for i in range(0, len(self.AngRangeList)): # вывод для красоты
            self.get_logger().info(self.AngRangeList[i].toStr())'''
        
        
        #self.get_logger().info(self.AngRangeList[5].toStr()) #вывод для проверки

        #------------------------------------------------- закончили обработку входных данных
        

class Forward(LaserScanSubscriberNode):

    def laser_callback(self, msg: LaserScan):
        global DIST

        self.angle_min = msg.angle_min #задаем поля для углов
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        
        ranges = msg.ranges #список расстояний
        angle_list = [] #список углов
        temp = self.angle_min
        while temp <= self.angle_max: #заполняем спсиок промежуточных значений углов 
            angle_list.append(temp)
            temp += self.angle_increment

        self.AngRangeList = [] #спсиок объектов класса угол-расстояние
   
        for i in range(0, len(angle_list)): #заполянем
            self.AngRangeList.append(LaserAngleAndDistance(angle_list[i], ranges[i]))
        
        #-----------------------

        indpi2 , ind3pi2= 0, 0
        eps1, eps2 = 10000000000, 10000000000
        for i in range(0, len(angle_list)):
            if abs(angle_list[i] - 3.14/2) < eps1:
                eps1 = abs(angle_list[i] - 3.14/2)
                indpi2 = i    
            if abs(angle_list[i] - (6.28 - 3.14/2)) < eps2:
                eps2 = abs(angle_list[i] - (6.28 - 3.14/2))
                ind3pi2 = i   

        msg = Twist()
        if self.AngRangeList[0].get_range() > DIST: #!!!!!!!!!!!
            msg.linear.x = 0.1
            if self.AngRangeList[ind3pi2] > DIST_R and self.AngRangeList[indpi2] > DIST_L:
                msg.angular.z = 0.0
            elif self.AngRangeList[ind3pi2] > DIST_R:
                msg.angular.z = -0.1
            elif self.AngRangeList[indpi2] > DIST_L:
                msg.angular.z = 0.1
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.0




def main(args=None):
    global DIST
    DIST = 0.4
    rclpy.init(args=args)
    #nodet = Very_talkative_Node()
    #rclpy.spin_until_future_complete(nodet, nodet.f)
    node = Forward()
    rclpy.spin(node)
    
    rclpy.shutdown()
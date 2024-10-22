#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.task import Future
import time

DIST = 0.4
DIST_L = 0.3
DIST_R = 0.3
MOVE = 0

TIME_END = -1
TIME_BEGIN = None

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

        self.st_Forward = Future()
        self.st_Correct_turn = Future()
        self.st_Check_node = Future()
        self.st_Left = Future()
        self.st_Right = Future()
        self.st_Turn = Future()

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

        indpi2 , ind3pi2= 0, 0 #!!!!!!!!!!!!
        eps1, eps2 = 10000000000, 10000000000 #погрешность
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
            if self.AngRangeList[ind3pi2].get_range() > DIST_R and self.AngRangeList[indpi2].get_range() > DIST_L: #!!!!!!!!!!!
                msg.angular.z = 0.0               
            elif self.AngRangeList[ind3pi2].get_range() > DIST_R:#!!!!!!!!!!!
                msg.angular.z = -0.075       
            elif self.AngRangeList[indpi2].get_range() > DIST_L:#!!!!!!!!!!!
                msg.angular.z = 0.075       
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.st_Forward.set_result(1)
        self.cmd_vel_pub_.publish(msg)     

           
class Correct_turn(LaserScanSubscriberNode):
      
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
        
        #----------------------- 
        msg = Twist()
        ind1, ind2 = 0 + 5, len(self.AngRangeList)-1 -5 #раствор угла !!!!!!!!!
        d = self.AngRangeList[ind1].get_range() - self.AngRangeList[ind2].get_range()
        self.get_logger().info(str(self.AngRangeList[ind1].get_range())+" "+ str(self.AngRangeList[ind2].get_range()))
        if d > 0.001: #погрешность
            msg.angular.z = -0.2
            self.get_logger().info('1')
        elif d < -0.001: #погрешность
            msg.angular.z = 0.2
            self.get_logger().info('2')
        else:     
            msg.angular.z = 0.0
            self.get_logger().info('ok')
            self.st_Correct_turn.set_result(1)
        self.cmd_vel_pub_.publish(msg)
           
class Forward_1(LaserScanSubscriberNode):

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
        
        #-----------------------

        indpi2 , ind3pi2= 0, 0 #!!!!!!!!!!!!
        eps1, eps2 = 10000000000, 10000000000 #погрешность
        for i in range(0, len(angle_list)):
            if abs(angle_list[i] - 3.14/2) < eps1:
                eps1 = abs(angle_list[i] - 3.14/2)
                indpi2 = i    
            if abs(angle_list[i] - (6.28 - 3.14/2)) < eps2:
                eps2 = abs(angle_list[i] - (6.28 - 3.14/2))
                ind3pi2 = i   
        
        msg = Twist()
        d = self.AngRangeList[ind3pi2].get_range() - self.AngRangeList[indpi2].get_range()
        if self.AngRangeList[0].get_range() > DIST: #!!!!!!!!!!!            
            msg.linear.x = 0.1
            if self.AngRangeList[ind3pi2].get_range() > DIST_R and self.AngRangeList[indpi2].get_range() > DIST_L: #!!!!!!!!!!!
                msg.angular.z = 0.0               
            elif d > 0:#!!!!!!!!!!!
                msg.angular.z = -0.075       
            elif d < 0:#!!!!!!!!!!!
                msg.angular.z = 0.075       
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.st_Forward.set_result(1)
        self.cmd_vel_pub_.publish(msg)  


class CheckNode(LaserScanSubscriberNode):

    def laser_callback(self, msg: LaserScan):
        global DIST, MOVE

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
        indpi2 , ind3pi2= 0, 0 #!!!!!!!!!!!!
        eps1, eps2 = 10000000000, 10000000000 #погрешность
        for i in range(0, len(angle_list)):
            if abs(angle_list[i] - 3.14/2) < eps1:
                eps1 = abs(angle_list[i] - 3.14/2)
                indpi2 = i    
            if abs(angle_list[i] - (6.28 - 3.14/2)) < eps2:
                eps2 = abs(angle_list[i] - (6.28 - 3.14/2))
                ind3pi2 = i   
        
        l_free, r_free = -1, -1
        
        if self.AngRangeList[ind3pi2].get_range() >= DIST: #!!!!!!!!!
            r_free = 1
        else:
            r_free = 0
        
        if self.AngRangeList[indpi2].get_range() >= DIST:#!!!!!!!!!
            l_free = 1
        else:
            l_free = 0

        if l_free == 0 and r_free == 0:
            self.get_logger().info('Разворот')
            MOVE = -1
        elif r_free == 1:
            self.get_logger().info('Направо')
            MOVE = 1
        elif r_free == 0 and l_free == 1:
            self.get_logger().info('Налево')
            MOVE = 2
        
        
        self.st_Check_node.set_result(1)
        

class Left(LaserScanSubscriberNode):
    def laser_callback(self, msg: LaserScan):
        global TIME_BEGIN, TIME_END
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
       
        #------------------------------------------------- закончили обработку входных данных
        self.get_logger().info("Налево")
        
        msg = Twist()
        
        msg.angular.z = 0.35
        self.cmd_vel_pub_.publish(msg)  

        TIME_END = time.time()  
        if TIME_END - TIME_BEGIN >= 4.5:
            self.st_Left.set_result(1)          

class Right(LaserScanSubscriberNode):
    def laser_callback(self, msg: LaserScan):
        global TIME_BEGIN, TIME_END
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
       
        #------------------------------------------------- закончили обработку входных данных
        self.get_logger().info("Направо")
        
        msg = Twist()
        
        msg.angular.z = -0.35
        self.cmd_vel_pub_.publish(msg)  

        TIME_END = time.time()  
        if TIME_END - TIME_BEGIN >= 4.5:
            self.st_Right.set_result(1) 

class Turn(LaserScanSubscriberNode):
    def laser_callback(self, msg: LaserScan):
        global TIME_BEGIN, TIME_END
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
       
        #------------------------------------------------- закончили обработку входных данных
        self.get_logger().info("Разворот")
        
        msg = Twist()
        
        msg.angular.z = -0.35
        self.cmd_vel_pub_.publish(msg)  

        TIME_END = time.time()  
        if TIME_END - TIME_BEGIN >= 9:
            self.st_Turn.set_result(1) 


def main(args=None):
    global DIST, MOVE, TIME_BEGIN
    DIST = 0.4
    rclpy.init(args=args)
    #nodet = Very_talkative_Node()
    #rclpy.spin_until_future_complete(nodet, nodet.f)
    
   
    #node = Forward()
    #rclpy.spin_until_future_complete(node, node.st_Forward)
    
    #node = Correct_turn()
    #rclpy.spin_until_future_complete(node, node.st_Correct_turn)
    
    #node = CheckNode()
    #rclpy.spin_until_future_complete(node, node.st_Check_node)
    while True:
        TIME_BEGIN = time.time()
        if MOVE == 0:
            node = Forward()
            print('ВПЕРЕД')
            rclpy.spin_until_future_complete(node, node.st_Forward)

            node = Correct_turn()
            print('КОРРЕКОТИРОВКА')
            rclpy.spin_until_future_complete(node, node.st_Correct_turn)

            node = CheckNode()
            print('ЕДЕМ...')
            rclpy.spin_until_future_complete(node, node.st_Check_node)
        elif MOVE == -1:
            node = Turn()
            print('РАЗВОРОТ')
            rclpy.spin_until_future_complete(node, node.st_Turn)

            MOVE = 0

        elif MOVE == 1:
            node = Right()
            print('НАПРАВО')
            rclpy.spin_until_future_complete(node, node.st_Right)

            MOVE = 0
        
        elif MOVE == 2:
            node = Left()
            print('НАЛЕВО')
            rclpy.spin_until_future_complete(node, node.st_Left)

            MOVE = 0 

    rclpy.shutdown()
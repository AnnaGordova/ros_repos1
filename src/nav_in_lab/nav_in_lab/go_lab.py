#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.task import Future
import time

FREE_SPACE = -1 # -1 - не определено, 1 - свободно справа, 0 - свободно слева, а справа нет, 2 - только разворот
TIME_END = -1
TIME_BEGIN = None

#Относительно стабильная версия. Без SLAM. Немного косячит калибровка иногда

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
    
    
class Very_talkative_Node(Node):
    def __init__(self):
        self.f = Future()
        super().__init__("Talkative_Node")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
    def laser_callback(self, msg: LaserScan):
        self.get_logger().info("I am very talkative node!") 
        self.f.set_result(1)
        
       
class LaserScanSubscriberNode(Node):
    def __init__(self):
        global TIME_BEGIN
        self.st1 = Future()
        self.st2 = Future()
        self.st3 = Future()
        self.st4_f = Future()
        self.st4_pi2 = Future()
        self.st4_3pi2 = Future()
        self.st4_turnpi = Future()
        TIME_BEGIN = time.time()
        
        super().__init__("Laser_subscriber")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)

    def get_st1(self): return self.st1  
    def get_st2(self): return self.st1  
    def get_st3(self): return self.st1   
    def get_st4_f(self): return self.st4_f   
    def get_st4_pi2(self): return self.st4_pi2    
    def get_st4_3pi2(self): return self.st4_3pi2    
    def get_st4_turnpi(self): return self.st4_turnpi

    

    def set_st4_f(self, f): self.st4_f = f
    def set_st4_pi2(self, f): self.st4_pi2 = f
    def set_st4_3pi2(self, f): self.st4_3pi2 = f
    def set_st4_turnpi(self, f): self.st4_turnpi = f

    TIME_BEGIN = time.time()

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
        
class TurnToRightSide(LaserScanSubscriberNode):
    
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
        minlenind = ranges.index(min(ranges))
        msg = Twist()
        
        d = abs(self.AngRangeList[minlenind].get_angle() - self.AngRangeList[0].get_angle()) 
        if not(0.0 <= d < 0.2):
            msg.angular.z = 0.35
        else:
            msg.angular.z = 0.0
            self.st1.set_result(1)
        self.get_logger().info(str(self.AngRangeList[minlenind].get_angle()) + " " + str(self.AngRangeList[0].get_angle()))
        self.cmd_vel_pub_.publish(msg)    
            
class GoForward(LaserScanSubscriberNode):

  
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
        self.need_indexes = [i for i in range(-3, 3+1)]
        min_acceptable_range = min(self.AngRangeList[i].get_range() for i in self.need_indexes)
        self.get_logger().info(str(min_acceptable_range))
        msg = Twist()
        if min_acceptable_range > 0.7:
            msg.linear.x = 0.3
            self.get_logger().info("Riding...")
        else:
            msg.linear.x = 0.0
            self.st2.set_result(1)
            self.get_logger().info("I am near the wall")
        self.cmd_vel_pub_.publish(msg)

class TurnAlongTheWall(LaserScanSubscriberNode):
    
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
        minlenind = ranges.index(min(ranges))
        msg = Twist()
        
        d = abs(self.AngRangeList[minlenind].get_angle() - self.AngRangeList[0].get_angle()) 
        if not(6.28 - 3.14/2 - 0.2 <= d < 6.28 - 3.14/2 ):
            msg.angular.z = 0.35
        else:
            msg.angular.z = 0.0
            self.st3.set_result(1)
        self.get_logger().info(str(self.AngRangeList[minlenind].get_angle()) + " " + str(self.AngRangeList[0].get_angle()))
        self.cmd_vel_pub_.publish(msg)   
        
class RightHandRule_Forward(LaserScanSubscriberNode):
    
    def laser_callback(self, msg: LaserScan):
        global FREE_SPACE

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
        msg = Twist()

        indpi2 , ind3pi2= 0, 0
        eps1, eps2 = 10000000000, 10000000000
        for i in range(0, len(angle_list)):
            if abs(angle_list[i] - 3.14/2) < eps1:
                eps1 = abs(angle_list[i] - 3.14/2)
                indpi2 = i    
            if abs(angle_list[i] - (6.28 - 3.14/2)) < eps2:
                eps2 = abs(angle_list[i] - (6.28 - 3.14/2))
                ind3pi2 = i          
          #Возможно доработать корректировку на угол
        if self.AngRangeList[0].get_range() > 0.5:
            msg.linear.x = 0.3
            self.get_logger().info("Chu-chu!" + " " + str(self.AngRangeList[0].get_range()))
            if self.AngRangeList[ind3pi2].get_range() >= 0.6 or self.AngRangeList[ind3pi2].get_range() > self.AngRangeList[ind3pi2].get_range():
                msg.angular.z = -0.2
                self.get_logger().info("Calibration 1..." + " " + str(self.AngRangeList[ind3pi2].get_range()))
            elif self.AngRangeList[ind3pi2].get_range() < 0.5:
                msg.angular.z = 0.2
                self.get_logger().info("Calibration 2..." + " " + str(self.AngRangeList[ind3pi2].get_range()))
            elif 0.5 <= self.AngRangeList[ind3pi2].get_range() < 0.6:
                msg.angular.z = 0.00
                self.get_logger().info("Calibration 3..." + " " + str(self.AngRangeList[ind3pi2].get_range()))
        else:
            msg.linear.x = 0.0
            self.get_logger().info("Stop")
            #проверка своодного места
            if self.AngRangeList[ind3pi2].get_range() < self.AngRangeList[indpi2].get_range():
                 #справа свободно
                FREE_SPACE = 1
                self.get_logger().info("справа свободно")
                self.st4_f.set_result(1)
            elif self.AngRangeList[ind3pi2].get_range() > self.AngRangeList[indpi2].get_range():
                 #слева свободно
                FREE_SPACE = 0
                self.get_logger().info("слева свободно")
                self.st4_f.set_result(1)
            elif self.AngRangeList[ind3pi2].get_range() < 0.5 and self.AngRangeList[indpi2].get_range() < 0.5:
                 #только разворот
                FREE_SPACE = 2
                self.get_logger().info("только разворот")
                self.st4_f.set_result(1)
            
            self.st4_f.set_result(1)
        self.cmd_vel_pub_.publish(msg)
        

class RightHandRule_Pi2(LaserScanSubscriberNode):
    def laser_callback(self, msg: LaserScan):
        global FREE_SPACE, TIME_BEGIN, TIME_END
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
        self.get_logger().info("I am pi2 node")
        
        msg = Twist()
        
        msg.angular.z = 0.35
        self.cmd_vel_pub_.publish(msg)  

        TIME_END = time.time()  
        if TIME_END - TIME_BEGIN >= 4.5:
            self.st4_pi2.set_result(1)
        
          



class RightHandRule_3Pi2(LaserScanSubscriberNode):

    def laser_callback(self, msg: LaserScan):
        global FREE_SPACE
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
        self.get_logger().info("I am 3pi2 node")
        msg = Twist()

        msg.angular.z = -0.35
        self.cmd_vel_pub_.publish(msg)   

        TIME_END = time.time()  
        self.get_logger().info(str(TIME_END) + " " + str(TIME_BEGIN) + " " + str(TIME_END - TIME_BEGIN))
        if TIME_END - TIME_BEGIN >= 4.5:
            self.get_logger().info('yes')
            self.st4_3pi2.set_result(1) 

        


class RightHandRule_Turnpi(LaserScanSubscriberNode):
    
    def laser_callback(self, msg: LaserScan):
        global FREE_SPACE
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
        self.get_logger().info("I am turnpi node")
        
        msg = Twist()
        msg.angular.z = 0.35
        self.cmd_vel_pub_.publish(msg)   

        TIME_END = time.time() 
        if TIME_END - TIME_BEGIN >= 9:
            self.st4_turnpi.set_result(1) 


def main(args=None):
    global FREE_SPACE

    rclpy.init(args=args)
    #nodet = Very_talkative_Node()
    #rclpy.spin_until_future_complete(node1, node1.f)
    
    node1 = TurnToRightSide() 
    rclpy.spin_until_future_complete(node1, node1.st1)

    node2 = GoForward()
    rclpy.spin_until_future_complete(node2, node2.st2)

    node3 = TurnAlongTheWall()
    rclpy.spin_until_future_complete(node3, node3.st3)
    
    
    
    while True:
        TIME_BEGIN = time.time()
        #print(TIME_BEGIN)

        if FREE_SPACE == -1:
            
            node4 = RightHandRule_Forward()
            rclpy.spin_until_future_complete(node4, node4.st4_f)
            
        elif FREE_SPACE == 1:

            node4 = RightHandRule_Pi2()
            rclpy.spin_until_future_complete(node4, node4.st4_pi2)
            FREE_SPACE = -1
            
        elif FREE_SPACE == 0:

            node4 = RightHandRule_3Pi2()
            rclpy.spin_until_future_complete(node4, node4.st4_3pi2)
            FREE_SPACE = -1
            
        elif FREE_SPACE == 2:

            node4 = RightHandRule_Turnpi()
            rclpy.spin_until_future_complete(node4, node4.st4_turnpi)
            
            FREE_SPACE = -1

        

    #print(node2.AngRangeList[int(3.14)].get_angle())
    rclpy.shutdown()
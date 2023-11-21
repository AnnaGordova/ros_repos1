#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.task import Future
import time

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
        global TIME_BEGIN, DIST
        DIST = True
        self.st1 = Future()
        self.st2 = Future()
        self.st3 = Future()
        self.st4_f = Future()
        self.st4_pi2 = Future()
        self.st4_3pi2 = Future()
        self.st4_turnpi = Future()
        TIME_BEGIN = time.time()
        super().__init__("Laser_subscriber")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/custom_sending_topic_scan", self.laser_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Start")

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
        
        
        for i in range(0, len(self.AngRangeList)): # вывод для красоты
            self.get_logger().info(self.AngRangeList[i].toStr())
        
        
        #self.get_logger().info(self.AngRangeList[5].toStr()) #вывод для проверки

        #------------------------------------------------- закончили обработку входных данных
        

def main(args=None):
    rclpy.init(args=args)
    #nodet = Very_talkative_Node()
    #rclpy.spin_until_future_complete(nodet, nodet.f)
    while True:
        node_l = LaserScanSubscriberNode()
        rclpy.spin(node_l)

        
    rclpy.shutdown()
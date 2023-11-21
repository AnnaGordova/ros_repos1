#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.task import Future
from std_msgs.msg import Int32MultiArray
import time

class Very_talkative_Node(Node):
    def __init__(self):
        self.f = Future()
        super().__init__("Talkative_Node")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
    def laser_callback(self, msg: LaserScan):
        self.get_logger().info("I am very talkative node!") 
        self.f.set_result(1)

class Rasp_LaserScanSubscriberNode(Node):    
    def __init__(self):
        self.f = Future()
        super().__init__("Laser_subscriber")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.custom_sending_topic_scan_pub_ = self.create_publisher(LaserScan, "/custom_sending_topic_scan", 10)

        '''
        self.pb_ = self.create_publisher(Int32MultiArray, "/custom_sending_topic_cmd_vel", 10)
        '''

    def laser_callback(self, msg: LaserScan):

        self.get_logger().info("Sending...")
        ranges = msg.ranges
        ss = ''
        for x in ranges:
            ss += str(x) + " "
        self.get_logger().info(ss)
        self.custom_sending_topic_scan_pub_.publish(msg)

        '''
        ll = Int32MultiArray()
        ll.data = [1, 1]
        self.pb_.publish(ll)
        self.get_logger().info("Sended...")
        '''

        self.f.set_result(1)

class Rasp_GPIOPublisherNode(Node):
    def __init__(self):
        self.f = Future()
        super().__init__("GPIO_publisher")
        self.gpio_subscriber_ = self.create_subscription(Int32MultiArray, "/custom_sending_topic_cmd_vel", self.gpio_callback, 10)
    
    def gpio_callback(self, msg: Int32MultiArray):
        self.get_logger().info("Received a message")
        #на вход список из двух элементов: скорость на левую и на правую пару соответсвенно
        l, r = msg[0], msg[1]
        #RGPIO...
        self.f.set_result(1)

        
def main(args=None):
    rclpy.init(args=args)
    #nodet = Very_talkative_Node()
    #rclpy.spin_until_future_complete(nodet, nodet.f)
    while True:
        node_l = Rasp_LaserScanSubscriberNode()
        rclpy.spin_until_future_complete(node_l, node_l.f)

        node_g = Rasp_GPIOPublisherNode()
        rclpy.spin_until_future_complete(node_g, node_g.f)
    rclpy.shutdown()
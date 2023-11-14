#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.task import Future
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
        super().__init__("Laser_subscriber")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.custom_sending_topic_scan_pub_ = self.create_publisher(LaserScan, "/custom_sending_topic_scan", 10)

    def laser_callback(self, msg: LaserScan):
        self.get_logger().info("Sending...")
        self.custom_sending_topic_scan_pub_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    #nodet = Very_talkative_Node()
    #rclpy.spin_until_future_complete(nodet, nodet.f)
    node_l = Rasp_LaserScanSubscriberNode()
    rclpy.spin(node_l)
    rclpy.shutdown()
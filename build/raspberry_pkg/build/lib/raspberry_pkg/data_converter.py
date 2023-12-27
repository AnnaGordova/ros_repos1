#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.task import Future
# задача - плавный разгон вперед и назад
#import RPi.GPIO as GPIO
# This module can only be run on a Raspberry Pi!
import time
from std_msgs.msg import Int32MultiArray


class Rasp_LaserScanSubscriberNode(Node):
    def __init__(self):
        self.f = Future()
        super().__init__("Laser_subscriber")
        self.laser_subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.custom_sending_topic_scan_pub_ = self.create_publisher(LaserScan, "/custom_sending_topic_scan", 10)
        self.get_logger().info("Start Rasp_LaserScanSubscriberNode")

    def laser_callback(self, msg: LaserScan):

        self.get_logger().info("Sending...")
        ranges = msg.ranges

        '''
        ss = ''
        for x in ranges:
            ss += str(x) + " "
        self.get_logger().info(ss)
        '''

        self.custom_sending_topic_scan_pub_.publish(msg)
        self.f.set_result(1)


class Rasp_GPIOPublisherNode(Node):
    def __init__(self):
        self.f = Future()
        super().__init__("GPIO_publisher")
        self.gpio_subscriber_ = self.create_subscription(Int32MultiArray, "/custom_sending_topic_cmd_vel",
                                                         self.gpio_callback, 10)
        self.get_logger().info("Start Rasp_GPIOPublisherNode")

    def gpio_callback(self, msg: Int32MultiArray):
        self.get_logger().info("Received a message")
        # на вход список из двух элементов: скорость на левую и на правую пару соответсвенно
        l, r = msg.data[0], msg.data[1] # 1, 0, -1
        # RGPIO...

        if l == 0:
            self.get_logger().info("Слева стоп")
        elif l == 1:
            self.get_logger().info("Слева вперед")
        else:
            self.get_logger().info("Слева назад")

        if r == 0:
            self.get_logger().info("Справа стоп")
        elif r == 1:
            self.get_logger().info("Справа вперед")
        else:
            self.get_logger().info("Справа назад")    

        '''
        #----------------------------------------------------
        in1 = 19  # направление HIGH LOW
        in2 = 26
        in_sh12 = 13
        in3 = 20  # направление HIGH LOW
        in4 = 21
        in_sh34 = 12

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(in3, GPIO.OUT)
        GPIO.setup(in4, GPIO.OUT)

        GPIO.setup(in_sh12, GPIO.OUT)
        GPIO.setup(in_sh34, GPIO.OUT)


        right_PWM = GPIO.PWM(in_sh12, 1000)
        right_PWM.start(0)
        left_PWM = GPIO.PWM(in_sh34, 1000)
        left_PWM.start(0)

        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)

        if l == 1:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        if r == 1:
            GPIO.output(in3, GPIO.HIGH)
            GPIO.output(in4, GPIO.LOW)
        if l == 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        if r == 0:
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.LOW)
        if l == -1:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        if r == -1:
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.HIGH)


        #left_PWM.ChangeDutyCycle(100)
        #right_PWM.ChangeDutyCycle(100)
        '''
        #-------------------------------------------------------
        self.f.set_result(1)

def main(args=None):
    rclpy.init(args=args)
    #nodet = Very_talkative_Node()
    #rclpy.spin_until_future_complete(nodet, nodet.f)
    print('Start')
    while True:
        node_l = Rasp_LaserScanSubscriberNode()
        rclpy.spin(node_l, node_l.f)

        node_r = Rasp_GPIOPublisherNode()
        rclpy.spin(node_r, node_r.f)



    rclpy.shutdown()
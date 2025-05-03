#!/usr/bin/env python3
import rclpy
import socket
import struct
from rclpy.node import Node

class Protocol_stm32_node(Node): 
    def __init__(self):
        super().__init__("Protocol_stm32_node") 
        #self.get_logger().info("Hello from ROS2")
        self.pub_motors = self.create_publisher("....")
        self.pub_telem = self.create_publisher("....")
        self.sub_telem_vel = self.create_publisher("....")
        self.sub_telem_speed = self.create_publisher("....")
        self.sub_telem_gps = self.create_publisher("....")
        '''
        H - uint8_t
        f - float
        h - int16_t
        '''

        self.host = '...'             
        self.port = '...'

        self.format_type = "=H"
        self.format_heartbeat = "=H" # отправить
        self.format_motors = "=Hffff"  #отправить
        self.format_telemetry_request = "=H" #отправить
        self.format_telemetry_answer = "=Hfffffffffhhhhf?fffff" #принять
        self.format_unknown_packet = "=H" # принять
    
    def spin(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((self.host, self.port))
            d = s.recvfrom(1)
            d = struct.unpack(self.format_type)
            if d == "0xAA": #hearbeat?
                d = s.recvfrom(1)
                pass
            elif d == "0xB0": #motors?
                d = s.recvfrom(5)
                pass
            elif d == "0xC0": #telem?
                pass
            elif d == "0xFF": #unknown?
                pass
                
    


def main(args=None): 
    rclpy.init(args=args) 
    node = Protocol_stm32_node()
    rclpy.spin(node) 
    rclpy.shutdown() 

# Запуск с терминала (перед сделать source ~/.bashrc)
# ros2 run lvr_project protocol_stm32
if __name__ == '__main__':
    main()

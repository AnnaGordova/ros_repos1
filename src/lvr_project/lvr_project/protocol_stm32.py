#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node): #создаем собственный класс узла, унаследованный от класса Node
    
    def __init__(self):
        super().__init__("first_node") #обращаемся к конструктору суперкласса. В аргументы передаем имя узла
        self.get_logger().info("Hello from ROS2")

def main(args=None): #по умолчанию тип аргументов None
    rclpy.init(args=args) #инициализируем пакет, чтобы работать с ros библиотекой
    node = MyNode()
    rclpy.spin(node) #не дает узлу отрубиться сразу после вызова, он будет "висеть в ожидании"
    rclpy.shutdown() #деактивируем пакет

if __name__ == '__main__':
    main()
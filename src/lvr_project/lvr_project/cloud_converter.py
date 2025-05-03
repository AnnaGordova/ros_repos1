#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class LaserScanToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_scan_to_point_cloud_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Топик с данными LaserScan
            self.laser_scan_callback,
            10)
        
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.get_logger().info('Конвертер запущен')

    def laser_scan_callback(self, msg):
        # Преобразование LaserScan в PointCloud2
        points = self.laser_scan_to_point_cloud(msg)
        point_cloud_msg = pc2.create_cloud_xyz32(msg.header, points)
        
        # Публикация PointCloud2
        self.publisher.publish(point_cloud_msg)

    def laser_scan_to_point_cloud(self, scan):
        # Преобразование данных LaserScan в массив точек
        points = []
        angle = scan.angle_min
        
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:  # Игнорируем недопустимые расстояния
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y, 0))  # z всегда 0 для 2D
            angle += scan.angle_increment
        return points

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
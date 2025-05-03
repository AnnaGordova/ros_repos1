import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import numpy as np

class IcpNode(Node):
    def __init__(self):
        super().__init__('icp_node')
        self.source_pcd = o3d.geometry.PointCloud()
        self.target_pcd = o3d.geometry.PointCloud()
        self.first_change = False
        # Подписка на топик LaserScan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Замените на ваш топик LaserScan
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # Преобразование LaserScan в облако точек
        self.target_pcd = self.laserscan_to_pointcloud(msg)
        if not(self.first_change):
            print('Облако пустое')
            self.source_pcd = self.target_pcd
            self.first_change = True
        else:
            print('Облако не пустое')
            self.get_logger().info('Received LaserScan data, performing ICP...')
            self.perform_icp()
            self.source_pcd = self.target_pcd
            

    def laserscan_to_pointcloud(self, scan):
        # Преобразование LaserScan в облако точек
        points = []
        angle_increment = scan.angle_increment
        angle = scan.angle_min

        for r in scan.ranges:
            if r < scan.range_max and r > scan.range_min:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y, 0])  # Z = 0 для 2D сканирования
            angle += angle_increment

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def perform_icp(self):
        if self.target_pcd.is_empty():
            self.get_logger().warn('Target point cloud is empty. Please set a target point cloud.')
            return
        
        threshold = 0.02  # Максимальное расстояние для соответствия
        reg_icp = o3d.pipelines.registration.registration_icp(
            self.source_pcd, self.target_pcd, threshold,
            np.eye(4),  # Начальная матрица трансформации
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        #self.get_logger().info(f'ICP converged: {reg_icp.convergence_criteria}')
        #self.get_logger().info(f'Transformation:\n{reg_icp.transformation}')

        translation_vector = reg_icp.transformation[:3, 3]
        self.get_logger().info(f'Translation vector: {translation_vector}')

        # Применение трансформации к исходной облачной точке
        self.source_pcd.transform(reg_icp.transformation)
        o3d.visualization.draw_geometries([self.source_pcd, self.target_pcd])

def main(args=None):
    rclpy.init(args=args)
    icp_node = IcpNode()
    rclpy.spin(icp_node)
    icp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
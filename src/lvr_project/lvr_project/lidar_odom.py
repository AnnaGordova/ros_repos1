import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree

class LaserScanToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_scan_to_point_cloud_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Топик с данными LaserScan
            self.laser_scan_callback,
            10)
        
        self.current_position = np.zeros(3)
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)

        self.previous_points = None

    def laser_scan_callback(self, msg):
        # Преобразование LaserScan в облако точек
        current_points = self.laser_scan_to_point_cloud(msg)

        if self.previous_points is not None:
            # Выполняем ICP, если предыдущее облако точек существует
            R, t = self.icp(self.previous_points, current_points)
            # Применяем трансформацию к текущему облаку точек
            current_points = np.dot(R, current_points.T).T + t
            

            print("Rotation Matrix:\n", R)
            print("Translation Vector:\n", t)
            self.current_position += t

        # Публикуем текущее облако точек
        point_cloud_msg = pc2.create_cloud_xyz32(msg.header, current_points)
        self.publisher.publish(point_cloud_msg)

        # Обновляем предыдущее облако точек
        self.previous_points = current_points

        
        print('Position:', self.current_position)


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
        
        return np.array(points)

    def nearest_neighbor(self, src, dst):
        """Находит ближайшие точки из src в dst."""
        dst_tree = KDTree(dst)
        distances, indices = dst_tree.query(src)
        return indices

    def compute_transformation(self, src, dst):
        """Вычисляет трансформацию между двумя наборами точек."""
        assert len(src) == len(dst)

        # Центрируем точки
        src_center = np.mean(src, axis=0)
        dst_center = np.mean(dst, axis=0)

        src_centered = src - src_center
        dst_centered = dst - dst_center

        # Вычисляем матрицу ковариации
        H = np.dot(src_centered.T, dst_centered)

        # SVD разложение
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # Обработка отражения
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = np.dot(Vt.T, U.T)

        t = dst_center - np.dot(R, src_center)

        return R, t

    def icp(self, src, dst, max_iterations=20, tolerance=1e-5):
        """Основной алгоритм ICP."""
        src = np.array(src)
        dst = np.array(dst)

        prev_error = 0

        for i in range(max_iterations):
            # Находим ближайшие точки
            indices = self.nearest_neighbor(src, dst)
            closest_dst = dst[indices]

            # Вычисляем трансформацию
            R, t = self.compute_transformation(src, closest_dst)

            # Применяем трансформацию
            src = np.dot(R, src.T).T + t

            # Вычисляем ошибку
            mean_error = np.mean(np.linalg.norm(closest_dst - src, axis=1))

            # Проверка сходимости
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        return R, t

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from math import sin, cos

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_calculator')
        self.publisher_ = self.create_publisher(Pose2D, 'odometry', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Начальные значения
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.L = 0.5  # Расстояние между колесами (в метрах)

    def timer_callback(self):
        # Пример скоростей колес
        v_left = 1.0  # Скорость левого колеса (м/с)
        v_right = 1.5  # Скорость правого колеса (м/с)
        dt = 0.1  # Интервал времени (с)

        # Расчет скорости и угловой скорости
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.L

        # Обновление положения
        self.x += v * cos(self.yaw) * dt
        self.y += v * sin(self.yaw) * dt
        self.yaw += omega * dt

        # Публикация одометрии
        odometry_msg = Pose2D()
        odometry_msg.x = self.x
        odometry_msg.y = self.y
        odometry_msg.theta = self.yaw
        self.publisher_.publish(odometry_msg)
        self.get_logger().info(f'Odometry: x={self.x}, y={self.y}, yaw={self.yaw}')

def main(args=None):
    rclpy.init(args=args)
    odometry_calculator = OdometryCalculator()
    rclpy.spin(odometry_calculator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher1 = self.create_publisher(Odometry, 'odometry_topic_1', 10)
        self.publisher2 = self.create_publisher(Odometry, 'odometry_topic_2', 10)
        
        self.timer = self.create_timer(0.1, self.publish_odometry)  # Публикация каждые 0.1 секунды
        
        self.angle = 0.0  # Начальный угол
        self.radius = 1.0  # Радиус круга
        
        # Списки для хранения координат
        self.x_positions_1 = []
        self.y_positions_1 = []
        self.x_positions_2 = []
        self.y_positions_2 = []

        # Запускаем поток для отрисовки траектории
        self.plot_thread = Thread(target=self.plot_trajectory)
        self.plot_thread.start()

    def publish_odometry(self):
        # Вычисляем координаты x и y по кругу для первого топика
        x1 = self.radius * np.cos(self.angle)
        y1 = self.radius * np.sin(self.angle)

        # Добавляем шум к координатам первого топика
        noise_x1 = np.random.normal(0, 0.05)  
        noise_y1 = np.random.normal(0, 0.05)

        noisy_x1 = x1 + noise_x1
        noisy_y1 = y1 + noise_y1

        # Сохраняем координаты для первого топика
        self.x_positions_1.append(noisy_x1)
        self.y_positions_1.append(noisy_y1)

        # Создаем сообщение Odometry для первого топика
        odom_msg_1 = Odometry()
        odom_msg_1.header.stamp = self.get_clock().now().to_msg()
        odom_msg_1.header.frame_id = 'odom'
        odom_msg_1.pose.pose.position.x = noisy_x1
        odom_msg_1.pose.pose.position.y = noisy_y1
        
        # Публикуем первое сообщение
        self.publisher1.publish(odom_msg_1)

        
        # Вычисляем координаты x и y по кругу для второго топика (можно использовать другой радиус или угол)
        x2 = x1 + np.random.normal(0, 0.02)  # Немного изменяем позицию для второго топика
        y2 = y1 + np.random.normal(0, 0.02)

        # Сохраняем координаты для второго топика
        self.x_positions_2.append(x2)
        self.y_positions_2.append(y2)

        # Создаем сообщение Odometry для второго топика
        odom_msg_2 = Odometry()
        odom_msg_2.header.stamp = self.get_clock().now().to_msg()
        odom_msg_2.header.frame_id = 'odom'
        odom_msg_2.pose.pose.position.x = x2
        odom_msg_2.pose.pose.position.y = y2
        
        # Публикуем второе сообщение
        # Публикуем второе сообщение
        self.publisher2.publish(odom_msg_2)

        # Увеличиваем угол для следующей итерации (движение по кругу)
        self.angle += np.pi / 30  

    def plot_trajectory(self):
        plt.figure(figsize=(10, 5))
         
        while rclpy.ok():
            plt.clf()  # Очищаем текущий график

            if len(self.x_positions_1) > 0:
                plt.subplot(121)  # Первый подграфик (левая часть)
                plt.plot(self.x_positions_1, self.y_positions_1, marker='o', markersize=3, linestyle='-', color='b')
                plt.title('Robot Trajectory - Topic 1')
                plt.xlabel('X Position')
                plt.ylabel('Y Position')
                plt.grid(True)
                plt.xlim(-self.radius - 0.5, self.radius + 0.5)
                plt.ylim(-self.radius - 0.5, self.radius + 0.5)

            if len(self.x_positions_2) > 0:
                plt.subplot(122)  # Второй подграфик (правая часть)
                plt.plot(self.x_positions_2, self.y_positions_2, marker='o', markersize=3, linestyle='-', color='r')
                plt.title('Robot Trajectory - Topic 2')
                plt.xlabel('X Position')
                plt.ylabel('Y Position')
                plt.grid(True)
                plt.xlim(-self.radius - 0.5, self.radius + 0.5)
                plt.ylim(-self.radius - 0.5, self.radius + 0.5)

            plt.pause(0.01)  # Обновляем график

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
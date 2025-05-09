import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.listener_callback,
            10)
        
        # Списки для хранения координат
        self.x_positions = []
        self.y_positions = []

        # Настройка графика
        plt.ion()  # Включение интерактивного режима
        
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'g-')  # Линия для траектории
        self.ax.set_xlim(-1.5, 1.5)  # Установите пределы по оси X
        self.ax.set_ylim(-1.5, 1.5)  # Установите пределы по оси Y
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Robot Filtered Trajectory')
        self.ax.grid(True)

    def listener_callback(self, msg):
        # Получение позиции из сообщения Odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Добавление координат в списки
        self.x_positions.append(x)
        self.y_positions.append(y)

        # Обновление графика
        self.update_plot()

    def update_plot(self):
        # Обновление данных линии и перерисовка графика
        self.line.set_xdata(self.x_positions)
        self.line.set_ydata(self.y_positions)
        
        # Перерисовка графика
        self.ax.relim()
        self.ax.autoscale_view()
        
        plt.draw()
        plt.pause(0.01)  # Небольшая пауза для обновления графика

def main(args=None):
    rclpy.init(args=args)
    trajectory_visualizer = TrajectoryVisualizer()

    try:
        rclpy.spin(trajectory_visualizer)
    except KeyboardInterrupt:
        pass

    trajectory_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
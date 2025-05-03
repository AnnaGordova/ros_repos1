from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Узел EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/kalman_publish/config/ekf.yaml']
        ),
        
        # Ваш существующий узел публикации
        Node(
            package='kalman_publish',
            executable='publish',
            name='odometry_publisher',
            output='screen'
        )
    ])

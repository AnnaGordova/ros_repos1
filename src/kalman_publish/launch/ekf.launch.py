from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # print(../config/ekf.yaml)

    return LaunchDescription([
        # Узел EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            #parameters=['../config/ekf.yaml'] запускаить из ~/ros2_ws1/src/kalman_publish/launch
            parameters=['/home/anna/ros2_ws1/src/kalman_publish/config/ekf.yaml']
        ),
        
        # Ваш существующий узел публикации
        Node(
            package='kalman_publish',
            executable='publish',
            name='odometry_publisher',
            output='screen'
        ),

        Node(
            package='kalman_publish',
            executable='painter',
            name='painter_filtered',
            output='screen'
        )
        
    ])

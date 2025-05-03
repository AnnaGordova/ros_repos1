from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{'use_sim_time': False}, 'ekf_config.yaml']
        ),
        Node(
            package='kalman_publish',
            executable='odometry_publisher',
            name='odometry_publisher_node',
            output='screen'
        )
    ])
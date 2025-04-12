from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LQR Action Server
        Node(
            package='lqr_control',
            executable='lqr_server',
            name='lqr_server',
            output='screen'
        ),
        
        # LQR Action Client
        Node(
            package='lqr_control',
            executable='lqr_client',
            name='lqr_client',
            output='screen'
        ),
    ])

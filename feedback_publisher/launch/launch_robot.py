from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_publisher',
            executable='cmd_publisher',
            name='cmd_publisher',
            output='screen'
        ),
        Node(
            package='encoders_node',
            executable='left_encoder',
            name='left_encoder',
            output='screen'
        ),
        Node(
            package='encoders_node',
            executable='right_encoder',
            name='right_encoder',
            output='screen'
        ),
        Node(
            package='feedback_publisher',
            executable='lqr_publisher',
            name='lqr_publisher',
            output='screen'
        ),
        Node(
            package='lqr_controller',
            executable='lqr_controller',
            name='lqr_controller',
            output='screen'
        ),
        Node(
            package='motor_controller',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='mpu6050_reader',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen'
        ),
        Node(
            package='mpu6050_reader',
            executable='mpu6050_filter',
            name='mpu6050_filter',
            output='screen'
        ),
    ])

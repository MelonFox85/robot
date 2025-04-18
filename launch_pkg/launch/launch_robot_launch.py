import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_publisher',
            executable='cmd_publisher',
            name='cmd_publisher_node',
            output='screen'
        ),
        Node(
            package='encoders_node',
            executable='left_encoder',
            name='left_encoder_node',
            output='screen'
        ),
        Node(
            package='encoders_node',
            executable='right_encoder',
            name='right_encoder_node',
            output='screen'
        ),
        Node(
            package='feedback_publisher',
            executable='lqr_publisher',
            name='lqr_publisher_node',
            output='screen'
        ),
        Node(
            package='lqr_controller',
            executable='lqr_controller',
            name='lqr_controller_node',
            output='screen'
        ),
        Node(
            package='motor_controller',
            executable='motor_controller',
            name='motor_controller_node',
            output='screen'
        ),
    ])


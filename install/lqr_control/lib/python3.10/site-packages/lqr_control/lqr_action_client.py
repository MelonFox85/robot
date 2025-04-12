import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from custom_interfaces.action import SetMotorSpeed
import math


class LQRClient(Node):
    def __init__(self):
        super().__init__('lqr_client')

        # Initialize Action Client
        self._action_client = ActionClient(self, SetMotorSpeed, 'set_motor_speed')

        # Subscribe to the desired robot parameters
        self.cmd_subscription = self.create_subscription(
            Float32MultiArray,
            'cmd',
            self.cmd_callback,
            10
        )

        # Subscribe to encoder data (left and right wheel speeds)
        self.encoder_left_subscription = self.create_subscription(
            Float32,
            'encoders_data/left',
            self.encoder_left_callback,
            10
        )
        self.encoder_right_subscription = self.create_subscription(
            Float32,
            'encoders_data/right',
            self.encoder_right_callback,
            10
        )

        # Subscribe to IMU data (filtered data for angle and angular velocities)
        self.imu_subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/filtered_data',
            self.imu_callback,
            10
        )

        # Parameters
        self.cmd_values = None  # Desired state
        self.current_values = [0.0] * 6  # Current state: [distance, body angular velocity, plane angular velocity, linear velocity, body angle, plane angle]
        self.left_wheel_speed = 0.0  # Left wheel angular velocity (revolutions per second)
        self.right_wheel_speed = 0.0  # Right wheel angular velocity (revolutions per second)
        self.previous_distance = 0.0  # Accumulated distance (m)
        self.previous_angle = 0.0  # Accumulated plane angle (radians)

        # Robot physical parameters
        self.wheel_radius = 0.0325  # Radius of the wheel (m)
        self.wheel_base = 0.1321  # Distance between wheels (m)

    def cmd_callback(self, msg):
        """Callback for desired robot parameters."""
        self.cmd_values = msg.data
        self.check_and_send_goal()

    def encoder_left_callback(self, msg):
        """Callback for left wheel encoder data."""
        self.left_wheel_speed = msg.data  # Speed in revolutions per second

    def encoder_right_callback(self, msg):
        """Callback for right wheel encoder data."""
        self.right_wheel_speed = msg.data  # Speed in revolutions per second

        # Update linear velocity, distance, and plane angle
        self.update_motion_estimation()

    def imu_callback(self, msg):
        """Callback for IMU data."""
        # Extract angular velocity and angle from IMU data
        self.current_values[1] = msg.data[1]  # Body angular velocity (degrees/sec)
        self.current_values[2] = msg.data[2]  # Plane angular velocity (degrees/sec)
        self.current_values[4] = msg.data[0]  # Body angle (degrees)

    def update_motion_estimation(self):
        """Update the robot's linear velocity, distance, and plane angle."""
        # Convert wheel speeds from revolutions per second to linear velocities (m/s)
        v_left = self.left_wheel_speed * 2 * math.pi * self.wheel_radius
        v_right = self.right_wheel_speed * 2 * math.pi * self.wheel_radius

        # Linear velocity (center of the robot)
        v_linear = (v_left + v_right) / 2.0

        # Angular velocity (rotation rate around the plane)
        omega_plane = (v_right - v_left) / self.wheel_base

        # Update distance (integral of linear velocity)
        self.previous_distance += v_linear * 0.01  # Assuming 10ms update rate
        self.current_values[0] = self.previous_distance

        # Update plane angle (integral of angular velocity)
        self.previous_angle += omega_plane * 0.01  # Assuming 10ms update rate
        self.current_values[5] = math.degrees(self.previous_angle)  # Convert to degrees

        # Update linear velocity in the current state
        self.current_values[3] = v_linear

        self.check_and_send_goal()

    def check_and_send_goal(self):
        """Check if both desired and current values are available, then calculate and send goal."""
        if self.cmd_values is None or self.current_values is None:
            return

        # Calculate the difference (delta) between desired and current states
        delta = [cmd - current for cmd, current in zip(self.cmd_values, self.current_values)]

        # Create goal message
        goal_msg = SetMotorSpeed.Goal()
        (
            goal_msg.delta_distance,
            goal_msg.delta_body_angular_velocity,
            goal_msg.delta_plane_angular_velocity,
            goal_msg.delta_linear_velocity,
            goal_msg.delta_body_angle,
            goal_msg.delta_plane_angle,
        ) = delta

        # Send the goal to the Action Server
        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal request...')
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response to the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle the result of the goal."""
        try:
            result = future.result().result  # Получаем результат
            if hasattr(result, 'success'):  # Проверяем наличие атрибута success
                if result.success:
                    self.get_logger().info('Goal succeeded')
                else:
                    self.get_logger().info('Goal failed')
            else:
                self.get_logger().error('Result does not have `success` attribute')
        except Exception as e:
            self.get_logger().error(f'Error in result callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LQRClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import math


class LQRPublisher(Node):
    """
    A node that gathers desired state information (cmd_values) and sensor data,
    calculates the difference between the desired and actual states, and then
    publishes this 'negative feedback' (delta) to the LQR regulator.
    """

    def __init__(self):
        super().__init__('lqr_publisher')

        # ROS Parameters
        self.declare_parameter('update_rate', 100.0)  # in Hz, or 100 for 10ms
        self.update_rate = self.get_parameter('update_rate').value
        self.dt = 1.0 / self.update_rate

        # Publisher for LQR feedback (negative feedback)
        self.feedback_publisher = self.create_publisher(Float32MultiArray, 'lqr_feedback', 10)

        # Subscribe to desired robot parameters (cmd), encoders, and IMU
        self.cmd_subscription = self.create_subscription(
            Float32MultiArray,
            'cmd',
            self.cmd_callback,
            10
        )
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
        self.imu_subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/filtered_data',
            self.imu_callback,
            10
        )

        # Desired state: set externally from 'cmd' topic
        self.cmd_values = None

        # Current robot state: 
        # [distance, body angular velocity, plane angular velocity, linear velocity, body angle, plane angle]
        self.current_values = [0.0] * 6

        # Local encoder data
        self.left_wheel_speed = 0.0  # rev/s
        self.right_wheel_speed = 0.0 # rev/s

        # Integration for distance and plane angle
        self.previous_distance = 0.0
        self.previous_angle = 0.0

        # Physical parameters
        self.wheel_radius = 0.0325
        self.wheel_base = 0.1321

        self.get_logger().info("LQRPublisher Node Initialized.")

    def cmd_callback(self, msg: Float32MultiArray):
        """
        Callback for receiving the desired robot parameters.
        """
        self.cmd_values = msg.data
        # Attempt to publish new feedback right away
        self.publish_feedback()

    def encoder_left_callback(self, msg: Float32):
        """
        Callback for receiving the left wheel speed (rev/s).
        """
        self.left_wheel_speed = msg.data

    def encoder_right_callback(self, msg: Float32):
        """
        Callback for receiving the right wheel speed (rev/s).
        """
        self.right_wheel_speed = msg.data
        self.update_motion_estimation()

    def imu_callback(self, msg: Float32MultiArray):
        """
        Callback for receiving IMU data.
        The data layout: [body_angle_deg, body_angular_velocity_deg_s, plane_angular_velocity_deg_s].
        """
        self.current_values[4] = msg.data[0]  # body angle (deg)
        self.current_values[1] = msg.data[1]  # body angular velocity (deg/s)
        self.current_values[2] = msg.data[2]  # plane angular velocity (deg/s)

    def update_motion_estimation(self):
        """
        Estimate the robot's linear velocity, distance, and plane angle
        based on encoder readings and known geometry.
        """
        # Convert wheel speeds (rev/s) to linear velocity (m/s).
        v_left = self.left_wheel_speed * 2.0 * math.pi * self.wheel_radius
        v_right = self.right_wheel_speed * 2.0 * math.pi * self.wheel_radius
        v_linear = (v_left + v_right) / 2.0

        # Angular velocity around plane (rad/s)
        omega_plane = (v_right - v_left) / self.wheel_base

        # Integrate distance
        self.previous_distance += v_linear * self.dt
        self.current_values[0] = self.previous_distance

        # Integrate plane angle (store as degrees)
        self.previous_angle += omega_plane * self.dt
        self.current_values[5] = math.degrees(self.previous_angle)

        # Current linear velocity
        self.current_values[3] = v_linear

        # Attempt to publish new feedback
        self.publish_feedback()

    def publish_feedback(self):
        """
        Publish the difference between desired and current state.
        If self.cmd_values is not set, no data is published.
        """
        if self.cmd_values is None:
            return

        # Negative feedback
        delta = [cmd - current for cmd, current in zip(self.cmd_values, self.current_values)]

        msg = Float32MultiArray()
        msg.data = delta
        self.feedback_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LQRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

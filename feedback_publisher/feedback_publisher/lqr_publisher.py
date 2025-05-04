import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import math


class LQRPublisher(Node):
    """
    A node that computes the difference (delta) between the desired and the actual state of the robot.
    The state is published as an array of 6 elements:
      [distance_traveled (m), linear_velocity (m/s),
       body_tilt_angle (rad), body_tilt_angular_velocity (rad/s),
       turning_angle (rad), turning_angular_velocity (rad/s)]
    """

    def __init__(self):
        super().__init__('lqr_publisher')

        # Rate at which we'll integrate and publish
        self.declare_parameter('update_rate', 400.0)   # 100 Hz => 10 ms
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.dt = 1.0 / self.update_rate

        # Publisher for "negative feedback" array
        self.feedback_publisher = self.create_publisher(Float32MultiArray, 'lqr_feedback', 10)

        # Subscription: Desired robot parameters (cmd) as 6-element array
        # The user must send [distance, linear_vel, body_angle, body_gyro, turn_angle, turn_gyro]
        self.cmd_subscription = self.create_subscription(
            Float32MultiArray,
            'cmd',
            self.cmd_callback,
            10
        )
        # Subscription: Encoders (left and right)
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
        # Subscription: IMU with partial data
        # We'll assume the layout: [body_angle_deg, body_angular_velocity_deg_s, turn_angular_velocity_deg_s]
        self.imu_subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/filtered_data',
            self.imu_callback,
            10
        )

        # Desired state
        self.cmd_values = None  # [distance, lin_vel, tilt_angle, tilt_gyro, turn_angle, turn_gyro]

        # Current state (units => [m, m/s, rad, rad/s, rad, rad/s])
        self.current_values = [0.0]*6

        # Internal
        self.left_wheel_speed = 0.0   # rev/s
        self.right_wheel_speed = 0.0  # rev/s

        # For distance and turning angle integration
        self.distance_accum = 0.0
        self.turn_accum = 0.0

        # Robot geometry
        self.wheel_radius = 0.0325
        self.wheel_base = 0.1587

        self.get_logger().info("LQRPublisher node initialized.")

    def cmd_callback(self, msg):
        """Receive desired state."""
        self.cmd_values = list(msg.data)   # store so we can compute delta
        self.publish_feedback()            # attempt immediate publish

    def encoder_left_callback(self, msg):
        """Left wheel speed in rev/s."""
        self.left_wheel_speed = msg.data

    def encoder_right_callback(self, msg):
        """Right wheel speed in rev/s."""
        self.right_wheel_speed = msg.data
        self.update_motion_estimation()

    def imu_callback(self, msg):
        """
        IMU update in degrees for angle/velocity, so convert them to radians here.
        Layout assumed: [body_angle_deg, body_ang_vel_deg_s, turn_ang_vel_deg_s].
        """
        body_angle_rad = math.radians(msg.data[0])
        body_ang_vel_rad_s = math.radians(msg.data[1])
        turn_ang_vel_rad_s = math.radians(msg.data[2])

        # Store in current_values
        self.current_values[2] = body_angle_rad
        self.current_values[3] = body_ang_vel_rad_s
        self.current_values[5] = turn_ang_vel_rad_s  # turn angular velocity
        # We'll handle turning angle integration in update_motion_estimation

    def update_motion_estimation(self):
        """
        Convert wheel speeds to linear velocity, distance, and turning angle in rad.
        """
        # rev/s -> (2*pi*r) m/s
        v_left = self.left_wheel_speed * 2.0 * math.pi * self.wheel_radius
        v_right = self.right_wheel_speed * 2.0 * math.pi * self.wheel_radius
        linear_vel = (v_left + v_right) / 2.0

        # Turning angle rate (rad/s)
        turning_rate = (v_right - v_left) / self.wheel_base

        # Integrate distance
        self.distance_accum += linear_vel * self.dt
        self.current_values[0] = self.distance_accum

        # Integrate turning angle
        self.turn_accum += turning_rate * self.dt
        self.current_values[4] = self.turn_accum

        # Update linear velocity
        self.current_values[1] = linear_vel

        # Attempt to publish
        self.publish_feedback()

    def publish_feedback(self):
        """Publish difference between cmd_values and current_values."""
        if self.cmd_values is None:
            return

        # Build array of delta = (cmd_value - current_value)
        # Order: [dist, lin_vel, body_tilt, body_tilt_vel, turn_angle, turn_vel]
        delta = []
        for c, a in zip(self.cmd_values, self.current_values):
            delta.append(c - a)

        msg_out = Float32MultiArray()
        msg_out.data = delta
        self.feedback_publisher.publish(msg_out)
        # debug print
        self.get_logger().debug("Published LQR feedback: {}".format(delta))


def main(args=None):
    rclpy.init(args=args)
    node = LQRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

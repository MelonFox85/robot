import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class LQRController(Node):
    """
    This node receives a 6-element delta (error) array from 'lqr_feedback'.
    We apply user-defined LQR gains K to compute left motor acceleration (L_accel) and
    right motor acceleration (R_accel) in a simplified approach, then transform them
    to motor RPM and publish to motor_rpm/left and motor_rpm/right.
    """

    def __init__(self):
        super().__init__('lqr_controller')

        # Create subscription for LQR feedback (the 6-element array)
        self.lqr_feedback_sub = self.create_subscription(
            Float32MultiArray,
            'lqr_feedback',
            self.feedback_callback,
            10
        )

        # Publishers for motor RPM signals
        self.left_rpm_pub = self.create_publisher(Float32, 'motor_rpm/left', 10)
        self.right_rpm_pub = self.create_publisher(Float32, 'motor_rpm/right', 10)

        # Example: Hardcode or declare a parameter for user to set K
        # K is 2x6 for [L_accel, R_accel]
        # Default from your snippet:
        #  [[-22.3607, -43.2739, -468.4965, -42.1292,  22.3607,   6.1385],
        #   [-22.3607, -43.2739, -468.4965, -42.1292, -22.3607,  -6.1385]]
        self.K = [
            [-22.3607, -43.2739, -468.4965, -42.1292,  22.3607,   6.1385],
            [-22.3607, -43.2739, -468.4965, -42.1292, -22.3607,  -6.1385]
        ]

        # Motor limits
        self.rpm_max = 366.0

        # Simple scale factor to convert "acceleration" to "rpm"
        # The exact logic depends on your hardware. Tune as needed.
        self.accel_to_rpm = 366.0

        self.get_logger().info("LQRController node initialized.")

    def feedback_callback(self, msg):
        """
        msg.data is a 6-element array (delta):
          [dist, lin_vel, body_tilt, body_tilt_vel, turn_angle, turn_vel]
        We'll compute:
            L_accel = -(K1*delta[0] + K2*delta[1] + K3*delta[2] + ...
                        K4*delta[3] + K5*delta[4] + K6*delta[5])
            R_accel = -(K1*delta[0] + K2*delta[1] + K3*delta[2] + ...
                        K4*delta[3] - K5*delta[4] - K6*delta[5])
          or more generally we've stored in self.K in a 2x6 structure.
        """
        delta = msg.data
        # Convert to 6 real values
        if len(delta) < 6:
            self.get_logger().error("Received insufficient data size for LQR feedback!")
            return

        # Approach: u = K * delta (2 outputs)
        L_accel = 0.0
        R_accel = 0.0
        for i in range(6):
            L_accel += self.K[0][i] * delta[i]
            R_accel += self.K[1][i] * delta[i]

        # We apply the negative sign if needed. Your snippet has the minus inside the K matrix.
        # If you want it external, do L_accel = -L_accel, R_accel = -R_accel. Check your math:
        # For demonstration, we assume K already includes negative sign factors.
        # L_accel, R_accel are dimensionless in this example. Convert them to RPM:
        left_rpm = L_accel * self.accel_to_rpm
        right_rpm = R_accel * self.accel_to_rpm

        # Clip RPM
        left_rpm_clamped = max(min(left_rpm, self.rpm_max), -self.rpm_max)
        right_rpm_clamped = max(min(right_rpm, self.rpm_max), -self.rpm_max)

        # Publish results
        l_msg = Float32()
        r_msg = Float32()
        l_msg.data = float(left_rpm_clamped)
        r_msg.data = float(right_rpm_clamped)
        self.left_rpm_pub.publish(l_msg)
        self.right_rpm_pub.publish(r_msg)

        self.get_logger().info(
            f"LQR->  L_accel={L_accel:.2f}, R_accel={R_accel:.2f} => L_rpm={left_rpm_clamped:.2f}, R_rpm={right_rpm_clamped:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

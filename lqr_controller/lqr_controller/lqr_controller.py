import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from scipy.linalg import solve_continuous_are

class LQRController(Node):
    """
    This node receives the error state (delta) from a 'lqr_feedback' topic,
    computes the control input through LQR, converts it into desired motor RPM,
    then publishes these RPM commands to 'motor_rpm/left' and 'motor_rpm/right'.
    """

    def __init__(self):
        super().__init__('lqr_controller')
        
        # Subscriber for LQR feedback (the delta array)
        self.feedback_subscription = self.create_subscription(
            Float32MultiArray,
            'lqr_feedback',
            self.feedback_callback,
            10
        )

        # Publishers for motor RPM signals
        self.motor_left_rpm_publisher = self.create_publisher(Float32, 'motor_rpm/left', 10)
        self.motor_right_rpm_publisher = self.create_publisher(Float32, 'motor_rpm/right', 10)

        # Declare/Initialize Robot Physical Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('m', 0.034),
                ('r', 0.065/2.0),
                ('M', 1.14 - 2.0*0.034),  # example default
                ('L', 0.5 * 0.119),
                ('g', 9.81),
            ]
        )
        # Additional derived or known parameters
        self.m = self.get_parameter('m').value
        self.r = self.get_parameter('r').value
        self.i = 0.5 * self.m * (self.r**2)
        self.M = self.get_parameter('M').value
        self.L = self.get_parameter('L').value
        self.J_p = (1.0 / 12.0) * self.M * (0.1640**2 + 0.0640**2)
        self.d = 0.1587
        self.J_delta = (1.0 / 12.0) * self.M * (0.1190**2 + 0.0640**2)
        self.g = self.get_parameter('g').value

        # Motor constraints
        self.Ke = 0.00103       # V/rpm
        self.KT = 0.00984       # Nm/A
        self.rpm_max = 366.0    # maximum rpm (no-load)
        self.voltage_supply = 12.0

        # Construct matrices, define Q & R, solve for LQR K
        self.A, self.B = self.calculate_matrices()
        self.Q = np.diag([1000, 0, 0, 1000, 1000, 0])
        self.R = np.diag([1.0, 1.0])
        self.K = self.calculate_lqr_gain()

        self.get_logger().info(f"LQRControllerRPM Node Initialized with K:\n{self.K}")

    def calculate_matrices(self):
        """
        Build the continuous-time state-space model for the robot.
        Returns (A, B) system matrices.
        """
        Q_eq = self.J_p * self.M + (self.J_p + self.M * self.L**2) * \
               (2.0*self.m + 2.0*self.i / self.r**2)

        A_23 = -(self.M**2 * self.L**2 * self.g) / Q_eq
        A_43 = (self.M * self.L * self.g *
                (self.M + 2.0*self.m + 2.0*self.i / self.r**2)) / Q_eq

        B_21 = (self.J_p + self.M * self.L**2 + self.M * self.L * self.r) / (Q_eq * self.r)
        B_41 = -(self.M * self.L / self.r + self.M + 2.0*self.m + 2.0*self.i / self.r**2) / Q_eq
        B_61 = 1.0 / (
            self.r *
            (self.m * self.d + self.i * self.d / self.r**2 + 2.0*self.J_delta / self.d)
        )

        A = np.array([
            [0.0,  1.0, 0.0, 0.0,  0.0, 0.0],
            [0.0,  0.0, A_23, 0.0, 0.0, 0.0],
            [0.0,  0.0, 0.0,  1.0,  0.0, 0.0],
            [0.0,  0.0, A_43, 0.0,  0.0, 0.0],
            [0.0,  0.0, 0.0,  0.0,  0.0, 1.0],
            [0.0,  0.0, 0.0,  0.0,  0.0, 0.0]
        ])

        # Scale by (self.i / self.r)
        B = (self.i / self.r) * np.array([
            [0.0,    0.0],
            [B_21,   B_21],
            [0.0,    0.0],
            [B_41,   B_41],
            [0.0,    0.0],
            [B_61,  -B_61]
        ])

        return A, B

    def calculate_lqr_gain(self):
        """
        Solve the continuous-time algebraic Riccati equation for LQR gain.
        Returns gain matrix K.
        """
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def feedback_callback(self, msg: Float32MultiArray):
        """
        Receive the error state (delta), compute LQR output, convert voltage to RPM,
        clamp the RPM, and publish.
        """
        delta = np.array(msg.data, dtype=float)

        u = -self.K @ delta  # 2-element control vector
        self.get_logger().info(f"LQR raw control input (approx voltage): {u}")

        # Clip voltage to ±12 V
        left_voltage = float(np.clip(u[0], -self.voltage_supply, self.voltage_supply))
        right_voltage = float(np.clip(u[1], -self.voltage_supply, self.voltage_supply))

        # Convert from voltage to rpm: rpm = voltage / Ke
        left_rpm = left_voltage / self.Ke
        right_rpm = right_voltage / self.Ke

        # Clip to ± rpm_max
        left_rpm_clamped = float(np.clip(left_rpm, -self.rpm_max, self.rpm_max))
        right_rpm_clamped = float(np.clip(right_rpm, -self.rpm_max, self.rpm_max))

        # Publish
        left_rpm_msg = Float32()
        right_rpm_msg = Float32()
        left_rpm_msg.data = left_rpm_clamped
        right_rpm_msg.data = right_rpm_clamped

        self.motor_left_rpm_publisher.publish(left_rpm_msg)
        self.motor_right_rpm_publisher.publish(right_rpm_msg)

        self.get_logger().info(
            f"RPM Output -> Left: {left_rpm_clamped:.2f}, Right: {right_rpm_clamped:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

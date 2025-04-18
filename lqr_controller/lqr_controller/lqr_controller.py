import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from scipy.linalg import solve_continuous_are


class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        # Subscriber for LQR feedback
        self.feedback_subscription = self.create_subscription(
            Float32MultiArray,
            'lqr_feedback',
            self.feedback_callback,
            10
        )

        # Publishers for motor control signals
        self.motor_left_publisher = self.create_publisher(Float32, 'motor/left', 10)
        self.motor_right_publisher = self.create_publisher(Float32, 'motor/right', 10)

        # Physical parameters of the robot
        self.m = 0.035
        self.M = 1.08
        self.r = 0.0325
        self.L = 0.045
        self.i = 1.85e-5
        self.J_p = 0.00589
        self.J_delta = 0.00327
        self.d = 0.026
        self.g = 9.81

        # Define state-space matrices A and B
        self.A, self.B = self.calculate_matrices()

        # Define Q and R matrices for LQR
        self.Q = np.diag([1000, 0, 0, 1000, 1000, 0])
        self.R = np.diag([1, 1])

        # Compute LQR gain matrix K
        self.K = self.calculate_lqr_gain()

    def calculate_matrices(self):
        """Calculate dynamic matrices A and B based on robot parameters."""
        Q_eq = self.J_p * self.M + (self.J_p + self.M * self.L**2) * (2 * self.m + 2 * self.i / self.r**2)
        A_23 = -(self.M**2 * self.L**2 * self.g) / Q_eq
        A_43 = self.M * self.L * self.g * (self.M + 2 * self.m + 2 * self.i / self.r**2) / Q_eq
        B_21 = (self.J_p + self.M * self.L**2 + self.M * self.L * self.r) / (Q_eq * self.r)
        B_41 = -(self.M * self.L / self.r + self.M + 2 * self.m + 2 * self.i / self.r**2) / Q_eq
        B_61 = 1 / (self.r * (self.m * self.d + self.i * self.d / self.r**2 + 2 * self.J_delta / self.d))

        # A matrix
        A = np.array([
            [0, 1, 0, 0, 0, 0],
            [0, 0, A_23, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, A_43, 0, 0, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0]
        ])

        # B matrix
        B = (self.i / self.r) * np.array([
            [0, 0],
            [B_21, B_21],
            [0, 0],
            [B_41, B_41],
            [0, 0],
            [B_61, -B_61]
        ])

        return A, B

    def calculate_lqr_gain(self):
        """Calculate LQR gain matrix K using the solution to the continuous-time Riccati equation."""
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def feedback_callback(self, msg):
        """Callback to process LQR feedback."""
        delta = np.array(msg.data)

        # Calculate control input u using LQR
        u = -self.K @ delta

        # Split control input into motor speeds
        motor_left_speed = u[0]
        motor_right_speed = u[1]

        # Publish motor speeds
        motor_left_msg = Float32()
        motor_right_msg = Float32()

        motor_left_msg.data = motor_left_speed
        motor_right_msg.data = motor_right_speed

        self.motor_left_publisher.publish(motor_left_msg)
        self.motor_right_publisher.publish(motor_right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

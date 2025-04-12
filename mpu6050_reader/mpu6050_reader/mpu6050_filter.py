import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MPU6050FilterNode(Node):
    def __init__(self):
        super().__init__('mpu6050_filter')

        # Subscriber for raw MPU6050 data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/data',
            self.mpu6050_callback,
            10
        )

        # Publisher for filtered data
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'mpu6050/filtered_data',
            10
        )

        # Complementary filter parameters
        self.alpha = 0.90 # weight for gyroscope data
        self.dt = 0.01    # time step in seconds (adjust as per your system)

        # State variables
        self.filtered_angle = 0.0  # Tilt angle for balancing
        self.angular_velocity_body = 0.0  # Angular velocity of the robot's body
        self.angular_velocity_plane = 0.0  # Angular velocity on the plane (z-axis)

    def mpu6050_callback(self, msg):
        # Extract accelerometer and gyroscope data
        accel_x, accel_y, accel_z = msg.data[0:3]
        gyro_x, gyro_y, gyro_z = msg.data[3:6]

        # Calculate tilt angle from accelerometer data
        accel_angle = self.calculate_accel_angle(accel_x, accel_y, accel_z)

        # Integrate gyroscope data to get rate of change of angle
        gyro_angle = self.filtered_angle + gyro_y * self.dt

        # Apply complementary filter for tilt angle
        self.filtered_angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle

        # Calculate angular velocity of the robot's body (balance, y-axis)
        self.angular_velocity_body = gyro_y

        # Calculate angular velocity on the plane (rotation, z-axis)
        self.angular_velocity_plane = gyro_z

        # Publish filtered data
        filtered_msg = Float32MultiArray()
        filtered_msg.data = [
            self.filtered_angle,
            self.angular_velocity_body,
            self.angular_velocity_plane
        ]
        self.publisher.publish(filtered_msg)

    @staticmethod
    def calculate_accel_angle(accel_x, accel_y, accel_z):
        import math
        # Calculate tilt angle (assuming rotation around y-axis)
        return math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050FilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

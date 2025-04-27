import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import RPi.GPIO as GPIO
import time


class MotorController(Node):
    """
    A node that receives desired RPMs for the left and right motors, translates
    them to PWM signals, and controls the motors through GPIO pins on a Raspberry Pi.
    """

    def __init__(self):
        super().__init__('motor_controller')

        # Declare critical angle parameter in case we want local checks
        # (Not strictly necessary here if running in LQR or safety node)
        self.declare_parameter('critical_angle', 40.0)
        self.critical_angle = self.get_parameter('critical_angle').value

        # GPIO Pin Configuration
        self.PWMA = 13  # PWM for right motor
        self.AIN1 = 27  # Direction 1 for right motor
        self.AIN2 = 17  # Direction 2 for right motor

        self.PWMB = 12  # PWM for left motor
        self.BIN1 = 14  # Direction 1 for left motor
        self.BIN2 = 4   # Direction 2 for left motor

        self.STBY = 15  # Standby pin (enable motors)

        # Motor constraints
        self.max_rpm = 366
        self.max_pwm_duty_cycle = 100.0  # 0–100%

        # PWM frequency (example: 7000 Hz from the original code)
        self.pwm_frequency = 7000

        # Emergency stop
        self.emergency_stop = False
        self.current_body_angle = 0.0

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.AIN1, self.AIN2, self.BIN1, self.BIN2, self.STBY], GPIO.OUT)
        GPIO.setup([self.PWMA, self.PWMB], GPIO.OUT)

        # Create PWM
        self.pwm_right = GPIO.PWM(self.PWMA, self.pwm_frequency)
        self.pwm_left = GPIO.PWM(self.PWMB, self.pwm_frequency)
        self.pwm_right.start(0)
        self.pwm_left.start(0)

        # Enable motors
        GPIO.output(self.STBY, GPIO.HIGH)

        # ROS2 Subscribers for RPM commands
        self.left_rpm_sub = self.create_subscription(
            Float32,
            'motor_rpm/left',
            self.left_rpm_callback,
            10
        )
        self.right_rpm_sub = self.create_subscription(
            Float32,
            'motor_rpm/right',
            self.right_rpm_callback,
            10
        )

        # IMU-subscription can be used for final-level safety checks if needed
        self.imu_subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/filtered_data',
            self.imu_callback,
            10
        )

        self.get_logger().info("MotorController Node Initialized.")

    def imu_callback(self, msg: Float32MultiArray):
        """
        Monitor the body angle from the IMU for emergency stop.
        If angle is beyond threshold, set emergency_stop.
        """
        self.current_body_angle = msg.data[4]
        if abs(self.current_body_angle) > self.critical_angle:
            if not self.emergency_stop:
                self.get_logger().warn(
                    f"Emergency stop triggered: |angle|>{self.critical_angle} deg!"
                )
            self.emergency_stop = True
        else:
            # If we have recovered from the tilt
            if self.emergency_stop:
                self.get_logger().info(
                    "Angle within safe range again. Clearing emergency stop."
                )
            self.emergency_stop = False

    def left_rpm_callback(self, msg: Float32):
        """Callback for controlling the left motor based on RPM."""
        if not self.emergency_stop:
            self.set_motor_rpm(self.BIN1, self.BIN2, self.pwm_left, msg.data)
        else:
            self.stop_motor(self.BIN1, self.BIN2, self.pwm_left, "left")

    def right_rpm_callback(self, msg: Float32):
        """Callback for controlling the right motor based on RPM."""
        if not self.emergency_stop:
            self.set_motor_rpm(self.AIN1, self.AIN2, self.pwm_right, msg.data)
        else:
            self.stop_motor(self.AIN1, self.AIN2, self.pwm_right, "right")

    def set_motor_rpm(self, in1_pin, in2_pin, pwm_instance, rpm):
        """
        Set motor speed and direction from an RPM command, converting to a PWM duty cycle.
        """
        limited_rpm = max(min(rpm, self.max_rpm), -self.max_rpm)

        # Direction
        if limited_rpm > 0:
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        elif limited_rpm < 0:
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)
        else:
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.LOW)

        # Convert RPM to duty cycle
        duty_cycle = (abs(limited_rpm) / self.max_rpm) * self.max_pwm_duty_cycle
        pwm_instance.ChangeDutyCycle(duty_cycle)

        self.get_logger().info(
            f"Requested RPM: {rpm:.2f}, Limited: {limited_rpm:.2f}, Duty Cycle: {duty_cycle:.2f}%"
        )

    def stop_motor(self, in1_pin, in2_pin, pwm_instance, side_label):
        """
        Force stop the motor by setting pins LOW and duty cycle to zero.
        """
        GPIO.output(in1_pin, GPIO.LOW)
        GPIO.output(in2_pin, GPIO.LOW)
        pwm_instance.ChangeDutyCycle(0.0)
        self.get_logger().warn(f"Emergency stop: {side_label} motor forced to stop.")

    def destroy(self):
        """Clean up GPIO and stop PWM."""
        self.get_logger().info("Shutting down Motor Controller Node.")
        self.pwm_right.stop()
        self.pwm_left.stop()
        GPIO.output(self.STBY, GPIO.LOW)  # Disable motors
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

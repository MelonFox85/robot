import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # GPIO Pin Configuration
        self.PWMA = 13  # PWM for right motor
        self.AIN1 = 27  # Direction 1 for right motor
        self.AIN2 = 17  # Direction 2 for right motor

        self.PWMB = 12  # PWM for left motor
        self.BIN1 = 14  # Direction 1 for left motor
        self.BIN2 = 4   # Direction 2 for left motor

        self.STBY = 15  # Standby pin (enable motors)

        # Motor Protection and Limitations
        self.max_rpm = 366
        self.max_pwm_duty_cycle = 100  # Max PWM percentage (0-100%)
        self.max_current = 3.36  # Max motor current in Amps

        # Constants
        self.pwm_frequency = 7000  # 20 kHz PWM frequency

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.AIN1, self.AIN2, self.BIN1, self.BIN2, self.STBY], GPIO.OUT)
        GPIO.setup([self.PWMA, self.PWMB], GPIO.OUT)

        # Create PWM instances
        self.pwm_right = GPIO.PWM(self.PWMA, self.pwm_frequency)
        self.pwm_left = GPIO.PWM(self.PWMB, self.pwm_frequency)

        # Start PWM with 0% duty cycle
        self.pwm_right.start(0)
        self.pwm_left.start(0)

        # Enable motors through STBY
        GPIO.output(self.STBY, GPIO.HIGH)

        # ROS2 Subscribers
        self.left_motor_sub = self.create_subscription(
            Float32,
            'motor/left',
            self.left_motor_callback,
            10
        )
        self.right_motor_sub = self.create_subscription(
            Float32,
            'motor/right',
            self.right_motor_callback,
            10
        )

        self.get_logger().info("Motor Controller Node Initialized.")

    def left_motor_callback(self, msg):
        """Callback to control the left motor."""
        speed = self.limit_speed(msg.data)
        self.set_motor(self.BIN1, self.BIN2, self.pwm_left, speed)

    def right_motor_callback(self, msg):
        """Callback to control the right motor."""
        speed = self.limit_speed(msg.data)
        self.set_motor(self.AIN1, self.AIN2, self.pwm_right, speed)

    def limit_speed(self, speed):
        """Limit the motor speed to safe values."""
        # Convert RPM to duty cycle percentage
        duty_cycle = abs(speed) / self.max_rpm * self.max_pwm_duty_cycle

        # Ensure duty cycle is within 0-100%
        duty_cycle = min(duty_cycle, self.max_pwm_duty_cycle)

        # Log warning if the speed exceeds maximum allowed values
        if abs(speed) > self.max_rpm:
            self.get_logger().warn(
                f"Requested speed {speed} RPM exceeds maximum limit of {self.max_rpm} RPM. Limiting to {self.max_rpm} RPM."
            )
        return duty_cycle

    def set_motor(self, in1_pin, in2_pin, pwm, speed):
        """Set motor speed and direction."""
        if speed > 0:
            # Forward direction
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        elif speed < 0:
            # Backward direction
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)
        else:
            # Stop motor
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.LOW)

        # Set PWM duty cycle
        pwm.ChangeDutyCycle(abs(speed))

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

#!/usr/bin/env python3
"""
motor_controller_node.py
────────────────────────
Motor driver node that consumes:
    • motor_cmd/left, motor_cmd/right   (Float32) – desired angular acceleration α [rad s⁻²]
    • encoders_data/left, encoders_data/right (Float32) – wheel speed ω [rev s⁻¹]
    • mpu6050/filtered_data             (Float32MultiArray) – tilt angle [deg] in data[0]

Voltage command:
    V_desired = (J * α * R) / K_T  +  K_E * ω
    V_desired ∈ [‑12 V … +12 V]

PWM duty:
    duty_% = (V_desired / 12) * 100
"""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import RPi.GPIO as GPIO


class MotorController(Node):
    def __init__(self) -> None:
        super().__init__("motor_controller")

        # ─── Parameters ──────────────────────────────────────
        self.declare_parameters(
            "",
            [
                ("critical_angle", 40.0),        # ° tilt limit
                ("pwm_frequency", 7000),         # Hz
                ("imu_topic", "mpu6050/filtered_data"),
                ("enc_left_topic", "encoders_data/left"),
                ("enc_right_topic", "encoders_data/right"),
                ("cmd_left_topic", "motor_cmd/left"),
                ("cmd_right_topic", "motor_cmd/right"),
                # Motor constants (SI)
                ("J", 4.0e-5),   # kg·m²
                ("R", 3.6),      # Ω
                ("K_e", 0.036),  # V·s/rad
                ("K_t", 0.036),  # N·m/A
            ],
        )
        self.crit_angle = self.get_parameter("critical_angle").value
        self.J = self.get_parameter("J").value
        self.R = self.get_parameter("R").value
        self.K_e = self.get_parameter("K_e").value
        self.K_t = self.get_parameter("K_t").value

        # Current state
        self.omega_left_rad = 0.0   # rad/s
        self.omega_right_rad = 0.0
        self.emergency_stop = False

        # GPIO pins (BCM)
        self.PWMA, self.AIN1, self.AIN2 = 13, 27, 17   # right
        self.PWMB, self.BIN1, self.BIN2 = 12, 14, 4    # left
        self.STBY = 15

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(
            [self.AIN1, self.AIN2, self.BIN1, self.BIN2, self.STBY], GPIO.OUT
        )
        GPIO.setup([self.PWMA, self.PWMB], GPIO.OUT)

        freq = int(self.get_parameter("pwm_frequency").value)
        self.pwm_r = GPIO.PWM(self.PWMA, freq)
        self.pwm_l = GPIO.PWM(self.PWMB, freq)
        self.pwm_r.start(0)
        self.pwm_l.start(0)
        GPIO.output(self.STBY, GPIO.HIGH)

        # ─── Subscriptions ──────────────────────────────────
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("imu_topic").value,
            self._imu_cb,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("enc_left_topic").value,
            self._enc_left_cb,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("enc_right_topic").value,
            self._enc_right_cb,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("cmd_left_topic").value,
            self._cmd_left_cb,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("cmd_right_topic").value,
            self._cmd_right_cb,
            10,
        )

        self.get_logger().info("MotorController ready.")

    # ──────────────────── CALLBACKS ──────────────────── #
    # IMU
    def _imu_cb(self, msg: Float32MultiArray) -> None:
        if not msg.data:
            return
        tilt = msg.data[0]
        if abs(tilt) > self.crit_angle:
            if not self.emergency_stop:
                self.get_logger().warn(
                    f"E‑STOP: |tilt|={tilt:.1f}° > {self.crit_angle}°"
                )
            self.emergency_stop = True
            self._stop(self.BIN1, self.BIN2, self.pwm_l, "left")
            self._stop(self.AIN1, self.AIN2, self.pwm_r, "right")
        elif self.emergency_stop:
            self.get_logger().info("Tilt normalised, E‑STOP cleared.")
            self.emergency_stop = False

    # Encoder speed: rev/s → rad/s
    def _enc_left_cb(self, msg: Float32) -> None:
        self.omega_left_rad = float(msg.data) * 2 * math.pi

    def _enc_right_cb(self, msg: Float32) -> None:
        self.omega_right_rad = float(msg.data) * 2 * math.pi

    # Desired acceleration α [rad/s²] from LQR
    def _cmd_left_cb(self, msg: Float32) -> None:
        self._drive(
            alpha=msg.data,
            omega=self.omega_left_rad,
            in1=self.BIN1,
            in2=self.BIN2,
            pwm=self.pwm_l,
            label="left",
        )

    def _cmd_right_cb(self, msg: Float32) -> None:
        self._drive(
            alpha=msg.data,
            omega=self.omega_right_rad,
            in1=self.AIN1,
            in2=self.AIN2,
            pwm=self.pwm_r,
            label="right",
        )

    # ───────────── CONTROL LOGIC ───────────── #
    def _drive(
        self,
        *,
        alpha: float,
        omega: float,
        in1: int,
        in2: int,
        pwm: GPIO.PWM,
        label: str,
    ) -> None:
        if self.emergency_stop:
            self._stop(in1, in2, pwm, label)
            return

        # Voltage command (all in rad units)
        V = (self.J * alpha * self.R) / self.K_t + self.K_e * omega
        V = max(min(V, 12.0), -12.0)  # clamp

        duty = (V / 12.0) * 100.0     # percentage −100…100
        self._apply(in1, in2, pwm, duty)

        self.get_logger().debug(
            f"{label}: α={alpha:.2f} rad/s², ω={omega:.2f} rad/s -> "
            f"V={V:.2f} V, duty={duty:.1f}%"
        )

    # ───────────── LOW‑LEVEL IO ───────────── #
    def _apply(self, in1: int, in2: int, pwm: GPIO.PWM, duty_pct: float) -> None:
        duty_pct = max(min(duty_pct, 100.0), -100.0)
        if duty_pct > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif duty_pct < 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(abs(duty_pct))

    def _stop(self, in1: int, in2: int, pwm: GPIO.PWM, label: str) -> None:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0.0)
        self.get_logger().debug(f"{label} motor stopped.")

    # ─────────────────── SHUTDOWN ─────────────────── #
    def destroy(self) -> None:
        self.get_logger().info("MotorController shutting down.")
        self.pwm_r.stop()
        self.pwm_l.stop()
        GPIO.output(self.STBY, GPIO.LOW)
        GPIO.cleanup()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

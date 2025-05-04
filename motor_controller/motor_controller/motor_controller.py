#!/usr/bin/env python3
"""
motor_controller_node.py
────────────────────────
Low‑level motor driver for a differential‑drive robot *with feedback*.

NEW FEATURES
------------
A feedback topic is added for every wheel:

    •  /motor_feedback/left   (Float32MultiArray) – [α_desired (rad/s²), duty_%]
    •  /motor_feedback/right  (Float32MultiArray) – [α_desired (rad/s²), duty_%]

This lets higher‑level diagnostics or loggers observe how the requested
acceleration is converted into a PWM duty cycle in real time.
The rest of the file is identical to the previous version.
"""

from __future__ import annotations

import math
from typing import Final

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import RPi.GPIO as GPIO


class MotorController(Node):
    """Implements equations (1)–(2), drives the H‑bridge and publishes feedback."""

    # GPIO mapping (BCM)
    PWMA, AIN1, AIN2 = 13, 27, 17   # right motor
    PWMB, BIN1, BIN2 = 12, 14, 4    # left  motor
    STBY: Final[int] = 15

    _TWO_PI = 2.0 * math.pi

    def __init__(self) -> None:
        super().__init__("motor_controller")

        # ─────── PARAMETERS ───────────────────────────────────────────────
        self.declare_parameters(
            "",
            [
                ("critical_angle_deg", 40.0),
                ("pwm_frequency", 7000),
                # Motor constants
                ("J_total", 4.0e-5),
                ("R_a", 2.3),
                ("L_a", 4.45e-3),
                ("K_e_rpm", 1.03e-3),
                ("K_t", 9.84e-3),
                ("V_static", 0.7),
                ("c_viscous", 0.01),
                # Topics
                ("imu_topic", "mpu6050/filtered_data"),
                ("enc_left_topic", "encoders_data/left"),
                ("enc_right_topic", "encoders_data/right"),
                ("cmd_left_topic", "motor_cmd/left"),
                ("cmd_right_topic", "motor_cmd/right"),
                ("V_bus", 12.0),
            ],
        )

        self.crit_angle = float(self.get_parameter("critical_angle_deg").value)
        self.J = float(self.get_parameter("J_total").value)
        self.R_a = float(self.get_parameter("R_a").value)
        K_e_rpm = float(self.get_parameter("K_e_rpm").value)
        self.K_e = K_e_rpm / (60.0 / self._TWO_PI)     # V·s/rad
        self.K_t = float(self.get_parameter("K_t").value)
        self.V_static = float(self.get_parameter("V_static").value)
        self.c_visc = float(self.get_parameter("c_viscous").value)
        self.V_bus = float(self.get_parameter("V_bus").value)

        self.emergency_stop = False
        self.omega_left = 0.0
        self.omega_right = 0.0

        # ─────── GPIO INIT ────────────────────────────────────────────────
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

        # ─────── PUB/SUB ─────────────────────────────────────────────────
        p = self.get_parameter  # shorthand

        self.create_subscription(Float32MultiArray, p("imu_topic").value, self._imu_cb, 10)
        self.create_subscription(Float32, p("enc_left_topic").value, self._enc_left_cb, 10)
        self.create_subscription(Float32, p("enc_right_topic").value, self._enc_right_cb, 10)
        self.create_subscription(Float32, p("cmd_left_topic").value, self._cmd_left_cb, 10)
        self.create_subscription(Float32, p("cmd_right_topic").value, self._cmd_right_cb, 10)

        self.left_fb_pub = self.create_publisher(Float32MultiArray, "motor_feedback/left", 10)
        self.right_fb_pub = self.create_publisher(Float32MultiArray, "motor_feedback/right", 10)

        self.get_logger().info(
            f"MotorController ready  (K_e={self.K_e:.3e} V·s/rad, K_t={self.K_t:.3e} N·m/A)"
        )

    # ─────────────── SENSOR CALLBACKS ──────────────────────────────────────
    def _imu_cb(self, msg: Float32MultiArray) -> None:
        if not msg.data:
            return
        tilt = msg.data[0]
        if abs(tilt) > self.crit_angle:
            if not self.emergency_stop:
                self.get_logger().warn(f"E‑STOP: |tilt| = {tilt:.1f}°")
            self.emergency_stop = True
            self._hard_stop()
        elif self.emergency_stop:
            self.get_logger().info("Tilt normalised – E‑STOP cleared.")
            self.emergency_stop = False

    def _enc_left_cb(self, msg: Float32) -> None:
        self.omega_left = msg.data * self._TWO_PI

    def _enc_right_cb(self, msg: Float32) -> None:
        self.omega_right = msg.data * self._TWO_PI

    # Desired accelerations α_d are already in rad/s²
    def _cmd_left_cb(self, msg: Float32) -> None:
        self._drive_wheel(
            side="left",
            alpha_d=msg.data,
            omega=self.omega_left,
            in1=self.BIN1,
            in2=self.BIN2,
            pwm=self.pwm_l,
            fb_pub=self.left_fb_pub,
        )

    def _cmd_right_cb(self, msg: Float32) -> None:
        self._drive_wheel(
            side="right",
            alpha_d=msg.data,
            omega=self.omega_right,
            in1=self.AIN1,
            in2=self.AIN2,
            pwm=self.pwm_r,
            fb_pub=self.right_fb_pub,
        )

    # ─────────────── CONTROL + PWM OUTPUT ─────────────────────────────────
    def _drive_wheel(
        self,
        *,
        side: str,
        alpha_d: float,
        omega: float,
        in1: int,
        in2: int,
        pwm: GPIO.PWM,
        fb_pub,
    ) -> None:
        if self.emergency_stop:
            self._stop(in1, in2, pwm, side)
            return

        V_cmd = (
            (self.J * alpha_d * self.R_a) / self.K_t
            + self.K_e * omega
            + self.V_static * (1.0 if alpha_d >= 0 else -1.0 if alpha_d < 0 else 0.0)
            + self.c_visc * omega
        )

        V_cmd = max(min(V_cmd, self.V_bus), -self.V_bus)
        duty_pct = (V_cmd / self.V_bus) * 100.0

        self._apply_pwm(in1, in2, pwm, duty_pct)

        # ── Publish feedback ───────────────────────────────────────────
        fb_msg = Float32MultiArray()
        fb_msg.data = [float(alpha_d), float(duty_pct)]
        fb_pub.publish(fb_msg)

        self.get_logger().debug(
            f"{side:<5}: α_d={alpha_d:8.3f}  ω={omega:7.2f}  "
            f"V={V_cmd:+6.2f} V  duty={duty_pct:+6.1f}%"
        )

    # ─────────────── LOW‑LEVEL GPIO HELPERS ───────────────────────────────
    def _apply_pwm(self, in1: int, in2: int, pwm: GPIO.PWM, duty_pct: float) -> None:
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

    def _hard_stop(self) -> None:
        self._stop(self.BIN1, self.BIN2, self.pwm_l, "left ")
        self._stop(self.AIN1, self.AIN2, self.pwm_r, "right")

    # ─────────────── SHUTDOWN ─────────────────────────────────────────────
    def destroy(self) -> None:
        self.get_logger().info("MotorController shutting down.")
        self._hard_stop()
        self.pwm_r.stop()
        self.pwm_l.stop()
        GPIO.output(self.STBY, GPIO.LOW)
        GPIO.cleanup()


# ─────────────────── MAIN ────────────────────
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

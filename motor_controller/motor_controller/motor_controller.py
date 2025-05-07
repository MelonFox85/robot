#!/usr/bin/env python3
"""
motor_controller_node.py
────────────────────────
Низкоуровневый драйвер H-моста для дифференциального робота
(Raspberry Pi 4 + ROS 2 Humble).

НОВОЕ
• «Умный» выбор частоты ШИМ (если запрошенная недопустима — берётся
  ближайшая из VALID_PWM_FREQ).
• E-STOP теперь корректно сравнивает угол, публикуемый фильтром
  (рад) с порогом, задаваемым в градусах.
"""

from __future__ import annotations
import math
from typing import Final

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, Float32MultiArray
import RPi.GPIO as GPIO

# ──────────────────────────────────────────────────────────────────────
VALID_PWM_FREQ: tuple[int, ...] = (
    50, 100, 250, 500, 1000, 2000,
    4000, 5000, 8000, 10000,
)


class MotorController(Node):
    # GPIO (BCM)
    PWMA, AIN1, AIN2 = 13, 27, 17     # right
    PWMB, BIN1, BIN2 = 12, 14, 4      # left
    STBY: Final[int] = 15
    _TWO_PI: Final[float] = 2.0 * math.pi
    _DEG2RAD: Final[float] = math.pi / 180.0

    # -----------------------------------------------------------------
    def __init__(self) -> None:
        super().__init__("motor_controller_node")

        # ───── параметры ────────────────────────────────────────────
        self.declare_parameters(
            "",
            [
                ("critical_angle_deg", 40.0),
                ("pwm_frequency", 20000),      # Hz
                ("alpha_clip", 200.0),         # rad/s²
                # Motor constants
                ("J_total", 4.0e-5),           # kg·m²
                ("R_a", 2.3),                 # Ω
                ("K_e_rpm", 1.03e-3),          # V·s/rpm
                ("K_t", 9.84e-3),              # N·m/A
                ("V_static", 0.7),             # V
                ("c_viscous", 0.01),           # N·m·s
                ("V_bus", 12.0),               # V
                # topics
                ("imu_topic", "mpu6050/filtered_data"),
                ("enc_left_topic", "encoders_data/left"),
                ("enc_right_topic", "encoders_data/right"),
                ("cmd_left_topic", "motor_cmd/left"),
                ("cmd_right_topic", "motor_cmd/right"),
            ],
        )

        p = self.get_parameter
        crit_angle_deg = float(p("critical_angle_deg").value)
        self.crit_angle_rad = crit_angle_deg * self._DEG2RAD  # ← СРАВНИВАЕМ В РАД

        self.alpha_clip = float(p("alpha_clip").value)
        self.J = float(p("J_total").value)
        self.R_a = float(p("R_a").value)
        self.K_e = float(p("K_e_rpm").value) / (60.0 / self._TWO_PI)
        self.K_t = float(p("K_t").value)
        self.V_static = float(p("V_static").value)
        self.c_visc = float(p("c_viscous").value)
        self.V_bus = float(p("V_bus").value)

        # ───── GPIO ────────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)
        for pin in (self.AIN1, self.AIN2, self.BIN1, self.BIN2, self.STBY):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        for pin in (self.PWMA, self.PWMB):
            GPIO.setup(pin, GPIO.OUT)

        req_freq = int(p("pwm_frequency").value)
        pwm_freq = self._closest_freq(req_freq)
        self.pwm_r = GPIO.PWM(self.PWMA, pwm_freq)
        self.pwm_l = GPIO.PWM(self.PWMB, pwm_freq)
        self.pwm_r.start(0.0)
        self.pwm_l.start(0.0)
        GPIO.output(self.STBY, GPIO.HIGH)
        self.get_logger().info(f"PWM at {pwm_freq} Hz (requested {req_freq})")

        # ───── состояние ───────────────────────────────────────────
        self.emergency_stop = False
        self.omega_left = 0.0
        self.omega_right = 0.0

        qos = QoSProfile(depth=1)
        self.create_subscription(Float32MultiArray, p("imu_topic").value,
                                 self._imu_cb, qos)
        self.create_subscription(Float32, p("enc_left_topic").value,
                                 self._enc_left_cb, qos)
        self.create_subscription(Float32, p("enc_right_topic").value,
                                 self._enc_right_cb, qos)
        self.create_subscription(Float32, p("cmd_left_topic").value,
                                 self._cmd_left_cb, qos)
        self.create_subscription(Float32, p("cmd_right_topic").value,
                                 self._cmd_right_cb, qos)

        self.left_fb_pub = self.create_publisher(Float32MultiArray,
                                                 "motor_feedback/left", qos)
        self.right_fb_pub = self.create_publisher(Float32MultiArray,
                                                  "motor_feedback/right", qos)

        self.get_logger().info(
            f"MotorController ready (E-STOP {crit_angle_deg}° = "
            f"{self.crit_angle_rad:.2f} rad)"
        )

    # ────────────────────────────────────────────────────────────────
    def _closest_freq(self, f: int) -> int:
        return min(VALID_PWM_FREQ, key=lambda x: abs(x - f))

    # ────────────── callbacks ───────────────────────────────────────
    def _imu_cb(self, msg: Float32MultiArray) -> None:
        if not msg.data:
            return
        tilt_rad = msg.data[0]              # уже rad
        if abs(tilt_rad) > self.crit_angle_rad:
            if not self.emergency_stop:
                self.get_logger().warn(f"E-STOP: |tilt| = {math.degrees(tilt_rad):.1f}°")
            self.emergency_stop = True
            self._hard_stop()
        elif self.emergency_stop:
            self.get_logger().info("Tilt normalized – E-STOP cleared.")
            self.emergency_stop = False
            GPIO.output(self.STBY, GPIO.HIGH)

    def _enc_left_cb(self, msg: Float32) -> None:
        self.omega_left = msg.data * self._TWO_PI

    def _enc_right_cb(self, msg: Float32) -> None:
        self.omega_right = msg.data * self._TWO_PI

    def _cmd_left_cb(self, msg: Float32) -> None:
        self._drive("left", self._clip(msg.data), self.omega_left,
                    self.BIN1, self.BIN2, self.pwm_l, self.left_fb_pub)

    def _cmd_right_cb(self, msg: Float32) -> None:
        self._drive("right", self._clip(msg.data), self.omega_right,
                    self.AIN1, self.AIN2, self.pwm_r, self.right_fb_pub)

    # ────────────────────────────────────────────────────────────────
    def _clip(self, a: float) -> float:
        return max(min(a, self.alpha_clip), -self.alpha_clip)

    def _drive(
        self, side: str, alpha_d: float, omega: float,
        in1: int, in2: int, pwm: GPIO.PWM, fb_pub
    ) -> None:
        if self.emergency_stop:
            self._brake(in1, in2, pwm, side)
            return

        V = (
            (self.J * alpha_d * self.R_a) / self.K_t
            + self.K_e * omega
            + self.V_static * (1.0 if alpha_d >= 0 else -1.0 if alpha_d < 0 else 0.0)
            + self.c_visc * omega
        )
        V = max(min(V, self.V_bus), -self.V_bus)
        duty = max(min((V / self.V_bus) * 100.0, 100.0), -100.0)

        # PWM + направление
        if duty > 0:
            GPIO.output(in1, GPIO.HIGH); GPIO.output(in2, GPIO.LOW)
        elif duty < 0:
            GPIO.output(in1, GPIO.LOW);  GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW);  GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(abs(duty))

        fb_pub.publish(Float32MultiArray(data=[alpha_d, duty]))

    def _brake(self, in1: int, in2: int, pwm: GPIO.PWM, label: str) -> None:
        GPIO.output(in1, GPIO.LOW); GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0.0)
        self.get_logger().debug(f"{label} motor stopped.")

    def _hard_stop(self) -> None:
        self._brake(self.BIN1, self.BIN2, self.pwm_l, "left ")
        self._brake(self.AIN1, self.AIN2, self.pwm_r, "right")

    # ───────────────────────── shutdown ────────────────────────────
    def destroy_node(self) -> None:
        self.get_logger().info("MotorController shutting down.")
        self._hard_stop()
        self.pwm_r.stop()
        self.pwm_l.stop()
        GPIO.output(self.STBY, GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()


# ─────────────────────────── MAIN ──────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
motor_controller_node.py
────────────────────────
Узел‑драйвер двигателей.  Подписывается на:

  • /wheel_pwm               (Float32MultiArray)  – [ duty_left, duty_right ]
  • /mpu6050/filtered_data   (Float32MultiArray)  – наклон робота / прочие данные

— duty ∈ [‑1 … 1] → направление + скважность PWM.
— Экстренный стоп, если |tilt| (первый элемент IMU‑сообщения, °) > critical_angle.

Изменение по сравнению с предыдущей версией
-------------------------------------------
•  Теперь угол наклона берётся из msg.data[0] и уже выражен в градусах –
   никакой конвертации через `math.degrees` не требуется.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import RPi.GPIO as GPIO


class MotorController(Node):
    # ────────────────────────── INIT ────────────────────────── #
    def __init__(self) -> None:
        super().__init__("motor_controller")

        # Параметры
        self.declare_parameters(
            "",
            [
                ("critical_angle", 40.0),       # °  – порог аварийного останова
                ("pwm_frequency",  7000),       # Гц
                ("pwm_topic",      "wheel_pwm"),
                ("imu_topic",      "mpu6050/filtered_data"),
            ],
        )
        self.crit_angle_deg: float = self.get_parameter("critical_angle").value
        self.emergency_stop: bool = False

        # Пины (BCM)
        self.PWMA, self.AIN1, self.AIN2 = 13, 27, 17   # правый мотор
        self.PWMB, self.BIN1, self.BIN2 = 12, 14, 4    # левый  мотор
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

        # Подписки
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("pwm_topic").value,
            self._wheel_pwm_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("imu_topic").value,
            self._imu_cb,
            10,
        )

        self.get_logger().info("MotorController initialised.")

    # ──────────────────── CALLBACKS ──────────────────── #
    def _imu_cb(self, msg: Float32MultiArray) -> None:
        """
        Tilt‑angle (°) находится в msg.data[0].
        Если |tilt| > critical_angle → emergency_stop.
        """
        if not msg.data:
            return

        tilt_deg = msg.data[0]

        if abs(tilt_deg) > self.crit_angle_deg:
            if not self.emergency_stop:
                self.get_logger().warn(
                    f"E‑STOP: |tilt| = {tilt_deg:.1f}°  > {self.crit_angle_deg}°"
                )
            self.emergency_stop = True
        else:
            if self.emergency_stop:
                self.get_logger().info("Tilt normalised, E‑STOP cleared.")
            self.emergency_stop = False

    def _wheel_pwm_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().error("wheel_pwm must contain 2 elements.")
            return

        d_left, d_right = float(msg.data[0]), float(msg.data[1])

        if self.emergency_stop:
            self._stop(self.BIN1, self.BIN2, self.pwm_l, "left")
            self._stop(self.AIN1, self.AIN2, self.pwm_r, "right")
            return

        self._apply(self.BIN1, self.BIN2, self.pwm_l,  d_left)
        self._apply(self.AIN1, self.AIN2, self.pwm_r, d_right)

    # ───────────── LOW‑LEVEL PWM/DIR ───────────── #
    def _apply(self, in1: int, in2: int, pwm: GPIO.PWM, duty: float) -> None:
        duty = max(min(duty, 1.0), -1.0)
        if duty > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif duty < 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(abs(duty) * 100.0)

    def _stop(self, in1: int, in2: int, pwm: GPIO.PWM, label: str) -> None:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0.0)
        self.get_logger().debug(f"{label} motor stopped (E‑STOP).")

    # ─────────────────── SHUTDOWN ─────────────────── #
    def destroy(self) -> None:
        self.get_logger().info("MotorController shutting down.")
        self.pwm_r.stop()
        self.pwm_l.stop()
        GPIO.output(self.STBY, GPIO.LOW)
        GPIO.cleanup()


# ────────────────────────── MAIN ───────────────────────── #
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

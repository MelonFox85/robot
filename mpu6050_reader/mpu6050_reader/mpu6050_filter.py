#!/usr/bin/env python3
"""
ROS 2 node:
    /mpu6050/data   (Float32MultiArray [ax ay az gx gy gz])  →  комплементарный фильтр → 
    /mpu6050/filtered_data [tilt_deg, ω_body_y, ω_plane_z]

•  Шаг интегрирования dt и коэффициент α подстраиваются к реальной частоте прихода
   сообщений (рассчитываются из ROS‑меток времени).
•  Постоянная времени τ определяет, насколько «инерционным» будет фильтр
   (чем меньше τ, тем сильнее верим акселерометру).
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray


class MPU6050FilterNode(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050_filter")

        # ────────── ROS I/O ───────────────────────────────────────────────
        self.sub = self.create_subscription(
            Float32MultiArray, "mpu6050/data", self.mpu_cb, 20
        )
        self.pub = self.create_publisher(
            Float32MultiArray, "mpu6050/filtered_data", 20
        )

        # ────────── Параметры фильтра ─────────────────────────────────────
        self.tau = 0.05           # [с] const time ≈ «склонность» к акселю (0.05 s → α≈0.83 при dt=0.01)
        self.filtered_angle = 0.0
        self.prev_stamp: Optional[Time] = None

        self.get_logger().info(
            f"Complementary filter ready (τ = {self.tau*1e3:.0f} ms)."
        )

    # ────────────────────────────────────────────────────────────────────
    def mpu_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            self.get_logger().error("mpu6050/data must contain 6 floats.")
            return

        ax, ay, az, gx_deg, gy_deg, gz_deg = msg.data[:6]

        # 1. dt из ROS‑меток
        now = self.get_clock().now()
        if self.prev_stamp is None:          # первый пакет — нечего сравнивать
            self.prev_stamp = now
            return
        dt = (now - self.prev_stamp).nanoseconds * 1e-9
        self.prev_stamp = now
        if dt <= 0.0:
            return                           # защита от некорректных меток

        # 2. α = τ / (τ + dt)  — классическая формула комплементарного фильтра
        alpha = self.tau / (self.tau + dt)

        # 3. Угол по акселерометру
        accel_angle = math.degrees(
            math.atan2(ax, math.sqrt(ay * ay + az * az))
        )

        # 4. Угол по гироскопу (интеграл)
        gyro_angle = self.filtered_angle + gy_deg * dt

        # 5. Комплементарное объединение
        self.filtered_angle = alpha * gyro_angle + (1.0 - alpha) * accel_angle

        # 6. Выходные величины
        out = Float32MultiArray()
        out.data = [
            self.filtered_angle,  # наклон (pitch) в °   — δ2 для LQR
            gy_deg,               # ω_y тела             — δ3
            gz_deg,               # ω_z плоскости        — δ5
        ]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050FilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

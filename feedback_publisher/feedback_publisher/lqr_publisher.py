#!/usr/bin/env python3
"""
Формирует δ-вектор (ошибку состояния) с частотой `control_rate`.

Изменения относительно исходника:
• Используются часы ROS2 (`self.get_clock().now()`), поэтому dt синхронизирован
  со всем графом и корректно работает в симуляции.
• Добавлены параметры `wheel_radius`, `wheel_base`, проверка их валидности.
• QoS `depth=1` – не накапливаем устаревшие сообщения.
• Обработаны крайние случаи: пустая команда, dt ≤ 0, некорректная длина msg.
• Имя узла и выходного топика унифицировано: `lqr_feedback`.
• Типовые аннотации повышают читаемость, а логика выделена в функции.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, Float32MultiArray


class LQRFeedback(Node):
    def __init__(self) -> None:
        super().__init__("lqr_feedback")

        # ─────────── параметры ────────────────────────────────────────
        self.declare_parameter("control_rate", 400.0)
        self.declare_parameter("wheel_radius", 0.0325)  # [m]
        self.declare_parameter("wheel_base", 0.1587)    # [m]
        self.declare_parameter("state_size", 6)

        self.control_rate: float = (
            self.get_parameter("control_rate").get_parameter_value().double_value
        )
        self.control_rate = max(self.control_rate, 1.0)  # защита
        self.dt_nom: float = 1.0 / self.control_rate

        self.r: float = self.get_parameter(
            "wheel_radius"
        ).get_parameter_value().double_value
        self.base: float = self.get_parameter(
            "wheel_base"
        ).get_parameter_value().double_value

        self.state_size: int = int(
            self.get_parameter("state_size").get_parameter_value().integer_value
        )
        if self.state_size < 1:
            self.get_logger().warning("state_size < 1, исправляю на 6")
            self.state_size = 6

        # ─────────── подписки ─────────────────────────────────────────
        qos = QoSProfile(depth=1)

        self.create_subscription(Float32MultiArray, "cmd", self._cmd_cb, qos)
        self.create_subscription(Float32, "encoders_data/left", self._enc_left_cb, qos)
        self.create_subscription(Float32, "encoders_data/right", self._enc_right_cb, qos)
        self.create_subscription(
            Float32MultiArray, "mpu6050/filtered_data", self._imu_cb, qos
        )

        # ─────────── публикация ───────────────────────────────────────
        self.pub = self.create_publisher(Float32MultiArray, "lqr_feedback", qos)
        self.create_timer(self.dt_nom, self._publish)

        # ─────────── внутреннее состояние ─────────────────────────────
        self.cmd: Optional[List[float]] = None
        self.state: List[float] = [0.0] * self.state_size

        self._prev_stamp = self.get_clock().now()

        # последние угловые скорости колёс (rev/s)
        self.w_l: float = 0.0
        self.w_r: float = 0.0

    # =================================================================
    #                           callbacks
    # =================================================================
    def _cmd_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) != self.state_size:
            self.get_logger().warn(
                f"cmd длины {len(data)} не совпадает с state_size={self.state_size}"
            )
        self.cmd = data

    def _enc_left_cb(self, msg: Float32) -> None:
        self.w_l = msg.data

    def _enc_right_cb(self, msg: Float32) -> None:
        self.w_r = msg.data

    def _imu_cb(self, msg: Float32MultiArray) -> None:
        """msg.data = [θ_rad, θ̇_rad_s, ψ̇_rad_s]."""
        if len(msg.data) < 3:
            self.get_logger().warn("/mpu6050/filtered_data too short")
            return
        θ, θdot, ψdot = msg.data[:3]
        self.state[2] = θ
        self.state[3] = θdot
        self.state[5] = ψdot      # ← больше не будет затираться

    # =================================================================
    #                      вспомогательные функции
    # =================================================================
    def _propagate_kinematics(self, dt: float) -> None:
        v_l = self.w_l * 2.0 * math.pi * self.r
        v_r = self.w_r * 2.0 * math.pi * self.r
        v = 0.5 * (v_l + v_r)
        ψdot = (v_r - v_l) / self.base

        self.state[0] += v * dt
        self.state[4] += ψdot * dt

        self.state[1] = v
        # state[5] (ψ̇) оставляем как пришло с IMU
    # =================================================================
    def _publish(self) -> None:
        if self.cmd is None:
            # Нет целевого вектора – ничего не публикуем
            return

        # вычисление реального dt по системным часам ROS
        now = self.get_clock().now()
        dt = (now - self._prev_stamp).nanoseconds * 1e-9
        self._prev_stamp = now

        if dt <= 0.0:
            return

        self._propagate_kinematics(dt)

        # «выравниваем» размеры, чтобы zip не урезал лишнее
        cmd_padded = (self.cmd + [0.0] * self.state_size)[: self.state_size]
        delta = [c - s for c, s in zip(cmd_padded, self.state)]

        self.pub.publish(Float32MultiArray(data=delta))

        self.get_logger().debug(
            f"dt={dt*1e3:.2f} ms  δ={', '.join(f'{x:+.3f}' for x in delta)}"
        )


# =====================================================================
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LQRFeedback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
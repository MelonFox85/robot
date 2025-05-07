#!/usr/bin/env python3
"""
Измерение угловой скорости правого колеса.

• Счёт импульсов от инкрементного энкодера через pigpio-callback.
• Каждые dt = 1 / control_rate секунд публикуется ω  [rev/s].
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

import pigpio


GPIO_A, GPIO_B = 8, 25   # выводы энкодера (правое колесо)


class RightEncoder(Node):
    def __init__(self) -> None:
        super().__init__("right_encoder")

        # ─────────── параметры ────────────────────────────────────────
        self.declare_parameter("control_rate", 400.0)
        self.declare_parameter("ticks_per_rev", 390)

        self.control_rate: float = (
            self.get_parameter("control_rate").get_parameter_value().double_value
        )
        if self.control_rate <= 0.0:
            self.get_logger().warning("control_rate ≤ 0, устанавливаю 400 Гц")
            self.control_rate = 400.0
        self.dt: float = 1.0 / self.control_rate

        self.ticks_per_rev: int = int(
            self.get_parameter("ticks_per_rev").get_parameter_value().integer_value
        )
        if self.ticks_per_rev <= 0:
            self.get_logger().warning("ticks_per_rev ≤ 0, устанавливаю 390")
            self.ticks_per_rev = 390

        # ─────────── pigpio ───────────────────────────────────────────
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal("Не удалось подключиться к pigpio-демону.")
            raise SystemExit(1)

        for gpio in (GPIO_A, GPIO_B):
            self.pi.set_mode(gpio, pigpio.INPUT)
            self.pi.set_pull_up_down(gpio, pigpio.PUD_UP)

        # ─────────── квадратурный счётчик ────────────────────────────
        self._pos: int = 0
        self._last_pos: int = 0

        self._state: int = (self.pi.read(GPIO_A) << 1) | self.pi.read(GPIO_B)

        self.pi.callback(GPIO_A, pigpio.EITHER_EDGE, self._cb)
        self.pi.callback(GPIO_B, pigpio.EITHER_EDGE, self._cb)

        # ─────────── ROS ──────────────────────────────────────────────
        qos = QoSProfile(depth=1)
        self.pub = self.create_publisher(Float32, "encoders_data/right", qos)

        self._prev_time = self.get_clock().now()
        self.create_timer(self.dt, self._publish_speed)

    # -----------------------------------------------------------------
    def _cb(self, gpio: int, level: int, tick: int) -> None:
        """Квадратурное декодирование таблицей переходов."""
        a = self.pi.read(GPIO_A)
        b = self.pi.read(GPIO_B)
        state = (a << 1) | b

        delta = (state - self._state) & 0x3  # mod 4
        if delta == 1:
            self._pos += 1
        elif delta == 3:
            self._pos -= 1
        # delta == 0 или 2 — игнорируем (дребезг/пропуск)

        self._state = state

    # -----------------------------------------------------------------
    def _publish_speed(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        if dt <= 0.0:
            return

        delta_ticks = self._pos - self._last_pos
        self._last_pos = self._pos

        rev_per_sec = delta_ticks / self.ticks_per_rev / dt
        self.pub.publish(Float32(data=float(rev_per_sec)))

        self.get_logger().debug(f"ω_R = {rev_per_sec:+.3f} rev/s")

    # -----------------------------------------------------------------
    def destroy_node(self) -> None:
        self.pi.stop()
        super().destroy_node()


# =====================================================================
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RightEncoder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
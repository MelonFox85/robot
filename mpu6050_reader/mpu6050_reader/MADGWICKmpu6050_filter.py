#!/usr/bin/env python3
"""
ROS 2 node: MPU‑6050 → Madgwick 6‑DoF filter.
Publishes [pitch, ω_body_y, ω_plane_z].

Совместим и с ahrs‑0.5+ (новый API), и со старыми ahrs‑0.4.*
"""

from __future__ import annotations
import math
from typing import Optional, Callable

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from ahrs.filters import Madgwick


def quat_to_pitch(q: np.ndarray) -> float:
    """Pitch (deg) из кватерниона [w, x, y, z]."""
    w, x, y, z = q
    return math.degrees(math.asin(-2.0 * (x * z - w * y)))


class MadgwickFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050_madgwick_filter")

        # ROS I/O
        self.sub = self.create_subscription(
            Float32MultiArray, "mpu6050/data", self.sensor_cb, 20
        )
        self.pub = self.create_publisher(
            Float32MultiArray, "mpu6050/filtered_data", 20
        )

        # Madgwick
        beta = 0.15
        self.madgwick = Madgwick(beta=beta)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        # Определяем, какой именно сигнатурой обладает updateIMU()
        self._update_fn: Callable
        try:
            # Попытка вызова «новой» сигнатуры (v0.5+)
            _ = self.madgwick.updateIMU(gyr=[0, 0, 0], acc=[0, 0, 1])
            self._update_fn = self._update_new
            self.get_logger().info("ahrs>=0.5 detected (no q arg).")
        except TypeError:
            # Откат на старую (v0.4.x)
            self._update_fn = self._update_old
            self.get_logger().info("ahrs<=0.4 detected (requires q arg).")

        self.prev_stamp: Optional[Time] = None
        self.get_logger().info(f"Madgwick filter node started (β = {beta:.3f}).")

    # ------------------------------------------------------------------
    def _update_new(self, gx: float, gy: float, gz: float,
                    ax: float, ay: float, az: float) -> None:
        """Для ahrs‑0.5+: q возвращается, q не передаётся."""
        q_new = self.madgwick.updateIMU(gyr=[gx, gy, gz], acc=[ax, ay, az])
        if q_new is not None:
            self.q = q_new / np.linalg.norm(q_new)

    def _update_old(self, gx: float, gy: float, gz: float,
                    ax: float, ay: float, az: float) -> None:
        """Для ahrs‑0.4.*: q нужно передать первым аргументом."""
        q_new = self.madgwick.updateIMU(self.q, gyr=[gx, gy, gz], acc=[ax, ay, az])
        if q_new is not None:
            self.q = q_new / np.linalg.norm(q_new)

    # ------------------------------------------------------------------
    def sensor_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            self.get_logger().error("Sensor message must contain 6 floats (ax ay az gx gy gz).")
            return
        ax, ay, az, gx_d, gy_d, gz_d = msg.data[:6]

        # Δt
        now = self.get_clock().now()
        if self.prev_stamp is None:
            self.prev_stamp = now
            return
        dt = (now - self.prev_stamp).nanoseconds * 1e-9
        self.prev_stamp = now
        if dt <= 0.0:
            return

        self.madgwick.sample_period = dt

        # deg/s → rad/s
        gx, gy, gz = np.deg2rad([gx_d, gy_d, gz_d])

        # вызов подходящего варианта updateIMU
        self._update_fn(gx, gy, gz, ax, ay, az)

        # публикация
        pitch = quat_to_pitch(self.q)
        out = Float32MultiArray()
        out.data = [pitch, gy_d, gz_d]
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MadgwickFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

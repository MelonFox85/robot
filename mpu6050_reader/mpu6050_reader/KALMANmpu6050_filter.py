#!/usr/bin/env python3
"""
ROS 2 node that fuses MPU‑6050 accelerometer and gyroscope data
with a (linearised) 2‑state Kalman filter.

State vector:  x = [θ, b]^T
   θ – body tilt angle (deg)
   b – gyro bias around y‑axis (deg/s)

System model:
   θ_k = θ_{k-1} + (ω_gyr_y − b_{k-1})·dt
   b_k = b_{k-1}

Measurement model:
   z_k = θ_k + v_k            (θ from accelerometer)

All matrices are rebuilt every cycle using the real Δt that is
calculated from ROS time stamps, eliminating artificial latency.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray


def accel_to_angle(ax: float, ay: float, az: float) -> float:
    """
    Convert accelerometer readings to tilt (pitch) angle around y‑axis.
    Result in degrees to match common control‑loop units.
    """
    return math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))


class KalmanTiltFilter(Node):
    """
    Two‑state Kalman filter for fast tilt estimation.
    Publishes [θ, ω_body, ω_plane] – совместимо с вашим LQR‑контроллером.
    """

    def __init__(self) -> None:
        super().__init__("mpu6050_kalman_filter")

        # ROS I/O -----------------------------------------------------------
        self.sub = self.create_subscription(
            Float32MultiArray, "mpu6050/data", self.sensor_cb, 10
        )
        self.pub = self.create_publisher(
            Float32MultiArray, "mpu6050/filtered_data", 10
        )

        # Kalman matrices ---------------------------------------------------
        # x̂ = [θ; b]
        self.x_hat = np.zeros((2, 1))       # initial state
        self.P = np.diag([10.0, 1.0])       # initial covariance

        # Noise parameters (tunable in runtime if нужно) -------------------
        self.Q_angle = 0.2   # (deg²)/s  – model noise for θ
        self.Q_bias = 0.01   # (deg/s)²·s – model noise for bias
        self.R_angle = 5.0   # deg²     – measurement noise

        self.H = np.array([[1.0, 0.0]])     # measurement matrix (1×2)

        # Internal ----------------------------------------------------------
        self.prev_stamp: Optional[Time] = None
        self.get_logger().info("Kalman tilt‑filter node started.")

    # ----------------------------------------------------------------------
    def sensor_cb(self, msg: Float32MultiArray) -> None:
        # Expecting (ax, ay, az, gx, gy, gz)
        if len(msg.data) < 6:
            self.get_logger().error("Sensor message must contain 6 floats.")
            return
        ax, ay, az, gx, gy, gz = msg.data[:6]

        # --- Δt ------------------------------------------------------------
        now = self.get_clock().now()
        if self.prev_stamp is None:
            self.prev_stamp = now
            return                            # need two samples to start
        dt = (now - self.prev_stamp).nanoseconds * 1e-9
        self.prev_stamp = now
        if dt <= 0.0:
            return

        # ------------------------------------------------------------------
        # 1. Predict
        # ------------------------------------------------------------------
        F = np.array([[1.0, -dt],
                      [0.0,  1.0]])
        B = np.array([[dt],
                      [0.0]])
        u = np.array([[gy]])                 # control: raw gyro y (deg/s)

        self.x_hat = F @ self.x_hat + B @ u

        Q = np.array([[self.Q_angle * dt, 0.0],
                      [0.0,              self.Q_bias * dt]])
        self.P = F @ self.P @ F.T + Q

        # ------------------------------------------------------------------
        # 2. Update
        # ------------------------------------------------------------------
        z = np.array([[accel_to_angle(ax, ay, az)]])  # measurement (deg)
        y = z - self.H @ self.x_hat                   # innovation
        S = self.H @ self.P @ self.H.T + self.R_angle
        K = self.P @ self.H.T / S                     # Kalman gain (2×1)

        self.x_hat += K * y
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P

        # ------------------------------------------------------------------
        out = Float32MultiArray()
        out.data = [
            float(self.x_hat[0, 0]),  # filtered tilt angle
            gy,                       # body angular rate (raw gyro y)
            gz                        # plane rotation (raw gyro z)
        ]
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KalmanTiltFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

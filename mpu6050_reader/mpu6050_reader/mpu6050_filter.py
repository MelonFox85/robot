#!/usr/bin/env python3
"""
/mpu6050/raw → /mpu6050/filtered_data
data = [ tilt_rad, gyro_y_rad_s, gyro_z_rad_s ]
"""
from __future__ import annotations
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Complementary(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050_filter")

        self.declare_parameters(
            "",
            [
                ("tau", 0.02),          # с
                ("gyro_clip", 30.0),    # rad/s
                ("invert_acc_x", False),
                ("invert_acc_y", False),
                ("invert_acc_z", False),
                ("invert_gyro_y", False),
                ("invert_gyro_z", False),
            ],
        )

        p = self.get_parameter
        self.tau = float(p("tau").value)
        self.clip = float(p("gyro_clip").value)

        # флаги инверсии читаем один раз
        self.inv_ax = bool(p("invert_acc_x").value)
        self.inv_ay = bool(p("invert_acc_y").value)
        self.inv_az = bool(p("invert_acc_z").value)
        self.inv_gy = bool(p("invert_gyro_y").value)
        self.inv_gz = bool(p("invert_gyro_z").value)

        qos = rclpy.qos.QoSProfile(depth=20)
        self.create_subscription(Float32MultiArray, "mpu6050/raw", self._cb, qos)
        self.pub = self.create_publisher(Float32MultiArray, "mpu6050/filtered_data", qos)

        self.angle = 0.0
        self.prev_ns: int | None = None

        self.get_logger().info(f"Complementary filter τ = {self.tau*1e3:.0f} ms")

    # -----------------------------------------------------------------
    def _cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            return

        ax, ay, az, gx, gy, gz = msg.data[:6]

        # apply sign conventions
        if self.inv_ax: ax = -ax
        if self.inv_ay: ay = -ay
        if self.inv_az: az = -az
        if self.inv_gy: gy = -gy
        if self.inv_gz: gz = -gz

        # dt from publisher (ns) or local clock
        now_ns = msg.layout.data_offset or self.get_clock().now().nanoseconds
        if self.prev_ns is None:
            self.prev_ns = now_ns
            return
        dt = (now_ns - self.prev_ns) * 1e-9
        self.prev_ns = now_ns
        if dt <= 0.0:
            return

        # complementary filter
        alpha = self.tau / (self.tau + dt)
        accel_ang = math.atan2(ax, math.sqrt(ay * ay + az * az))
        gyro_ang  = self.angle + gy * dt
        self.angle = alpha * gyro_ang + (1.0 - alpha) * accel_ang

        gy = max(min(gy, self.clip), -self.clip)
        gz = max(min(gz, self.clip), -self.clip)

        self.pub.publish(Float32MultiArray(data=[self.angle, gy, gz]))


def main() -> None:
    rclpy.init()
    node = Complementary()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
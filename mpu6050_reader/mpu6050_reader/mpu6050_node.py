#!/usr/bin/env python3
"""
ROS 2 node that reads MPU‑6050 at 1 kHz and publishes
[ax, ay, az, gx, gy, gz]  (g / ° s‑1) in topic  /mpu6050/data.
"""

from __future__ import annotations

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus

# ---------------------------------------------------------------------#
MPU6050_ADDR   = 0x68
ACCEL_XOUT_H   = 0x3B

PWR_MGMT_1     = 0x6B
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C

ACCEL_SENS = 16384.0            # ±2 g  -> g/LSB
GYRO_SENS  = 131.0              # ±250 °/s -> °/s / LSB
# ---------------------------------------------------------------------#


class MPU6050Node(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050_node_1khz")

        # I²C -------------------------------------------------------------
        self.bus = SMBus(1)
        self._mpu_init()

        # Publisher (QoS depth = 1 – только самое свежее)
        self.pub = self.create_publisher(Float32MultiArray, "mpu6050/data", 1)

        # «ручной» троттлинг лог‑сообщений
        self.last_error_log: Time | None = None

        # 1 kHz таймер
        self.create_timer(0.001, self._read_and_publish)

        self.get_logger().info("MPU6050 node started at 1 kHz.")

    # ------------------------------------------------------------------
    def _mpu_init(self) -> None:
        """Конфигурация датчика под частоту 1 kHz."""
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)   # wake‑up
        time.sleep(0.05)

        self.bus.write_byte_data(MPU6050_ADDR, CONFIG, 0x01)       # DLPF=184 Hz
        self.bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 0x00)   # 1 kHz/(1+0)
        self.bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00) # ±2 g
        self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)  # ±250 °/s

        self.get_logger().info("MPU6050 configured: Fs = 1 kHz, DLPF = 184 Hz.")

    # ------------------------------------------------------------------
    def _read_and_publish(self) -> None:
        try:
            raw = self.bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
            if len(raw) != 14:
                raise IOError(f"expected 14 bytes, got {len(raw)}")

            ax, ay, az, _temp, gx, gy, gz = struct.unpack(">hhhhhhh", bytes(raw))

            msg = Float32MultiArray()
            msg.data = [
                ax / ACCEL_SENS,
                ay / ACCEL_SENS,
                az / ACCEL_SENS,
                gx / GYRO_SENS,
                gy / GYRO_SENS,
                gz / GYRO_SENS,
            ]
            self.pub.publish(msg)

        except Exception as exc:
            # выводим предупреждение не чаще раза в 5 с
            now = self.get_clock().now()
            if self.last_error_log is None or \
               (now - self.last_error_log).nanoseconds > 5e9:
                self.get_logger().warn(f"I²C read error: {exc}")
                self.last_error_log = now

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self.bus.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

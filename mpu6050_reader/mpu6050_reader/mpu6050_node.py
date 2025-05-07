#!/usr/bin/env python3
"""
MPU-6050 → /mpu6050/raw
data = [ax_g, ay_g, az_g, gx_rad_s, gy_rad_s, gz_rad_s]
"""

from __future__ import annotations
import struct, time, statistics, collections
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus

MPU, ACC_H = 0x68, 0x3B
PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CFG, ACC_CFG = 0x6B, 0x19, 0x1A, 0x1B, 0x1C
ACC_SENS, G_SENS_DPS = 16384.0, 131.0
DEG2RAD = 0.017453292519943295

class MPU6050(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050_reader_node")

        self.bus = SMBus(1)
        self._init_chip()

        qos = rclpy.qos.QoSProfile(depth=20)
        self.raw_pub = self.create_publisher(Float32MultiArray, "mpu6050/raw", qos)

        self.bias_buf = collections.deque(maxlen=400)
        self.gyro_bias = [0.0, 0.0, 0.0]

        self.create_timer(0.001, self._tick)  # 1 kHz

        self.get_logger().info("MPU-6050 node – 1 kHz, rad/s output.")

    # ---------------------------------------------------------------
    def _init_chip(self) -> None:
        self.bus.write_byte_data(MPU, PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self.bus.write_byte_data(MPU, CONFIG, 0x01)
        self.bus.write_byte_data(MPU, SMPLRT_DIV, 0x00)
        self.bus.write_byte_data(MPU, ACC_CFG, 0x00)
        self.bus.write_byte_data(MPU, GYRO_CFG, 0x00)

    # ---------------------------------------------------------------
    def _tick(self) -> None:
        raw = self.bus.read_i2c_block_data(MPU, ACC_H, 14)
        ax, ay, az, _, gx, gy, gz = struct.unpack(">hhhhhhh", bytes(raw))

        gx_r, gy_r, gz_r = (v / G_SENS_DPS * DEG2RAD for v in (gx, gy, gz))

        if len(self.bias_buf) < self.bias_buf.maxlen:
            self.bias_buf.append((gx_r, gy_r, gz_r))
            if len(self.bias_buf) == self.bias_buf.maxlen:
                self.gyro_bias = [statistics.mean(c[i] for c in self.bias_buf) for i in range(3)]
                self.get_logger().info(f"Gyro bias = {self.gyro_bias}")
            return

        gx_r -= self.gyro_bias[0]
        gy_r -= self.gyro_bias[1]
        gz_r -= self.gyro_bias[2]

        self.raw_pub.publish(
            Float32MultiArray(
                data=[
                    ax / ACC_SENS,
                    ay / ACC_SENS,
                    az / ACC_SENS,
                    gx_r,
                    gy_r,
                    gz_r,
                ]
            )
        )

    # ---------------------------------------------------------------
    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = MPU6050()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
LQR-controller → desired wheel ACCELERATION (α*, rad/s²).

Input:
    /lqr_feedback            – Float32MultiArray, 6-el. δ-vector
Output:
    /motor_cmd/left          – Float32, α_L*  [rad/s²]
    /motor_cmd/right         – Float32, α_R*  [rad/s²]
"""

from __future__ import annotations
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, Float32MultiArray


class LQRTorqueController(Node):
    def __init__(self) -> None:
        super().__init__("lqr_torque_controller")

        # ── параметры ────────────────────────────────────────────────
        self.declare_parameter(
            "K", [22.3607, 43.2739, 468.4965, 42.1292, 22.3607, 6.1385]
        )
        self.declare_parameter("max_alpha", 200.0)   # |α|max, rad/s²
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)

        K_raw = self.get_parameter("K").get_parameter_value().double_array_value
        if len(K_raw) != 6:
            self.get_logger().fatal("Parameter K must contain 6 elements.")
            raise SystemExit(1)
        self.K = tuple(float(k) for k in K_raw)

        self.max_alpha: float = float(self.get_parameter("max_alpha").value)
        self.inv_l: bool = bool(self.get_parameter("invert_left").value)
        self.inv_r: bool = bool(self.get_parameter("invert_right").value)

        qos = QoSProfile(depth=1)

        self.create_subscription(
            Float32MultiArray, "lqr_feedback", self._fb_cb, qos
        )
        # ------------- ИМЕНА ТОПИКОВ ИЗМЕНЕНЫ -------------
        self.left_pub  = self.create_publisher(Float32, "motor_cmd/left",  qos)
        self.right_pub = self.create_publisher(Float32, "motor_cmd/right", qos)

        self.get_logger().info("LQR torque-controller ready.")

    # -----------------------------------------------------------------
    def _fb_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) != 6 or any(math.isinf(x) or math.isnan(x) for x in data):
            self.get_logger().error("lqr_feedback must be a 6-element sane vector.")
            return

        K1, K2, K3, K4, K5, K6 = self.K
        d0, d1, d2, d3, d4, d5 = data

        # u = −K·δ
        alpha_L = (K1*d0 + K2*d1 + K3*d2 + K4*d3 + K5*d4 + K6*d5)
        alpha_R = (K1*d0 + K2*d1 + K3*d2 + K4*d3 - K5*d4 - K6*d5)

        if self.inv_l:
            alpha_L = -alpha_L
        if self.inv_r:
            alpha_R = -alpha_R

        alpha_L = max(min(alpha_L, self.max_alpha), -self.max_alpha)
        alpha_R = max(min(alpha_R, self.max_alpha), -self.max_alpha)

        self.left_pub.publish (Float32(data=float(alpha_L)))
        self.right_pub.publish(Float32(data=float(alpha_R)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LQRTorqueController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
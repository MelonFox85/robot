#!/usr/bin/env python3
"""
LQR‑контроллер → PWM‑доля (‑1 … +1).

Главная идея: матрица K после умножения на k_scale сразу даёт значение,
нормированное на напряжение питания, поэтому duty меняется плавно,
а не «залипает» на насыщении.
"""

from __future__ import annotations
import typing as _t
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from scipy import linalg
except ImportError:
    linalg = None


class LQRController(Node):
    def __init__(self) -> None:
        super().__init__("lqr_controller")

        # ─────────── параметры ─────────── #
        self.declare_parameters(
            "",
            [
                ("update_rate", 100.0),
                ("voltage_supply", 12.0),
                ("v_sat", 0.95),          # |duty| ≤ v_sat
                ("k_scale", 0.0),         # 0 ⇒ подобрать автоматически
                ("diagnostics", True),

                # модель (для расчёта K)
                ("wheel_radius", 0.065/2),
                ("d", 0.1587),
                ("m", 0.034),
                ("M", 1.14 - 2*0.034),
                ("L", 0.5*0.119),
                ("g", 9.81),

                # топики
                ("feedback_topic", "lqr_feedback"),
                ("pwm_topic", "wheel_pwm"),
            ],
        )
        self.diagnostics = bool(self.get_parameter("diagnostics").value)

        # ─────────── LQR gain ─────────── #
        K_volt = self._compute_K()             # вольты
        k_scale = self.get_parameter("k_scale").value
        if k_scale == 0.0:                     # автоподбор
            k_scale = self._auto_scale(K_volt)
        self.K = k_scale * K_volt              # сразу в долях питания

        self._print_gain(self.K, k_scale)

        # ─────────── ROS I/O ─────────── #
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("feedback_topic").value,
            self._feedback_cb, 10)

        self.pwm_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter("pwm_topic").value,
            10)

        self.get_logger().info("LQRController ready.")

    # ================================================================== #
    #                        LQR  gain  (вольты)                         #
    # ================================================================== #
    def _compute_K(self) -> np.ndarray:
        if linalg is None:
            self.get_logger().warn("SciPy not найден – K по умолчанию.")
            return np.array(
                [[-22.36, -43.27, -468.94, -42.14,  22.36,  6.14],
                 [-22.36, -43.27, -468.94, -42.14, -22.36, -6.14]]
            )

        p = self.get_parameter
        m, r, M, L, d, g = [p(x).value for x in
                            ("m", "wheel_radius", "M", "L", "d", "g")]
        i  = 0.5 * m * r**2
        Jp = (1/12) * M * (0.164**2 + 0.064**2)
        Jd = (1/12) * M * (0.119**2 + 0.064**2)

        Qeq = Jp*M + (Jp + M*L**2)*(2*m + 2*i/r**2)
        A23 = -(M**2 * L**2 * g) / Qeq
        A43 =  (M*L*g*(M + 2*m + 2*i/r**2)) / Qeq
        B21 = (Jp + M*L**2 + M*L*r) / (Qeq*r)
        B41 = -(M*L/r + M + 2*m + 2*i/r**2) / Qeq
        B61 = 1 / (r*(m*d + i*d/r**2 + 2*Jd/d))

        A = np.array([[0,1,0,0,0,0],
                      [0,0,A23,0,0,0],
                      [0,0,0,1,0,0],
                      [0,0,A43,0,0,0],
                      [0,0,0,0,0,1],
                      [0,0,0,0,0,0]])
        B = (i/r)*np.array([[0,0],
                            [B21,B21],
                            [0,0],
                            [B41,B41],
                            [0,0],
                            [B61,-B61]])

        Q = np.diag([200,0,0,200,50,0])   # умеренные веса
        R = np.diag([5,5])                # повысили R → мягче K
        P = linalg.solve_continuous_are(A, B, Q, R)
        return np.linalg.inv(R) @ (B.T @ P)

    # ================================================================== #
    #                   Автоматическое масштабирование                   #
    # ================================================================== #
    def _auto_scale(self, K_v: np.ndarray) -> float:
        """
        Подбираем коэффициент так, чтобы при 'разумной' максимальной ошибке
        |duty| ≈ v_sat.  Это простая грубая оценка, но работает.
        """
        # грубая оценка предельной ошибки (можно подстроить)
        e_max = np.array([0.5,   # θ   ~ 0.5 рад (28°)
                          0.0,   # θ_dot
                          1.0,   # φ   ~ 1 рад (57°)
                          5.0,   # φ_dot
                          0.2,   # x   ~ 0.2 м
                          2.0])  # x_dot
        U_pred = np.max(np.abs(K_v @ e_max))
        v_sat  = float(self.get_parameter("v_sat").value) \
               * float(self.get_parameter("voltage_supply").value)
        k_scale = v_sat / U_pred if U_pred > 1e-6 else 1.0
        self.get_logger().info(
            f"Auto‑scale K: |U_pred|={U_pred:.1f} V  -> k_scale={k_scale:.4f}"
        )
        return k_scale

    # ================================================================== #
    #                             CALLBACK                               #
    # ================================================================== #
    def _feedback_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 6:
            self.get_logger().error("feedback size must be 6")
            return

        e = np.asarray(msg.data).reshape((6,1))
        duty = (self.K @ e).flatten()                 # уже нормировано
        sat  = float(self.get_parameter("v_sat").value)
        duty = np.clip(duty, -sat, sat)

        if self.diagnostics:
            self.get_logger().info(
                f"e={np.round(e.flatten(),3)} duty=[{duty[0]:.3f},{duty[1]:.3f}]"
            )

        out = Float32MultiArray()
        out.data = duty.astype(np.float32).tolist()
        self.pwm_pub.publish(out)

    # ================================================================== #
    def _print_gain(self, K: np.ndarray, k_scale: float) -> None:
        lines = [f"K (scaled), k_scale={k_scale:.4f}:"]
        for r in K:
            lines.append("  " + "  ".join(f"{v:8.3f}" for v in r))
        self.get_logger().info("\n".join(lines))


# ====================================================================== #
def main(args: _t.Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LQRController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

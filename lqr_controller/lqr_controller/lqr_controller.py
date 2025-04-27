"""
ROS 2 node that implements a Linear‑Quadratic Regulator (LQR) exactly as
described below.

The node
    • subscribes to  `lqr_feedback`   (std_msgs/Float32MultiArray) that contains
      the 6‑element state‑error vector

          δ = [dist, lin_vel, body_tilt, body_tilt_vel, turn_angle, turn_vel]

    • computes raw motor commands

          L_accel = −(K1·δ0 + K2·δ1 + K3·δ2 + K4·δ3 + K5·δ4 + K6·δ5)
          R_accel = −(K1·δ0 + K2·δ1 + K3·δ2 + K4·δ3 – K5·δ4 – K6·δ5)

      No additional scaling, clipping, or unit conversion is performed.

    • publishes these raw values to

          motor_cmd/left   (std_msgs/Float32)
          motor_cmd/right  (std_msgs/Float32)

Only the feedback_callback() function below performs the LQR calculation so
that you can easily see or modify the control law.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class LQRController(Node):
    """ROS 2 node that converts the error vector to raw motor commands."""

    # Positive gain coefficients   K1 … K6
    K1, K2, K3, K4, K5, K6 = (
        22.3607,
        43.2739,
        468.4965,
        42.1292,
        22.3607,
        6.1385,
    )

    def __init__(self) -> None:
        super().__init__("lqr_controller")

        # ---------- I/O ----------------------------------------------------
        self.create_subscription(
            Float32MultiArray, "lqr_feedback", self.feedback_callback, 10
        )
        self.left_pub = self.create_publisher(Float32, "motor_cmd/left", 10)
        self.right_pub = self.create_publisher(Float32, "motor_cmd/right", 10)

        self.get_logger().info("LQRController ready.")

    # ---------------------------------------------------------------------
    # Callback that *actually* implements the control law requested by user
    # ---------------------------------------------------------------------
    def feedback_callback(self, msg: Float32MultiArray) -> None:
        """
        Implements the exact equations given in the user’s description:

            L_accel = −(K1·δ0 + K2·δ1 + K3·δ2 + K4·δ3 + K5·δ4 + K6·δ5)
            R_accel = −(K1·δ0 + K2·δ1 + K3·δ2 + K4·δ3 − K5·δ4 − K6·δ5)

        The result is published as‑is without further processing.
        """
        delta = list(msg.data)
        if len(delta) != 6:
            self.get_logger().error(
                f"lqr_feedback must contain 6 elements, got {len(delta)}."
            )
            return

        d0, d1, d2, d3, d4, d5 = delta  # unpack for readability

        # --------- LQR equations (verbatim) --------------------------------
        left_accel = -(
            self.K1 * d0
            + self.K2 * d1
            + self.K3 * d2
            + self.K4 * d3
            + self.K5 * d4
            + self.K6 * d5
        )

        right_accel = -(
            self.K1 * d0
            + self.K2 * d1
            + self.K3 * d2
            + self.K4 * d3
            - self.K5 * d4
            - self.K6 * d5
        )
        # -------------------------------------------------------------------

        # Publish raw values
        self.left_pub.publish(Float32(data=float(left_accel)))
        self.right_pub.publish(Float32(data=float(right_accel)))

        # Debug log (visible only if log level is DEBUG)
        self.get_logger().debug(
            f"δ={delta}  →  L_accel={left_accel:.3f}, R_accel={right_accel:.3f}"
        )


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LQRController()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

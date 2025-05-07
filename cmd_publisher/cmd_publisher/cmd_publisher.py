"""
Command stub for a self-balancing robot.
Publishes a Float32MultiArray with zeros (or a user-defined reference)
at the global control frequency `control_rate` (Hz).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray


class CmdPublisher(Node):
    def __init__(self) -> None:
        super().__init__("cmd_publisher")

        # -------------------- parameters --------------------
        # глобальная частота управления, Гц
        self.declare_parameter("control_rate", 400.0)

        self.control_rate: float = (
            self.get_parameter("control_rate").get_parameter_value().double_value
        )
        if self.control_rate <= 0.0:
            self.get_logger().warning(
                "control_rate <= 0, принудительно устанавливаю 400 Гц"
            )
            self.control_rate = 400.0

        self.dt: float = 1.0 / self.control_rate

        # -------------------- publishers --------------------
        qos = QoSProfile(depth=1)  # храним только самую новую команду
        self.pub = self.create_publisher(Float32MultiArray, "cmd", qos)

        # -------------------- timers ------------------------
        self.timer = self.create_timer(self.dt, self._timer_cb)
        self._prev_stamp = self.get_clock().now()

    # ======================================================
    def _timer_cb(self) -> None:
        """
        Build and publish the reference trajectory.
        Currently publishes six zeros.
        """
        now = self.get_clock().now()
        dt_real = (now - self._prev_stamp).nanoseconds * 1e-9
        self._prev_stamp = now

        # вывод только при DEBUG-уровне
        self.get_logger().debug(f"cmd tick – {dt_real*1e3:.2f} ms")

        msg = Float32MultiArray()
        msg.data = [0.0] * 6
        self.pub.publish(msg)


# ---------------------------------------------------------
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CmdPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
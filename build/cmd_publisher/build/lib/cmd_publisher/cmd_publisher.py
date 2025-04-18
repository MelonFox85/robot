import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'cmd', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Публикация каждую секунду

        # Переменные для интегралов
        self.integral_1 = 0.0
        self.integral_3 = 0.0

        # Время для вычисления интегралов
        self.previous_time = time.time()

    def timer_callback(self):
        # Текущее время
        current_time = time.time()
        delta_time = current_time - self.previous_time
        self.previous_time = current_time

        # Вычисление интегралов
        self.integral_1 += 1.0 * delta_time  # Интеграл от единицы
        self.integral_3 += 1.0 * delta_time  # Интеграл от единицы

        # Формирование сообщения
        msg = Float32MultiArray()
        msg.data = [
            0.0,  # Первая переменная: интеграл от единицы
            0.0,              # Вторая переменная: ноль
            0.0,  # Третья переменная: интеграл от единицы
            0.0,              # Четвертая переменная: единица
            0.0,              # Пятая переменная: ноль
            0.0               # Шестая переменная: единица
        ]

        # Публикация сообщения
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

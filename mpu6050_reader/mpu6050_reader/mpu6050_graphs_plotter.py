import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from collections import deque


class MPU6050GraphNode(Node):
    def __init__(self):
        super().__init__('mpu6050_graph_node')

        # Подписка на топик `mpu6050_reader/filtered_data`
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/filtered_data',
            self.listener_callback,
            10
        )
        self.subscription  # Предотвращение удаления

        # Инициализация переменных для хранения данных
        self.data_roll = deque(maxlen=100)
        self.data_pitch = deque(maxlen=100)
        self.data_gyro_x = deque(maxlen=100)
        self.data_gyro_y = deque(maxlen=100)
        self.data_gyro_z = deque(maxlen=100)
        self.time_stamps = deque(maxlen=100)

        # Настройка времени
        self.time_counter = 0

        # Подготовка графиков
        plt.ion()  # Включаем интерактивный режим matplotlib
        self.fig, self.axes = plt.subplots(5, 1, figsize=(10, 12))  # 5 графиков
        self.graph_titles = [
            "Roll (Крен)",
            "Pitch (Тангаж)",
            "Gyro X (Угловая скорость X)",
            "Gyro Y (Угловая скорость Y)",
            "Gyro Z (Угловая скорость Z)"
        ]

        # Настраиваем каждый из графиков
        for ax, title in zip(self.axes, self.graph_titles):
            ax.set_title(title)
            ax.set_xlim(0, 100)  # Ось X: 100 последовательно принятых точек
            ax.set_ylim(-10, 10)  # Ось Y: задайте диапазон вручную (по необходимости)
            ax.grid()

        self.lines = [ax.plot([], [])[0] for ax in self.axes]

        # Запускаем таймер обновления графиков (каждые 100 мс)
        self.timer = self.create_timer(0.1, self.update_graphs)

        self.get_logger().info("MPU6050 Graph Node запущен.")


    def listener_callback(self, msg):
        """Обработка входящих данных из топика."""
        # Проверяем, что сообщение содержит 5 переменных
        if len(msg.data) != 5:
            self.get_logger().warning("Сообщение не содержит 5 переменных!")
            return

        # Распаковываем переменные
        roll, pitch, gyro_x, gyro_y, gyro_z = msg.data

        # Добавляем переменные в соответствующие очереди
        self.data_roll.append(roll)
        self.data_pitch.append(pitch)
        self.data_gyro_x.append(gyro_x)
        self.data_gyro_y.append(gyro_y)
        self.data_gyro_z.append(gyro_z)

        # Обновляем время
        self.time_stamps.append(self.time_counter)
        self.time_counter += 1


    def update_graphs(self):
        """Обновляет графики новыми данными."""
        datasets = [
            self.data_roll,
            self.data_pitch,
            self.data_gyro_x,
            self.data_gyro_y,
            self.data_gyro_z
        ]

        # Обновляем данные линий графиков
        for line, data, ax in zip(self.lines, datasets, self.axes):
            line.set_data(self.time_stamps, list(data))
            ax.set_xlim(max(0, self.time_counter - 100), self.time_counter)

        # Перерисовываем графики
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)

    # Запускаем ноду
    graph_node = MPU6050GraphNode()

    try:
        rclpy.spin(graph_node)
    except KeyboardInterrupt:
        graph_node.get_logger().info("Остановка узла MPU6050 Graph Node...")
    finally:
        graph_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


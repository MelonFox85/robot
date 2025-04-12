import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from collections import deque


class CompareGraphsNode(Node):
    def __init__(self):
        super().__init__('compare_graphs_node')

        # Буферы для данных (храним последние 100 точек для каждой оси)
        self.raw_data_buffer = deque(maxlen=100)
        self.filtered_data_buffer = deque(maxlen=100)

        # Подписка на топик с сырыми данными
        self.raw_subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/data',
            self.raw_callback,
            10
        )

        # Подписка на топик с отфильтрованными данными
        self.filtered_subscription = self.create_subscription(
            Float32MultiArray,
            'mpu6050/filtered_data',
            self.filtered_callback,
            10
        )

        self.get_logger().info("Compare Graphs Node initialized and listening...")

        # Запуск графиков в отдельном потоке
        self.run_graph_thread()
    def raw_callback(self, msg):
        """Сохраняем сырые данные"""
        if len(msg.data) == 6:  # Проверим, что каждое сообщение имеет 6 значений
            self.raw_data_buffer.append(msg.data)

    def filtered_callback(self, msg):
        """Сохраняем отфильтрованные данные"""
        if len(msg.data) == 6:  # Проверим, что каждое сообщение имеет 6 значений
            self.filtered_data_buffer.append(msg.data)

    def run_graph_thread(self):
        """Запуск визуализации в отдельном потоке"""
        thread = threading.Thread(target=self.plot_graphs)
        thread.daemon = True
        thread.start()

    def plot_graphs(self):
        """Построение и обновление графиков в режиме реального времени"""

        # Настройка графиков (6.subplot, по 3 для каждого типа данных)
        fig, axs = plt.subplots(2, 3, figsize=(12, 8))
        fig.suptitle('Raw vs Filtered MPU6050 Data', fontsize=16)

        accel_axes = ['Accel X', 'Accel Y', 'Accel Z']
        gyro_axes = ['Gyro X', 'Gyro Y', 'Gyro Z']

        # Линии для графиков
        lines = []
        for i, ax in enumerate(axs.flatten()):
            ax.set_xlim(0, 100)  # Ограничиваем ось X (100 точек данных)
            ax.set_ylim(-5, 5)  # Предполагаемый диапазон значений MPU6050
            ax.set_title(accel_axes[i] if i < 3 else gyro_axes[i - 3])
            ax.grid(True)
            # Добавим данные элементов (линии) на график
            line, = ax.plot([], [], label='Raw data')
            line_filtered, = ax.plot([], [], label='Filtered data')
            ax.legend()
            lines.append((line, line_filtered))
        # Лямбда для анимации
        def update(frame):
            # Проверяем, есть ли данные
            if len(self.raw_data_buffer) > 0 and len(self.filtered_data_buffer) > 0:
                raw_data = list(self.raw_data_buffer)
                filtered_data = list(self.filtered_data_buffer)

                # Обновляем графики
                for i in range(6):
                    line, line_filtered = lines[i]
                    # Данные i-го измерения
                    raw_y = [data[i] for data in raw_data]
                    filtered_y = [data[i] for data in filtered_data]

                    x = range(len(raw_y))

                    line.set_data(x, raw_y)  # Сырые данные
                    line_filtered.set_data(x, filtered_y)  # Отфильтрованные данные

        # Запуск анимации
        ani = FuncAnimation(fig, update, interval=500)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = CompareGraphsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped manually.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


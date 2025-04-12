#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import pigpio
import threading
import signal
import time

# Инициализируем переменные
GPIO_A = 23
GPIO_B = 24
levA = 0
levB = 0
lastGpio = None
pos = 0
last_pos = 0

class EncoderNodeLeft(Node):
    def __init__(self):
        super().__init__('left_encoder')
        
        self.publisher_ = self.create_publisher(Float32, 'encoders_data/left', 10)
        self.pos = 0
        self.last_pos = 0
        self.pi = pigpio.pi()

        if not self.pi.connected:
            self.get_logger().error("Ошибка подключения к pigpio!")
            rclpy.shutdown()

        self.get_logger().info("Инициализация энкодера...")

        self.pi.set_mode(GPIO_A, pigpio.INPUT)
        self.pi.set_mode(GPIO_B, pigpio.INPUT)

        self.pi.set_pull_up_down(GPIO_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(GPIO_B, pigpio.PUD_UP)

        # Регистрация обратных вызовов
        self.cbA = self.pi.callback(GPIO_A, pigpio.EITHER_EDGE, self.callback)
        self.cbB = self.pi.callback(GPIO_B, pigpio.EITHER_EDGE, self.callback)

        # Запуск расчёта скорости в отдельном потоке
        threading.Thread(target=self.calculate_speed, daemon=True).start()

    def callback(self, gpio, level, tick):
        """Функция обратного вызова для обработки импульсов энкодера."""
        global levA, levB, lastGpio, pos

        if gpio == GPIO_A:
            levA = level
        else:
            levB = level

        if gpio != lastGpio:  # Простой дебаунс
            lastGpio = gpio

            if gpio == GPIO_A and level == 1:
                if levB == 1:
                    self.pos += 1
            elif gpio == GPIO_B and level == 1:
                if levA == 1:
                    self.pos -= 1
        
    def calculate_speed(self):
        """Функция для вычисления скорости вращения."""
        while rclpy.ok():
            time.sleep(1)

            delta_pos = self.pos - self.last_pos
            speed = float(delta_pos) / (-390.0)
            self.last_pos = self.pos

            # Публикация данных о скорости в топик
            self.publisher_.publish(Float32(data=speed))
            self.get_logger().info(f"{speed} revolution/sec")

    def destroy_node(self):
        """Функция для освобождения ресурсов при остановке ноды."""
        self.running = False
        self.cbA.cancel()
        self.cbB.cancel()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNodeLeft()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка работы по запросу пользователя.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



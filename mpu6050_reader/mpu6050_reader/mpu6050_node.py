import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32MultiArray
import time

# I2C адрес MPU6050
MPU6050_ADDR = 0x68

# Ranges
ACCEL_RANGE = 16384.0 #+-2g
GYRO_RANGE = 131.0 #+-250/c

# Регистры MPU6050
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

def read_word(bus, address, reg):
    """Чтение 16-битного значения из регистра устройства"""
    high = bus.read_byte_data(address, reg)
    low = bus.read_byte_data(address, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:  # Обработка отрицательных чисел
        value = -((65535 - value) + 1)
    return value

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Подключение к I2C
        self.get_logger().info('Initializing MPU6050...')
        self.bus = SMBus(1)

        # Инициализация MPU6050
        try:
            self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
            self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)
            self.bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)
            self.get_logger().info('MPU6050 initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {e}')
            exit(1)

        # Настройка таймера для публикации данных
        self.publisher_ = self.create_publisher(Float32MultiArray, 'mpu6050/data', 10)
        self.timer = self.create_timer(0.1, self.read_and_publish_data)  # 10 Г
    def read_and_publish_data(self):
        try:
            # Чтение данных акселерометра
            accel_x = read_word(self.bus, MPU6050_ADDR, ACCEL_XOUT_H) 
            accel_y = read_word(self.bus, MPU6050_ADDR, ACCEL_YOUT_H) 
            accel_z = read_word(self.bus, MPU6050_ADDR, ACCEL_ZOUT_H) 

            gyro_x = read_word(self.bus, MPU6050_ADDR, GYRO_XOUT_H) 
            gyro_y = read_word(self.bus, MPU6050_ADDR, GYRO_YOUT_H)
            gyro_z = read_word(self.bus, MPU6050_ADDR, GYRO_ZOUT_H)

            # Normalization

            accel_x /= ACCEL_RANGE
            accel_y /= ACCEL_RANGE
            accel_z /= ACCEL_RANGE
            gyro_x /= GYRO_RANGE
            gyro_y /= GYRO_RANGE
            gyro_z /= GYRO_RANGE

            # Calibration

            accel_x += 0.0286328125
            accel_y += 0.01318359375
            accel_z += (1-0.933651123046875)
            gyro_x += 2.7733206106870223
            gyro_y += 0.8731297709923658
            gyro_z += 1.020419847328245

            # Подготовка и публикация сообщения
            msg = Float32MultiArray()
            msg.data = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
            self.publisher_.publish(msg)

            self.get_logger().info(f'Accel Data -> X: {accel_x:.2f}, Y: {accel_y:.2f}, Z: {accel_z:.2f} ' f'Gyro Data -> X: {gyro_x:.2f}, Y: {gyro_y:.2f}, Z: {gyro_z:.2f} ')
        except Exception as e:
            self.get_logger().error(f'Failed to read data from MPU6050: {e}')

    def destroy_node(self):
        self.bus.close()  # Закрываем I2C-шину перед завершением
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

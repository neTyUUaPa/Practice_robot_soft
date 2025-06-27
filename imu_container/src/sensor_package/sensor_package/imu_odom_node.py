import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import smbus
import time
import math

# Регистры MPU9255
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Инициализация публикатора
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # ~20 Гц

        # Инициализация I2C
        self.bus = smbus.SMBus(1)  # Для Raspberry Pi 3/4 используй bus 1
        self.device_address = 0x68  # Адрес MPU9255 по I2C

        # Инициализация MPU9255
        self.MPU_Init()

        # Калибровка гироскопа
        self.gx_bias, self.gy_bias, self.gz_bias = self.calibrate_gyro()

        # Буферы для фильтрации
        self.gx_buffer, self.gy_buffer, self.gz_buffer = [], [], []

    def MPU_Init(self):
        """Инициализация MPU9255"""
        self.bus.write_byte_data(self.device_address, SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.device_address, PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.device_address, CONFIG, 0)
        self.bus.write_byte_data(self.device_address, GYRO_CONFIG, 24)  # ±2000 dps
        self.bus.write_byte_data(self.device_address, ACCEL_CONFIG, 24)  # ±16g

    def read_raw_data(self, addr):
        """Чтение сырых данных из регистров"""
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def get_acceleration(self):
        """Получение данных акселерометра"""
        x = self.read_raw_data(ACCEL_XOUT_H) / 2048.0
        y = self.read_raw_data(ACCEL_XOUT_H + 2) / 2048.0
        z = self.read_raw_data(ACCEL_XOUT_H + 4) / 2048.0
        return x, y, z

    def get_gyro(self):
        """Получение данных гироскопа"""
        x = self.read_raw_data(GYRO_XOUT_H) / 16.4
        y = self.read_raw_data(GYRO_XOUT_H + 2) / 16.4
        z = self.read_raw_data(GYRO_XOUT_H + 4) / 16.4
        return x, y, z

    def calibrate_gyro(self, samples=100):
        """Калибровка гироскопа"""
        print("Calibrating Gyro... (keep the sensor still)")
        gx_bias = gy_bias = gz_bias = 0.0
        for _ in range(samples):
            gx, gy, gz = self.get_gyro()
            gx_bias += gx
            gy_bias += gy
            gz_bias += gz
            time.sleep(0.02)
        gx_bias /= samples
        gy_bias /= samples
        gz_bias /= samples
        print(f"Gyro Bias: X={gx_bias:.2f}, Y={gy_bias:.2f}, Z={gz_bias:.2f}\n")
        return gx_bias, gy_bias, gz_bias

    def moving_average(self, buffer, new_value, window_size=10):
        """Фильтр бегущего среднего"""
        buffer.append(new_value)
        if len(buffer) > window_size:
            buffer.pop(0)
        return sum(buffer) / len(buffer)

    def dead_zone_filter(self, value, threshold=0.1):
        """Фильтр подавления малых значений ("мертвый зон")"""
        return value if abs(value) > threshold else 0.0

    def timer_callback(self):
        """Основной цикл: чтение данных и публикация в топик"""
        ax, ay, az = self.get_acceleration()
        gx, gy, gz = self.get_gyro()

        # Вычитаем смещение
        gx -= self.gx_bias
        gy -= self.gy_bias
        gz -= self.gz_bias

        # Применяем фильтр "мертвой зоны"
        gx = self.dead_zone_filter(gx, threshold=0.1)
        gy = self.dead_zone_filter(gy, threshold=0.1)
        gz = self.dead_zone_filter(gz, threshold=0.1)

        # Применяем бегущее среднее
        gx_filt = self.moving_average(self.gx_buffer, gx)
        gy_filt = self.moving_average(self.gy_buffer, gy)
        gz_filt = self.moving_average(self.gz_buffer, gz)

        # Создание сообщения Imu
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Линейное ускорение
        imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)

        # Угловая скорость
        imu_msg.angular_velocity = Vector3(x=gx_filt, y=gy_filt, z=gz_filt)

        # Ориентация (здесь используется простой пример кватерниона)
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        yaw = 0.0  # MPU9255 не предоставляет магнитометр, поэтому yaw = 0
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Публикация сообщения
        self.publisher_.publish(imu_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Конвертация углов Эйлера в кватернион"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
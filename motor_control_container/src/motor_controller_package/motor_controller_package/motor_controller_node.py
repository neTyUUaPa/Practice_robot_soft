#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct

class HoverboardTestNode(Node):
    def __init__(self):
        super().__init__('hoverboard_test_node')

        # Параметры UART
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyAMA0').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value

        # Подключение к последовательному порту
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            rclpy.shutdown()

        # Объявление параметров для steer и speed
        self.declare_parameter('steer', 0)  # Угол поворота (по умолчанию 0)
        self.declare_parameter('speed', 500)  # Скорость (по умолчанию 500)

        # Таймер для отправки данных
        self.timer = self.create_timer(1.0, self.timer_callback)  # Отправка каждую секунду

    def calculate_crc32(self, data):
        """
        Вычисление CRC32 для данных.
        """
        crc = 0xFFFFFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
        return crc ^ 0xFFFFFFFF

    def send_packet(self, steer, speed):
        """
        Отправка пакета на плату.
        """
        # Формирование структуры Serialcommand
        command = struct.pack('<hh', steer, speed)  # '<' для little-endian, 'h' для int16
        crc = self.calculate_crc32(command)
        packet = command + struct.pack('<I', crc)  # Добавление CRC как uint32

        # Отправка пакета
        self.ser.write(packet)
        self.get_logger().info(f"Sent packet: steer={steer}, speed={speed}")

    def timer_callback(self):
        """
        Отправка данных на плату.
        """
        # Получение текущих значений параметров
        test_steer = self.get_parameter('steer').value
        test_speed = self.get_parameter('speed').value

        # Отправка данных
        self.send_packet(test_steer, test_speed)

def main(args=None):
    rclpy.init(args=args)
    node = HoverboardTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
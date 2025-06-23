import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3, Pose, Twist, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from math import sin, cos

class OdomFromIMU(Node):
    def __init__(self):
        super().__init__('odom_from_imu')

        # Переменные для расчета одометрии
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

        # Подписка на /imu/data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Публикация в /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Публикация трансформации odom -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg):
        # Извлечение данных из IMU
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.z  # Угловая скорость вокруг оси Z (yaw)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Обновление положения
        self.theta += gx * dt
        self.x += ax * cos(self.theta) * dt
        self.y += ay * sin(self.theta) * dt

        # Создание сообщения Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Положение
        odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom_msg.pose.pose.orientation = msg.orientation

        # Скорость
        odom_msg.twist.twist.linear = Vector3(x=ax, y=ay, z=az)
        odom_msg.twist.twist.angular = Vector3(z=gx)

        # Публикация
        self.odom_pub.publish(odom_msg)

        # Публикация трансформации odom -> base_link
        self.broadcast_odom_transform(current_time, self.x, self.y, self.theta, msg.orientation)

    def broadcast_odom_transform(self, current_time, x, y, theta, orientation):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Задаем трансформацию
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = orientation

        # Публикуем трансформацию
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFromIMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
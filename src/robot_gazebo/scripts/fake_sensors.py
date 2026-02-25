#!/usr/bin/env python3
"""
Sahte sensör verisi yayınlar:
- /line_error (robot_interfaces/LineError): Şerit takibi; lateral/heading sapma.
- /imu/data (sensor_msgs/Imu): EKF için; /odom'dan yaw.
- /gps/fix (sensor_msgs/NavSatFix): NavSat + EKF için; sabit enlem/boylam.
"""

import math
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import LineError
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry


class FakeSensorsNode(Node):
    def __init__(self):
        super().__init__('fake_sensors')
        self.declare_parameter('line_error_hz', 20.0)
        self.declare_parameter('imu_hz', 50.0)
        self.declare_parameter('gps_hz', 1.0)
        self.declare_parameter('line_error_value', 0.0)  # 0 = düz, + sol, - sağ
        self.declare_parameter('latitude', 41.0082)
        self.declare_parameter('longitude', 28.9784)

        self._line_error_pub = self.create_publisher(LineError, 'line_error', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)

        self._odom_yaw = 0.0
        self._odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10
        )

        line_hz = self.get_parameter('line_error_hz').value
        imu_hz = self.get_parameter('imu_hz').value
        gps_hz = self.get_parameter('gps_hz').value

        self._line_timer = self.create_timer(1.0 / line_hz, self._publish_line_error)
        self._imu_timer = self.create_timer(1.0 / imu_hz, self._publish_imu)
        self._gps_timer = self.create_timer(1.0 / gps_hz, self._publish_gps)

        self.get_logger().info(
            'Fake sensors aktif: line_error, imu/data, gps/fix (sahte veri)'
        )

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _publish_line_error(self):
        val = self.get_parameter('line_error_value').value
        msg = LineError()
        msg.lateral_error = float(val)
        msg.heading_error = 0.0
        self._line_error_pub.publish(msg)

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.orientation.w = math.cos(self._odom_yaw / 2.0)
        msg.orientation.z = math.sin(self._odom_yaw / 2.0)
        msg.orientation_covariance[0] = -1.0  # orientation not provided
        self._imu_pub.publish(msg)

    def _publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.latitude = self.get_parameter('latitude').value
        msg.longitude = self.get_parameter('longitude').value
        msg.altitude = 10.0
        msg.status.status = 0
        msg.status.service = 1
        self._gps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

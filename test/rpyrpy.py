import sys
import os
import math
import time

import numpy as np
import busio
import board
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class RollPitchYaw(Node):
    def __init__(self):
        super().__init__('RollPitchYaw')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        print(f"roll={self.roll}, pitch{self.pitch}, yaw={self.yaw}")

def main(args=None):
    rclpy.init(args=args)
    node_name = RollPitchYaw()
    rclpy.spin(node_name)
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

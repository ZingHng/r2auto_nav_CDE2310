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

from helper_funcs import euler_from_quaternion

ROTATECHANGE = 0.1

class RollPitchYaw(Node):
    def __init__(self):
        super().__init__('RollPitchYaw')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
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

    def rotatebot(self, rot_angle):
        twist = Twist()
        current_yaw = self.yaw
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        target_yaw = current_yaw + math.radians(rot_angle)
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        c_change = c_target_yaw / c_yaw
        c_change_dir = np.sign(c_change.imag)
        twist.linear.x = 0.0
        twist.angular.z = c_change_dir * ROTATECHANGE
        self.publisher_.publish(twist)
        c_dir_diff = c_change_dir
        while(c_change_dir * c_dir_diff > 0):
            rclpy.spin_once(self)
            current_yaw = self.yaw
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            c_change = c_target_yaw / c_yaw
            c_dir_diff = np.sign(c_change.imag)
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def looper(self):
        while rclpy.ok():
            angle = int(input("Input: Angle"))
            self.rotatebot(angle)
            time.sleep(3)
        

def main(args=None):
    rclpy.init(args=args)
    node_name = RollPitchYaw()
    node_name.looper()
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import sys
import os
import math
import time

import numpy as np
import busio
import board
import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, BatteryState

from helper_funcs import fire_sequence, euler_from_quaternion

def Reader(Node):
    def __init__(self):
        super().__init__('Reader')
        self.scan_subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            qos_profile_sensor_data)
        self.battery = 0.0
    
    def battery_callback(self, msg):
        self.battery = msg.percentage

    def looper(self):
        counter = 1
        while rclpy.ok():
            print(self.battery)
            rclpy.spin_once(self, timeout_sec=0.1) # timeout_sec=0.1 in case lidar doesnt work
            counter += 1


def main(args=None):
    rclpy.init(args=args)
    node_name = Reader()
    node_name.looper()
    node_name.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
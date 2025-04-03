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
import adafruit_amg88xx

from helper_funcs import fire_sequence, euler_from_quaternion

i2c_bus = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)
os.putenv("SDL_FBDEV", "/dev/fb1")
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

def Reader(Node):
    def __init__(self):
        super().__init__('Reader')
            # create subscription to track occupancy
    def looper(self):
        counter = 1
        while rclpy.ok():
            pixels = np.array(sensor.pixels)
            print(pixels)
            print("\n")

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
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""This example is for Raspberry Pi (Linux) only!
   It will not work on microcontrollers running CircuitPython!"""

import sys
import os
import math
import time

import numpy as np
import busio
import board
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
import adafruit_amg88xx

i2c_bus = busio.I2C(board.SCL, board.SDA)

# low range of the sensor (this will be blue on the screen)
MINTEMP = 26.0

# high range of the sensor (this will be red on the screen)
MAXTEMP = 32.0

# how many color values we can have
COLORDEPTH = 1024

os.putenv("SDL_FBDEV", "/dev/fb1")

# initialize the sensor
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

'''# some utility functions
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min'''


# let the sensor initialize
time.sleep(0.1)

class SurvivorZoneDetector(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.temp_publisher = self.create_publisher(Float32MultiArray, 'temperature', 10)
        self.survivor_publisher = self.create_publisher(String, 'survivor', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("temp publisher has started")
    def timer_callback(self):
        temp_msg = Float32MultiArray()
        pixels = np.array(sensor.pixels)
        pixels = np.reshape(pixels, 64)
        pixel_list = pixels.tolist()
        temp_msg.data = pixel_list
        self.temp_publisher.publish(temp_msg)

        survivor_msg = String()
        if max(pixel_list) > MAXTEMP:
            survivor_msg.data = True
        else:
            survivor_msg.data = False
        self.survivor_publisher.publish(survivor_msg)
        

def main(args=None):
    rclpy.init(args=args)

    node_name = SurvivorZoneDetector()
    rclpy.spin(node_name)
    node_name.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





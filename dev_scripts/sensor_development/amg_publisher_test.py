# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# Adapted from https://github.com/adafruit/Adafruit_CircuitPython_AMG88xx/blob/main/examples/amg88xx_rpi_thermal_cam.py

# This code publishes sensor data from AMG8833 as a one-dimensional array of 64 entries
# Data is published to a node named 'sensor'
# This code should be run on the Raspberry Pi, with amg_listener_test.py running simultaneously on the remote PC

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


# MINTEMP = 26.0


# MAXTEMP = 32.0

# how many color values we can have
# COLORDEPTH = 1024

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


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Publisher has started")
    def timer_callback(self):
        msg = Float32MultiArray()
        pixels = np.array(sensor.pixels)
        # convert 8x8 temperature reading array to list for publishing
        pixels = np.reshape(pixels, 64)
        msg.data = pixels.tolist()
        self.publisher_.publish(msg)
        #print(msg)
        

def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = SensorPublisher()

    rclpy.spin(sensor_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





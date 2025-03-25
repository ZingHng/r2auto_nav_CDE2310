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

from std_msgs.msg import String, Float32MultiArray, Twist 
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid

import cmath

i2c_bus = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)

# low range of the sensor (this will be blue on the screen)
MINTEMP = 26.0

# high range of the sensor (this will be red on the screen)
MAXTEMP = 32.0

# how many color values we can have
COLORDEPTH = 1024

os.putenv("SDL_FBDEV", "/dev/fb1")

# initialize the sensor
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

# let the sensor initialize

TIMERPERIOD = 0.1
MAXSPEED = -0.05
ROTATECHANGE = 0.1
SAFETYDISTANCE = 0.230

class SurvivorZoneSequence(Node):
    def __init__(self):
        super().__init__('Survivor_Zone_Sequence')
        self.temp_publisher = self.create_publisher(Float32MultiArray, 'temperature', 10)
        self.survivor_publisher = self.create_publisher(String, 'survivor', 10)
        self.survivor_sequence = False
        self.temp_grid = None

    def approach_child(self):
        twist = Twist()
        if self.laser_range.size != 0:
            lr2i = np.nanargmin(self.laser_range)
        else:
            lr2i = 0
        if child == left:
            twist.angular.z -= ROTATECHANGE
        elif child == right:
            twist.angular.z += ROTATECHANGE
        elif self.laser_range[lr2i] > SAFETYDISTANCE:
            twist.linear.x = MAXSPEED

        
        twist.angular.z = ROTATECHANGE
        time.sleep(0.5) # FOR VIBES
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def looper(self):
        temp_msg = Float32MultiArray()
        pixels = np.array(sensor.pixels)
        self.temp_grid = np.reshape(pixels, 64)
        pixel_list = self.temp_grid.tolist()
        temp_msg.data = pixel_list
        self.temp_publisher.publish(temp_msg)

        if max(pixel_list) > MAXTEMP and not self.survivor_sequence:
            survivor_msg = String()
            survivor_msg.data = True
            self.survivor_sequence = True
            self.odom_subscription = self.create_subscription(
                OccupancyGrid,
                'map',
                self.occ_callback,
                qos_profile_sensor_data)
            self.survivor_publisher.publish(survivor_msg)

        if self.survivor_sequence:
            

def main(args=None):
    rclpy.init(args=args)

    node_name = SurvivorZoneSequence()
    node_name.looper()
    node_name.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





import sys
import os
import math
import time

import numpy as np
import busio
import board
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid

from helper_funcs import fire_sequence

DELTASPEED = 0.05
MAXTEMP = 32.0
ROTATESLOW = 0.1
ROTATEFAST = 0.25
SAFETYDISTANCE = 0.250
TIMERPERIOD = 0.1
TEMPDIFFTOLERANCE = 8 #Huat
VIEWANGLE = 45

# initialize the sensor
i2c_bus = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)
os.putenv("SDL_FBDEV", "/dev/fb1")
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

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

class Ramp(Node):
    def __init__(self):
        super().__init__('Ramp')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.laser_range = np.array([])
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.ramp_subscription = self.create_subscription(
            Bool,
            'rampsequence',
            self.ramp_callback,
            10)
        self.ramp_seq = False

    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan
    
    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def calibrate(self):
        pass

    def looper(self):
        while rclpy.ok():
            rclpy.spin_once(self) # timeout_sec=0.1 in case lidar doesnt work
            pixels = np.array(sensor.pixels)
            left_half, right_half = np.hsplit(pixels, 2)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.laser_range.size != 0:
                laser_points = len(self.laser_range)
                within_viewing_angle = math.ceil(VIEWANGLE/360 * laser_points)
                self.laser_range[:laser_points - within_viewing_angle][within_viewing_angle:] = np.nan
                lidar_shortest = np.nanmin(self.laser_range)
            else:
                lidar_shortest = 0
            left_sum = np.sum(left_half)
            right_sum = np.sum(right_half)
            left_right_error = left_sum - right_sum
            if left_right_error > TEMPDIFFTOLERANCE:
                twist.angular.z = ROTATESLOW
            elif left_right_error < -TEMPDIFFTOLERANCE:
                twist.angular.z = -ROTATESLOW
            elif lidar_shortest > SAFETYDISTANCE:
                twist.linear.x = DELTASPEED
            self.publisher_.publish(twist)
            if (twist.linear.x == 0.0) and (twist.angular.z == 0.0):
                fire_sequence()

            
            print(f"LOOP{counter} DONE")
            counter += 1

def main(args=None):
    rclpy.init(args=args)
    node_name = Ramp()
    node_name.check()
    node_name.looper()
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
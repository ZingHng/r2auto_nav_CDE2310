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

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from helper_funcs import fire_sequence, euler_from_quaternion

DELTASPEED = 0.05
MAXTEMP = 32.0
ROTATECHANGE = 0.1
SAFETYDISTANCE = 0.250
TIMERPERIOD = 0.1
TEMPDIFFTOLERANCE = 8 #Huat
FIRINGSAFETYZONESQ = 1

# initialize the sensor
i2c_bus = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)
os.putenv("SDL_FBDEV", "/dev/fb1")
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

class SurvivorZoneSequence(Node):
    def __init__(self):
        super().__init__('Survivor_Zone_Sequence')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.survivor_publisher = self.create_publisher(String, 'survivor', 10)
        self.survivor_sequence = False
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
        self.activations = []
        self.tfBuffer = tf2_ros.Buffer()
        self.temp_grid = None

    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan

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
        print(f"{c_change_dir} c_dir_diff={c_dir_diff}")
        while(c_change_dir * c_dir_diff > 0):
            rclpy.spin_once(self)
            current_yaw = self.yaw
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            c_change = c_target_yaw / c_yaw
            c_dir_diff = np.sign(c_change.imag)
            print(f"c_dir_diff={c_dir_diff}, current_yaw={current_yaw}")
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def approach_victim(self, left, right):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if self.laser_range.size != 0:
            lidar_shortest = np.nanmin(self.laser_range)
        else:
            lidar_shortest = 0
        left_sum = np.sum(left)
        right_sum = np.sum(right)
        left_right_error = left_sum - right_sum
        if left_right_error > TEMPDIFFTOLERANCE:
            twist.angular.z = ROTATECHANGE
        elif left_right_error < -TEMPDIFFTOLERANCE:
            twist.angular.z = -ROTATECHANGE
        elif lidar_shortest > SAFETYDISTANCE:
            twist.linear.x = DELTASPEED
        self.publisher_.publish(twist)
        if (twist.linear.x == 0.0) and (twist.angular.z == 0.0):
            fire_sequence()
            return False
        return True
    
    def current_position(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return (0,0)
        return trans.transform.translation # real world coordinates of robot relative to robot start point
    
    def looper(self):
        counter = 1
        while rclpy.ok():
            pixels = np.array(sensor.pixels)
            if not self.survivor_sequence and np.max(pixels) > MAXTEMP:
                x, y = self.current_position()
                print(f"{counter} current{x, y}")
                nearest_fire = min([(i[0] - x) ** 2 + (i[1] - y) ** 2 for i in self.activations]+[math.inf])
                if nearest_fire > FIRINGSAFETYZONESQ:
                    survivor_msg = String()
                    survivor_msg.data = "FOUND"
                    self.survivor_sequence = True
                    self.survivor_publisher.publish(survivor_msg)
                else:
                    print(f"Too close to past firing: current=({x, y}) nearest={nearest_fire}")

            if self.survivor_sequence:
                left_half, right_half = np.hsplit(pixels, 2)
                stay_survivor_sequence = self.approach_victim(left_half, right_half)
                if not stay_survivor_sequence:
                    survivor_msg = String()
                    survivor_msg.data = "RESCUED"
                    self.survivor_publisher.publish(survivor_msg)
                    self.activations.append(self.current_position())
                    self.rotatebot(180)

            rclpy.spin_once(self, timeout_sec=0.1) # timeout_sec=0.1 in case lidar doesnt work
            counter += 1

def main(args=None):
    rclpy.init(args=args)
    node_name = SurvivorZoneSequence()
    node_name.looper()
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
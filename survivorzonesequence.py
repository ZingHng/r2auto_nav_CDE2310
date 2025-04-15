import sys
import os
import math
import time

import numpy as np
import busio
import board
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from helper_funcs import fire_sequence, euler_from_quaternion

DELTASPEED = 0.1
ROTATESLOW = 0.15
ROTATEFAST = 0.5
SAFETYDISTANCE = 0.30
TIMERPERIOD = 0.1
TEMPTOLERANCE = 8 #Huat
FIRINGSAFETYZONESQ = 0.25
VIEWANGLE = 45 # 0 +-ViewAngle
TAU = 2*math.pi
OFFRAMPHEATSOURCES = 2

try:
    max_temp = float(input("Max Temp? "))
except:
    max_temp = 30.0

print(f"Max Temp {max_temp}")

# initialize the sensor
i2c_bus = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)
os.putenv("SDL_FBDEV", "/dev/fb1")
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

class SurvivorZoneSequence(Node):
    def __init__(self):
        super().__init__('Survivor_Zone_Sequence')
        self.cmd_vel_publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.survivor_publisher = self.create_publisher(Bool, 'survivorzonesequenceactive', 10)
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
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.position = (0,0)
        self.nearest_fire_sq = None
        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            qos_profile_sensor_data)
        self.battery = 0.0
        self.voltage = 0.0
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

    def occ_callback(self, msg):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout = Duration(seconds=0.05))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return None
        self.position = [trans.transform.translation.x, trans.transform.translation.y] # real world coordinates of robot relative to robot start point
    
    def ramp_callback(self, msg):
        self.ramp_seq = msg.data

    def battery_callback(self, msg):
        self.battery = round(msg.percentage, 2) if msg.percentage > 40 else f"__LOW_BATTERY__ {round(msg.percentage, 2)}"
        self.voltage = round(msg.voltage, 2)

    def rotate(self, rad_angle):  # rad_angle in radians, < 2π
        def normalize_angle(angle):
            return np.arctan2(np.sin(angle), np.cos(angle))  # [-π, π]
        start_yaw = self.yaw
        target_angle = normalize_angle(start_yaw + rad_angle)
        twist = Twist()
        # Determine rotation direction
        if rad_angle > 0:  # CCW
            twist.angular.z = ROTATESLOW
        elif rad_angle < 0:  # CW
            twist.angular.z = -ROTATESLOW
        else:
            twist.angular.z = 0
            print("No Rotation")
            return  # No rotation needed
        self.cmd_vel_publisher.publish(twist)
        angle_diff = normalize_angle(target_angle - self.yaw)
        while abs(angle_diff) > 0.01:
            rclpy.spin_once(self)
            angle_diff = normalize_angle(target_angle - self.yaw)
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def approach_victim(self, left, right):
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
            left_sum = np.sum(left)
            right_sum = np.sum(right)
            left_right_error = left_sum - right_sum
            if left_right_error > TEMPTOLERANCE:
                twist.angular.z = ROTATESLOW
            elif left_right_error < -TEMPTOLERANCE:
                twist.angular.z = -ROTATESLOW
            elif lidar_shortest > SAFETYDISTANCE:
                twist.linear.x = DELTASPEED
            self.cmd_vel_publisher.publish(twist)
            if (twist.linear.x == 0.0) and (twist.angular.z == 0.0):
                return False
            return True
    
    def smart_flip(self):
        left_lidar_half, right_lidar_half = np.hsplit(self.laser_range[len(self.laser_range)%2:], 2)
        if np.sum(left_lidar_half) > np.sum(right_lidar_half):
            self.rotate(math.pi)
        else:
            self.rotate(-math.pi)
        
    def stop_bot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def move_away_from_wall(self, safety=SAFETYDISTANCE):
        print("Move away from wall")
        while len(self.laser_range) == 0:
            rclpy.spin_once(self)
        while np.nanmin(self.laser_range) < safety:
            angle = np.nanargmin(self.laser_range)/len(self.laser_range) * TAU
            angle = angle if angle < math.pi else angle - TAU
            self.rotate(angle)
            twist = Twist()
            twist.linear.x = -DELTASPEED
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.3)
            self.stop_bot()
        

    def survivorzones(self):
        print("SurvivorZoneSequence")
        while rclpy.ok() and len(self.activations) < OFFRAMPHEATSOURCES:
            rclpy.spin_once(self)
            pixels = np.array(sensor.pixels)
            if not self.survivor_sequence and np.max(pixels) > max_temp:
                x, y = self.position
                self.nearest_fire_sq = min([(i[0] - x) ** 2 + (i[1] - y) ** 2 for i in self.activations]+[math.inf])
                if self.nearest_fire_sq > FIRINGSAFETYZONESQ:
                    survivor_msg = Bool()
                    survivor_msg.data = True
                    self.survivor_sequence = True
                    self.survivor_publisher.publish(survivor_msg)
                else:
                    print(f"Too close to past firing: current=({x, y}) nearest={self.nearest_fire_sq}")

            if self.survivor_sequence:
                left_heat_half, right_heat_half = np.hsplit(pixels, 2)
                self.survivor_sequence = self.approach_victim(left_heat_half, right_heat_half)
                if not self.survivor_sequence:
                    self.smart_flip()
                    self.stop_bot()
                    fire_sequence()
                    print("Post Fire Seq")
                    self.activations.append(self.position)
                    survivor_msg = Bool()
                    survivor_msg.data = False
                    self.survivor_publisher.publish(survivor_msg)
    
    def rampcheck(self):
        while not self.ramp_seq:
            print(f"ramp_seq= {self.ramp_seq} sleep timeeee")
            time.sleep(2)
            rclpy.spin_once(self)

        print("Ramp Check")
        if max_temp > 20: # theres 2 guys in end zone
            print(f"Searching for other guy of temp {max_temp}")
            not_found = True
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = np.sign(self.yaw) * ROTATESLOW
            self.cmd_vel_publisher.publish(twist)
            while not_found:
                rclpy.spin_once(self)
                pixels = np.array(sensor.pixels)
                if np.max(pixels) > max_temp:
                    left_heat_half, right_heat_half = np.hsplit(pixels, 2)
                    not_found = self.approach_victim(left_heat_half, right_heat_half)
                    if not not_found:
                        self.stop_bot()
                        self.smart_flip()
                        fire_sequence()
                max_temp -= 0.00001
        print("Move away from wall")
        self.move_away_from_wall()
        
        rclpy.spin_once(self)
        print("Align Perpendicular")
        if 1.55 < abs(self.yaw) < 1.59: # align to 90
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = -np.sign(self.yaw) * ROTATESLOW
            self.cmd_vel_publisher.publish(twist)
            while 1.55 < abs(self.yaw) < 1.59:
                rclpy.spin_once(self)
            self.stop_bot()
        
        if self.laser_range.size != 0:
            rclpy.spin_once(self)
        print("Wiggle")
        laser_points = len(self.laser_range)
        back_view = self.laser_range[laser_points//4: 3*laser_points//4].copy()
        self.laser_range[laser_points//4: 3*laser_points//4] = np.nan
        front_view = self.laser_range
        side_to_side_error = np.nanmin(front_view) - np.nanmin(back_view)
        if abs(side_to_side_error) > SAFETYDISTANCE:
            twist = Twist()
            twist.linear.x = -np.sign(side_to_side_error) * DELTASPEED
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            while np.nanmin(front_view) > 0.4 and np.nanmin(back_view) > 0.4:
                rclpy.spin_once(self)
            self.stop_bot
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = np.sign(self.yaw) * ROTATESLOW
            self.cmd_vel_publisher.publish(twist)
            while 3.138 > abs(self.yaw):
                rclpy.spin_once(self)
                if np.nanmin(self.laser_range):
                    continue
            self.stop_bot()

        if 3.138 < abs(self.yaw): # align to math.pi (victim)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = -np.sign(self.yaw) * ROTATESLOW
            self.cmd_vel_publisher.publish(twist)
            while 3.138 > abs(self.yaw):
                rclpy.spin_once(self)
            self.stop_bot()

        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


    def rampclimb(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            pixels = np.array(sensor.pixels)
            left_half, right_half = np.hsplit(pixels, 2)
            ended = self.approach_victim(left_half, right_half)
            if not ended:
                self.smart_flip()
                self.stop_bot()
                fire_sequence()
                break

def main(args=None):
    rclpy.init(args=args)
    node_name = SurvivorZoneSequence()
    node_name.survivorzones()
    node_name.rampcheck()
    node_name.rampclimb()
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
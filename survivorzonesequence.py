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
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from helper_funcs import fire_sequence, euler_from_quaternion

DELTASPEED = 0.075
ROTATESLOW = 0.1
ROTATEFAST = 0.25
SAFETYDISTANCE = 0.30
TIMERPERIOD = 0.1
TOLERANCE = 8 #Huat
FIRINGSAFETYZONESQ = 0.25
VIEWANGLE = 45 # 0 +-ViewAngle
DEBUG = False

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
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        self.position = [trans.transform.translation.x, trans.transform.translation.y] # real world coordinates of robot relative to robot start point
    
    def ramp_callback(self, msg):
        self.ramp_seq = msg.data

    def battery_callback(self, msg):
        self.battery = round(msg.percentage, 2) if msg.percentage > 40 else "__LOW_BATTERY__LOW_BATTERY__LOW_BATTERY__LOW_BATTERY__LOW_BATTERY__"
        self.voltage = round(msg.voltage, 2)

    def rotatebot(self, rot_angle):
        print("Start Rotate")
        twist = Twist()
        current_yaw = self.yaw
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        target_yaw = current_yaw + math.radians(rot_angle)
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        c_change = c_target_yaw / c_yaw
        c_change_dir = np.sign(c_change.imag)
        twist.linear.x = 0.0
        twist.angular.z = c_change_dir * ROTATEFAST
        self.cmd_vel_publisher.publish(twist)
        c_dir_diff = c_change_dir
        while(c_change_dir * c_dir_diff > 0):
            rclpy.spin_once(self)
            current_yaw = self.yaw
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            c_change = c_target_yaw / c_yaw
            c_dir_diff = np.sign(c_change.imag)
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
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
        if left_right_error > TOLERANCE:
            twist.angular.z = ROTATESLOW
        elif left_right_error < -TOLERANCE:
            twist.angular.z = -ROTATESLOW
        elif lidar_shortest > SAFETYDISTANCE:
            twist.linear.x = DELTASPEED
        self.cmd_vel_publisher.publish(twist)
        if (twist.linear.x == 0.0) and (twist.angular.z == 0.0):
            return False
        return True
    
    def debugger(self):
        if not DEBUG:
            pass
        if len(self.laser_range):
            closest_LIDAR_index = np.nanargmin(self.laser_range)
            print(f"""\n\n\n\n\n\n
{time.strftime("%H:%M:%S.%f",time.localtime())}
LIDAR    | closest={np.nanmin(self.laser_range)}m @ {closest_LIDAR_index} - {closest_LIDAR_index / len(self.laser_range) * 360 }*
ODOM     | roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}
TEMP     | target={max_temp}, max={np.max(sensor.pixels)}*C
BATTERY  | voltage={self.voltage}V percentage={self.battery}%
POSITION | (x, y)=({self.position[0]}, {self.position[1]})
STORAGE  | nearestfire={self.nearest_fire_sq}, survivor sequence?={self.survivor_sequence}
         | activations={self.activations}
""")
        else:
            print(f"""\n\n\n\n\n\n
{time.strftime("%H:%M:%S.%f",time.localtime())}
ODOM     | roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}
TEMP     | target={max_temp}, max={np.max(sensor.pixels)}*C
BATTERY  | voltage={self.voltage}V percentage={self.battery}%
POSITION | (x, y)=({self.position[0]}, {self.position[1]})
STORAGE  | nearestfiresq={self.nearest_fire_sq}, survivor sequence?={self.survivor_sequence}
         | activations={self.activations}
""")
    def smart_flip(self):
        left_lidar_half, right_lidar_half = np.hsplit(self.laser_range, 2)
        if np.sum(left_lidar_half) > np.sum(right_lidar_half):
            self.rotatebot(180)
        else:
            self.rotatebot(-180)
        
    def stop_bot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def survivorzones(self):
        print("SurvivorZoneSequence")
        while rclpy.ok() and not self.ramp_seq:
            rclpy.spin_once(self)
            pixels = np.array(sensor.pixels)
            self.debugger()
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
                    self.activations.append(self.position)
                    survivor_msg = Bool()
                    survivor_msg.data = False
                    self.survivor_publisher.publish(survivor_msg)
    
    def rampcheck(self):
        if len(self.activations) < 2: # theres 2 guys in end zone
            # spin
            # rotate around, find anything above 26 degrees
            # whack target
            pass
        aligned = False
        while not aligned:
            rclpy.spin_once(self)
            if 1.55 < abs(self.yaw) < 1.59: # align to 90
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = -np.sign(self.yaw) * ROTATESLOW
                self.cmd_vel_publisher.publish(twist)
                while 3.138 > abs(self.yaw):
                    rclpy.spin_once(self)
                self.stop_bot()
            
            if 3.138 < abs(self.yaw): # check
                aligned = True
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


    def rampclimb(self):
        print("Ramp Climber")
        while rclpy.ok():
            rclpy.spin_once(self)
            pixels = np.array(sensor.pixels)
            self.debugger()
            left_half, right_half = np.hsplit(pixels, 2)
            self.survivor_sequence = self.approach_victim(left_half, right_half)
            if not self.survivor_sequence:
                self.smart_flip()
                self.stop_bot()
                fire_sequence()

def main(args=None):
    rclpy.init(args=args)
    node_name = SurvivorZoneSequence()
    try:
        node_name.survivorzones()
        node_name.rampcheck()
        node_name.rampclimb()
    except:
        node_name.stop_bot()
    finally:
        node_name.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
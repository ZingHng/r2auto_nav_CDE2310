import sys
import os
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, BatteryState


from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from helper_funcs import fire_sequence, euler_from_quaternion

def Reader(Node):
    def __init__(self):
        super().__init__('Reader')
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
        self.szs_subscription = self.create_subscription(
            Bool,
            'survivorzonesequenceactive',
            self.szs_callback,
            10)
        self.szsactive = False

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

    def szs_callback(self, msg):
        self.szsactive = msg.data
        if self.szsactive:
            print('SURVIVOR ZONE SEQUENCE ACTIVE')
            self.survivorsfound += 1
        else:
            print('SURVIVOR ZONE SEQUENCE COMPLETE')
    
    def debugger(self):
        if len(self.laser_range):
            closest_LIDAR_index = np.nanargmin(self.laser_range)
            print(f"""\n\n\n\n\n\n
{time.strftime("%H:%M:%S",time.localtime())}
LIDAR    | closest={np.nanmin(self.laser_range)}m @ {closest_LIDAR_index} - {closest_LIDAR_index / len(self.laser_range) * 360 }*
ODOM     | roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}
BATTERY  | voltage={self.voltage}V percentage={self.battery}%
POSITION | (x, y)=({self.position[0]}, {self.position[1]})
SZS      | survivor sequence={self.survivor_sequence}
""")
        else:
            print(f"""\n\n\n\n\n\n
{time.strftime("%H:%M:%S",time.localtime())}
ODOM     | roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}
BATTERY  | voltage={self.voltage}V percentage={self.battery}%
POSITION | (x, y)=({self.position[0]}, {self.position[1]})
STORAGE  | nearestfiresq={self.nearest_fire_sq}, survivor sequence?={self.survivor_sequence}
         | activations={self.activations}
""")

    def looper(self):
        self.debugger()




def main(args=None):
    node_name = Reader()
    node_name.looper()
    node_name.destroy_node()


if __name__ == '__main__':
    main()
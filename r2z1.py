import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from rclpy.qos import qos_profile_sensor_data # type: ignore
from sensor_msgs.msg import LaserScan # type: ignore
from nav_msgs.msg import OccupancyGrid # type: ignore
import scipy.stats # type: ignore
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np 
import math
import cmath
import time

from r2occupancy import euler_from_quaternion

OCC_BINS = [-1, 0, 50, 100]

def CostMap(Node):
    def __init__(self):
        super().__init__('CostMap')
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

            # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        occdata = np.array(msg.data)
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=OCC_BINS)
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        img = Image.fromarray(odata)
        print(odata)
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        plt.pause(0.00000000001)

    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        pass


def main(args=None):
    rclpy.init(args=args)
    auto_nav = CostMap()
    auto_nav.mover()
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
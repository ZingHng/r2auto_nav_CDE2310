# This code subscribes to the 'sensor' node and listens for AMG8833 sensor data published by amg_publisher_test.py
# The temperature data from the sensor is visualised as a heatmap on the remote PC
# This code should be run on the remote PC, with amg_publisher_test.py running simultaneously on the RPi


import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray

import numpy as np
import matplotlib.pyplot as plt

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info("receiving") # for debugging
        # convert the one-dimensional array to 8x8 array
        x = np.reshape(msg.data, (8, 8))
        print(x)
        plt.imshow(x, cmap='hot', interpolation='nearest') 
        plt.draw()
        plt.pause(0.1)
        plt.cla()


def main(args=None):
    rclpy.init(args=args)

    sensor_subscriber = SensorSubscriber()

    rclpy.spin(sensor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
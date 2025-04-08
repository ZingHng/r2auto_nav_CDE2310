import time
import adafruit_amg88xx
import busio
import board
import rclpy
from rclpy.node import Node
import numpy as np
import os

i2c_bus = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)
os.putenv("SDL_FBDEV", "/dev/fb1")
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

class SensorTest(Node):
    def __init__(self):
        super().__init__('SensorTest')
    
    def looper(self):
        while rclpy.ok():
            pixels = np.array(sensor.pixels)
            pixels = np.reshape(pixels, (8,8))
            print(pixels)
            print("\n")
            time.sleep()

def main(args=None):
    rclpy.init(args=args)
    node_name = SensorTest()
    node_name.looper()
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import sys
import os
import math
import time
import RPi.GPIO as GPIO

import numpy as np
import busio
import board
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid

i2c_bus = busio.I2C(board.SCL, board.SDA)
time.sleep(0.1)

os.putenv("SDL_FBDEV", "/dev/fb1")

# initialize the sensor
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

MAXSPEED = -0.05
MAXTEMP = 32.0
ROTATECHANGE = 0.1
SAFETYDISTANCE = 0.250
TIMERPERIOD = 0.1
TEMPDIFFTOLERANCE = 8 #Huat
STEPSPERREV = 512

class SurvivorZoneSequence(Node):
    def __init__(self):
        super().__init__('Survivor_Zone_Sequence')
#        self.temp_publisher = self.create_publisher(Float32MultiArray, 'temperature', 10)
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.survivor_publisher = self.create_publisher(String, 'survivor', 10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.survivor_sequence = False
        self.temp_grid = None
        self.laser_range = np.array([])

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan

    def fire_sequence(self):
        print("FIRE AAAAAAAAAAAAAA")
        GPIO.setmode(GPIO.BCM)

        in1 = 20
        in2 = 16
        en = 21
        mospwm = 12

        #Stepper setup
        control_pins = [26,19,13,6]
        for pin in control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        ''''fullstep_seq = [
            [1,1,0,0],
            [0,1,1,0],
            [0,0,1,1],
            [1,0,0,1]]''' # reversed
        fullstep_seq = [
            [1,0,0,1],
            [0,0,1,1],
            [0,1,1,0],
            [1,1,0,0]]

        #DC motor setup
        GPIO.setup(in1,GPIO.OUT)
        GPIO.setup(in2,GPIO.OUT)
        GPIO.setup(en,GPIO.OUT)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        GPIO.setup(mospwm,GPIO.OUT)
        p=GPIO.PWM(en,1000)
        p.start(25)
        ps = GPIO.PWM(mospwm, 1000)
        ps.start(0)

        def FlywheelStart():
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(in1,GPIO.OUT)
            GPIO.setup(in2,GPIO.OUT)
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)

        def FlywheelStop():
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(in1,GPIO.OUT)
            GPIO.setup(in2,GPIO.OUT)
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)

        def StepperTurn():
            GPIO.setmode(GPIO.BCM)
            x = 90 #degrees
            angle = int((float(x)/360)*STEPSPERREV)
            for i in range(angle):
                for fullstep in range(4):
                    for pin in range(4):
                        GPIO.output(control_pins[pin], fullstep_seq[fullstep][pin])
                    time.sleep(0.001)
        
        ps.ChangeDutyCycle(75)
        FlywheelStart()
        time.sleep(2)
        StepperTurn()
        time.sleep(4)
        StepperTurn()
        time.sleep(2)
        StepperTurn()
        time.sleep(5)
        FlywheelStop()
        ps.ChangeDutyCycle(0)
        time.sleep(3)

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
        print(f"left{left_sum} | right{right_sum}, error = {left_right_error}")
        print(left_right_error)
        if left_right_error > TEMPDIFFTOLERANCE:
            twist.angular.z = ROTATECHANGE
            print("LEFT")
        elif left_right_error < -TEMPDIFFTOLERANCE:
            twist.angular.z = -ROTATECHANGE
            print("RIGHT")
        elif lidar_shortest > SAFETYDISTANCE:
            print(f"GO {lidar_shortest}")
            twist.linear.x = MAXSPEED
        self.publisher_.publish(twist)
        print(f"PUBBED twist.linear.x{twist.linear.x} twist.angular.z{twist.angular.z}")
        if (twist.linear.x == 0.0) and (twist.angular.z == 0.0):
            print("FIRE")
            self.fire_sequence()
            print("FIRED")
            return False
        return True

    def looper(self):
        counter = 1
        while rclpy.ok():
            self.get_logger().info(f"LOOP{counter}")
            pixels = np.array(sensor.pixels)
            
            if not self.survivor_sequence and np.max(pixels) > MAXTEMP:
                survivor_msg = String()
                survivor_msg.data = "HELP ME IM DYING"
                self.survivor_sequence = True
                self.survivor_publisher.publish(survivor_msg)

            if self.survivor_sequence:
                left_half, right_half = np.hsplit(pixels, 2)
                self.survivor_sequence = self.approach_victim(left_half, right_half)
            rclpy.spin_once(self, timeout_sec=0.1) # timeout_sec=0.1 in case lidar doesnt work
            self.get_logger().info(f"LOOP{counter} DONE")
            counter += 1

def main(args=None):
    rclpy.init(args=args)
    node_name = SurvivorZoneSequence()
    node_name.looper()
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
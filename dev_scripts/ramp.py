import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import RPi.GPIO as GPIO
import math

ROTATESPEED = 0.5
FORWARDSPEED = 0.2
DURATION = 0.3

class CelebrationDance(Node):
    def __init__(self):
        super().__init__('celebration_dance')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def spin_in_place(self, duration, direction=1):
        twist = Twist()
        twist.angular.z = direction * ROTATESPEED
        self.cmd_vel_publisher.publish(twist)
        time.sleep(duration)

    def shimmy(self, duration):
        twist = Twist()
        twist.linear.x = FORWARDSPEED
        self.cmd_vel_publisher.publish(twist)
        time.sleep(duration)
        twist.linear.x = -FORWARDSPEED
        self.cmd_vel_publisher.publish(twist)
        time.sleep(duration)
        self.stop()

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_publisher.publish(twist)

    def detonate(self):
        STEPSPERREV = 512
        GPIO.setmode(GPIO.BCM)

        in1 = 20
        in2 = 16
        en = 12

        control_pins = [26,19,13,6]
        for pin in control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        GPIO.setup(in1,GPIO.OUT)
        GPIO.setup(in2,GPIO.OUT)
        GPIO.setup(en,GPIO.OUT)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        p=GPIO.PWM(en,1000)
        p.start(100)
        
        fullstep_seq = [
            [1,0,0,1],
            [0,0,1,1],
            [0,1,1,0],
            [1,1,0,0],
            ]

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
        try:
            print("Flywheel Start")
            FlywheelStart()
            time.sleep(3)
            print("Stepper Turn")
            StepperTurn()
            time.sleep(1)
            print("Stepper Turn")
            StepperTurn()
            time.sleep(1)
            print("Stepper Turn")
            StepperTurn()
            time.sleep(0.75)
            print("Flywheel Stop")
            FlywheelStop()
        except:
            FlywheelStop()
        finally:
            GPIO.cleanup()

    def dance(self):
        self.get_logger().info("🎉 Time to Celebrate! 💃🕺")
        self.spin_in_place(1.5)      # spin right
        self.stop()
        self.shimmy(0.5)             # shimmy forward/back
        self.spin_in_place(1.5, -1)  # spin left
        self.stop()
        self.shimmy(0.5)             # shimmy again
        self.spin_in_place(1.0)      # little twirl
        self.detonate()
        self.stop()
        self.get_logger().info("🎊 Dance complete!")

def main(args=None):
    rclpy.init(args=args)
    node = CelebrationDance()
    node.dance()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

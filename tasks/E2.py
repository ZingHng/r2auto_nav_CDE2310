import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

GPIO.setmode(GPIO.BCM)
solenoid_pin = 21
servo_pin = 20
GPIO.setup(solenoid_pin, GPIO.OUT)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(7.5)
GPIO.output(solenoid_pin, GPIO.LOW)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('I have started with minimal errors')

        

    def listener_callback(self, msg):
        try:
            if msg.data == "fire":
                a = 45
                d = (a/180)*10 + 2.5
                GPIO.output(solenoid_pin, GPIO.HIGH)
                pwm.ChangeDutyCycle(d)
                self.get_logger().info('I heard: "%s"' % msg.data)
            elif msg.data == "reset":
                GPIO.output(solenoid_pin, GPIO.LOW)
                pwm.ChangeDutyCycle(2.5)
                self.get_logger().info('I heard: "%s"' % msg.data)
        except KeyboardInterrupt:
                pwm.stop()
                GPIO.cleanup()



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    print("55")
    rclpy.spin(minimal_subscriber)
    print("57")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#Set solenoid_pin numbering convention
#use the Broadcom solenoid_pin numbering convention
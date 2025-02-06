import time
import RPi.GPIO as GPIO

#Set pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose an appropriate pwm channel to be used to control the servo
servo_pin = 12

# Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialise the servo to be controlled by pwm with 50 Hz frequency
p = GPIO.PWM(servo_pin, 50)

# Set servo to 90 degrees as it's starting position
p.start(7.5)

def demo(p):
    a = int(input())
    d = (a/180)*10 + 2.5
    p.ChangeDutyCycle(d)


try:
    while True:
        demo(p)
        #p.ChangeDutyCycle(7.5) #90 deg position
        time.sleep(1) #delay 1 second
        #p.ChangeDutyCycle(2.5) #0 deg position
        #time.sleep(1) #delay 1 second again
        #p.ChangeDutyCycle(12.5) #180 deg position
        #time.sleep(1) #delay 1 second again... ...



except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
import time
import RPi.GPIO as GPIO

#Set solenoid_pin numbering convention
#use the Broadcom solenoid_pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose an appropriate pwm channel to be used to control the servo
solenoid_pin = 21
servo_pin = 20


# Set the solenoid_pin as an output
GPIO.setup(solenoid_pin, GPIO.OUT)
GPIO.setup(servo_pin, GPIO.OUT)

p = GPIO.PWM(servo_pin, 50)

p.start(7.5)

a = 45
d = (a/180)*10 + 2.5

try:
    while True:
        print("HI")
        GPIO.output(solenoid_pin, GPIO.HIGH)
        p.ChangeDutyCycle(d)
        time.sleep(3)
        print("BYE")
        GPIO.output(solenoid_pin, GPIO.LOW)
        p.ChangeDutyCycle(2.5)
        time.sleep(1)

except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()

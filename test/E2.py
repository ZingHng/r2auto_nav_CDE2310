import time
import RPi.GPIO as GPIO

#Set pin numbering convention
#use the Broadcom pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose an appropriate pwm channel to be used to control the servo
pin = 21

# Set the pin as an output
GPIO.setup(pin, GPIO.OUT)

try:
    while True:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(3)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(3)

except KeyboardInterrupt:
    GPIO.cleanup()

import time
import RPi.GPIO as GPIO


sensor_pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor_pin, GPIO.IN)
counter = 0
stronk = ""
stronk2 = ""
try:
    while True:
        if counter == 8:
            counter = 0
            stronk2 += stronk[0]
            stronk = ""
        stronk += str(GPIO.input(sensor_pin))
        counter += 1
        if len(stronk2) == 8:
            print(stronk2)
            stronk2 = ""
except KeyboardInterrupt:
    GPIO.cleanup()

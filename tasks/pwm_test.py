import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

test_point = 21

GPIO.setup(test_point, GPIO.OUT)

#initialise pwm object with 1kHz frequency
pwm = GPIO.PWM(test_point,1000)
pwm.start(0)

#Begin pwm experiment
print("Start")

for i in range(0,100,1):
    pwm.ChangeDutyCycle(i)
    print("Brightness is",i,"%")
    time.sleep(0.2)

else:
    print("Finished")

#End the script and exit
pwm.stop()
GPIO.cleanup()
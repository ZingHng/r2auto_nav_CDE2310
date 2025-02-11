import time
import RPi.GPIO as GPIO

#Set pin numbering convention
#use the Broadcom pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose an appropriate pwm channel to be used to control the servo
#set GPIO pin 12 as the servo pin
servo_pin = 12

# Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialise the servo to be controlled by pwm with 50 Hz frequency
p = GPIO.PWM(servo_pin, 50)

# Set servo to 90 degrees as it's starting position
p.start(7.5)

def demo(p):
    #a in the input angle
    a = None
    while a is None:
        try:
            #ask the user for an angle
            print("Input an angle between 0 and 180")
            a = int(float(input()))
        except KeyboardInterrupt:
            #stop the pwm object
            p.stop()
            #clean up the GPIO port
            GPIO.cleanup()
        except:
            pass
    
    #calculation to convert angle to duty cycle
    if a < 0:
        a = 0
    elif a > 180:
        a = 180
    d = (a/180)*10 + 2.5
    #set the servo to the input angle
    p.ChangeDutyCycle(d)


try:
    while True:
        #run demo function
        demo(p)
        #p.ChangeDutyCycle(7.5) #90 deg position
        time.sleep(1) #delay 1 second
        #p.ChangeDutyCycle(2.5) #0 deg position
        #time.sleep(1) #delay 1 second again
        #p.ChangeDutyCycle(12.5) #180 deg position
        #time.sleep(1) #delay 1 second again... ...



except KeyboardInterrupt:
    #stop the pwm object
    p.stop()
    #clean up the GPIO port
    GPIO.cleanup()

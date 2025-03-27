import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

in1 = 20
in2 = 16
# in3 = 16
# in4 = 20
en = 21
mospwm = 12

#Stepper setup
control_pins = [26,19,13,6]
for pin in control_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

fullstep_seq = [
    [1,1,0,0],
    [0,1,1,0],
    [0,0,1,1],
    [1,0,0,1]]

RUNNING = True
STEPSPERREV = 512

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

while True:
    try:
        ps.ChangeDutyCycle(75)
        time.sleep(1)
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
    except KeyboardInterrupt:
        GPIO.cleanup()




#End the script and exit
GPIO.cleanup()
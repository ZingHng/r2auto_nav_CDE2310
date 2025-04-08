import time
import RPi.GPIO as GPIO
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def fire_sequence():
    print("FIRE AAAAAAAAAAAAAA")
    STEPSPERREV = 512
    GPIO.setmode(GPIO.BCM)

    in1 = 20
    in2 = 16
    en = 21

    #Stepper setup
    control_pins = [26,19,13,6]
    for pin in control_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)

    fullstep_seq = [
        [1,0,0,1],
        [0,0,1,1],
        [0,1,1,0],
        [1,1,0,0],
        ]

    #DC motor setup
    GPIO.setup(in1,GPIO.OUT)
    GPIO.setup(in2,GPIO.OUT)
    GPIO.setup(en,GPIO.OUT)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    p=GPIO.PWM(en,1000)
    p.start(15)

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
        time.sleep(5)
        print("Stepper Turn")
        StepperTurn()
        time.sleep(4)
        print("Stepper Turn")
        StepperTurn()
        time.sleep(2)
        print("Stepper Turn")
        StepperTurn()
        time.sleep(5)
        print("Flywheel Stop")
        FlywheelStop()
        print("Rest")
        time.sleep(3)
    except:
        FlywheelStop()
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    fire_sequence()
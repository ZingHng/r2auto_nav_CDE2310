import time
import RPi.GPIO as GPIO

def fire_sequence():
    print("FIRE AAAAAAAAAAAAAA")
    STEPSPERREV = 512
    GPIO.setmode(GPIO.BCM)

    in1 = 20
    in2 = 16
    en = 21
    mospwm = 12

    #Stepper setup
    control_pins = [26,19,13,6]
    for pin in control_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)

    fullstep_seq = [
        [1,0,0,1],
        [0,0,1,1],
        [0,1,1,0],
        [1,1,0,0]]

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
    
    ps.ChangeDutyCycle(75)
    print("Flywheel Start")
    FlywheelStart()
    time.sleep(2)
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
    ps.ChangeDutyCycle(0)
    print("Rest")
    time.sleep(3)
    GPIO.cleanup()

if __name__ == '__main__':
    fire_sequence()
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

control_pins = [26,19,13,6]
for pin in control_pins:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)

# Halfstep has more resolution, but lower torque, more responsive means approx same speed (not that it matters)
#halfstep_seq = [
#  [1,0,0,0],
#  [1,1,0,0],
#  [0,1,0,0],
#  [0,1,1,0],
#  [0,0,1,0],
#  [0,0,1,1],
#  [0,0,0,1],
#  [1,0,0,1]]

# Halfstep has less resolution, but higher torque
fullstep_seq = [
    [1,1,0,0],
    [0,1,1,0],
    [0,0,1,1],
    [1,0,0,1]]

RUNNING = True
STEPSPERREV = 512

while RUNNING:
  x = input("Angle in degrees")
  try:
    angle = int((float(x)/360)*STEPSPERREV)
  except:
    RUNNING = False
    angle = 0
    GPIO.cleanup()
  for i in range(angle):
    for fullstep in range(4):
      for pin in range(4):
        GPIO.output(control_pins[pin], fullstep_seq[fullstep][pin])
      time.sleep(0.001)

#End the script and exit
GPIO.cleanup()

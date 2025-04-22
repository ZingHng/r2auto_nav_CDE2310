import time
import busio
import board
import adafruit_amg88xx
import numpy as np
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

try:
    while True:
        #print(amg.pixels)
        '''for row in amg.pixels:
            print([temp for temp in row])
            print("")'''
        pixels = np.array(amg.pixels)
        pixels = np.reshape(pixels, 64)
        print(pixels)
        print("\n")
        time.sleep(1)
except KeyboardInterrupt:
    print("end")
import serial
import time
from struct import *

COM = '/dev/ttyAMA0'

ser = serial.Senrial(COM, 115200)   

x=109
# Y=209
# Z=98



while True:

    # data = [x,y,z]
    data = x
    ser.write(data)
    print("send")

    time.sleep(1)
    
ser.close() 

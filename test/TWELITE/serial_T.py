import serial
import time
from struct import *

COM = '/dev/ttyAMA0'

ser = serial.Serial(COM, 115200)   

x=109
y=209
z=98

#x=str(x)
#y=str(y)
#z=str(z)

while True:

    data = [x,y,z]
    ser.write(data)
    print("send")

    time.sleep(1)
    
ser.close()          




import serial
import time
from struct import *


COM = '/dev/ttyAMA0'


ser = serial.Serial(COM, 115200)   


X=109.243
Y=-209
Z=3098


while True:


    data = pack('>ddd', X, Y, Z)
    print(data)
    data = data + b'\n'
    ser.write(data)
    print("send")


    time.sleep(1)
    
ser.close()  


import serial
from serial.tools import list_ports

ser = serial.Serial()
devices = [info.device for info in list_ports.comports()]
print(devices)

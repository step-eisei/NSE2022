import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(22,  GPIO.OUT)

GPIO.output(22, True)
#ここの数字は実験次第
time.sleep(5)
GPIO.output(22, False)

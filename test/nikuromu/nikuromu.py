import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17,  GPIO.OUT)

GPIO.output(17, True)
#ここの数字は実験次第
time.sleep(10)
GPIO.output(17, False)
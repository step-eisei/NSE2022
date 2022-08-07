# Cleanupのみ

import RPi.GPIO as GPIO

# モータのセッティング
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
print("GPIO cleanup fin.")

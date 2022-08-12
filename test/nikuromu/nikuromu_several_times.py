import RPi.GPIO as GPIO
import time
for:
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(22,  GPIO.OUT)

  for i in range(4):
    GPIO.output(22, True)
    print("nchrm "+str(i))
    time.sleep(2)
    GPIO.output(22, False)
    time.sleep(5)

  GPIO.cleanup()
except KeyboardInterrupt:
  GPIO.cleanup()
  print("cleanup fin.")

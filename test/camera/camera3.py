import picamera
from time import sleep

camera = picamera.PiCamera()

camera.capture('image5.jpg')


camera.start_preview()

for i in range(60):
    camera.brightness = i
    sleep(0.2)
    
   
camera.stop_preview()  

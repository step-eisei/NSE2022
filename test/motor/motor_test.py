# 参考サイト：https://www.denshi.club/parts/2020/11/gpiozero8import-motor-2-tb6612.html



from gpiozero import Motor
from time import sleep

motor1 = Motor(23, 24)
motor2 = Motor(16, 26)
motor1.forward(0.2)
motor2.forward(0.2)
sleep(2)
motor1.backward(0.5)
motor2.backward(0.5)
sleep(3)
motor1.stop()
motor2.stop()

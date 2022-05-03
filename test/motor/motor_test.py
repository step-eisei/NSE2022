# 参考サイト：https://www.denshi.club/parts/2020/11/gpiozero8import-motor-2-tb6612.html



from gpiozero import Motor
from time import sleep

motor = Motor(17, 18)
motor.forward(0.2)
sleep(2)
motor.backward(0.5)
sleep(3)
motor.stop()

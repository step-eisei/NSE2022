import RPi.GPIO as GPIO
import time

PIN_AIN1 = 24
PIN_AIN2 = 23
PIN_PWMA = 12
PIN_BIN1 = 26
PIN_BIN2 = 16
PIN_PWMB = 13
DUTY_A = 59 # 念のため20より上には上げないように
DUTY_B = 65 # 念のため20より上には上げないように

GPIO.setmode(GPIO.BCM)

# 左モータ
GPIO.setup(PIN_AIN1, GPIO.OUT)
GPIO.setup(PIN_AIN2, GPIO.OUT)

# 左モータPWM
GPIO.setup(PIN_PWMA, GPIO.OUT)
pwm_left = GPIO.PWM(PIN_PWMA, 300)
pwm_left.start(10)
pwm_left.ChangeDutyCycle(0)

# 右モータ
GPIO.setup(PIN_BIN1, GPIO.OUT)
GPIO.setup(PIN_BIN2, GPIO.OUT)

# 右モータPWM
GPIO.setup(PIN_PWMB, GPIO.OUT)
pwm_right = GPIO.PWM(PIN_PWMB, 300)
pwm_right.start(10)
pwm_right.ChangeDutyCycle(0)

# sleep
time.sleep(2)

# ----------------------------------------

# 前進
GPIO.output(PIN_AIN1, GPIO.LOW)
GPIO.output(PIN_AIN2, GPIO.HIGH)
GPIO.output(PIN_BIN1, GPIO.HIGH)
GPIO.output(PIN_BIN2, GPIO.LOW)

for i in range(0, 101, 2):
  pwm_left.ChangeDutyCycle(i*DUTY_A/100)
  pwm_right.ChangeDutyCycle(i*DUTY_B/100)
  time.sleep(0.1)

for i in range(0, 101, 2):
  pwm_left.ChangeDutyCycle((100-i)*DUTY_A/100)
  pwm_right.ChangeDutyCycle((100-i)*DUTY_B/100)
  time.sleep(0.1)
time.sleep(3)

# # 後進
# GPIO.output(PIN_AIN1, GPIO.HIGH)
# GPIO.output(PIN_AIN2, GPIO.LOW)
# GPIO.output(PIN_BIN1, GPIO.LOW)
# GPIO.output(PIN_BIN2, GPIO.HIGH)
# time.sleep(3)

# # 右旋回
# GPIO.output(PIN_AIN1, GPIO.HIGH)
# GPIO.output(PIN_AIN2, GPIO.LOW)
# GPIO.output(PIN_BIN1, GPIO.HIGH)
# GPIO.output(PIN_BIN2, GPIO.LOW)
# time.sleep(3)

# # 左旋回
# GPIO.output(PIN_AIN1, GPIO.LOW)
# GPIO.output(PIN_AIN2, GPIO.HIGH)
# GPIO.output(PIN_BIN1, GPIO.LOW)
# GPIO.output(PIN_BIN2, GPIO.HIGH)
# time.sleep(3)


# # ----------------------------------------

# # # 右モータ後進
# # GPIO.output(PIN_AIN1, GPIO.HIGH)
# # GPIO.output(PIN_AIN2, GPIO.LOW)
# # time.sleep(3)

# # # 右モータブレーキ？
# # GPIO.output(PIN_AIN1, GPIO.HIGH)
# # GPIO.output(PIN_AIN2, GPIO.HIGH)
# # time.sleep(1)

# # # 右モータ前進
# # GPIO.output(PIN_AIN1, GPIO.LOW)
# # GPIO.output(PIN_AIN2, GPIO.HIGH)
# # time.sleep(3)

# # # 右モータ停止？
# # GPIO.output(PIN_AIN1, GPIO.LOW)
# # GPIO.output(PIN_AIN2, GPIO.LOW)
# # time.sleep(3)

# #----------------------------------------
# # 左モータ前進
# GPIO.output(PIN_BIN1, GPIO.HIGH)
# GPIO.output(PIN_BIN2, GPIO.LOW)
# time.sleep(3)

# # 左モータブレーキ？
# GPIO.output(PIN_BIN1, GPIO.HIGH)
# GPIO.output(PIN_BIN2, GPIO.HIGH)
# time.sleep(1)

# # 左モータ後進
# GPIO.output(PIN_BIN1, GPIO.LOW)
# GPIO.output(PIN_BIN2, GPIO.HIGH)
# time.sleep(3)

# # 左モータ停止？
# GPIO.output(PIN_BIN1, GPIO.LOW)
# GPIO.output(PIN_BIN2, GPIO.LOW)
# time.sleep(3)

#----------------------------------------
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()

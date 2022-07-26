import RPi.GPIO as GPIO
import time

PIN_AIN1 = 24
PIN_AIN2 = 23
PIN_PWMA = 12
PIN_BIN1 = 16
PIN_BIN2 = 26
PIN_PWMB = 13
DUTY_A = 5 # 念のため20より上には上げないように
DUTY_B = 5 # 念のため20より上には上げないように

GPIO.setmode(GPIO.BCM)

# 左モータ
GPIO.setup(PIN_AIN1, GPIO.OUT)
GPIO.setup(PIN_AIN2, GPIO.OUT)

# 左モータPWM
GPIO.setup(PIN_PWMA, GPIO.OUT)
pwm_left = GPIO.PWM(PIN_PWMA, 300)
pwm_left.start(10)
pwm_left.ChangeDutyCycle(DUTY_A)

# 右モータ
GPIO.setup(PIN_BIN1, GPIO.OUT)
GPIO.setup(PIN_BIN2, GPIO.OUT)

# 右モータPWM
GPIO.setup(PIN_PWMB, GPIO.OUT)
pwm_right = GPIO.PWM(PIN_PWMB, 300)
pwm_right.start(10)
pwm_right.ChangeDutyCycle(DUTY_B)

# sleep
time.sleep(2)

# ----------------------------------------
# 右モータ後進
"""
print("motorA starts going forward")
GPIO.output(PIN_AIN1, GPIO.HIGH)
GPIO.output(PIN_AIN2, GPIO.LOW)
time.sleep(5)
"""
# 右モータブレーキ？
# GPIO.output(PIN_AIN1, GPIO.HIGH)
# GPIO.output(PIN_AIN2, GPIO.HIGH)
# time.sleep(1)

# 右モータ前進
print("motorA starts going backward")
GPIO.output(PIN_AIN1, GPIO.LOW)
GPIO.output(PIN_AIN2, GPIO.HIGH)
#time.sleep(1)

"""
# 右モータ停止？
GPIO.output(PIN_AIN1, GPIO.LOW)
GPIO.output(PIN_AIN2, GPIO.LOW)
time.sleep(3)
"""
#----------------------------------------
# 左モータ前進
print("motorB starts going forward")
GPIO.output(PIN_BIN1, GPIO.HIGH)
GPIO.output(PIN_BIN2, GPIO.LOW)
time.sleep(1)

# 左モータブレーキ？
# GPIO.output(PIN_BIN1, GPIO.HIGH)
# GPIO.output(PIN_BIN2, GPIO.HIGH)
# time.sleep(1)

"""
# 左モータ後進
print("motorB starts going backward")
GPIO.output(PIN_BIN1, GPIO.LOW)
GPIO.output(PIN_BIN2, GPIO.HIGH)
time.sleep(5)
"""
"""
# 左モータ停止？
GPIO.output(PIN_BIN1, GPIO.LOW)
GPIO.output(PIN_BIN2, GPIO.LOW)
time.sleep(3)
"""
#----------------------------------------
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()

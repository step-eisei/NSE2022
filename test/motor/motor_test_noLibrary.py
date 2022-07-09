import RPi.GPIO as GPIO
import time

PIN_AIN1 = 24
PIN_AIN2 = 23
PIN_PWMA = 12
PIN_BIN1 = 16
PIN_BIN2 = 26
PIN_PWMB = 13
DUTY_A = 30
DUTY_B = 30

GPIO.setmode(GPIO.BCM)

# # 左モータ
# GPIO.setup(14, GPIO.OUT)
# GPIO.setup(15, GPIO.OUT)
GPIO.setup(PIN_AIN1, GPIO.OUT)
GPIO.setup(PIN_AIN2, GPIO.OUT)

# # 左モータPWM
# GPIO.setup(18, GPIO.OUT)
# pwm_left = GPIO.PWM(18, 50)
# pwm_left.start(10)
# pwm_left.ChangeDutyCycle(80)
GPIO.setup(PIN_PWMA, GPIO.OUT)
pwm_left = GPIO.PWM(PIN_PWMA, DUTY_A)
pwm_left.start(10)
pwm_left.ChangeDutyCycle(DUTY_A)


# # 右モータ
# GPIO.setup(23, GPIO.OUT)
# GPIO.setup(24, GPIO.OUT)

# # 右モータPWM
# GPIO.setup(25, GPIO.OUT)
# pwm_right = GPIO.PWM(25, 50)
# pwm_right.start(10)
# pwm_right.ChangeDutyCycle(80)

# sleep
# time.sleep(2)

# # ----------------------------------------
# # 左モータ前進
# GPIO.output(14, GPIO.HIGH)
# GPIO.output(15, GPIO.LOW)
# time.sleep(3)

# # 左モータブレーキ？
# GPIO.output(14, GPIO.HIGH)
# GPIO.output(15, GPIO.HIGH)
# time.sleep(1)

# # 左モータ後進
# GPIO.output(14, GPIO.LOW)
# GPIO.output(15, GPIO.HIGH)
# time.sleep(3)

# # 左モータ停止？
# GPIO.output(14, GPIO.LOW)
# GPIO.output(15, GPIO.LOW)
# time.sleep(3)

# #----------------------------------------
# # 右モータ前進
# GPIO.output(23, GPIO.HIGH)
# GPIO.output(24, GPIO.LOW)
# time.sleep(3)

# # 右モータブレーキ？
# GPIO.output(23, GPIO.HIGH)
# GPIO.output(24, GPIO.HIGH)
# time.sleep(1)

# # 右モータ後進
# GPIO.output(23, GPIO.LOW)
# GPIO.output(24, GPIO.HIGH)
# time.sleep(3)

# # 右モータ停止？
# GPIO.output(23, GPIO.LOW)
# GPIO.output(24, GPIO.LOW)
# time.sleep(3)

#----------------------------------------
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()

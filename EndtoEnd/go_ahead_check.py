# モータによる前後進能力，回転能力を把握するプログラム

import RPi.GPIO as GPIO
import math
import time

# モータのピン割り当て(GPIO 〇〇)
PIN_AIN1 = 24   # 右モータ(A)
PIN_AIN2 = 23
PIN_PWMA = 12
PIN_BIN1 = 26   # 左モータ(B)
PIN_BIN2 = 16
PIN_PWMB = 13
# 左右のduty比(定義域：0~100)
DUTY_A = 59 # 20~40でICが高温になります．60~70が妥当です
DUTY_B = 65 # 20~40でICが高温になります．60~70が妥当です
freq = 300 # PWMの周波数

x_now = 10
y_now = 10
T_straight = 0

# 機体を前進させる関数
def go_ahead():
#     # モータのセッティング
#     GPIO.setmode(GPIO.BCM)
#     # 左モータ
#     GPIO.setup(PIN_AIN1, GPIO.OUT)
#     GPIO.setup(PIN_AIN2, GPIO.OUT)
#     # 左モータPWM
#     GPIO.setup(PIN_PWMA, GPIO.OUT)
#     pwm_left = GPIO.PWM(PIN_PWMA, freq)
#     pwm_left.start(10)
#     # 右モータ
#     GPIO.setup(PIN_BIN1, GPIO.OUT)
#     GPIO.setup(PIN_BIN2, GPIO.OUT)
#     # 右モータPWM
#     GPIO.setup(PIN_PWMB, GPIO.OUT)
#     pwm_right = GPIO.PWM(PIN_PWMB, freq)
#     pwm_right.start(10)
#     # sleep
#     time.sleep(2)
    # 右モータ前進
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.HIGH)
    # 左モータ前進
    GPIO.output(PIN_BIN1, GPIO.HIGH)
    GPIO.output(PIN_BIN2, GPIO.LOW)
    # 0からDUTYまで数秒かけて上げる
    for i in range(0, 101, 2):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle(i*DUTY_A/100)
            pwm_right.ChangeDutyCycle(i*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle(i*DUTY_A/200)
            pwm_right.ChangeDutyCycle(i*DUTY_B/200)
            time.sleep(0.07)
    # sleep
    time.sleep(T_straight)
    # DUTYから0まで数秒かけて下げる
    for i in range(0, 101, 2):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/100)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/200)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/200)
            time.sleep(0.07)
    time.sleep(2)
    # モータの解放
#     pwm_right.stop()
#     pwm_left.stop()
#     GPIO.cleanup()



# 機体を後進，急停止させる関数
def go_stop():
    # 右モータ後進
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    # 左モータ後進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    # 0からDUTYまで数秒かけて上げる
    for i in range(0, 101, 2):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle(i*DUTY_A/100)
            pwm_right.ChangeDutyCycle(i*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle(i*DUTY_A/200)
            pwm_right.ChangeDutyCycle(i*DUTY_B/200)
            time.sleep(0.07)
    # sleep
    time.sleep(T_straight)
    # DUTYから0まで数秒かけて下げる
    for i in range(0, 101, 50):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/100)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/200)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/200)
            time.sleep(0.07)
    time.sleep(2)

def go_stop2(): # 前進速め
    # 右モータ後進
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    # 左モータ後進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    # 0からDUTYまで数秒かけて上げる
    for i in range(0, 101, 5):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle(i*DUTY_A/100)
            pwm_right.ChangeDutyCycle(i*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle(i*DUTY_A/200)
            pwm_right.ChangeDutyCycle(i*DUTY_B/200)
            time.sleep(0.07)
    # sleep
    time.sleep(T_straight)
    # DUTYから0まで数秒かけて下げる
    for i in range(0, 101, 50):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/100)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/200)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/200)
            time.sleep(0.07)
    time.sleep(2)

def go_stop3(): # 急前進
    # 右モータ後進
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    # 左モータ後進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    # 0からDUTYまで数秒かけて上げる
    for i in range(0, 101, 50):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle(i*DUTY_A/100)
            pwm_right.ChangeDutyCycle(i*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle(i*DUTY_A/200)
            pwm_right.ChangeDutyCycle(i*DUTY_B/200)
            time.sleep(0.07)
    # sleep
    time.sleep(T_straight)
    # DUTYから0まで数秒かけて下げる
    for i in range(0, 101, 2):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/100)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/200)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/200)
            time.sleep(0.07)
    time.sleep(2)

def go_stop4(): # 急前進，減速速め
    # 右モータ後進
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    # 左モータ後進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    # 0からDUTYまで数秒かけて上げる
    for i in range(0, 101, 50):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle(i*DUTY_A/100)
            pwm_right.ChangeDutyCycle(i*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle(i*DUTY_A/200)
            pwm_right.ChangeDutyCycle(i*DUTY_B/200)
            time.sleep(0.07)
    # sleep
    time.sleep(T_straight)
    # DUTYから0まで数秒かけて下げる
    for i in range(0, 101, 5):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/100)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle((100-i)*DUTY_A/200)
            pwm_right.ChangeDutyCycle((100-i)*DUTY_B/200)
            time.sleep(0.07)
    time.sleep(2)

# motorをセットアップする
INITIAL_DUTY_A = 0
INITIAL_DUTY_B = 0
ini_freq = 300          # pwm周波数
# モータのセッティング
GPIO.setmode(GPIO.BCM)
# 左モータ
GPIO.setup(PIN_AIN1, GPIO.OUT)
GPIO.setup(PIN_AIN2, GPIO.OUT)
# 左モータPWM
GPIO.setup(PIN_PWMA, GPIO.OUT)
pwm_left = GPIO.PWM(PIN_PWMA, ini_freq)
pwm_left.start(10)
pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
# 右モータ
GPIO.setup(PIN_BIN1, GPIO.OUT)
GPIO.setup(PIN_BIN2, GPIO.OUT)
# 右モータPWM
GPIO.setup(PIN_PWMB, GPIO.OUT)
pwm_right = GPIO.PWM(PIN_PWMB, ini_freq)
pwm_right.start(10)
pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
# sleep
time.sleep(2)

# ループ(3mゴールまで)
try:
    go_stop()
    print("went ahead")
    pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
    pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
        
except KeyboardInterrupt:
    pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
    pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()

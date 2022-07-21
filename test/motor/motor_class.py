import RPi.GPIO as GPIO
import time

DUTY_A = 20 # 念のため20より上には上げないように
DUTY_B = 20 # 念のため20より上には上げないように
stright_time = 5

class motor:
  def __init__(self):
    PIN_AIN1 = 24
    PIN_AIN2 = 23
    PIN_PWMA = 12
    PIN_BIN1 = 16
    PIN_BIN2 = 26
    PIN_PWMB = 13
    GPIO.setmode(GPIO.BCM)
    # 左モータ
    GPIO.setup(PIN_AIN1, GPIO.OUT)
    GPIO.setup(PIN_AIN2, GPIO.OUT)
    # 左モータPWM
    GPIO.setup(PIN_PWMA, GPIO.OUT)
    pwm_left = GPIO.PWM(PIN_PWMA, DUTY_A)
    pwm_left.start(10)
    pwm_left.ChangeDutyCycle(DUTY_A)
    # 右モータ
    GPIO.setup(PIN_BIN1, GPIO.OUT)
    GPIO.setup(PIN_BIN2, GPIO.OUT)
    # 右モータPWM
    GPIO.setup(PIN_PWMB, GPIO.OUT)
    pwm_right = GPIO.PWM(PIN_PWMB, DUTY_B)
    pwm_right.start(10)
    pwm_right.ChangeDutyCycle(DUTY_B)

  # ----------------------------------------
  
  # 前進
  def go_ahead(self):
    # 右モータの前進
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.HIGH)
    # 左モータの前進
    GPIO.output(PIN_BIN1, GPIO.HIGH)
    GPIO.output(PIN_BIN2, GPIO.LOW)
    time.sleep(stright_time)

  # 後進
　def go_back(self):
    # 右モータの前進
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    # 左モータの前進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    time.sleep(stright_time)

  # 旋回
  def rotate(self,theta_relative):
    
    const = 1/27 # 単位角度における回転所要時間
    
    if theta_relative > 0: # 右旋回
      # 右モータ前進 
      GPIO.output(PIN_AIN1, GPIO.HIGH)
      GPIO.output(PIN_AIN2, GPIO.LOW)
      # 左モータ前進
      GPIO.output(PIN_BIN1, GPIO.HIGH)
      GPIO.output(PIN_BIN2, GPIO.LOW)
    else: # 左旋回  
      # 右モータ前進
      GPIO.output(PIN_AIN1, GPIO.LOW)
      GPIO.output(PIN_AIN2, GPIO.HIGH)
      # 左モータ前進
      GPIO.output(PIN_BIN1, GPIO.LOW)
      GPIO.output(PIN_BIN2, GPIO.HIGH)
      
    time.sleep(math.fabs(theta_relative)*const)

  # モータの解放
  pwm_left.stop()
  pwm_right.stop()
  GPIO.cleanup()

  
  

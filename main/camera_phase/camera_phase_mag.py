'''
- ダミーデータ
 - x_now, y_now
 - mag.csv
 
- 変更点
 - printの追加
 - csv_write(*data)のコメントアウト
'''

import time
import math
import RPi.GPIO as GPIO
import numpy as np
import serial
import csv
import threading

import FaBo9Axis_MPU9250
import sys
import datetime
import re

from PIL import Image,ImageOps
import picamera

image=Image
imageo=ImageOps
import RPi.GPIO as GPIO
camera=picamera.PiCamera()

mpu9250 = FaBo9Axis_MPU9250.MPU9250()


# モータのピン割り当て(GPIO 〇〇)
PIN_AIN1 = 24   # 右モータ(A)
PIN_AIN2 = 23
PIN_PWMA = 12
PIN_BIN1 = 16   # 左モータ(B)
PIN_BIN2 = 26
PIN_PWMB = 13
# 左右のduty比(定義域：0~100)
DUTY_A = 62 # 20~40でICが高温になります．60~70が妥当です
DUTY_B = 60 # 20~40でICが高温になります．60~70が妥当です
freq = 300 # PWMの周波数

T_straight = 0
final_distance = 3

satellites_used = 0
stack = False

takepic_counter = 1
borderprop = 3
theta_relative = 0
prop = 0

x_now = 0.1 # ダミー
y_now = 0.1 # ダミー

# ダミー
with open ('mag.csv', 'r' ) as f :
    reader = csv.reader(f)
    line = [row for row in reader]
    magX_max = float(line[1][0])
    magX_min = float(line[1][1])
    magY_max = float(line[1][2])
    magY_min = float(line[1][3])
print("magX_max = " + str(magX_max))
print("magX_min = " + str(magX_min))
print("magY_max = " + str(magY_max))
print("magY_min = " + str(magY_min))
    
# 機体を旋回させる関数
def rotate(theta_relative):
    R_DUTY_A = 10
    R_DUTY_B = 10
    const = 10/765        # 単位角度における回転所要時間
    if(theta_relative > 0): # 左に旋回
        # 右モータ前進
        GPIO.output(PIN_AIN1, GPIO.LOW)
        GPIO.output(PIN_AIN2, GPIO.HIGH)
        # 左モータ後進
        GPIO.output(PIN_BIN1, GPIO.LOW)
        GPIO.output(PIN_BIN2, GPIO.HIGH)
    if(theta_relative < 0): # 右に旋回
        # 右モータ後進
        GPIO.output(PIN_AIN1, GPIO.HIGH)
        GPIO.output(PIN_AIN2, GPIO.LOW)
        # 左モータ前進
        GPIO.output(PIN_BIN1, GPIO.HIGH)
        GPIO.output(PIN_BIN2, GPIO.LOW)
    # カクカクver.
    pwm_left.ChangeDutyCycle(R_DUTY_A)
    pwm_right.ChangeDutyCycle(R_DUTY_B)
    time.sleep(math.fabs(theta_relative)*const)
    """
    # なめらかver.
    for i in range(0, 101, 5):
        pwm_left.ChangeDutyCycle(R_DUTY_A*i/100)
        pwm_right.ChangeDutyCycle(R_DUTY_B*i/100)
        time.sleep(math.fabs(theta_relative)*const/20)
    for i in range(0, 101, 5):
        pwm_left.ChangeDutyCycle(R_DUTY_A*(100-i)/100)
        pwm_right.ChangeDutyCycle(R_DUTY_B*(100-i)/100)
        time.sleep(math.fabs(theta_relative)*const/20)
    """
    pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
    pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
    time.sleep(3)
    """
     # モータのセッティング
     GPIO.setmode(GPIO.BCM)
     # 左モータ
     GPIO.setup(PIN_AIN1, GPIO.OUT)
     GPIO.setup(PIN_AIN2, GPIO.OUT)
     # 左モータPWM
     GPIO.setup(PIN_PWMA, GPIO.OUT)
     pwm_left = GPIO.PWM(PIN_PWMA, freq)
     pwm_left.start(10)
    pwm_left.ChangeDutyCycle(R_DUTY_A)
     # 右モータ
     GPIO.setup(PIN_BIN1, GPIO.OUT)
     GPIO.setup(PIN_BIN2, GPIO.OUT)
     # 右モータPWM
     GPIO.setup(PIN_PWMB, GPIO.OUT)
     pwm_right = GPIO.PWM(PIN_PWMB, freq)
     pwm_right.start(10)
    pwm_right.ChangeDutyCycle(R_DUTY_B)
     # sleep
     time.sleep(2)
     print("set up finished")
    """
#     # モータの解放
#     pwm_right.stop()
#     pwm_left.stop()
#     GPIO.cleanup()

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
            time.sleep(0.05)
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
            time.sleep(0.05)
    time.sleep(2)
    # モータの解放
#     pwm_right.stop()
#     pwm_left.stop()
#     GPIO.cleanup()

# 機体を後進させる関数
def go_back():
    # モータのセッティング
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
    # 右モータ後進
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    pwm_left.ChangeDutyCycle(DUTY_A)
    # 左モータ後進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    pwm_right.ChangeDutyCycle(DUTY_B)
    # 0からDUTYまで数秒かけて上げる
    for i in range(0, 101, 2):
        if(math.sqrt( x_now**2 + y_now**2 ) > 7): 
            pwm_left.ChangeDutyCycle(i*DUTY_A/100)
            pwm_right.ChangeDutyCycle(i*DUTY_B/100)
            time.sleep(0.1)
        else: 
            pwm_left.ChangeDutyCycle(i*DUTY_A/200)
            pwm_right.ChangeDutyCycle(i*DUTY_B/200)
            time.sleep(0.05)
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
            time.sleep(0.05)
    time.sleep(2)
    # モータの解放
#     pwm_right.stop()
#     pwm_left.stop()
#     GPIO.cleanup()

    
# 角度取得関数
def magnet():
    # データを取り始めて数データはノイズが大きい可能性
    # サンプリング周期が小さいとノイズが大きい可能性
    magXs=[]
    magYs=[]
    for i in range(5):
        mag = mpu9250.readMagnet()
        # print(" mx = " , ( mag['x']   ), end='')
        # print(" my = " , ( mag['y']   ), end='')
        # print(" mz = " , ( mag['z'] ))
        # print()

        # キャリブレーション
        magX_calibrated = (mag['x']-(magX_max + magX_min)/2) / ((magX_max - magX_min)/2)
        magY_calibrated = (mag['y']-(magY_max + magY_min)/2) / ((magY_max - magY_min)/2)
        
        #リスト追加
        magXs.append(magX_calibrated)
        magYs.append(magY_calibrated)
        time.sleep(0.2)
        
    # ローパスフィルタ
    magX_mean = sum(magXs)/len(magXs)
    magY_mean = sum(magYs)/len(magYs)

    # とりあえずatan2に入れたものをtheta_absoluteとしているが，本当に欲しいtheta_absoluteにするには演算が必要かも
    theta_absolute = math.atan2(-magY_calibrated, -magX_calibrated)*180/math.pi
    theta_absolute_lowPass = math.atan2(-magY_mean, -magX_mean)*180/math.pi
    print(f"lowPass = {theta_absolute_lowPass}, notlowPass = {theta_absolute}")
    return theta_absolute_lowPass


# 以下カメラ用関数
# ----------------------------------------------------------------------------------------------

def hsv_binary(img_hsv,sat_avg,val_avg):
    #画像のチャンネルを分ける
    #画像のチャンネルを分ける img[縦，横，[h,s,v]]　この行程でグレースケールになる
    im_h= img_hsv[:, :, 0] + 0*img_hsv[:, :, 1] + 0*img_hsv[:, :, 2] # h以外の情報を消去してim_hに格納
    im_s= 0*img_hsv[:, :, 0] + img_hsv[:, :, 1] + 0*img_hsv[:, :, 2] # s以外の情報を消去してim_sに格納
    im_v= 0*img_hsv[:, :, 0] + 0*img_hsv[:, :, 1] + img_hsv[:, :, 2] # v以外の情報を消去してim_vに格納


    # 条件を満たしていれば1, それ以外は0に置換
    # np.where(条件, Trueの時に置換する数, Falseの時に置換する数)
    #色相環は360度→0～255に変換
    #色相環の内、赤色は0度と360度をまたぐ
    img_h_th = np.where((im_h < 15/360*255) | (im_h > 175/360*255), 1, 0)
    img_s_th = np.where(im_s > sat_avg*0.9, 1, 0)
    img_v_th = np.where(im_v > val_avg*0.9, 1, 0)

    #行列の掛け算ではなく各要素の掛け算をする 上記の条件で一つでも満たしていないものがあれば，0となる．
    #検出された物を白にするために最後に255を掛ける(この時点で2値化)
    img_th = img_h_th * img_s_th * img_v_th * 255 # 条件を満たした要素は255，それ以外は0

    img_th = np.uint8(img_th) # np.uint8 unsigned int (0 ～ 255)

    return img_th

def sv_scan(img_hsv):
    # img[:,:,0]　色相(Hue)
    # img[:,:,1]　彩度(Saturation)
    # img[:,:,2]　明度(Value)
    
    im_s = 0*img_hsv[:, :, 0] + img_hsv[:, :, 1] + 0*img_hsv[:, :, 2] #彩度以外の情報を消去して、im_sに格納
    sat_avg = np.average(im_s) #画像の彩度の平均を求める
    
    im_v = 0*img_hsv[:, :, 0] + 0*img_hsv[:, :, 1] + img_hsv[:, :, 2]
    val_avg = np.average(im_v)
    
    return sat_avg,val_avg

def scantheta(img_th):

    # 行列サイズ
    # img_th[縦,横,色]
    width=img_th.shape[1] #横の長さ
    # hight=img_th.shape[1]

    #列の和
    im_thx=np.sum(img_th,axis=0)

    theta_row=np.argmax(im_thx) # 最大の要素のインデックスを取得

    theta=-(theta_row/width*62.2-31.1)

    return theta

def scanprop(img_th):
    #赤の割合を計算する
    width=img_th.shape[1]
    height=img_th.shape[0]
    size=width*height

    red_area=np.count_nonzero(img_th)

    prop=(red_area/size)*100

    return prop

def takepic():
    # 撮影
    global takepic_counter
    now_time_camera = datetime.datetime.now()
    filename_camera = now_time_camera.strftime('%m%d_%H%M_')+str(takepic_counter)
    camera.capture("image"+filename_camera+".jpg")

    # 読み込み
    img = image.open ("image"+filename_camera+".jpg")
    #hsv空間に変換 「色相(Hue)」「彩度(Saturation)」「明度(Value)」
    img_hsv = image.open("image"+filename_camera+".jpg").convert('HSV')
    #それぞれ上下左右反転し，Pillow → Numpyへ変換
    # 上下反転メソッド　flip()
    # 左右反転メソッド　mirror()
    img = np.array(imageo.mirror(imageo.flip(img)))
    img_hsv=np.array(imageo.mirror(imageo.flip(img_hsv)))

    # 解析
    sv_avg = sv_scan(img_hsv) # 画像全体の彩度(Saturation),明度(Value)の平均を取得
    sat_avg = sv_avg[0]
    val_avg = sv_avg[1]
    
    img_th = hsv_binary(img_hsv,sat_avg,val_avg) #条件を満たす要素を255，それ以外を0とする配列
    (image.fromarray(img_th)).save("scanth"+filename_camera+".jpg")
    
    takepic_counter += 1
    theta=scantheta(img_th)
    prop=scanprop(img_th)

    data = (theta,prop)
    # csv_write(*data)

    return theta,prop

# -----------------------------------------------------------------------------------------------

# ここからメイン
print("main started")

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
print("set up finished")


try:
    # 赤コーン探索フェーズ
    print("searching the red cone...")
    max_prop_mag = magnet()
    max_prop = 0
    
    for i in range(15):
        data = takepic()
        prop = data[1]
        
        print(f"prop={prop}")
        if prop > max_prop:
            max_prop_mag = magnet()
            max_prop = prop
        
        print("rotate 30 deg")
        rotate(30)
        
    mag = magnet()
    print("rotate mag = " + str(mag) + " deg")
    rotate(mag)
    print("rotate -max_prop_mag = " + str(-max_prop_mag) + " deg")
    rotate(-max_prop_mag)

    # 赤コーン接近フェーズ 
    print("approaching the red cone...")
    DUTY_A = 31
    DUTY_B = 30   
    for i in range(4):
        data = takepic()
        theta = data[0]
        print(f"theta={theta}")
        rotate(theta_relative)
        go_ahead()
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    print("goal!")
        
except KeyboardInterrupt:
    pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
    pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()

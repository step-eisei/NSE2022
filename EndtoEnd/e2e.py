# 案1 rungps()関数をthreadする方法
# 未定義：モータ制御の時間など，座標計算時の定数をループするか，スタックの条件
# magnetは作っていません
# 制御履歴の保存先はphase1_record.csv
# 移動検知は3.5mとした．スタックの過剰検知に寄せる．後々検知幅を小さくする予定

from xml.dom.expatbuilder import parseString
from xmlrpc.client import NOT_WELLFORMED_ERROR

from smbus import SMBus

import time
import math
import RPi.GPIO as GPIO
import numpy as np
import serial
import micropyGPS
import csv
import threading

import FaBo9Axis_MPU9250
import sys
import datetime
import re
import os

from PIL import Image,ImageOps
import picamera

image=Image
imageo=ImageOps
camera=picamera.PiCamera()

mpu9250 = FaBo9Axis_MPU9250.MPU9250()
image_folder="image_jpg_folder"
scanth_folder="scanth_jpg_folder"
os.makedirs(image_folder, exist_ok=True)
os.makedirs(scanth_folder, exist_ok=True)
# ゴール座標を保存したCSVファイルの読み込み
with open ('goal.csv', 'r') as f :
    reader = csv.reader(f)
    line = [row for row in reader]
    goal_latitude = float(line[ 1 ] [ 0 ])
    goal_longitude = float(line[ 1 ] [ 1 ])


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
min_satellites_used = 10

gps_latitude = 0
gps_longitude = 0
x_now = 0
y_now = 0
x_past = 0
y_past = 0
x_goal = 0
y_goal = 0
satellites_used = 0
stack = False

takepic_counter = 1
borderprop = 3
theta_relative = 0
prop = 0


# 以下，キャリブレーションにより計算した最大値と最小値
with open ('mag.csv', 'r' ) as f :
    reader = csv.reader(f)
    line = [row for row in reader]
    magX_max = float(line[1][0])
    magX_min = float(line[1][1])
    magY_max = float(line[1][2])
    magY_min = float(line[1][3])

magXs = [0]*5
magYs = [0]*5

def nchrm(): #ニクロム線加熱
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(22,  GPIO.OUT)

    GPIO.output(22, True)
    #ここの数字は実験次第
    time.sleep(2)
    GPIO.output(22, False)
    time.sleep(5)

def get_pressure():
    bus_number  = 1
    i2c_address = 0x76
    bus = SMBus(bus_number)

    digT = []
    digP = []
    digH = []

    t_fine = 0.0

    def writeReg(reg_address, data):
        bus.write_byte_data(i2c_address,reg_address,data)

    def get_calib_param():
        calib = []

        for i in range (0x88,0x88+24):
            calib.append(bus.read_byte_data(i2c_address,i))
        calib.append(bus.read_byte_data(i2c_address,0xA1))
        for i in range (0xE1,0xE1+7):
            calib.append(bus.read_byte_data(i2c_address,i))

        digT.append((calib[1] << 8) | calib[0])
        digT.append((calib[3] << 8) | calib[2])
        digT.append((calib[5] << 8) | calib[4])
        digP.append((calib[7] << 8) | calib[6])
        digP.append((calib[9] << 8) | calib[8])
        digP.append((calib[11]<< 8) | calib[10])
        digP.append((calib[13]<< 8) | calib[12])
        digP.append((calib[15]<< 8) | calib[14])
        digP.append((calib[17]<< 8) | calib[16])
        digP.append((calib[19]<< 8) | calib[18])
        digP.append((calib[21]<< 8) | calib[20])
        digP.append((calib[23]<< 8) | calib[22])
        digH.append( calib[24] )
        digH.append((calib[26]<< 8) | calib[25])
        digH.append( calib[27] )
        digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        digH.append( calib[31] )

        for i in range(1,2):
            if digT[i] & 0x8000:
                digT[i] = (-digT[i] ^ 0xFFFF) + 1

        for i in range(1,8):
            if digP[i] & 0x8000:
                digP[i] = (-digP[i] ^ 0xFFFF) + 1

        for i in range(0,6):
            if digH[i] & 0x8000:
                digH[i] = (-digH[i] ^ 0xFFFF) + 1  

    def readData():
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(bus.read_byte_data(i2c_address,i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]

        compensate_T(temp_raw)
        x=compensate_P(pres_raw)

        return x

    def compensate_P(adc_P):
        global  t_fine

        v1 = (t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
        v2 = v2 + ((v1 * digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (digP[3] * 65536.0)
        v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * digP[0]) / 32768

        #if v1 == 0:
            #return 0
        pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)  

        pressure = pressure/100
        return pressure
        #print('pressure : {} hPa'.format(pressure/100))
        

        # print "pressure : %7.2f hPa" % (pressure/100)

    def compensate_T(adc_T):
        global t_fine
        v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
        v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
        t_fine = v1 + v2


    def setup():
        osrs_t = 1			#Temperature oversampling x 1
        osrs_p = 1			#Pressure oversampling x 1
        osrs_h = 1			#Humidity oversampling x 1
        mode   = 3			#Normal mode
        t_sb   = 5			#Tstandby 1000ms
        filter = 0			#Filter off
        spi3w_en = 0			#3-wire SPI Disable

        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
        ctrl_hum_reg  = osrs_h

        writeReg(0xF2,ctrl_hum_reg)
        writeReg(0xF4,ctrl_meas_reg)
        writeReg(0xF5,config_reg)

    setup()
    get_calib_param()


    if __name__ == '__main__':
        try:
            x=readData() #気圧の値読み取り
            
            return x #pressure関数が呼び出されたら渡す
        except KeyboardInterrupt:
            pass


def average_pressure():
    sum=0.0
    land=0.0
    
    for i in range(20):
        land=get_pressure()
        sum+=land
        time.sleep(0.1)

    average_pressure=sum/20
    return average_pressure


# threadにする関数.gpsを取得し続ける
def rungps(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/ttySOFT0', 4800, timeout=20)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    print("read gps")
    while True:
        try:
            sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
            if sentence[0] != '$': # 先頭が'$'でなければ捨てる
                continue
            for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
                gps.update(x)
        except:
            print("GPS error, but ignored")

# gpsを取得する関数
def getgps():
    global gps_latitude
    global gps_longitude
    global gps
    while True:
        if gps.clean_sentences > 20: # ちゃんとしたデーターがある程度たまったら出力する
            h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
            #print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
            #print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))

            gps_latitude = gps.latitude[0]
            gps_longitude = gps.longitude[0]


            #print('海抜: %f' % gps.altitude)
            #print(gps.satellites_used)
            #print('衛星番号: (仰角, 方位角, SN比)')
            #for k, v in gps.satellite_data.items():
            #    print('%d: %s' % (k, v))
            #print('')
            #print(gps_latitude)
            #print(gps_longitude)
            break
        time.sleep(3)
    
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

# ゴール角度，機体の角度から機体の回転角度を求める関数
def angle(x_now, y_now, theta_absolute):
    # ゴール角度算出
    theta_gps = math.atan2(y_now, x_now) * 180/math.pi
    # 機体正面を0として，左を正，右を負とした変数(-180~180)を作成
    theta_relative = theta_gps + theta_absolute + 90
    if(theta_relative > 180): theta_relative -= 360
    if(theta_relative < -180): theta_relative += 360
    return theta_relative

# gpsからゴール基準で自己位置を求める関数(国土地理院より)
def calc_xy(gps_latitude, gps_longitude, goal_latitude, goal_longitude):
    
    """ 緯度経度を平面直角座標に変換する
    - input:
        (gps_latitude, gps_longitude): 変換したい緯度・経度[度]（分・秒でなく小数であることに注意）
        (goal_latitude, goal_longitude): 平面直角座標系原点の緯度・経度[度]（分・秒でなく小数であることに注意）
    - output:
        x: 変換後の平面直角座標[m]
        y: 変換後の平面直角座標[m]
    """
    # 緯度経度・平面直角座標系原点をラジアンに直す
    phi_rad = np.deg2rad(gps_latitude)
    lambda_rad = np.deg2rad(gps_longitude)
    phi0_rad = np.deg2rad(goal_latitude)
    lambda0_rad = np.deg2rad(goal_longitude)

    # 補助関数
    def A_array(n):
        A0 = 1 + (n**2)/4. + (n**4)/64.
        A1 = -     (3./2)*( n - (n**3)/8. - (n**5)/64. ) 
        A2 =     (15./16)*( n**2 - (n**4)/4. )
        A3 = -   (35./48)*( n**3 - (5./16)*(n**5) )
        A4 =   (315./512)*( n**4 )
        A5 = -(693./1280)*( n**5 )
        return np.array([A0, A1, A2, A3, A4, A5])

    def alpha_array(n):
        a0 = np.nan # dummy
        a1 = (1./2)*n - (2./3)*(n**2) + (5./16)*(n**3) + (41./180)*(n**4) - (127./288)*(n**5)
        a2 = (13./48)*(n**2) - (3./5)*(n**3) + (557./1440)*(n**4) + (281./630)*(n**5)
        a3 = (61./240)*(n**3) - (103./140)*(n**4) + (15061./26880)*(n**5)
        a4 = (49561./161280)*(n**4) - (179./168)*(n**5)
        a5 = (34729./80640)*(n**5)
        return np.array([a0, a1, a2, a3, a4, a5])

    # 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
    m0 = 0.9999 
    a = 6378137.
    F = 298.257222101

    # (1) n, A_i, alpha_iの計算
    n = 1. / (2*F - 1)
    A_array = A_array(n)
    alpha_array = alpha_array(n)

    # (2), S, Aの計算
    A_ = ( (m0*a)/(1.+n) )*A_array[0] # [m]
    S_ = ( (m0*a)/(1.+n) )*( A_array[0]*phi0_rad + np.dot(A_array[1:], np.sin(2*phi0_rad*np.arange(1,6))) ) # [m]

    # ここまで定数．今後はA_，　S_，　alpha_arrayのみを利用
    
    # (3) lambda_c, lambda_sの計算
    lambda_c = np.cos(lambda_rad - lambda0_rad)
    lambda_s = np.sin(lambda_rad - lambda0_rad)

    # (4) t, t_の計算
    t = np.sinh( np.arctanh(np.sin(phi_rad)) - ((2*np.sqrt(n)) / (1+n))*np.arctanh(((2*np.sqrt(n)) / (1+n)) * np.sin(phi_rad)) )
    t_ = np.sqrt(1 + t*t)

    # (5) xi', eta'の計算
    xi2  = np.arctan(t / lambda_c) # [rad]
    eta2 = np.arctanh(lambda_s / t_)

    # (6) x, yの計算
    X = A_ * (xi2 + np.sum(np.multiply(alpha_array[1:],
                                       np.multiply(np.sin(2*xi2*np.arange(1,6)),
                                                   np.cosh(2*eta2*np.arange(1,6)))))) - S_ # [m]
    Y = A_ * (eta2 + np.sum(np.multiply(alpha_array[1:],
                                        np.multiply(np.cos(2*xi2*np.arange(1,6)),
                                                    np.sinh(2*eta2*np.arange(1,6)))))) # [m]
    
    # return
    return (Y, X) # [m]，(x_now, y_now)で，軸が反転している．

# スタック処理をする関数
def stack():
    global gps_latitude
    global gps_longitude
    global x_now
    global y_now
    global x_past
    global y_past
    p = 0
    x_past = x_now
    y_past = y_now
    # 後進を3回くらい
    for i in range(3):
        go_back()
        # 位置情報の取得と移動判定
        getgps()
        x_now, y_now = calc_xy(gps_latitude, gps_longitude, goal_latitude, goal_longitude)
        if(math.sqrt((x_now - x_past)**2 + (y_now - y_past)**2) > 1):
            p = 1
            break
    # 3回くらい暴れてみる
    for i in range(3):
        rotate(60 + i*30)
        rotate(-60 - i*30)
        go_back()
        # 位置情報の取得と移動判定
        getgps()
        x_now, y_now = calc_xy(gps_latitude, gps_longitude, goal_latitude, goal_longitude)
        if(math.sqrt((x_now - x_past)**2 + (y_now - y_past)**2) > 1):
            p = 1
            break
        go_ahead()
        # 位置情報の取得と移動判定
        getgps()
        x_now, y_now = calc_xy(gps_latitude, gps_longitude, goal_latitude, goal_longitude)
        if(math.sqrt((x_now - x_past)**2 + (y_now - y_past)**2) > 1):
            p = 2
            break
    # 回避した移動向きから条件分岐
    if(p == 1):         # 後進でスタック回避
        go_back()
        rotate(90)
        go_ahead()
        rotate(-90)
        go_ahead()
    if(p == 2):         # 前進でスタック回避
        go_ahead()
    """
    stack関数はメインのループに含まれるので無限ループにしない
    gpsの測定精度より完璧なスタック検知が困難である
     スタックを前進により抜けたがスタック検知した場合,後進で再度スタックする危険がある
    """

# 制御履歴の保存
def record(theta,gps_latitude,gps_longitude,x_now,y_now,i):

    distance = math.sqrt(x_now**2+y_now**2)
    stacking = ["True", "False"]
    with open('phase1_record.csv','a',newline='') as f: 
        writer = csv.writer(f)
        writer.writerow([theta,gps_latitude,gps_longitude,x_now,y_now,distance,stacking[i]])
    f.close()
    
# 角度取得関数
def magnet():
#     magXs=[]
#     magYs=[]
#     for lowpass_ in range(5):
    mag = mpu9250.readMagnet()
    # print(" mx = " , ( mag['x']   ), end='')
    # print(" my = " , ( mag['y']   ), end='')
    # print(" mz = " , ( mag['z'] ))
    # print()

    # キャリブレーション
    print("mag['x']=" + str(mag['x']))
    print("mag['y']=" + str(mag['y']))
    magX_calibrated = (mag['x']-(magX_max + magX_min)/2) / ((magX_max - magX_min)/2)
    magY_calibrated = (mag['y']-(magY_max + magY_min)/2) / ((magY_max - magY_min)/2)
    print("magX_calibrated" + str(magX_calibrated))
    print("magY_calibrated" + str(magY_calibrated))

    #リスト追加
#     magXs.append(magX_calibrated)
#     magYs.append(magY_calibrated)

    # ローパスフィルタ
#     magX_mean = sum(magXs)/len(magXs)
#     magY_mean = sum(magYs)/len(magYs)

    # とりあえずatan2に入れたものをtheta_absoluteとしているが，本当に欲しいtheta_absoluteにするには演算が必要かも
    theta_absolute = math.atan2(-magY_calibrated, -magX_calibrated)*180/math.pi
    # print(theta_absolute)
#     theta_absolute_lowPass = math.atan2(-magY_mean, -magX_mean)*180/math.pi
    # print(theta_absolute_lowPass)
    # ローパスが悪さをしている可能性があったので，未ローパスの値を使っている
    return theta_absolute


# 以下カメラ用関数
# ----------------------------------------------------------------------------------------------

def csv_write_f():

    flag = True
    filename = ""

    def write(x,y):

        import csv

        nonlocal flag
        nonlocal filename

        if flag:
            now_time = datetime.datetime.now()
            filename = 'test_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

            with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow(["theta", "prop"])
            flag = False


        with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow([x, y])

    return write

csv_write = csv_write_f()


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
    camera.capture(os.path.join(image_folder,"image"+filename_camera+".jpg"))

    # 読み込み
    img = image.open ("image_jpg_folder/image"+filename_camera+".jpg")
    #hsv空間に変換 「色相(Hue)」「彩度(Saturation)」「明度(Value)」
    img_hsv = image.open("image_jpg_folder/image"+filename_camera+".jpg").convert('HSV')
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
    (image.fromarray(img_th)).save(os.path.join(scanth_folder,"scanth"+filename_camera+".jpg"))
    
    takepic_counter += 1
    theta=scantheta(img_th)
    prop=scanprop(img_th)

    data = (theta,prop)
    csv_write(*data)

    return theta,prop

# -----------------------------------------------------------------------------------------------



# ここからメイン
print("main started")

# 制御履歴CSVファイルの作成
with open('phase1_record.csv','w',newline='') as f: 
    writer = csv.writer(f)
    writer.writerow(["theta","gps_latitude","gps_longitude","x_now","y_now","distance","stack"])
f.close()
print("csv created")
'''
# ---ここから着地・展開検知---
land_pressure=average_pressure() #基準となる地表での気圧を取得
print('land_pressure : {} hPa'.format(land_pressure))

i=0
while(i<=10): #上昇したかを判断
    pressure=get_pressure()
    time.sleep(0.1)
    
    if pressure<(land_pressure-1.21923): #3階用 
    #if pressure<(land_pressure-7.84011):#50m以上になったら上がったと判断
        i+=1
        print("In the sky")
        print(i)
    else: #50m地点に上がりきるまでyetを出力
        i=0
        print("yet") 
print("next\n") #10回連続50m以上の値になったら着地判定へ


i=0
while(i<=10): #着地したかを判断
    pressure=get_pressure()

    if pressure>(land_pressure-0.05): 
        i=i+1
        print(i)
    else: 
        i=0
        print("yet")
    time.sleep(0.1)
print("On the land")


#展開検知
for j in range(5): #赤の割合が一定以下になるまで繰り返す
    nchrm()
    print("nchrm "+str(j))

    data=takepic()
    prop=data[1] #Rの割合取得
    
    
    if prop<60: 
       print(prop)
       break 

    else:
        print("yet")
        print(prop) 
        continue
   
print("open!")
GPIO.cleanup()
'''
# ---ここまで着地・展開検知---

# ---ここからGPSフェーズ---
print("enter GPS phase")
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

# gpsの設定
gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSオブジェクトを生成する。
                                     # 引数はタイムゾーンの時差と出力フォーマット
gpsthread = threading.Thread(target=rungps, args=()) # 上の関数を実行するスレッドを生成
gpsthread.setDaemon(True)
gpsthread.start() # スレッドを起動
print("thread got up")

# gpsから緯度・経度取得
getgps()
print("got gps")

# calc_xyから座標取得
x_now, y_now = calc_xy(gps_latitude, gps_longitude, goal_latitude, goal_longitude)
print("calced xy¥n")
print("x_now, y_now = ", x_now, y_now)
# magnetから絶対角度取得
theta_absolute = magnet()
print("got theta_absolute=", theta_absolute)
# angleから回転角度取得
theta_relative = angle(x_now, y_now, theta_absolute)
print("got theta_relative=", theta_relative)
# ループ(3mゴールまで)
try:
    while math.sqrt( x_now**2 + y_now**2 ) > final_distance :
        print("entered while")
        """
        # スタックの条件分岐(移動距離が3.5m以内)
        if(math.sqrt((x_now - x_past)**2 + (y_now - y_past)**2) <= 3.5):
            # スタック処理
            stack()
            print("stack")
            stack = True
        else:
            # 旋回，直進
            rotate(theta_relative)
            print("rotated")
            go_ahead()
            print("went ahead")
            stack = False
        """
        # stack無しバージョン
        # 旋回，直進
        while True:
            # 10°固定
            if(theta_relative > 0): rotate(10)
            if(theta_relative < 0): rotate(-10)
            print("10 deg rotated")
            """
            # 必要角度に応じて回転角を算出
            rotate(theta_relative/1.5)
            print(f"{theta_relative/1.5} deg rotated")
            """
            # 旋回後に角度のフィードバック
            time.sleep(3)
            theta_absolute = magnet()
            theta_relative = angle(x_now, y_now, theta_absolute)
            print(f"theta_absolute = {theta_absolute}\ntheta_relative = {theta_relative}")
            if(theta_relative > -10 and theta_relative < 10): break
            # x_now, y_now を表示したい
            getgps()
            print("got gps")
            # calc_xyから座標取得
            x_now, y_now = calc_xy(gps_latitude, gps_longitude, goal_latitude, goal_longitude)
            print("calced xy\n")
            print("x_now, y_now =", x_now, y_now)
            
        go_ahead()
        print("went ahead")
#         stack = False
        # 履歴の保存
        record(theta_relative, gps_latitude, gps_longitude, x_now, y_now, 1)
        print("recorded")
        # 過去データの一時保存(移動検知のため)
        x_past = x_now
        y_past = y_now
        # gpsから緯度・経度取得
        time.sleep(5)
        getgps()
        print("got gps")
        # calc_xyから座標取得
        x_now, y_now = calc_xy(gps_latitude, gps_longitude, goal_latitude, goal_longitude)
        print("calced xy\n")
        print("x_now, y_now =", x_now, y_now)
        # magnetから絶対角度取得
        theta_absolute = magnet()
        print("got theta_absolute=", theta_absolute)
        # angleから回転角度取得
        theta_relative = angle(x_now, y_now, theta_absolute)
        print("got theta_relative=", theta_relative)     

    print("3m goal")
        
    # 赤コーン探索フェーズ
    while True:
        data = takepic()
        prop = data[1]
        print(f"prop={prop}")
        if prop > borderprop:
            break
        rotate(20)
    print("find!!")

    # 赤コーン接近フェーズ 
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

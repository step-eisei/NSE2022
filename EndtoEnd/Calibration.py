# 結果のcsvをmainフォルダに格納するように設定しています．
# 名前は'mag_record_calib_mebunryo_max_min.csv'

# coding: utf-8
## @package faboMPU9250
#  This is a library for the FaBo 9AXIS I2C Brick.
#
#  http://fabo.io/202.html
#
#  Released under APACHE LICENSE, VERSION 2.0
#
#  http://www.apache.org/licenses/
#
#  FaBo <info@fabo.io>
from lib2to3.pgen2.token import NEWLINE
from tkinter import W
import serial
import micropyGPS
import threading
import time
import csv
import FaBo9Axis_MPU9250
import sys
import datetime
import re
import math
import RPi.GPIO as GPIO

#GPSモジュールからデータをmy_gps(GPSオブジェクト)に追加，＊引数はタイムゾーンの時差（日本は＋9時間）と，経度緯度の出力フォーマットを指定（ddm,dms,ddなどから）
my_gps = micropyGPS.MicropyGPS(9, 'dd')

def percentpick(listdata):
    n = int(len(listdata) *p/100)
    listdata = sorted(listdata) # 昇順
    min = listdata[n-1]
    max = listdata[len(listdata)-n]
    return max, min

#GPSモジュールを読み，my_gpsを更新する関数
def rungps():
    #シリアルポートの指定，シリアル通信の速度[bps]を指定
    ser = serial.Serial("/dev/ttySOFT0",4800,timeout=20)
    ser.readline()
    while True:
        #受信したデータをバイナリデータからテキストデータへ変換
        sentence = ser.readline().decode('utf-8')
        #先頭が'$'でなければ捨てる
        if sentence[0] != '$':
            continue
        #
        for x in sentence:
            my_gps.update(x)

#上の関数を実行するスレッドを生成
gpsthread = threading.Thread(target=rungps, args=())
#gpsスレッドをデーモン化
gpsthread.daemon = True
#スレッドを起動
gpsthread.start()
'''
以下3行に渡ってcsvファイル名を作成している．
datetimeで得られる現在時刻にはスペース，コロン，ピリオドが含まれており，
これはLinuxでは保存できるがWindowsでは保存できないため，
それらの文字をreplaceでアンダーバーやハイフンに置換している．
もっといい方法があると思うが，とりあえずこれで実装した．
'''
DIFF_JST_FROM_UTC = 9
jp_time = datetime.datetime.utcnow() + datetime.timedelta(hours=DIFF_JST_FROM_UTC)
gps_name = 'calibration_gps/gps_record_' + str(jp_time).replace(' ', '_').replace(':', '-').replace('.', '_') + '.csv'
mag_name = 'calibration_9axis/mag_record_' + str(jp_time).replace(' ', '_').replace(':', '-').replace('.', '_') + '.csv'
# print(jp_time)
# >> 2022-07-07 13:28:32.197486
# print(csv_name)
# >> mag_record_2022-07-07_13-28-32_197156

#引数には（保存先のパス，読み書きの指定やバイナリ，テキストの指定はmodeを使う,）
with open(gps_name,'w',newline='') as f:
    writer = csv.writer(f)
    #一行書き込み
    writer.writerow(["goal_latitude", "goal_longitude"])

with open(mag_name,'w',newline='') as f: 
    writer = csv.writer(f)
    writer.writerow(["magX", "magY", "magZ"])
f.close()

#10回データを書き込めば終了
gps_latitude = 100
gps_longitude = 100
sum_latitude = 0
sum_longitude = 0
for i in range(10):
    while True:
        #ちゃんとしたデータがある程度たまってから出力
        if my_gps.clean_sentences > 20:
            if(gps_latitude != my_gps.latitude[0]):
                gps_latitude = my_gps.latitude[0]
                gps_longitude = my_gps.longitude[0]
                sum_latitude += gps_latitude
                sum_longitude += gps_longitude
                #衛星数を出力
                print(f"gps_latitude = {gps_latitude}, gps_longitude = {gps_longitude}")
                #mode='a'は追記モードでファイルを開く
                with open(gps_name,mode='a',newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([gps_latitude,gps_longitude]) 
                print("data add")
                break

#平均値を算出しgoal.pyに書き込み
goal_la = sum_latitude / 10
goal_lo = sum_longitude / 10

with open('goal.csv',mode='w',newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["goal_latitude", "goal_longitude"])

with open('goal.csv',mode='a',newline='') as f:
           writer = csv.writer(f)
           writer.writerow([goal_la,goal_lo]) 

# ここから9軸
mpu9250 = FaBo9Axis_MPU9250.MPU9250()

try:
    magX_save = []
    magY_save = []
    # duty比1でモータを旋回
    # モータのピン割り当て(GPIO 〇〇)
    PIN_AIN1 = 24   # 右モータ(A)
    PIN_AIN2 = 23
    PIN_PWMA = 12
    PIN_BIN1 = 16   # 左モータ(B)
    PIN_BIN2 = 26
    PIN_PWMB = 13
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
    # 左に旋回
    # 右モータ前進
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.HIGH)
    # 左モータ後進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    magx = []
    magy = []
    while True:
        pwm_left.ChangeDutyCycle(1)
        pwm_right.ChangeDutyCycle(1)
        time.sleep(0.2)
        pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
        pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
        time.sleep(0.2)
        mag = mpu9250.readMagnet()
        magx.append(mag['x'])
        magy.append(mag['y'])
        print(f"x = {mag['x']}, y = {mag['y']}")
        with open(mag_name,'a',newline='') as f: 
            writer = csv.writer(f)
            writer.writerow([mag['x'], mag['y'], mag['z']])
        f.close()
        
    
except KeyboardInterrupt:
    pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
    pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    # 最大値，最小値の算出
    p = 3 # 上位何%をpickするか
    Xmax, Xmin = percentpick(magx)
    Ymax, Ymin = percentpick(magy)
    Xcenter = (Xmax + Xmin) / 2
    Ycenter = (Ymax + Ymin) / 2
    with open('mag.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['magX_max', 'magX_min', 'magY_max', 'magY_min'])
        f.close()
    with open('mag.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([Xmax, Xmin, Ymax, Ymin])
        f.close()
    sys.exit()

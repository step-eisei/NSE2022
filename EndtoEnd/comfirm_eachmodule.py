import end_to_end


#各モジュール
######################
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

from struct import *
#####################################
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
gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSオブジェクトを生成する

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

gps_latitude = 0.0
gps_longitude = 0.0
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
theta_relative = 0.0
prop = 0

data = pack('>ddd', float(theta_relative), float(gps_latitude), float(gps_longitude))
data = data + b'\n'

with open ('mag.csv', 'r' ) as f :
    reader = csv.reader(f)
    line = [row for row in reader]
    magX_max = float(line[1][0])
    magX_min = float(line[1][1])
    magY_max = float(line[1][2])
    magY_min = float(line[1][3])


if __name__=="__main__":
#     image=Image
#     imageo=ImageOps
#     camera=picamera.PiCamera()
    
    mpu9250 = FaBo9Axis_MPU9250.MPU9250()
    
    
    COM = '/dev/ttyAMA0'
    ser = serial.Serial(COM, 115200)
    # ここからメイン
    print("main started")

    # 制御履歴CSVファイルの作成
    with open('phase1_record.csv','w',newline='') as f: 
        writer = csv.writer(f)
        writer.writerow(["theta","gps_latitude","gps_longitude","x_now","y_now","distance","stack"])
    f.close()
    print("csv created")

    # ---ここから着地・展開検知---
    land_pressure=end_to_end.average_pressure() #基準となる地表での気圧を取得
    print('land_pressure : {} hPa'.format(land_pressure))

    i=0
    while(i<=5): #上昇したかを判断
        pressure=end_to_end.get_pressure()
        print(f"pressure={pressure}")
        time.sleep(0.1)
        i+=1
        
    print("pressure_got_data") #10回連続50m以上の値になったら着地判定へ
    print()
    time.sleep(0.5)

    #展開検知
    for j in range(2): #赤の割合が一定以下になるまで繰り返す
        """
        nchrm()
        print("nchrm "+str(j))
        """

        data=end_to_end.takepic()
        prop=data[1] #Rの割合取得
        print(f"camera_R_prop={prop}")
        time.sleep(2)
    
    print("camera_got_data") 
    print()
    GPIO.cleanup()
    time.sleep(0.5)
    # ---ここまで着地・展開検知---

    # ---ここからGPSフェーズ---
    
                                        # 引数はタイムゾーンの時差と出力フォーマット
    gpsthread = threading.Thread(target=end_to_end.rungps, args=()) # 上の関数を実行するスレッドを生成
    gpsthread.setDaemon(True)
    gpsthread.start() # スレッドを起動
    print("thread got up")

    for times in range(3):
        # gpsから緯度・経度取得
        end_to_end.getgps()
        print(f"got gps[lat,long]={gps_latitude},{gps_longitude}")

        mag = mpu9250.readMagnet()
        print(f"magx={mag['x']}")
        print(f"magy={mag['y']}")
        
        ser.write(data)

    GPIO.cleanup()

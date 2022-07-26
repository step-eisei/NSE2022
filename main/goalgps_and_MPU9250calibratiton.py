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

#GPSモジュールからデータをmy_gps(GPSオブジェクト)に追加，＊引数はタイムゾーンの時差（日本は＋9時間）と，経度緯度の出力フォーマットを指定（ddm,dms,ddなどから）
my_gps = micropyGPS.MicropyGPS(9, 'dd')

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

#引数には（保存先のパス，読み書きの指定やバイナリ，テキストの指定はmodeを使う,）
with open('goal_gps.csv',mode='w',newline='') as f:
    writer = csv.writer(f)
    #一行書き込み
    writer.writerow(["goal_latitude", "goal_longitude"])

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
                print("num =", my_gps.satellites_used)
                #mode='a'は追記モードでファイルを開く
                with open('goal_gps.csv',mode='a',newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([gps_latitude,gps_longitude]) 
                print("data add")
                break

#平均値を算出しgoal.pyに書き込み
goal_la = sum_latitude / 10
goal_lo = sum_longitude / 10

with open('goal.csv',mode='a',newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["goal_latitude", "goal_longitude"])

with open('goal.csv',mode='a',newline='') as f:
           writer = csv.writer(f)
           writer.writerow([goal_la,goal_lo]) 

# ここから9軸
mpu9250 = FaBo9Axis_MPU9250.MPU9250()

# 以下，目分量で得られた最大値と最小値
magX_max = 35.5
magX_min = -6.9
magY_max = -385.3
magY_min = -430.7

magXs = [0]*4
magYs = [0]*4

'''
以下3行に渡ってcsvファイル名を作成している．
datetimeで得られる現在時刻にはスペース，コロン，ピリオドが含まれており，
これはLinuxでは保存できるがWindowsでは保存できないため，
それらの文字をreplaceでアンダーバーやハイフンに置換している．
もっといい方法があると思うが，とりあえずこれで実装した．
'''
DIFF_JST_FROM_UTC = 9
jp_time = datetime.datetime.utcnow() + datetime.timedelta(hours=DIFF_JST_FROM_UTC)
csv_name = 'mag_record_calib_mebunryo_' + str(jp_time).replace(' ', '_').replace(':', '-').replace('.', '_') + '.csv'
# print(jp_time)
# >> 2022-07-07 13:28:32.197486
# print(csv_name)
# >> mag_record_2022-07-07_13-28-32_197156

with open('9axis_rawdata/' + csv_name,'w',newline='') as f: 
    writer = csv.writer(f)
    writer.writerow(["magX", "magY", "magZ", "magX_calibrated", "magY_calibrated", "theta_absolute", "magX_mean", "magY_mean", "theta_absolute_lowPass"])
f.close()

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
    freq = 300 # PWMの周波数
    while True:
#         accel = mpu9250.readAccel()
#         print(" ax = " , ( accel['x'] ))
#         print(" ay = " , ( accel['y'] ))
#         print(" az = " , ( accel['z'] ))

#         gyro = mpu9250.readGyro()
#         print(" gx = " , ( gyro['x'] ))
#         print(" gy = " , ( gyro['y'] ))
#         print(" gz = " , ( gyro['z'] ))

        mag = mpu9250.readMagnet()
#         print(" mx = " , ( mag['x']   ), end='')
#         print(" my = " , ( mag['y']   ), end='')
#         print(" mz = " , ( mag['z'] ))
#         print()
           
        # キャリブレーション
        magX_calibrated = (mag['x']-(magX_max + magX_min)/2) / ((magX_max - magX_min)/2)
        magY_calibrated = (mag['y']-(magY_max + magY_min)/2) / ((magY_max - magY_min)/2)
        
        # ローパスフィルタ
        magXs.append(magX_calibrated)
        magX_mean = sum(magXs)/5
        del magXs[0]
        
        
        magYs.append(magY_calibrated)
        magY_mean = sum(magYs)/5
        del magYs[0]
 
        
        # とりあえずatan2に入れたものをtheta_absoluteとしているが，本当に欲しいtheta_absoluteにするには演算が必要かも
        theta_absolute = math.atan2(-magY_calibrated, -magX_calibrated)*180/math.pi
        # print(theta_absolute)
        theta_absolute_lowPass = math.atan2(-magY_mean, -magX_mean)*180/math.pi
        print(theta_absolute_lowPass)
        
        # dataの抜き出し
        magX_save.append(float(mag['x']))
        magY_save.append(float(mag['y']))
      
        with open('9axis_rawdata/' + csv_name,'a',newline='') as f: 
            writer = csv.writer(f)
            writer.writerow([mag['x'], mag['y'], mag['z'], magX_calibrated, magY_calibrated, theta_absolute, magX_mean, magY_mean, theta_absolute_lowPass])
        f.close()
         
        time.sleep(0.3)
    
except KeyboardInterrupt:
    # 最大値，最小値の算出
    magX_max = max(magX_save)
    magX_min = min(magX_save)
    magY_max = max(magY_save)
    magY_min = min(magY_save)
    with open('9axis_rawdata/mag_record_calib_mebunryo_max_min.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['magX_max', 'magX_min', 'magY_max', 'magY_min'])
        writer.writerow([magX_max, magX_min, magY_max, magY_min])
        f.close()
    sys.exit()

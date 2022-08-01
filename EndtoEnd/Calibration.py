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
    magx = []
    magy = []
    print("Rotate Cansat.")
    while True:
        mag = mpu9250.readMagnet()
        magx.append(mag['x'])
        magy.append(mag['y'])
        print(f"x = {mag['x']}, y = {mag['y']}")
        with open(mag_name,'a',newline='') as f: 
            writer = csv.writer(f)
            writer.writerow([mag['x'], mag['y'], mag['z']])
        f.close()
        time.sleep(0.2)
        
    
except KeyboardInterrupt:
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

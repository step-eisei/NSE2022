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

def percentpick(listdata):
    n = int(len(listdata) *p/100)
    listdata = sorted(listdata) # 昇順
    min = listdata[n-1]
    max = listdata[len(listdata)-n]
    return max, min

'''
以下3行に渡ってcsvファイル名を作成している．
datetimeで得られる現在時刻にはスペース，コロン，ピリオドが含まれており，
これはLinuxでは保存できるがWindowsでは保存できないため，
それらの文字をreplaceでアンダーバーやハイフンに置換している．
もっといい方法があると思うが，とりあえずこれで実装した．
'''
DIFF_JST_FROM_UTC = 9
jp_time = datetime.datetime.utcnow() + datetime.timedelta(hours=DIFF_JST_FROM_UTC)
mag_name = 'calibration_9axis/mag_record_' + str(jp_time).replace(' ', '_').replace(':', '-').replace('.', '_') + '.csv'
# print(jp_time)
# >> 2022-07-07 13:28:32.197486
# print(csv_name)
# >> mag_record_2022-07-07_13-28-32_197156

#引数には（保存先のパス，読み書きの指定やバイナリ，テキストの指定はmodeを使う,）
with open(mag_name,'w',newline='') as f: 
    writer = csv.writer(f)
    writer.writerow(["magX", "magY", "magZ"])
f.close()

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
    p = 5 # 上位何%をpickするか
    Xmax, Xmin = percentpick(magx)
    Ymax, Ymin = percentpick(magy)
    with open('mag.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['magX_max', 'magX_min', 'magY_max', 'magY_min'])
        f.close()
    with open('mag.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([Xmax, Xmin, Ymax, Ymin])
        f.close()
    sys.exit()

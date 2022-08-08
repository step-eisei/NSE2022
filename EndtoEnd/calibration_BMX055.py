# 9軸のbmx055を用いるバージョンのプログラムです．
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
import sys
import datetime
import re
import math
from smbus import SMBus

def bmx_setup():
    # mag_data_setup : 地磁気値をセットアップ
    data = i2c.read_byte_data(MAG_ADDR, 0x4B)
    if(data == 0):
        i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
        time.sleep(0.5)
    i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
    i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
    i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
    i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
    i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
    time.sleep(0.5)

def mag_value():
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    mag_data = [0.0, 0.0, 0.0]

    try:
        for i in range(8):
            data[i] = i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)

        for i in range(3):
            if i != 2:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                if mag_data[i] > 4095:
                    mag_data[i] -= 8192
            else:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                if mag_data[i] > 16383:
                    mag_data[i] -= 32768

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return mag_data

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
mag_name = 'calibration_bmx/mag_record_' + str(jp_time).replace(' ', '_').replace(':', '-').replace('.', '_') + '.csv'
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
# I2C
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42

i2c = SMBus(1)

try:
    bmx_setup()
    print("setup fin.")
    time.sleep(0.5)
    
    magx = []
    magy = []
    print("Rotate Cansat.")
    while True:
        mag = mag_value()
        magx.append(mag[0])
        magy.append(mag[1])
        print(f"x = {mag[0]}, y = {mag[1]}")
        with open(mag_name,'a',newline='') as f: 
            writer = csv.writer(f)
            writer.writerow([mag[0], mag[1], mag[2]])
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

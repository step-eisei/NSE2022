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

import FaBo9Axis_MPU9250
import time
import sys
import csv
import datetime
import re

mpu9250 = FaBo9Axis_MPU9250.MPU9250()

# 以下，目分量で得られた最大値と最小値
magX_max = 35.0
magX_min = -24.0
magY_max = 68.0
magY_min = 11.0

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

with open('result/' + csv_name,'w',newline='') as f: 
    writer = csv.writer(f)
    writer.writerow(["magX", "magY", "magZ", "magX_calibrated", "magY_calibrated", "theta_absolute"])
f.close()

try:
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
        magX_calibrated = (mag['x']-(magX_max + magX_min)/2) / ((magX_max - magX_min)/2)
        magY_calibrated = (mag['y']-(magY_max + magY_min)/2) / ((magY_max - magY_min)/2)
        
#         とりあえずatan2に入れたものをtheta_absoluteとしているが，本当に欲しいtheta_absoluteにするには演算が必要かも
        theta_absolute = atan2(magY_calibrated, magX_calibrated)
        print(theta_absolute)

        with open('result/' + csv_name,'a',newline='') as f: 
            writer = csv.writer(f)
            writer.writerow([mag['x'], mag['y'], mag['z'], magX_calibrated, magY_calibrated, theta_absolute])
        f.close()
         
        time.sleep(0.5)

except KeyboardInterrupt:
    sys.exit()

# 結果のcsvをresultフォルダに格納するように設定しています．
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

import time
import sys
import csv
import datetime
import math


# 以下，目分量で得られた最大値と最小値
magX_max = 35.5
magX_min = -6.9
magY_max = -385.3
magY_min = -430.7

magXs = [0]*4
magYs = [0]*4

# I2C
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42

i2c = SMBus(1)

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

if __name__ =="__main__":
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
        writer.writerow(["magX", "magY", "magZ", "magX_calibrated", "magY_calibrated", "theta_absolute", "magX_mean", "magY_mean", "theta_absolute_lowPass"])

    try:
        magX_save = []
        magY_save = []
        while True:

            magx,magy,magz = mag_value()
            
            # キャリブレーション
            magX_calibrated = (magx-(magX_max + magX_min)/2) / ((magX_max - magX_min)/2)
            magY_calibrated = (magy-(magY_max + magY_min)/2) / ((magY_max - magY_min)/2)
            
            # ローパスフィルタ
            magXs.append(magX_calibrated)
            magX_mean = sum(magXs)/5
            del magXs[0]

            magYs.append(magY_calibrated)
            magY_mean = sum(magYs)/5
            del magXs[0]
            
    
            
            # とりあえずatan2に入れたものをtheta_absoluteとしているが，本当に欲しいtheta_absoluteにするには演算が必要かも
            theta_absolute = math.atan2(-magY_calibrated, -magX_calibrated)*180/math.pi
            # print(theta_absolute)
            theta_absolute_lowPass = math.atan2(-magY_mean, -magX_mean)*180/math.pi
            print(theta_absolute_lowPass)
            
            # dataの抜き出し
            magX_save.append(float(magx))
            magY_save.append(float(magy))
        
            with open('result/' + csv_name,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow([magx, magy, magz, magX_calibrated, magY_calibrated, theta_absolute, magX_mean, magY_mean, theta_absolute_lowPass])
            
            time.sleep(0.3)
        
        # 最大値，最小値の計算
        magX_max = max(magX_save)
        magX_min = min(magX_save)
        magY_max = max(magY_save)
        magY_min = min(magY_save)
        with open('result/mag_record_calib_mebunryo_max_min.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['magX_max', 'magX_min', 'magY_max', 'magY_min'])
            writer.writerow([magX_max, magX_min, magY_max, magY_min])
            
    except KeyboardInterrupt:
        sys.exit()

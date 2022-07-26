# theta_relativeをチェックする関数
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

mpu9250 = FaBo9Axis_MPU9250.MPU9250()

# 以下，キャリブレーションにより計算した最大値と最小値
with open ('9axis_rawdata/mag_record_calib_mebunryo_max_min.csv', 'r' ) as f :
    reader = csv.reader(f)
    line = [row for row in reader]
    magX_max = float(line[1][0])
    magX_min = float(line[1][1])
    magY_max = float(line[1][2])
    magY_min = float(line[1][3])

magXs = [0]*5
magYs = [0]*5

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

# 角度取得関数
def magnet():

    mag = mpu9250.readMagnet()
    # print(" mx = " , ( mag['x']   ), end='')
    # print(" my = " , ( mag['y']   ), end='')
    # print(" mz = " , ( mag['z'] ))
    # print()

    # キャリブレーション
    magX_calibrated = (mag['x']-(magX_max + magX_min)/2) / ((magX_max - magX_min)/2)
    magY_calibrated = (mag['y']-(magY_max + magY_min)/2) / ((magY_max - magY_min)/2)

    # ローパスフィルタ
    magXs[0] = magXs[1]
    magXs[1] = magXs[2]
    magXs[2] = magXs[3]
    magXs[3] = magXs[4]
    magXs[4] = magX_calibrated
    magX_mean = sum(magXs)/5

    magYs[0] = magYs[1]
    magYs[1] = magYs[2]
    magYs[2] = magYs[3]
    magYs[3] = magYs[4]
    magYs[4] = magY_calibrated
    magY_mean = sum(magYs)/5

    # とりあえずatan2に入れたものをtheta_absoluteとしているが，本当に欲しいtheta_absoluteにするには演算が必要かも
    theta_absolute = math.atan2(-magY_calibrated, -magX_calibrated)*180/math.pi
    # print(theta_absolute)
    theta_absolute_lowPass = math.atan2(-magY_mean, -magX_mean)*180/math.pi
    # print(theta_absolute_lowPass)
    return theta_absolute_lowPass


# ここからメイン
try:
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

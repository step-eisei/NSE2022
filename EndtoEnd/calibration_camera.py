import RPi.GPIO as GPIO
import numpy as np
import serial
import csv
import threading

import FaBo9Axis_MPU9250
import sys
import datetime
import re

from PIL import Image,ImageOps
import picamera

image=Image
imageo=ImageOps
import RPi.GPIO as GPIO
camera=picamera.PiCamera()

takepic_counter = 0
def sv_scan(img_hsv):
    # img[:,:,0]　色相(Hue)
    # img[:,:,1]　彩度(Saturation)
    # img[:,:,2]　明度(Value)
    
    im_s = 0*img_hsv[:, :, 0] + img_hsv[:, :, 1] + 0*img_hsv[:, :, 2] #彩度以外の情報を消去して、im_sに格納
    sat_avg = np.average(im_s) #画像の彩度の平均を求める
    
    im_v = 0*img_hsv[:, :, 0] + 0*img_hsv[:, :, 1] + img_hsv[:, :, 2]
    val_avg = np.average(im_v)
    
    return sat_avg,val_avg

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
    img_h_th = np.where((im_h < 15/360*255) | (im_h > 345/360*255), 1, 0)
    img_s_th = np.where(im_s > sat_avg*0.9, 1, 0)
    img_v_th = np.where(im_v > val_avg*0.9, 1, 0)

    #行列の掛け算ではなく各要素の掛け算をする 上記の条件で一つでも満たしていないものがあれば，0となる．
    #検出された物を白にするために最後に255を掛ける(この時点で2値化)
    img_th = img_h_th * img_s_th * img_v_th * 255 # 条件を満たした要素は255，それ以外は0

    img_th = np.uint8(img_th) # np.uint8 unsigned int (0 ～ 255)

    return img_th
  
# 撮影
now_time_camera = datetime.datetime.now()
filename_camera = now_time_camera.strftime('%m%d_%H%M_')+str(takepic_counter)
camera.capture("calibration_camera/image"+filename_camera+".jpg")

# 読み込み
img = image.open ("calibration_camera/image"+filename_camera+".jpg")
#hsv空間に変換 「色相(Hue)」「彩度(Saturation)」「明度(Value)」
img_hsv = image.open("calibration_camera/image"+filename_camera+".jpg").convert('HSV')
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
(image.fromarray(img_th)).save("calibration_camera/scanth"+filename_camera+".jpg")
    
takepic_counter += 1
    
print(sat_avg)
print(val_avg)

import RPi.GPIO as GPIO
import numpy as import serial
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

def sv_scan(img_hsv):
    # img[:,:,0]　色相(Hue)
    # img[:,:,1]　彩度(Saturation)
    # img[:,:,2]　明度(Value)
    
    im_s = 0*img_hsv[:, :, 0] + img_hsv[:, :, 1] + 0*img_hsv[:, :, 2] #彩度以外の情報を消去して、im_sに格納
    sat_avg = np.average(im_s) #画像の彩度の平均を求める
    
    im_v = 0*img_hsv[:, :, 0] + 0*img_hsv[:, :, 1] + img_hsv[:, :, 2]
    val_avg = np.average(im_v)
    
    return sat_avg,val_avg
  
  # 撮影
global takepic_counter
now_time_camera = datetime.datetime.now()
filename_camera = now_time_camera.strftime('%m%d_%H%M_')+str(takepic_counter)
camera.capture("image"+filename_camera+".jpg")

# 読み込み
img = image.open ("image"+filename_camera+".jpg")
#hsv空間に変換 「色相(Hue)」「彩度(Saturation)」「明度(Value)」
img_hsv = image.open("image"+filename_camera+".jpg").convert('HSV')
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
(image.fromarray(img_th)).save("scanth"+filename_camera+".jpg")
    
takepic_counter += 1
theta=scantheta(img_th)
prop=scanprop(img_th)
    
print(sat_avg)
print(val_avg)

from PIL import Image,ImageOps
import numpy as np
import picamera
import math
import cv2
if dis<0.79
  image=Image
  imageo=ImageOps
  camera=picamera.PiCamera()


  def rgbbinary(img,val):
    #画像のチャンネルを分ける
    #画像のチャンネルを分ける img[縦，横，色]　この行程でグレースケールになる
    im_R= img[:, :, 0] + 0*img[:, :, 1] + 0*img[:, :, 2]
    im_G= 0*img[:, :, 0] + img[:, :, 1] + 0*img[:, :, 2]
    im_B= 0*img[:, :, 0] + 0*img[:, :, 1] + img[:, :, 2]


    #比較を満たしていれば1, それ以外は0を書き込み
    img_r_th = np.where(im_R>val, 1, 0)
    img_g_th = np.where(im_G<val*0.8, 1, 0)
    img_b_th = np.where(im_B<val*0.8, 1, 0)

    #行列の掛け算ではなく各要素の掛け算をする
    #検出された物を白にするために最後に255を掛ける(この時点で2値化)
    img_th = img_r_th * img_g_th * img_b_th * 255

    img_th = np.uint8(img_th)

    return img_th
    
#赤の割合を計算する
width=img_th.shape[1]
height=img_th.shape[0]
img_th.size=width*height

red_area=cv2.countNonZero(img_th)

prop=(red_area/img_th.size)*100

print (prop) 

if prop==100
 exit
  
data=takepic()

# カメラで画像を撮影し，thetaを出力するまでのプログラムです．動作確認済みです．

from PIL import Image,ImageOps
import numpy as np
import picamera
import math

image=Image
imageo=ImageOps
camera=picamera.PiCamera()


def rgbbinary(img,val):
    #画像のチャンネルを分ける
    #画像のチャンネルを分ける img[縦，横，[R,G,B]]　この行程でグレースケールになる
    im_R= img[:, :, 0] + 0*img[:, :, 1] + 0*img[:, :, 2] # R以外の情報を消去してim_Rに格納
    im_G= 0*img[:, :, 0] + img[:, :, 1] + 0*img[:, :, 2] # G以外の情報を消去してim_Gに格納
    im_B= 0*img[:, :, 0] + 0*img[:, :, 1] + img[:, :, 2] # B以外の情報を消去してim_Bに格納


    # 条件を満たしていれば1, それ以外は0に置換
    # np.where(条件, Trueの時に置換する数, Falseの時に置換する数)
    img_r_th = np.where(im_R>val, 1, 0)
    img_g_th = np.where(im_G<val*0.8, 1, 0)
    img_b_th = np.where(im_B<val*0.8, 1, 0)

    #行列の掛け算ではなく各要素の掛け算をする 上記の条件で一つでも満たしていないものがあれば，0となる．
    #検出された物を白にするために最後に255を掛ける(この時点で2値化)
    img_th = img_r_th * img_g_th * img_b_th * 255 # 条件を満たした要素は255，それ以外は0

    img_th = np.uint8(img_th) # np.uint8 unsigned int (0 ～ 255)

    return img_th

def vscan(img):
    # img[:,:,0]　色相(Hue)
    # img[:,:,1]　彩度(Saturation)
    # img[:,:,2]　明度(Value)
    im_v=0*img[:, :, 0] + 0*img[:, :, 1] + img[:, :, 2] # 明度以外の情報を消去して，im_Vに格納

    val=np.average(im_v) # 画像の明度の平均を求める
    return val

def scantheta(img_th):

    # 行列サイズ
    # img_th[縦,横,色]
    width=img_th.shape[1] #横の長さ
    # hight=img_th.shape[1]

    #列の和
    im_thx=np.sum(img_th,axis=0)

    theta_row=np.argmax(im_thx) # 最大の要素のインデックスを取得

    theta=-(theta_row/width*180-90)

    return theta

def scanprop(img_th):
    #赤の割合を計算する
    width=img_th.shape[1]
    height=img_th.shape[0]
    img_th.size=width*height

    red_area=np.countnonzero(img_th)

    prop=(red_area/img_th.size)*100

    return prop

def takepic():
    # 撮影
    camera.capture('image.jpg')

    # 読み込み
    img= image.open ('image.jpg')
    #hsv空間に変換 「色相(Hue)」「彩度(Saturation)」「明度(Value)」
    img_hsv=image.open('image.jpg').convert('HSV')
    #それぞれ上下左右反転し，Pillow → Numpyへ変換
    # 上下反転メソッド　flip()
    # 左右反転メソッド　mirror()
    img = np.array(imageo.mirror(imageo.flip(img)))
    img_hsv=np.array(imageo.mirror(imageo.flip(img_hsv)))

    # 解析
    val=vscan(img_hsv) # 画像全体の明度(Value)の平均を取得
    img_th=rgbbinary(img,val) #条件を満たす要素を255，それ以外を0とする配列
    (image.fromarray(img_th)).save('scanth.jpg')
    theta=scantheta(img_th)
    prop=scanprop(im_th)

    return theta,prop



data = takepic()

theta = takepic[0]

print("theta="+str(theta))

   

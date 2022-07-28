from PIL import Image,ImageOps
import numpy as np
import picamera
import math

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

def vscan(img):
    im_v=0*img[:, :, 0] + 0*img[:, :, 1] + img[:, :, 2]
    val=np.average(im_v)
    return val

def scantheta(img_th):

    #行列サイズ
    width=img_th.shape[1]
    # hight=img_th.shape[1]

    #列の和
    im_thx=np.sum(img_th,axis=0)

    #ソート
    theta_row=np.argmax(im_thx)

    theta=-(theta_row/width*62.2-90)

    return theta
    
def scandis(img_th):
    #列の和
    col_sum=np.sum(img_th,axis=0)
    #列の和の最大値
    col_m=max(col_sum)
    #写真の高さのピクセル
    height_p=img_th.shape[0]
    #高さの比
    prop_c=(col_m/255)/height_p
    #コーンの高さ
    height_c=0.7
    #tanΘの値
    tan_c=math.tan(math.radians(48.8))

    #dis=area_r/(img_th.shape[0]*img_th.shape[1])
    dis=height_c/prop_c*tan_c
    return dis


def enddecision(img_th):
    #赤の割合を計算する
    width=img_th.shape[1]
    height=img_th.shape[0]
    img_th.size=width*height

    red_area=np.countnonzero(img_th)


    prop=(red_area/img_th.size)*100

    return prop

def takepic1():
    # 撮影
    # camera.capture('image1.jpg')

    # 読み込み
    img= image.open ('image1.jpg')
    #hsv空間に変換
    img_hsv=image.open('image1.jpg').convert('HSV')
    #それぞれ上下左右反転
    img = np.array(imageo.mirror(imageo.flip(img)))
    img_hsv=np.array(imageo.mirror(imageo.flip(img_hsv)))

    # 解析
    val=vscan(img_hsv)
    img_th=rgbbinary(img,val)
    (image.fromarray(img_th)).save('scanth.jpg')
    theta=scantheta(img_th)
    dis=scandis(img_th)


    return theta,dis

def takepic2():
    # 撮影
    # camera.capture('image1.jpg')

    # 読み込み
    img= image.open ('image1.jpg')
    #hsv空間に変換
    img_hsv=image.open('image1.jpg').convert('HSV')
    #それぞれ上下左右反転
    img = np.array(imageo.mirror(imageo.flip(img)))
    img_hsv=np.array(imageo.mirror(imageo.flip(img_hsv)))
        # 解析
    val=vscan(img_hsv)
    img_th=rgbbinary(img,val)
    (image.fromarray(img_th)).save('scanth.jpg')
    prop=enddecision(img_th)
    
    theta=scantheta(img_th)
    dis=0.8
    return theta,dis,prop



data=takepic1()
theta=data[0]
dis=data[1]
   
print("theta="+str(theta)+", dis="+str(dis))
   

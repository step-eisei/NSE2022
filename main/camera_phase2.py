# 案1 rungps()関数をthreadする方法
# 未定義：モータのピン番号，モータ制御のduty比や時間など，座標計算時の定数をループするか，スタックの条件，スタックの緯度・経度
# magnetは作っていません
import time
from gpiozero import Motor
from PIL import Image,ImageOps
import numpy as np
import picamera
import math

# モータのピン割り当て(GPIO 〇〇)
motor_right = Motor(17, 18)
motor_left = Motor(15, 16)
# 左右のduty比(定義域：0~1)
duty_right = 0
duty_left = 0

T_straight = 5

theta = 60

borderprop = 20

takepic_counter = 0

image=Image
imageo=ImageOps
camera=picamera.PiCamera()

# 機体を旋回させる関数
def rotate(theta_relative):
    global motor_right
    global motor_left
    global duty_right
    global duty_left
    const = 0       # 単位角度における回転所要時間
    if(theta_relative > 0):
        motor_right.forward(duty_right)
        motor_left.backward(duty_left)
    if(theta_relative < 0):
        motor_right.backward(duty_right)
        motor_left.forward(duty_left)
    time.sleep(math.fabs(theta_relative)*const)
    motor_right.stop()
    motor_left.stop()

# 機体を前進させる関数
def go_ahead():
    global motor_right
    global motor_left
    global duty_right
    global duty_left
    global T_straight
    motor_right.forward(duty_right)
    motor_left.forward(duty_left)
    time.sleep(T_straight)
    motor_right.stop()
    motor_left.stop()

# 機体を後進させる関数
def go_back():
    global motor_right
    global motor_left
    global duty_right
    global duty_left
    global T_straight
    motor_right.backward(duty_right)
    motor_left.backward(duty_left)
    time.sleep(T_straight)
    motor_right.stop()
    motor_left.stop()

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
    size=width*height

    red_area=np.count_nonzero(img_th)

    prop=(red_area/size)*100

    return prop

def takepic():
    global takepic_counter
    # 撮影
    camera.capture('image'+str(takepic_counter)+'.jpg')

    # 読み込み
    img= image.open ('image'+str(takepic_counter)+'.jpg')
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
    (image.fromarray(img_th)).save('scanth'+str(takepic_counter)+'.jpg')
    theta=scantheta(img_th)
    prop=scanprop(img_th)
    takepic_counter += 1
    
    return theta,prop




theta = data[0]
prop = data[1]

while True:
    data = takepic()
    prop = data[1]
    if prop > borderprop:
        break
for i in range(5):
    data = takepic()
    theta = data[0]
    rotate(theta)
    go_ahead()

print("finish")


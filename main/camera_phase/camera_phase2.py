from PIL import Image,ImageOps
import numpy as np
import picamera
import math
import time

image=Image
imageo=ImageOps
import RPi.GPIO as GPIO
camera=picamera.PiCamera()

takepic_counter=1
borderprop = 0.5
theta = 0
prop = 0

# -------------------------------------------------------------
# モータのピン割り当て(GPIO 〇〇)
PIN_AIN1 = 24   # 左モータ(A)
PIN_AIN2 = 23
PIN_PWMA = 12
PIN_BIN1 = 16   # 右モータ(B)
PIN_BIN2 = 26
PIN_PWMB = 13
# 左右のduty比(定義域：0~100)
DUTY_A = 13 # 念のため20より上には上げないように
DUTY_B = 13 # 念のため20より上には上げないように

T_straight = 10
# --------------------------------------------------------------


# 機体を旋回させる関数
def rotate(theta_relative):
    global PIN_AIN1
    global PIN_AIN2
    global PIN_PWMA
    global PIN_BIN1
    global PIN_BIN2
    global PIN_PWMB
    global DUTY_A
    global DUTY_B
    const = 1/27       # 単位角度における回転所要時間
    # モータのセッティング
    GPIO.setmode(GPIO.BCM)
    # 左モータ
    GPIO.setup(PIN_AIN1, GPIO.OUT)
    GPIO.setup(PIN_AIN2, GPIO.OUT)
    # 左モータPWM
    GPIO.setup(PIN_PWMA, GPIO.OUT)
    pwm_left = GPIO.PWM(PIN_PWMA, DUTY_A)
    pwm_left.start(10)
    pwm_left.ChangeDutyCycle(DUTY_A)
    # 右モータ
    GPIO.setup(PIN_BIN1, GPIO.OUT)
    GPIO.setup(PIN_BIN2, GPIO.OUT)
    # 右モータPWM
    GPIO.setup(PIN_PWMB, GPIO.OUT)
    pwm_right = GPIO.PWM(PIN_PWMB, DUTY_B)
    pwm_right.start(10)
    pwm_right.ChangeDutyCycle(DUTY_B)
    # sleep
    time.sleep(2)
    
    if(theta_relative > 0): # 左に旋回
        # 右モータ前進
        GPIO.output(PIN_AIN1, GPIO.LOW)
        GPIO.output(PIN_AIN2, GPIO.HIGH)
        # 左モータ後進
        GPIO.output(PIN_BIN1, GPIO.LOW)
        GPIO.output(PIN_BIN2, GPIO.HIGH)
    if(theta_relative < 0): # 右に旋回
        # 右モータ後進
        GPIO.output(PIN_AIN1, GPIO.HIGH)
        GPIO.output(PIN_AIN2, GPIO.LOW)
        # 左モータ前進
        GPIO.output(PIN_BIN1, GPIO.HIGH)
        GPIO.output(PIN_BIN2, GPIO.LOW)
    time.sleep(math.fabs(theta_relative)*const)
    # モータの解放
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()

# 機体を前進させる関数
def go_ahead():
    global PIN_AIN1
    global PIN_AIN2
    global PIN_PWMA
    global PIN_BIN1
    global PIN_BIN2
    global PIN_PWMB
    global DUTY_A
    global DUTY_B
    global T_straight
    # モータのセッティング
    GPIO.setmode(GPIO.BCM)
    # 左モータ
    GPIO.setup(PIN_AIN1, GPIO.OUT)
    GPIO.setup(PIN_AIN2, GPIO.OUT)
    # 左モータPWM
    GPIO.setup(PIN_PWMA, GPIO.OUT)
    pwm_left = GPIO.PWM(PIN_PWMA, DUTY_A)
    pwm_left.start(10)
    pwm_left.ChangeDutyCycle(DUTY_A)
    # 右モータ
    GPIO.setup(PIN_BIN1, GPIO.OUT)
    GPIO.setup(PIN_BIN2, GPIO.OUT)
    # 右モータPWM
    GPIO.setup(PIN_PWMB, GPIO.OUT)
    pwm_right = GPIO.PWM(PIN_PWMB, DUTY_B)
    pwm_right.start(10)
    pwm_right.ChangeDutyCycle(DUTY_B)
    # sleep
    time.sleep(2)
    # 右モータ前進
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.HIGH)
    # 左モータ前進
    GPIO.output(PIN_BIN1, GPIO.HIGH)
    GPIO.output(PIN_BIN2, GPIO.LOW)
    # sleep
    time.sleep(T_straight)
    # モータの解放
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()

# 機体を後進させる関数
def go_back():
    global PIN_AIN1
    global PIN_AIN2
    global PIN_PWMA
    global PIN_BIN1
    global PIN_BIN2
    global PIN_PWMB
    global DUTY_A
    global DUTY_B
    global T_straight
    # モータのセッティング
    GPIO.setmode(GPIO.BCM)
    # 左モータ
    GPIO.setup(PIN_AIN1, GPIO.OUT)
    GPIO.setup(PIN_AIN2, GPIO.OUT)
    # 左モータPWM
    GPIO.setup(PIN_PWMA, GPIO.OUT)
    pwm_left = GPIO.PWM(PIN_PWMA, DUTY_A)
    pwm_left.start(10)
    pwm_left.ChangeDutyCycle(DUTY_A)
    # 右モータ
    GPIO.setup(PIN_BIN1, GPIO.OUT)
    GPIO.setup(PIN_BIN2, GPIO.OUT)
    # 右モータPWM
    GPIO.setup(PIN_PWMB, GPIO.OUT)
    pwm_right = GPIO.PWM(PIN_PWMB, DUTY_B)
    pwm_right.start(10)
    pwm_right.ChangeDutyCycle(DUTY_B)
    # sleep
    time.sleep(2)
    # 右モータ後進
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    # 左モータ後進
    GPIO.output(PIN_BIN1, GPIO.LOW)
    GPIO.output(PIN_BIN2, GPIO.HIGH)
    # sleep
    time.sleep(T_straight)
    # モータの解放
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()


# --------------------------------------------------------------------

def csv_write_f():

    flag = True
    filename = ""

    def write(x,y):

        import datetime
        import csv

        nonlocal flag
        nonlocal filename

        if flag:
            now_time = datetime.datetime.now()
            filename = 'test_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

            with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow(["theta", "prop"])
            flag = False


        with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow([x, y])

    return write

csv_write = csv_write_f()


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

    theta=-(theta_row/width*62.2-31.1)

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
    # 撮影
    global takepic_counter
    camera.capture("image"+str(takepic_counter)+".jpg")

    # 読み込み
    img= image.open ("image"+str(takepic_counter)+".jpg")
    #hsv空間に変換 「色相(Hue)」「彩度(Saturation)」「明度(Value)」
    img_hsv=image.open("image"+str(takepic_counter)+".jpg").convert('HSV')
    #それぞれ上下左右反転し，Pillow → Numpyへ変換
    # 上下反転メソッド　flip()
    # 左右反転メソッド　mirror()
    img = np.array(imageo.mirror(imageo.flip(img)))
    img_hsv=np.array(imageo.mirror(imageo.flip(img_hsv)))

    # 解析
    val=vscan(img_hsv) # 画像全体の明度(Value)の平均を取得
    img_th=rgbbinary(img,val) #条件を満たす要素を255，それ以外を0とする配列
    (image.fromarray(img_th)).save("scanth"+str(takepic_counter)+".jpg")
    
    takepic_counter += 1
    theta=scantheta(img_th)
    prop=scanprop(img_th)

    data = (theta,prop)
    csv_write(*data)

    return theta,prop


while True:
    data = takepic()
    prop = data[1]
    print(prop)
    if prop > borderprop:
        break
    rotate(50)
    
print("find!!")

for i in range(25):
    data = takepic()
    theta = data[0]
    print(theta)
    rotate(theta)
    go_ahead()


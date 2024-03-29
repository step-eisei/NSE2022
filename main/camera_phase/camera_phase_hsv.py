from PIL import Image,ImageOps
import numpy as np
import picamera
import math
import time
import datetime

image=Image
imageo=ImageOps
import RPi.GPIO as GPIO
camera=picamera.PiCamera()

takepic_counter = 1
borderprop = 1
theta_relative = 0
prop_past = 0

# -------------------------------------------------------------
# モータのピン割り当て(GPIO 〇〇)
PIN_AIN1 = 24   # 左モータ(A)
PIN_AIN2 = 23
PIN_PWMA = 12
PIN_BIN1 = 16   # 右モータ(B)
PIN_BIN2 = 26
PIN_PWMB = 13
# 左右のduty比(定義域：0~100)
DUTY_A = 60 # 20~40でICが高温になります．60~70が妥当です
DUTY_B = 60 # 20~40でICが高温になります．60~70が妥当です

T_straight = 0
# --------------------------------------------------------------

# 機体を旋回させる関数
def rotate(theta_relative):
    global PIN_AIN1
    global PIN_AIN2
    global PIN_PWMA
    global PIN_BIN1
    global PIN_BIN2
    global PIN_PWMB
    R_DUTY_A = 10
    R_DUTY_B = 10
    freq = 300          # pwm周波数
    const = 10/765        # 単位角度における回転所要時間
    
    pwm_left.ChangeDutyCycle(R_DUTY_A)
    pwm_right.ChangeDutyCycle(R_DUTY_B)
    
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
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    time.sleep(2)


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
    freq = 300          # pwm周波数

    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.HIGH)
    # 左モータ前進
    GPIO.output(PIN_BIN1, GPIO.HIGH)
    GPIO.output(PIN_BIN2, GPIO.LOW)
    # DUTY_A = DUTY_Bという仮定の下，
    # 0からDUTY_Aまで1ずつ上げる
    for i in range(0, DUTY_A + 1, 1):
        pwm_left.ChangeDutyCycle(i)
        pwm_right.ChangeDutyCycle(i)
        time.sleep(0.1)
    # sleep
    time.sleep(T_straight)
    # DUTY_A = DUTY_Bという仮定の下，
    # DUTY_Aから0まで1ずつさげる
    for i in range(0, DUTY_A + 1, 1):
        pwm_left.ChangeDutyCycle(DUTY_A - i)
        pwm_right.ChangeDutyCycle(DUTY_A - i)
        time.sleep(0.05)
    time.sleep(2)
 

# --------------------------------------------------------------------

def csv_write_f():

    flag = True
    filename = ""

    def write(x,y):

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
    img_h_th = np.where((im_h < 15/360*255) | (im_h > 175/360*255), 1, 0)
    img_s_th = np.where(im_s > sat_avg*0.9, 1, 0)
    img_v_th = np.where(im_v > val_avg*0.9, 1, 0)

    #行列の掛け算ではなく各要素の掛け算をする 上記の条件で一つでも満たしていないものがあれば，0となる．
    #検出された物を白にするために最後に255を掛ける(この時点で2値化)
    img_th = img_h_th * img_s_th * img_v_th * 255 # 条件を満たした要素は255，それ以外は0

    img_th = np.uint8(img_th) # np.uint8 unsigned int (0 ～ 255)

    return img_th

def sv_scan(img_hsv):
    # img[:,:,0]　色相(Hue)
    # img[:,:,1]　彩度(Saturation)
    # img[:,:,2]　明度(Value)
    
    im_s = 0*img_hsv[:, :, 0] + img_hsv[:, :, 1] + 0*img_hsv[:, :, 2] #彩度以外の情報を消去して、im_sに格納
    sat_avg = np.average(im_s) #画像の彩度の平均を求める
    
    im_v = 0*img_hsv[:, :, 0] + 0*img_hsv[:, :, 1] + img_hsv[:, :, 2]
    val_avg = np.average(im_v)
    
    return sat_avg,val_avg

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

    data = (theta,prop)
    csv_write(*data)

    return theta,prop


# motorをセットアップする
INITIAL_DUTY_A = 0
INITIAL_DUTY_B = 0
ini_freq = 300          # pwm周波数
# モータのセッティング
GPIO.setmode(GPIO.BCM)
# 左モータ
GPIO.setup(PIN_AIN1, GPIO.OUT)
GPIO.setup(PIN_AIN2, GPIO.OUT)
# 左モータPWM
GPIO.setup(PIN_PWMA, GPIO.OUT)
pwm_left = GPIO.PWM(PIN_PWMA, ini_freq)
pwm_left.start(10)
pwm_left.ChangeDutyCycle(INITIAL_DUTY_A)
# 右モータ
GPIO.setup(PIN_BIN1, GPIO.OUT)
GPIO.setup(PIN_BIN2, GPIO.OUT)
# 右モータPWM
GPIO.setup(PIN_PWMB, GPIO.OUT)
pwm_right = GPIO.PWM(PIN_PWMB, ini_freq)
pwm_right.start(10)
pwm_right.ChangeDutyCycle(INITIAL_DUTY_B)
# sleep
time.sleep(2)
print("set up finished")

DUTY_A = 31
DUTY_B = 30


try:
    while True:
        data = takepic()
        prop_now = data[1]
        print(f"prop={prop_now}")
        if prop_now > borderprop:
            break
        rotate(30)

    print("find!!")

    for i in range(5):
        flag = True
        data = takepic()
        prop_now = data[1]
        theta_relative = data[0]
        
        if prop_now > 60:
            break
            
        if prop_now - prop_past < 0:
            break
        
        print(f"theta={theta_relative},prop ={prop_now}")
        rotate(theta_relative)
        go_ahead()
        
        if flag:
            if prop_now > 10:
                DUTY_A = 21
                DUTY_B = 20
                flag = False
            
    print("goal!!")
    
except KeyboardInterrupt:
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()


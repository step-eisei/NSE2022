from xml.dom.expatbuilder import parseString
from xmlrpc.client import NOT_WELLFORMED_ERROR
import time
from smbus import SMBus
import math
import numpy as np
import threading
import time
import timeout_decorator
from PIL import Image,ImageOps
import picamera

prop_closed=60 # この値よりも，Rの割合が低くなるとカプセルが展開したと判断

image=Image
imageo=ImageOps
camera=picamera.PiCamera()

takepic_counter=1


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

def nchrm(): #ニクロム線加熱
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17,  GPIO.OUT)

    GPIO.output(17, True)
    #ここの数字は実験次第
    time.sleep(2)
    GPIO.output(17, False)
    time.sleep(5)


def get_pressure():
    bus_number  = 1
    i2c_address = 0x76
    bus = SMBus(bus_number)

    digT = []
    digP = []
    digH = []

    t_fine = 0.0

    def writeReg(reg_address, data):
        bus.write_byte_data(i2c_address,reg_address,data)

    def get_calib_param():
        calib = []

        for i in range (0x88,0x88+24):
            calib.append(bus.read_byte_data(i2c_address,i))
        calib.append(bus.read_byte_data(i2c_address,0xA1))
        for i in range (0xE1,0xE1+7):
            calib.append(bus.read_byte_data(i2c_address,i))

        digT.append((calib[1] << 8) | calib[0])
        digT.append((calib[3] << 8) | calib[2])
        digT.append((calib[5] << 8) | calib[4])
        digP.append((calib[7] << 8) | calib[6])
        digP.append((calib[9] << 8) | calib[8])
        digP.append((calib[11]<< 8) | calib[10])
        digP.append((calib[13]<< 8) | calib[12])
        digP.append((calib[15]<< 8) | calib[14])
        digP.append((calib[17]<< 8) | calib[16])
        digP.append((calib[19]<< 8) | calib[18])
        digP.append((calib[21]<< 8) | calib[20])
        digP.append((calib[23]<< 8) | calib[22])
        digH.append( calib[24] )
        digH.append((calib[26]<< 8) | calib[25])
        digH.append( calib[27] )
        digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        digH.append( calib[31] )

        for i in range(1,2):
            if digT[i] & 0x8000:
                digT[i] = (-digT[i] ^ 0xFFFF) + 1

        for i in range(1,8):
            if digP[i] & 0x8000:
                digP[i] = (-digP[i] ^ 0xFFFF) + 1

        for i in range(0,6):
            if digH[i] & 0x8000:
                digH[i] = (-digH[i] ^ 0xFFFF) + 1  

    def readData():
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(bus.read_byte_data(i2c_address,i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]

        compensate_T(temp_raw)
        x=compensate_P(pres_raw)

        return x

    def compensate_P(adc_P):
        global  t_fine

        v1 = (t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
        v2 = v2 + ((v1 * digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (digP[3] * 65536.0)
        v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * digP[0]) / 32768

        #if v1 == 0:
            #return 0
        pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)  

        pressure = pressure/100
        return pressure
        #print('pressure : {} hPa'.format(pressure/100))
        

        # print "pressure : %7.2f hPa" % (pressure/100)

    def compensate_T(adc_T):
        global t_fine
        v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
        v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
        t_fine = v1 + v2


    def setup():
        osrs_t = 1			#Temperature oversampling x 1
        osrs_p = 1			#Pressure oversampling x 1
        osrs_h = 1			#Humidity oversampling x 1
        mode   = 3			#Normal mode
        t_sb   = 5			#Tstandby 1000ms
        filter = 0			#Filter off
        spi3w_en = 0			#3-wire SPI Disable

        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
        ctrl_hum_reg  = osrs_h

        writeReg(0xF2,ctrl_hum_reg)
        writeReg(0xF4,ctrl_meas_reg)
        writeReg(0xF5,config_reg)

    setup()
    get_calib_param()


    if __name__ == '__main__':
        try:
            x=readData() #気圧の値読み取り
            print(x)
            return x #pressure関数が呼び出されたら渡す
        except KeyboardInterrupt:
            pass


def average_pressure():
    sum=0.0
    land=0.0
    
    for i in range(20):
        land=get_pressure()
        sum+=land
        time.sleep(0.1)

    average_pressure=sum/20
    return average_pressure


# land_pressure=average_pressure() #基準となる地表での気圧を取得
# print('land_pressure : {} hPa'.format(land_pressure))

# i=0
# while(i<=10): #上昇したかを判断
#     pressure=get_pressure()
#     time.sleep(0.1)
    
#     if pressure<(land_pressure-1.21923): #3階用 
#     #if pressure<(land_pressure-7.84011):#50m以上になったら上がったと判断
#         i+=1
#         print("In the sky")
#         print(i)
#     else: #50m地点に上がりきるまでyetを出力
#         i=0
#         print("yet") 
# print("next\n") #10回連続50m以上の値になったら着地判定へ


# i=0
# while(i<=10): #着地したかを判断
#     pressure=get_pressure()

#     if pressure>(land_pressure-0.05): 
#         i=i+1
#         print(i)
#     else: 
#         i=0
#         print("yet")
#     time.sleep(0.1)
# print("On the land")


#展開検知
while True: #赤の割合が一定以下になるまで繰り返す
    #nhrm()
    print("nhrm")

    data=takepic()
    prop=data[1] #Rの割合取得
    
    print("prop:"+str(prop))
    

#     if prop　<　prop_closed: #red_closeは具体的な値入れる
#         break
#     else:
#         print("yet")
#         continue
# print("succeed")

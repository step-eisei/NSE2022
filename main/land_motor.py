#気圧⇒GPS⇒ニクロム⇒前進⇒GPS（⇒ニクロム…）

from xml.dom.expatbuilder import parseString
from xmlrpc.client import NOT_WELLFORMED_ERROR
from gpiozero import Motor
import time
import RPi.GPIO as GPIO
from smbus import SMBus
import math
from gpiozero import Motor
import serial
import micropyGPS
import csv
import threading

theta = 0
gps_latitude = 0
gps_longitude = 0
x_now = 0
y_now = 0
x_goal = 0
y_goal = 0
satellites_used = 0


def go_ahead():
    motor = Motor(17, 18)
    #motor.forward(2)
    motor.stop()

def nchrm(): #ニクロム線加熱
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17,  GPIO.OUT)

    GPIO.output(17, True)
    #ここの数字は実験次第
    time.sleep(2)
    GPIO.output(17, False)
    time.sleep(5)

def rungps(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/ttySOFT0', 4800, timeout=20)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    while True:
        try:
            sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
            if sentence[0] != '$': # 先頭が'$'でなければ捨てる
                continue
            for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
                gps.update(x)
        except:
            print("GPS error, but ignored")

def getgps():
    global gps_latitude
    global gps_longitude
    global gps
    while True:
        if gps.clean_sentences > 20: # ちゃんとしたデーターがある程度たまったら出力する
            h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24

            gps_latitude = gps.latitude[0]
            gps_longitude = gps.longitude[0]

            break
        time.sleep(3)
    return gps_latitude,gps_longitude

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
        print('pressure : {} hPa'.format(pressure))
        

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
            return x #main関数が呼び出されたら渡す
        except KeyboardInterrupt:
            pass

#　↑ここまでが気圧を測定するプログラム

def average_pressure():
    sum=0.0
    land=0.0
    
    for i in range(20):
        land=get_pressure()
        sum+=land
        time.sleep(0.1)

    average_pressure=sum/20
    return average_pressure

def csv_write_f(x,y):
   

    lat=x
    long=y
    flag = True

    def write():

        import datetime
        import csv

        nonlocal flag

        if flag:
            now_time = datetime.datetime.now()

            with open('gps.csv','w',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow(["GPS"])
            flag = False


        with open('gps.csv','a') as f: 
                writer = csv.writer(f)
                writer.writerow([lat,long])

    return write


#ここまでが関数の定義



# land_pressure=average_pressure() #地表での気圧を打ち上げ前に取得
# print('land_pressure : {} hPa'.format(land_pressure))


# print("閾値: "+str(land_pressure-1.2193))
# i=0
# while(i<=10):
#     pressure=get_pressure()
#     time.sleep(0.1)


#     if pressure<(land_pressure-1.21923): #50m以上になったら上がったと判断
#         i+=1
#         print("In the sky")
#         print(i)
#     else: #50m地点に上がりきるまでyetを出力
#         i=0
#         print("yet") 
# print("next\n") #10回連続50m以上の値になったら着地判定へ

# i=0
# while(i<=10):
#     pressure=get_pressure()

#     if pressure>land_pressure-0.05: #地面の値に近いとき着地
#         i=i+1
#         print(i)
#     else: #地面での値より小さいときまだ飛んでいると判断
#         i=0
#         print("yet")

#     time.sleep(0.1)

# print("On the land")
#着地検知

#gpsの設定
gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSオブジェクトを生成する。
                                     # 引数はタイムゾーンの時差と出力フォーマット
gpsthread = threading.Thread(target=rungps, args=()) # 上の関数を実行するスレッドを生成
gpsthread.setDaemon(True)
gpsthread.start() 

while True:#展開検知
    while True:
        getgps()
        past_lat=gps_latitude
        past_long=gps_longitude
        if past_lat==0 or past_long==0:
            continue
        else:
            break
    csv_write_f(past_lat,past_long)
    nchrm() #10s
    #go_ahead() #2s
    print("go")
    while True:
        getgps()
        lat_1=gps_latitude
        long_1=gps_longitude
        if lat_1==0 or long_1==0:
            continue
        else:
            break
    csv_write_f(lat_1,long_1)
    #go_ahead() #2s
    print("go")
    while True:
        getgps()
        lat_2=gps_latitude
        long_2=gps_longitude
        if lat_2==0 or long_2==0:
            continue
        else:
            break
    csv_write_f(lat_2,long_2)
    #go_ahead() #2s
    print("go")
    while True:
        getgps()
        lat_3=gps_latitude
        long_3=gps_longitude
        if lat_3==0 or long_3==0:
            continue
        else:
            break
    csv_write_f(lat_3,long_3)

    lat_range=0.00006 #誤差，若干大きめにとってる
    long_range=0.00003

    if past_lat - lat_range < lat_1< past_lat + lat_range and past_long - long_range < long_1< past_long + long_range: 
        print("stopping")
        print("past:"+str(past_lat)+"/"+str(past_long))
        print("now1:"+str(lat_1)+"/"+str(long_1))
        continue
    elif past_lat - lat_range < lat_2 < past_lat + lat_range and past_long - long_range < long_2 <past_long + long_range:
        print("stopping")
        print("past:"+str(past_lat)+"/"+str(past_long))
        print("now2:"+str(lat_2)+"/"+str(long_2))
        continue
    elif past_lat - lat_range < lat_3 < past_lat + lat_range and past_long - long_range < long_3 < past_long + long_range:
        print("stopping")
        print("past:"+str(past_lat)+"/"+str(past_long))
        print("now3:"+str(lat_3)+"/"+str(long_3))
        continue
           
    else: 
        print("moving")
        print("past:"+str(past_lat)+"/"+str(past_long)+"\n")
        print("now:"+str(lat_2)+"/"+str(long_2)+"\n")
        break

print("open!")

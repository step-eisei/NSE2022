from xml.dom.expatbuilder import parseString
from xmlrpc.client import NOT_WELLFORMED_ERROR
from gpiozero import Motor
import time
from smbus import SMBus
import math
from gpiozero import Motor
import numpy as np
import csv
import threading
import time
import timeout_decorator

def nchrm(): #ニクロム線加熱
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17,  GPIO.OUT)

    GPIO.output(17, True)
    #ここの数字は実験次第
    time.sleep(2)
    GPIO.output(17, False)
    time.sleep(5)


def pressure():
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
            return x #pressure関数が呼び出されたら渡す
        except KeyboardInterrupt:
            pass

#　↑ここまでが気圧を測定するプログラム

def average_pressure(): #投下前に地上での気圧の値を取得，閾値とする
    sum=0
    
    for i  in range(20):
        pressure=pressure()
        sum+=pressure
        time.sleep(0.1)

    average_pressure=sum/20
    return average_pressure

#ここまでが関数の定義



land_pressure=average_pressure() #地表での気圧，閾値
print('land_pressure : {} hPa'.format(land_pressure))
print("閾値: "+str(land_pressure-7.84011))
i=0

while(i<=10): #上昇したかを判断
    pressure=pressure()
    time.sleep(0.1)

    if pressure<(land_pressure-7.84011): #50 m以上になったら上がったと判断
        i+=1
        print("flying\n")
        print('pressure1 : {} hPa'.format(pressure))
        print(i)
    else: #50 m地点に上がりきるまでyetを出力
        i=0
        print("yet") 
print("next\n") #10 回連続50 m以上の値になったら着地判定へ

i=0
while(i<=10): #着地したかを判断
    pressure=pressure()

    if pressure>land_pressure-0.05: #地面の値に近いとき着地
        i=i+1
        print('pressure2 : {} hPa'.format(pressure))
        print(i)
    else: #地面での値より小さいときまだ飛んでいると判断
        i=0
        print("yet")

    time.sleep(0.1)

print("land")
#着地検知


#展開検知
while True: #赤の割合が減るまで繰り返す
    nchrm() #7s

    timeout_decorator.timeout(10) #タイムアウトの制限時間を10sに設定
    if __name__=='__main__':
        try:
            data=takepic() #カメラの関数，ここには書いてない
            prop=data[1] #赤の割合取得
        except:
            print("try again")
        else:
            print("end")

    red_close=80 #閉じてる時の割合(%)，今は適当
    range=60 #開閉時の差，今は適当

    if prop<red_close-range:
        break
    else:
        print("close\n")
        print("red:"+prop+"\n")
        continue
print("open!!")

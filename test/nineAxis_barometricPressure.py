# -*- coding: utf-8 -*-

# ------------------- nineAxis -------------------
# reference webpage :
# https://taku-info.com/bmx055howtouse-mag/

from smbus import SMBus
import time
import math
import datetime
import csv

# I2C
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42

i2c = SMBus(1)

def bmx_setup():
    # acc_data_setup : 加速度の値をセットアップ
    i2c.write_byte_data(ACCL_ADDR, 0x0F, 0x03)
    i2c.write_byte_data(ACCL_ADDR, 0x10, 0x08)
    i2c.write_byte_data(ACCL_ADDR, 0x11, 0x00)
    time.sleep(0.5)

    # gyr_data_setup : ジャイロ値をセットアップ
    i2c.write_byte_data(GYRO_ADDR, 0x0F, 0x04)
    i2c.write_byte_data(GYRO_ADDR, 0x10, 0x07)
    i2c.write_byte_data(GYRO_ADDR, 0x11, 0x00)
    time.sleep(0.5)

    # mag_data_setup : 地磁気値をセットアップ
    data = i2c.read_byte_data(MAG_ADDR, 0x4B)
    if(data == 0):
        i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
        time.sleep(0.5)
    i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
    i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
    i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
    i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
    i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
    time.sleep(0.5)

def acc_value():
    data = [0, 0, 0, 0, 0, 0]
    acc_data = [0.0, 0.0, 0.0]

    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i)

        for i in range(3):
            acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16
            if acc_data[i] > 2047:
                acc_data[i] -= 4096
            acc_data[i] *= 0.0098

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return acc_data

def gyro_value():
    data = [0, 0, 0, 0, 0, 0]
    gyro_data = [0.0, 0.0, 0.0]

    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(GYRO_ADDR, GYRO_R_ADDR + i)

        for i in range(3):
            gyro_data[i] = (data[2*i + 1] * 256) + data[2*i]
            if gyro_data[i] > 32767:
                gyro_data[i] -= 65536
            gyro_data[i] *= 0.0038

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return gyro_data

def mag_value():
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    mag_data = [0.0, 0.0, 0.0]

    try:
        for i in range(8):
            data[i] = i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)

        for i in range(3):
            if i != 2:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                if mag_data[i] > 4095:
                    mag_data[i] -= 8192
            else:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                if mag_data[i] > 16383:
                    mag_data[i] -= 32768

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return mag_data



bmx_setup()
time.sleep(0.1)

now_time = datetime.datetime.now()
filename = 'test_' + now_time.strftime('%m%d_%H%M') + '.csv'
# ファイル，1行目(カラム)の作成
with open(filename, 'a') as f:
    writer = csv.writer(f)
    writer.writerow(["acc1","acc2","acc3","gyro1","gyro2","gyro3","mag1","mag2","mag3"])
f.close()

# ------------------- nineAxis_end -------------------

# ------------------- BME280 -------------------

# from smbus import SMBus
# import time

bus_number  = 1
i2c_address = 0x76

# ------------------- BME280_end -------------------

while True:
    # ------------------- nineAxis -------------------
    acc = acc_value()
    gyro= gyro_value()
    mag = mag_value()

    print("Accl -> x:{}, y:{}, z: {}".format(acc[0], acc[1], acc[2]))
    print("Gyro -> x:{}, y:{}, z: {}".format(gyro[0], gyro[1], gyro[2]))
    print("Mag -> x:{}, y:{}, z: {}".format(mag[0], mag[1], mag[2]))
    print("\n")
    time.sleep(0.1)

    with open(filename, 'a', newline="") as f:
        writer = csv.writer(f)
        writer.writerow([acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2],mag[0],mag[1],mag[2]])
    f.close()
    # ------------------- nineAxis_end -------------------
    
    # ------------------- BME280 -------------------
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
      compensate_P(pres_raw)
      compensate_H(hum_raw)

    def compensate_P(adc_P):
      global  t_fine
      pressure = 0.0

      v1 = (t_fine / 2.0) - 64000.0
      v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
      v2 = v2 + ((v1 * digP[4]) * 2.0)
      v2 = (v2 / 4.0) + (digP[3] * 65536.0)
      v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
      v1 = ((32768 + v1) * digP[0]) / 32768

      if v1 == 0:
        return 0
      pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
      if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1
      else:
        pressure = (pressure / v1) * 2
      v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
      v2 = ((pressure / 4.0) * digP[7]) / 8192.0
      pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)  

      print('pressure : {} hPa'.format(pressure/100))

        # print "pressure : %7.2f hPa" % (pressure/100)

    def compensate_T(adc_T):
      global t_fine
      v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
      v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
      t_fine = v1 + v2
      temperature = t_fine / 5120.0
      print('temp : {} ℃'.format(temperature))
        # print "temp : %-6.2f ℃" % (temperature) 

    def compensate_H(adc_H):
      global t_fine
      var_h = t_fine - 76800.0
      if var_h != 0:
        var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
      else:
        return 0
      var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
      if var_h > 100.0:
        var_h = 100.0
      elif var_h < 0.0:
        var_h = 0.0
      print('hum : {}'.format(var_h))
        # print "hum : %6.2f ％" % (var_h)


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
        readData()
      except KeyboardInterrupt:
        pass
    
    # ------------------- BME280_end -------------------
    

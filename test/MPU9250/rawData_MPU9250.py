# -*- coding: utf-8 -*-
import smbus
import time

I2C = smbus.SMBus(1)    # I2C smbus

MP9250_ADR = 0x68    # MPU9250 I2C slave address
AK8963_ADR = 0x0C    # AK8963 I2C slave address
DEVICE_ID = 0x71    ## Device id

# MPU-9250 Register Addresses
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
INT_PIN_CFG    = 0x37
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43
PWR_MGMT_1     = 0x6B
WHO_AM_I       = 0x75

# Gyro Full Scale Index
GFS_250  = 0
GFS_500  = 1
GFS_1000 = 2
GFS_2000 = 3
GYR_FS = [0x00,0x08, 0x10, 0x18]        # ジャイロFSレンジのBit設定 Reg29[4:3]
GYR_RNG = [250.0, 500.0, 1000.0, 2000.0]    # ジャイロFSレンジ(単位=deg/sec)

# Accel Full Scale Index
AFS_2G   = 0
AFS_4G   = 1
AFS_8G   = 2
AFS_16G  = 3
ACC_FS = [0x00, 0x08, 0x10,0x18]    # 加速度FSレンジのBit設定 Reg28[4:3]
ACC_RNG = [2.0, 4.0, 8.0, 16.0]        # 加速度FSレンジ(単位=G)

# AK8963 Register Addresses
AK8963_ST1     = 0x02    # ステータス
AK8963_MAG_OUT = 0x03    # 測定データ
AK8963_CNTL1   = 0x0A    # コントロール
AK8963_ASAX    = 0x10    # 感度調整値

# CNTL1 Mode select
AK8963_MODE_DOWN   = 0x00    # パワーダウンモード
AK8963_MODE_ONE    = 0x01    # 単発測定モード
AK8963_MODE_C8HZ   = 0x02    # 連続測定モード１(8Hz)
AK8963_MODE_C100HZ = 0x06    # 連続測定モード２(100Hz)
AK8963_BIT_14      = 0x00    # 14bits output
AK8963_BIT_16      = 0x10    # 16bits output


class MPU9250:
    def __init__(self, mpuAdr=MP9250_ADR):
        self.mpuAdr = mpuAdr
        self.configMPU9250(gfsNo=GFS_250, afsNo=AFS_2G)    # Default
        self.configAK8963(mode=AK8963_MODE_C8HZ, mfs=AK8963_BIT_16)    # Default

    def searchDevice(self):    # Device Check
        who_am_I = I2C.read_byte_data(self.mpuAdr, WHO_AM_I)
        if(who_am_I == DEVICE_ID):
            return True
        else:
            return False

    def configMPU9250(self, gfsNo, afsNo):    # Configure MPU-9250
        if gfsNo < 0 or gfsNo > 3:
            gfsNo = 3
        self.gres = GYR_RNG[gfsNo] / 32768.0

        if afsNo < 0 or afsNo > 3:
            afsNo = 3
        self.ares = ACC_RNG[afsNo] / 32768.0

        I2C.write_byte_data(self.mpuAdr, PWR_MGMT_1, 0x00)    # sleep off
        time.sleep(0.1)
        I2C.write_byte_data(self.mpuAdr, PWR_MGMT_1, 0x01)    # auto select clock source
        time.sleep(0.1)
        I2C.write_byte_data(self.mpuAdr, CONFIG, 0x03)    # DLPF_CFG(Low Pass Filter 41Hz)
        I2C.write_byte_data(self.mpuAdr, SMPLRT_DIV, 0x04)    # sample rate divider
        I2C.write_byte_data(self.mpuAdr, GYRO_CONFIG, GYR_FS[gfsNo])    # gyro full scale select
        I2C.write_byte_data(self.mpuAdr, ACCEL_CONFIG, ACC_FS[afsNo])    # accel full scale select
        I2C.write_byte_data(self.mpuAdr, ACCEL_CONFIG_2, 0x03)    # A_DLPFCFG(Low Pass Filter 41Hz)
        I2C.write_byte_data(self.mpuAdr, INT_PIN_CFG, 0x02)    # BYPASS_EN
        time.sleep(0.1)

    def configAK8963(self, mode, mfs):    # Configure AK8963
        if mfs == AK8963_BIT_14:
            self.mres = 4912.0/8190.0
        else: #  mfs == AK8963_BIT_16:
            self.mres = 4912.0/32760.0

        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, 0x00)
        time.sleep(0.01)
        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, 0x0F)    # set read FuseROM mode
        time.sleep(0.01)
        data = I2C.read_i2c_block_data(AK8963_ADR, AK8963_ASAX, 3)    # read coef data
        self.magCoef = [0., 0., 0.]
        for ax in range(3):
            self.magCoef[ax] = (data[ax] - 128) / 256.0 + 1.0

        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, 0x00)    # set power down mode
        time.sleep(0.01)
        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, (mfs | mode))# set scale&continous mode
        time.sleep(0.01)

    def checkDataReady(self):
        dataReady = I2C.read_byte_data(self.mpuAdr, INT_STATUS)
        if dataReady & 0x01:
            return True
        else:
            return False

    # Read accelerometer and gyro
    def readAccGyro(self):
        data = I2C.read_i2c_block_data(self.mpuAdr, ACCEL_OUT, 14)
        # ACC=data[0:5], TEMP=data[6:7], GYRO=data[8:13] ---- MSB:LSB
        accData = [0., 0., 0.]
        for ax in range(3):    # set accelerometer data
            accData[ax] = self.byte2data(data[ax*2], data[ax*2+1])    # (MSB, LSB)
            accData[ax] = round(accData[ax] * self.ares, 3)
        gyrData = [0., 0., 0.]
        for ax in range(3):    # set gyro data
            gyrData[ax] = self.byte2data(data[8+ax*2], data[9+ax*2])    # (MSB, LSB)
            gyrData[ax] = round(gyrData[ax] * self.gres, 3)
        return accData, gyrData


    # Read mag
    def readMag(self):
        magData = [0., 0., 0.]
        dataReady = I2C.read_byte_data(AK8963_ADR, AK8963_ST1)
        if dataReady & 0x01 :    # Check data ready
            data = I2C.read_i2c_block_data(AK8963_ADR, AK8963_MAG_OUT, 7)
            # MAG=data[0:5] --- (LSB:MSB), OverFlow=data[6]
            if (data[6] & 0x08) != 0x08:    # Check overflow
                for ax in range(3):
                    magData[ax] = self.byte2data(data[ax*2+1], data[ax*2])    # (MSB, LSB)
                    magData[ax] = round(magData[ax] * self.mres * self.magCoef[ax], 3)
        return magData

    # Read temperature
    def readTemp(self):
        data = I2C.read_i2c_block_data(self.mpuAdr, TEMP_OUT, 2)
        tempData = self.byte2data(data[0], data[1])    # (MSB, LSB)
        tempData = round((tempData / 333.87 + 21.0), 3)
        return tempData

    def byte2data(self, byteMSB, byteLSB):
        value = (byteMSB << 8) | byteLSB
        value = - (value & 0x8000) | (value & 0x7FFF)    # 2の補数
        return value

if __name__ == '__main__':
    mpu9250 = MPU9250(MP9250_ADR)        # 0x68
    if mpu9250.searchDevice():
        print("Who am I? OK")
    else:
        print("Who am I? Error")

    mpu9250.configMPU9250(gfsNo = GFS_2000, afsNo = AFS_16G)
    # Full Scale : Gyro=2000deg/sec, Acc=16g
    mpu9250.configAK8963(mode = AK8963_MODE_C8HZ, mfs = AK8963_BIT_16)
    # mode=Continous 8Hz, 16bit output

    for k in range(1000):
        if not mpu9250.checkDataReady():
            print("Data not ready")
            continue
        accXYZ, gyrXYZ = mpu9250.readAccGyro()
        magXYZ = mpu9250.readMag()
        temp = mpu9250.readTemp()

        strAcc ="Acc = {:7.2f}, {:7.2f}, {:7.2f}, ".format(accXYZ[0], accXYZ[1], accXYZ[2])
        strGyro ="Gyro = {:7.2f}, {:7.2f}, {:7.2f}, ".format(gyrXYZ[0], gyrXYZ[1], gyrXYZ[2])
        strMag ="Mag = {:7.2f}, {:7.2f}, {:7.2f}, ".format(magXYZ[0], magXYZ[1], magXYZ[2])
        strTemp ="Temp = {:7.2f}".format(temp)
        print(strAcc + strGyro + strMag + strTemp)

import smbus
import time
i2c = smbus.SMBus(1) # ラズパイのI2Cオブジェクト

you_are_lps = i2c.read_i2c_block_data(0x5C, 0x0F, 1)

print(you_are_lps)
print(hex(you_are_lps[0]))

# 各アドレスを定数化
ADDR = 0x5C
CTRL_REG1 = 0x20
WRITE_REG1 = 0x90
PRESS_OUT_XL = 0x28

while True:
     # 読み取りモードに設定
    bus = smbus.SMBus(1)
    bus.write_byte_data(ADDR, CTRL_REG1, WRITE_REG1)

    # ここを書き換えた
    data = bus.read_i2c_block_data(ADDR, 0xA8, 3)

    pressure = (data[2]<<16 | data[1] << 8 | data[0]) / 4096.0

    print ("気圧は・・・ : %.2f hPa" %pressure)
    time.sleep(1)
   

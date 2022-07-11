# 受信側の端末で実行するプログラムです．

import serial
import time
from struct import *
 
COM="COM4"
bitRate=115200

ser = serial.Serial(COM, bitRate, timeout=0.1) # 使用するポート番号，ビットレートを宣言

def bytes2float(data_bytes):
    n=0
    first = 0 
    count = 0

    for i in list(str(data_bytes)):
        count += 1
        if i == ';':
            n += 1
            if n == 7:
                first = count
                break
    data_bytes = data_bytes[first-2:-6] #必要なデータだけを取り出す 
    data_float = unpack('>ddd',data_bytes) #デコードする
    return data_float

def csv_write_f():

    flag = True
    filename = ""

    def write(x,y,z):

        import datetime
        import csv

        nonlocal flag
        nonlocal filename

        if flag:
            now_time = datetime.datetime.now()
            filename = 'test_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

            with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z"])
            flag = False


        with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow([x, y, z])

    return write

csv_write = csv_write_f()


try:

    while True:
        time.sleep(0.1)
        
        data = ser.read_all() #データ(バイト型)を受信

        if data != b'':
            print(data)
            data = bytes2float(data) #バイト型からフロート型に変換
            print(data)
            csv_write(*data)
            data = b''
		

except KeyboardInterrupt:
    print('stop!')
    ser.close()


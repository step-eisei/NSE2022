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

try:

    while True:
        time.sleep(0.1)
        
        data = ser.read_all() #データ(バイト型)を受信

        if data != b'':
            print(data)
            data = bytes2float(data) #バイト型からフロート型に変換
            print(data)
            data = b''
		

except KeyboardInterrupt:
    print('stop!')
    ser.close()

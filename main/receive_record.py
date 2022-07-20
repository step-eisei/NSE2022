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

    phase_number = str(data_bytes)[5]
    print(phase_number)


    if phase_number == '1':
        data_float = unpack('>bd',data_bytes) #デコードする
    elif phase_number == '2':
        data_float = unpack('>bd',data_bytes) #デコードする
    elif phase_number == '3':
        data_float = unpack('>bddd',data_bytes) #デコードする
    else:
        data_float = unpack('>bdd',data_bytes) #デコードする

    return data_float

def csv_write_f():

    flag1 = True
    flag2 = True
    flag3 = True
    flag4 = True

    filename = ""

    def write(*data):

        import datetime
        import csv

        nonlocal flag1
        nonlocal flag2
        nonlocal flag3
        nonlocal flag4

        nonlocal filename


        if data[0] == 1:
            if flag1:
                now_time = datetime.datetime.now()
                filename = 'land_detect_phase_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

                with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow(["pressure"])
                flag1 = False


            with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow([data[1]])

        elif data[0] == 2:
            if flag2:
                now_time = datetime.datetime.now()
                filename = 'open_detect_phase_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

                with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow(["prop"])
                flag2 = False

            with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow([data[1]])


        elif data[0] == 3:
            if flag3:
                now_time = datetime.datetime.now()
                filename = 'guide_phase1_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

                with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow(["theta", "gps_latitude", "gps_longitude"])
                flag3 = False

            with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow([data[1], data[2], data[3]])

        else:
            if flag4:
                now_time = datetime.datetime.now()
                filename = 'guide_phase2_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

                with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow(["theta", "prop"])
                flag4 = False

            with open(filename,'a',newline='') as f: 
                    writer = csv.writer(f)
                    writer.writerow([data[1], data[2]])

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
            csv_write(*data) # (phase,変数1,変数2,変数3,,,)
            data = b''
		
except KeyboardInterrupt:
    print('stop!')
    ser.close()


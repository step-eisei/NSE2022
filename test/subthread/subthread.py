import time
import threading
import serial
from struct import *

def subThread():

    COM = '/dev/ttyAMA0'
    ser_sub = serial.Serial(COM, 115200)   
    
    def send_data(*data):

        if data[0] == "land_detect":
            data = pack('>bd',1, data[1])
            data = data + b'\n'
            ser_sub.write(data)

        elif data[0] == "open_detect":
            data = pack('>bd', 2, data[1])
            data = data + b'\n'
            ser_sub.write(data)

        elif data[0] == "guide_phase1":
            data = pack('>bdddd',3, data[1], data[2], data[3], data[4], data[5])
            data = data + b'\n'
            ser_sub.write(data)

        else:
            data = pack('>bdd', 4, data[1], data[2])
            data = data + b'\n'
            ser_sub.write(data)


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


            if data[0] == "land_detect":
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

            elif data[0] == "open_detect":
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


            elif data[0] == "guide_phase1":
                if flag3:
                    now_time = datetime.datetime.now()
                    filename = 'guide_phase1_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

                    with open(filename,'a',newline='') as f: 
                        writer = csv.writer(f)
                        writer.writerow(["theta", "gps_latitude", "gps_longitude","x_now","y_now"])
                    flag3 = False


                with open(filename,'a',newline='') as f: 
                        writer = csv.writer(f)
                        writer.writerow([data[1], data[2], data[3], data[4], data[5]])

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



    while True:

        csv_write(*read_data)
        send_data(*read_data)

        time.sleep(1)


pressure = 1.0
gps_latitude = 2.0 
gps_longitude = 3.0
theta = 4.0
x_now = 5.0
y_now = 6.0
prop = 7.0


read_data = ("land_detect",pressure) 

th_subthread = threading.Thread(target=subThread)

th_subthread.setDaemon(True)

th_subthread.start()

time.sleep(7)
read_data = ("open_detect",prop)
time.sleep(7)
read_data = ("guide_phase1",theta,gps_latitude,gps_longitude,x_now,y_now)
time.sleep(7)
read_data = ("guide_phase2",theta,prop)
time.sleep(7)

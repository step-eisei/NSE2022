from lib2to3.pgen2.token import NEWLINE
from tkinter import W
import serial
import micropyGPS
import threading
import time
import csv

#GPSモジュールからデータをmy_gps(GPSオブジェクト)に追加，＊引数はタイムゾーンの時差（日本は＋9時間）と，経度緯度の出力フォーマットを指定（ddm,dms,ddなどから）
my_gps = micropyGPS.MicropyGPS(9, 'dd')

#GPSモジュールを読み，my_gpsを更新する関数
def rungps():
    #シリアルポートの指定，シリアル通信の速度[bps]を指定
    ser = serial.Serial("/dev/ttySOFT0",4800,timeout=20)
    ser.readline()
    while True:
        #受信したデータをバイナリデータからテキストデータへ変換
        sentence = ser.readline().decode('utf-8')
        #先頭が'$'でなければ捨てる
        if sentence[0] != '$':
            continue
        #
        for x in sentence:
            my_gps.update(x)

#上の関数を実行するスレッドを生成
gpsthread = threading.Thread(target=rungps, args=())
#gpsスレッドをデーモン化
gpsthread.daemon = True
#スレッドを起動
gpsthread.start()

#引数には（保存先のパス，読み書きの指定やバイナリ，テキストの指定はmodeを使う,）
with open('goal_gps.csv',mode='w',newline='') as f:
    writer = csv.writer(f)
    #一行書き込み
    writer.writerow(["goal_latitude", "goal_longitude"])

#10回データを書き込めば終了
gps_latitude = 100
gps_longitude = 100
sum_latitude = 0
sum_longitude = 0
for i in range(10):
    #ちゃんとしたデータがある程度たまってから出力
    if my_gps.clean_sentences > 20:

        gps_latitude = my_gps.latitude[0]
        gps_longitude = my_gps.longitude[0]
        sum_latitude += gps_latitude
        sum_longitude += gps_longitude
        #衛星数を出力
        print(my_gps.satellites_used)
        #mode='a'は追記モードでファイルを開く
        with open('goal_gps.csv',mode='a',newline='') as f:
            writer = csv.writer(f)
            writer.writerow([gps_latitude,gps_longitude]) 
    #time.sleep(20.0)
    #変化した値を入れていく
    for t in range(100000):
        if(gps_latitude != my_gps.latitude[0] and gps_longitude != my_gps.longitude[0]):
            break

#平均値を算出しgoal.pyに書き込み
goal_la = sum_latitude / 10
goal_lo = sum_longitude / 10

with open('goal.csv',mode='a',newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["goal_latitude", "goal_longitude"])

with open('goal.csv',mode='a',newline='') as f:
           writer = csv.writer(f)
           writer.writerow([goal_la,goal_lo]) 

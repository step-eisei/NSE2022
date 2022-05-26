#データを受け取ると真球化
import numpy as np
#import matplotlib.pyplot as plt
import os

fname='test_0525_2128.csv'
#data00csv.txtの最後の３列がmG（ミリガウス）単位の地磁気データx、y、zとなっている
nineaxis_data=np.loadtxt(fname, delimiter=',',skiprows=1)

#地磁気データ抽出
magx=-nineaxis_data[:,6]
magy=nineaxis_data[:,7]
magz=-nineaxis_data[:,8]
def true_sphere_parameter(fname):
    parameter_file=open("./parameter/"+"parameter_"+fname+"_.csv")
    parameter=np.loadtxt(parameter_file, delimiter=',',skiprows=1)
    x0,y0,z0,sx,sy,sz,P_00,P_01,P_02,P_10,P_11,P_12,P_20,P_21,P_22=parameter
    P=np.matrix([[P_00,P_01,P_02],
                [P_10,P_11,P_12],
                [P_20,P_21,P_22]])
    return x0,y0,z0,sx,sy,sz,P

def true_sphere(magx,magy,magz):
    
                
    #print(type(P))#<class 'numpy.matrix'>

    #移動計算のためデータ整形
    mag2=np.vstack([magx,magy,magz],)

    #回転
    mag2=P.T*mag2
    #並行移動
    magx2=np.array(mag2)[0]-x0
    magy2=np.array(mag2)[1]-y0
    magz2=np.array(mag2)[2]-z0

    #拡大縮小
    magx2=sx*magx2
    magy2=sy*magy2
    magz2=sz*magz2
    return magx2,magy2,magz2

x0,y0,z0,sx,sy,sz,P=true_sphere_parameter(fname)
magx2,magy2,magz2=true_sphere(magx,magy,magz)



if __name__=="__main__":
    #データのプロット
    # グラフの枠を作成
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111, projection='3d')

    # X,Y,Z軸にラベルを設定
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim([-500,500])
    ax.set_ylim([-400,600])
    ax.set_zlim([-600,400])
    #ax.set_box_aspect((1,1,1)) #自分のverだとエラー吐く

    # 生データ描画
    ax.plot(magx,magy,magz,marker=".",ms=1, linestyle='None')

    #回転・平行移動後の描画
    ax.plot(magx2,magy2,magz2,marker=".",ms=1, linestyle='None')


    #xyz軸描画
    ax.plot([-500,500],[0,0],[0,0])
    ax.plot([0,0],[-400,600],[0,0])
    ax.plot([0,0],[0,0],[-400,600])

    # グラフ表示
    plt.show()

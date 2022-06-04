import numpy as np
from smbus import SMBus
import time
from nineAxisSensor_test import bmx_setup,mag_value
from sinnen import true_sphere,true_sphere_parameter
import matplotlib.pyplot as plt

fname='data00csv.txt'
x0,y0,z0,sx,sy,sz,P=true_sphere_parameter(fname)

# I2C
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42

i2c = SMBus(1)

bmx_setup()
time.sleep(0.1)


loop=True
r_history,theta_history=[],[]
while loop==True: 
    
    magx,magy,magz = mag_value()
    magx2,magy2,magz2=true_sphere(magx,magy,magz)
    theta=np.degrees(np.arctan2(magy2, magx2))
    print(theta)
    
    #グラフ表示
    r_history.append(np.sqrt(magy2**2+magx2**2))
    theta_history.append(theta)
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='polar')
    scatter=ax.scatter(theta_history, r_history)
    ax.set_title('nine_axis_calc_angle',fontsize=18)
    #次の描画まで（）秒待つ
    plt.pause(5)
    # グラフをクリア
    scatter.remove()

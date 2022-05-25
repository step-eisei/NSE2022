import numpy as np
import matplotlib.pyplot as plt
import time

import matplotlib.pyplot as plt



r_history,theta_history=[],[]
for r in range(4):
    theta = 45*2*np.pi/360
    r_history.append(r)
    theta_history.append(theta)
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='polar')
    line=ax.scatter(theta_history, r_history)
    ax.set_title('nine_axis_calc_angle',fontsize=18)
    
    print(r_history,theta_history)
    plt.pause(5)
    # グラフをクリア
    line.remove()
    
    
# #print(np.degrees(np.arctan2(1, 1)))
# 90

# print(np.degrees(np.arctan2(-1, 0)))
# # -180.0
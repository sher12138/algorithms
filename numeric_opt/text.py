
import math
import matplotlib.pyplot as plt
import numpy as np



# ego (0,0) 10  
# obs (0,1) 13   3 


def draw_circle(v1,v2,obs_x,obs_y, ego_x,ego_y,a, heading):

    # t =np.linspace(1, 10, 100)
    # x1 =  v1 * t + ego_x + 0.5 * a * t**2
    # y1 =  t * 0 + ego_y 

    # x2 =  v2 * np.cos(heading / 180 * math.pi) * t + obs_x
    # y2 =  v2 * np.sin(heading / 180 * math.pi) * t + obs_y 
    x0 = []
    y0 = []
    x01 = []
    y01 = []
    for i in range(1,8):
        t0 = obs_y / (v2 * np.sin(heading / 180 * math.pi))
        x0.append(v1 * t0 + ego_x + 0.5 * i * t0**2)
        y0.append(0)
        x01.append(v2 * np.cos(heading / 180 * math.pi) * t0 + obs_x)
        y01.append(v2 * np.sin(heading / 180 * math.pi) * t0 + obs_y)
    print(x0,y0)
    print(x01,y01)
    plt.plot(x0,y0, label='ego')
    plt.plot(x01,y01, label='obs')
    plt.xlabel('a')
    plt.ylabel('t')
    plt.show()


def sin():
    x = np.linspace(0,10,100)
    y = np.sin(x * np.pi / 180)
    print(x,y)
    plt.plot(x,y)
    plt.show()

# draw_circle(11,13 , -2,1,0,0,-1,-3) # 对方正在超车，但

sin()
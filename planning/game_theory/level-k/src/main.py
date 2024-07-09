"""
Game Theoretic Modeling of Vehicle Interactions at
Unsignalized Intersections and Application to Autonomous Vehicle Control


// 地图模块： 十字路口，无信号灯，纯交互转向，处理自车直行如何与转向车辆交互问题

需要模块：

1. ego 定义， obs 定义 
    -ego:  状态， 动作信息， 轨迹， 信念
    -obs: 状态，动作信息，轨迹 
2. cost 计算：
    - R = w1 * c + w2 * s + w3 * o + w4 * l + w5 * d

    
    c 碰撞 
    s 安全区
    o 道路边界
    l 交规
    d 偏离
"""


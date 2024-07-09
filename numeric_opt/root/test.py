import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import fsolve

# 样条曲线1的控制点
points1 = np.array([
    [0, 0],
    [1, 2],
    [2, 0],
    [3, 3]
])

# 样条曲线2的控制点
points2 = np.array([
    [0, 3],
    [1, 1],
    [2, 4],
    [3, 0]
])

# 生成样条曲线
tck1, _ = splprep(points1.T, s=0)
tck2, _ = splprep(points2.T, s=0)

# 定义差函数
def diff_func(params):
    t, u = params
    x1, y1 = splev(t, tck1)
    x2, y2 = splev(u, tck2)
    return [x1 - x2, y1 - y2]

# 初始猜测点
initial_guess = [0.5, 0.5]

# 使用fsolve求解
solution = fsolve(diff_func, initial_guess)

# 交点参数
t_intersect, u_intersect = solution

# 计算交点坐标
x_intersect, y_intersect = splev(t_intersect, tck1)

print(f"交点坐标: ({x_intersect}, {y_intersect})")

"""
LQR迭代解法：

min J = ∫[0,T] (x^T Q x + u^T R u) dt + x^T Qf x
s.t. x_dot = Ax + Bu
x(0) = x0
x(T) = xT
"""
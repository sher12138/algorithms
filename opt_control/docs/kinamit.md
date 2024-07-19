# 常用运动学模型 
![alt text](image.png)
基本状态量：
- $x$,
- $y$
- $\psi$  车辆横摆角
横摆角速度：
$$
(\tan(\delta_f) - \tan(\delta_r)) \cos(\beta) = \frac{l_f + l_r}{R} , 用于表示转向半径\\ 
\mathrm{低速情况，可以认为横摆角速度等于车辆质心加速度} \\ 
\dot{\psi} =   V /R \\
\mathrm{质心侧偏角:} \\ 
\beta = \tan^{-1}(\frac{l_r\tan(\delta_f) + l_f\tan(\delta_r)}{l_f + l_r}) \\
$$
- $\beta$  质心侧偏角
- $v$  车辆速度
- $a$  车辆加速度
- $\delta$  车辆前轮转角
- $\delta_{f}$ 车辆前轮转角
- $l_f$ 前轮轴距

质心为参考点的模型： 
$$\begin{aligned}
\dot{X}&=V\cos(\psi+\beta)\\
\dot{Y}&=V\sin(\psi+\beta)\\
\dot{\psi}&=\frac{V\cos(\beta)}{l_f+l_r}\left(\tan(\delta_f)-tan(\delta_r)\right)\\
\beta&=\tan^{-1}(\frac{l_f\tan\delta_r+l_r\tan\delta_f}{l_f+l_r})\end{aligned}$$

*各个状态量的描述方法*:


## 1 横向模型

### 1.1 阿克曼转向模型

假设后轮转角$\delta_r$为0，前轮转角很小，则$\tan(\delta_f) \approx \delta_f$;质心侧偏角$\beta$很小，$\sin(\beta) \approx \beta$，$\cos(\beta) \approx 1$，则有：
车辆横摆角速度:
$$
\dot{\psi} = \frac{V}{l} \delta
$$


### 扩展：PP
后轴中心模型，车身切线，控制前轮转角，满足$\tan(\delta_f) = L / R$，其中$L$为前后轮轴距，$R$为转向半径。
目标：使车辆沿着一条经过目标路点的圆弧行驶。

前视距离$l_d$圆弧上最近点作为目标点：

计算目标点与车辆的横向误差$e_{ld}=l_d \sin(\alpha)$，

$$
R = \frac{l_d}{2 \sin(\delta)} \\
tan(\delta) = \frac{L}{R} \\
\delta = \tan^{-1}(\frac{2 L * \sin\alpha(t)}{l_d}) \\
e_{ld} = ld^2 / {2L} \tan(\delta) = \frac{l_d^2}{2L} \delta
$$

### 扩展：Stanley 
前轮反馈控制法：基于前轮中心的路径跟踪偏差量对方向盘转向控制量进行计算。
![alt text](image-1.png)
- 航向误差引起的转角$\delta_e$，即当前车身方向与参考轨迹最近的点的切线方向的夹角。
- 横向误差$e$，引起的转角$\delta_y$，
- 前轮转角有最大值限制，

于是，前轮转角：
$$
\delta(t) = \delta_e(t) + \tan^{-1}(\frac{ke(t)}{v(t)})
$$


## 2 纵向模型


# PGM 

式中的$I$表示意图，$V$表示速度分量，$T_m$表示合并车辆到达时间，$T_h$表示主车到达时间。我们的目标是求解$P(I | V, T_m, T_h)$，即给定视觉、语音和手势特征，求解图像的概率分布。
$$
\log{P(I | V, T_m, T_h)} = \log{P(I)} + \log{P(V | I)} + \log{P(T_m | I)} + \log{P(T_h | I)} \\ 

\log{P(V | I)} = \alpha \sum_{i=2}^{N} \log{P(V_i | V_{i-1}, I)} \\
\\

I^{\star} = \arg \max_{I} P(I | V, T_m, T_h) \\
$$
# cost build 

自车代价由 安全性，舒适性，效率三部分组成。
$$
J^{EC} = w^{EC}_{ds}   J^{EC}_{ds} + w^{EC}_{rc}   J^{EC}_{rc}  + w^{EC}_{pe}   J^{EC}_{pe}
$$

安全性代价分别由横纵向安全性代价组成。
$$
J^{EC}_{ds} =(\sigma^2 -1)^2 J^{EC}_{ds - log} + \sigma^2 J^{EC}_{ds - log}
$$
纵向安全代价是纵向gap 和相对速度的函数
$$
J^{EC}_{ds - log} = \kappa^{EC}_{ds-log} \lambda^{EC}_{v}(\Delta v^{EC}_{x,v})^2  + \frac{\kappa^{EC}_{ds-log} }{{\Delta s^{EC}_{x,v}}^2 + \epsilon }
$$

$$
\Delta s^{EC}_{x,v} = \sqrt{(X_v^{LC} - X_v^{EC})^2 + (Y_v^{LC} - Y_v^{EC})^2} - l_v
$$


## payoff matrix 


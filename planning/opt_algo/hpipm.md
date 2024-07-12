## 实例 

## 1.1 模型：
$$
x_{k+1} = Ax_{k} + Bu_{k} \\
y = Cx + Du   \\ 输出:v
$$

状态量：
$\begin{bmatrix}
s,v,a
\end{bmatrix}^T
$

控制量：$jerk$

$$
A = \begin{bmatrix}
1,T,0.5T^2 \\
0,1,T \\
0,0,1
\end{bmatrix}
$$

$$
b = \begin{bmatrix}
1/6T^3,1/2T^2,T
\end{bmatrix}
$$

$$
c = \begin{bmatrix}
0,1,0
\end{bmatrix}
$$

其它量：$g$,$dl$,$du$,

## 1.2 求解器变量

```c++
    // 维度信息
  std::vector<int> nx_;    // number of states  状态量的状态数
  std::vector<int> nu_;    // number of inputs  控制量的状态数 
  std::vector<int> nbx_;   // number of bounds on x
  std::vector<int> nbu_;   // number of bounds on u
  std::vector<int> ng_;    // number of polytopic constratins
  std::vector<int> nsbx_;  // number of slacks variables on x
  std::vector<int> nsbu_;  // number of slacks variables on u
  std::vector<int> nsg_;  // number of slacks variables on polytopic constraints

  // LTV dynamcis 线性时变模型
  std::vector<double *> hA_;  // hA[k] = A_k 0时刻没有状态转移方程A，
  std::vector<double *> hB_;  // hB[k] = B_k 
  std::vector<double *> hb_;  // hb[k] = b_k

  // cost function
  std::vector<double *> hQ_;  // hQ[k] = Q_k
  std::vector<double *> hS_;  // hS[k] = S_k
  std::vector<double *> hR_;  // hR[k] = R_k
  std::vector<double *> hq_;  // hq[k] = q_k
  std::vector<double *> hr_;  // hr[k] = r_k

  // Polytopic constraints  输出约束 
  // g_lower,k <= D_k*x_k + C_k*u_k
  // D_k*x_k + C_k*u_k  <= g_upper,k
  std::vector<double *> hlg_;  // hlg[k] =  g_lower,k
  std::vector<double *> hug_;  // hug[k] =  g_upper,k
  std::vector<double *> hC_;   // hC[k] = C_k
  std::vector<double *> hD_;   // hD[k] = D_k

  // General bounds 边界，状态量 控制量
  // x_lower,k <= x_k <= x_upper,k
  // hidxbx can be used to select bounds on a subset of states
  std::vector<int *> hidxbx_;  // hidxbx[k] = {0,1,2,...,nx} for bounds on all
  // and states
  std::vector<double *> hlbx_;  // x_lower,k
  std::vector<double *> hubx_;  // x_upper,k
  // u_lower,k <= u_k <=  u_upper,k
  // hidxbu can be used to select bounds on a subset of inputs
  std::vector<int *> hidxbu_;  // hidxbuk] = {0,1,2,...,nu} for bounds on all

  // and states
  std::vector<double *> hlbu_;  // u_lower,k
  std::vector<double *> hubu_;  // u_upper,k

  // Cost (only soft constriants)
  // s_lower,k -> slack variable of lower polytopic constraint (3) + lower
  // bounds s_upper,k -> slack variable of upper polytopic constraint (4) +
  // upper bounds min_x,u sum_k=0^N_K
  // 1/2*[s_lower,k;s_upper,k]^T*[Z_lower,k ,
  // 0; 0 , Z_upper,k]*[s_lower,k;s_upper,k] + [z_lower,k;
  // z_upper,k]^T*[s_lower,k;s_upper,k]
  std::vector<double *> hZl_;  // hZl[k] = Z_lower,k
  std::vector<double *> hZu_;  // hZu[k] = Z_upper,k
  std::vector<double *> hzl_;  // hzl[k] = z_lower,k
  std::vector<double *> hzu_;  // hzu[k] = z_upper,k

  // Bounds on the soft constraint multipliers
  // s_lower,k >= s_lower,bound,k
  // s_upper,k >= s_upper,bound,k
  std::vector<double *> hlls_;
  std::vector<double *> hlus_;
  // index of the bounds and constraints that are softened
  // order is not really clear
  //   int **hidxs_;
  std::vector<int *> hidxs_;

  // bounds that are different to stages bounds and need to be stored somewhere
  // such the a pointer can point
  std::vector<ThirdOrderSplineSpeed::HpipmBound> hpipm_bounds_;
  Eigen::MatrixXd b0_;
```


## 1.3 调用参数解释：

```c++
/*


npc  多项式约束 
ns   软约束 
*/
ThirdOrderSplineSpeedMPC(const double delta_t, const int n, const int n_x, const int n_u,
    const int n_p_c, const int n_s,
    const ThirdOrderSplineSpeed::State& init_state,
    const ThirdOrderSplineSpeed::Control& init_control,
    const double speed_limit, const std::vector<double>& upper_s_bounds,
    const std::vector<double>& lower_s_bounds,
    const std::vector<double>& upper_v_bounds,
    const std::vector<double>& lower_v_bounds,
    const std::vector<double>& upper_a_bounds,
    const std::vector<double>& lower_a_bounds,
    const std::vector<double>& upper_jerk_bounds,
    const std::vector<double>& lower_jerk_bounds,
    const std::vector<STGoalSInfo>& goal_upper_s,
    const std::vector<STGoalVInfo>& goal_upper_v,
    const std::vector<STGoalSInfo>& goal_lower_s,
    const std::vector<STGoalVInfo>& goal_lower_v,
    const std::vector<STGoalAInfo>& goal_a){}


bool ThirdOrderSplineSpeedMPC::SetStage(
    const ThirdOrderSplineSpeed::State& x_k,
    const ThirdOrderSplineSpeed::Control& u_k, const int step,
    const double weight_s, const double weight_v, const double weight_a) {
  // LOG_INFO("Check S weight: {}", weight_s);
  stages_[step].nx = nx_; // 阶段的状态数 
  stages_[step].nu = nu_; // 阶段的控制量
  if (step == 0) {
    stages_[step].ng = 0;
    stages_[step].ns = 0;  // 0阶段没约束 
  } else { // 其它阶段开始有约束，正常v给硬约束，  
    stages_[step].ng = npc_; // polygon 约束 
    stages_[step].ns = (init_state_.state_v + 0.01 < speed_limit_) ? 0 : ns_; // 如果当前速度小于限速，不设置限速约束，不设置速度的软约束
  }

  // 1. model
  stages_[step].line_model = model_->line_model_matrix(); // 同一套模型

  // 2. cost
  auto config = config::PlanningConfig::Instance()
                    ->planning_research_config()
                    .third_order_spline_speed_optimizer_config;
  // s
  double s_upper_weight{0.}, s_lower_weight{0.};
  stages_[step].cost.Q(0, 0) =  weight_s +  weight_s;  // 二次项约束 
  stages_[step].cost.q(0, 0) = -weight_s * goal_upper_s_[step].goal_t_s.s() +
                               -weight_s * goal_lower_s_[step].goal_t_s.s();  // 一次项约束 
  LOG_DEBUG("step: goal_upper/lower_s {:.3f}, {:.3f}",
            goal_upper_s_[step].goal_t_s.s(), goal_lower_s_[step].goal_t_s.s());
  // v
  double v_upper_weight{0.}, v_lower_weight{0.};

  stages_[step].cost.Q(1, 1) = weight_v + weight_v;
  stages_[step].cost.q(1, 0) = -weight_v * goal_upper_v_[step].goal_t_v.s() +
                               -weight_v * goal_lower_v_[step].goal_t_v.s();
  LOG_DEBUG("step: goal_upper/lower_v {:.3f}, {:.3f}",
            goal_upper_v_[step].goal_t_v.s(), goal_lower_v_[step].goal_t_v.s());
  // a
  stages_[step].cost.Q(2, 2) = weight_a;
  stages_[step].cost.q(2, 0) = -0.5 * weight_a * goal_a_[step].goal_t_a.s();
  // jerk
  stages_[step].cost.R(0, 0) = config.weight_jerk;

  stages_[step].cost.r = ThirdOrderSplineSpeed::r::Zero();
  stages_[step].cost.S = ThirdOrderSplineSpeed::S::Zero();
  stages_[step].cost.Z << config.weight_slack *
                              ThirdOrderSplineSpeed::Z::Identity();
  stages_[step].cost.z << ThirdOrderSplineSpeed::z::Zero();

  // 3. polytopic constraints: v as the soft constraint，取约束的方式，
  stages_[step].constraints.C = model_->polytopic_constraints().C;  // 取v，其它置0
  stages_[step].constraints.D = model_->polytopic_constraints().D;
  double upper_v_bounds{0.0};
  stages_[step].constraints.du << std::fmax(upper_v_bounds_[step], 0); // 软约束？
  stages_[step].constraints.dl << lower_v_bounds_[step] - 0.01; // 

  // 4. box constraints:
  stages_[step].upper_bounds_x << upper_s_bounds_[step],
      2 * ThirdOrderSplineSpeed::INF, upper_a_bounds_[step];
  stages_[step].lower_bounds_x << lower_s_bounds_[step], lower_v_bounds_[step],
      lower_a_bounds_[step];

  stages_[step].upper_bounds_u << upper_jerk_bounds_[step];
  stages_[step].lower_bounds_u << lower_jerk_bounds_[step];

  stages_[step].upper_bounds_s << 0.1;
  stages_[step].lower_bounds_s << 0.1;

  return true;
}


// 设置动力学模型

void ThirdOrderSplineSpeedHpipmSolver::SetDynamics(
    const ThirdOrderSplineSpeed::State &x0,
    std::vector<ThirdOrderSplineSpeed::Stage> &stages) {
  b0_ = (stages[0].line_model.A * ThirdOrderSplineSpeed::StateToVector(x0) +
         stages[0].line_model.g);
  for (int i = 0; i < N_; i++) {
    if (i == 0) {
      hA_[i] = nullptr; // 第一项中需要置空？但是会导致使用模型求下个状态时，缺少 Ax 的值，所以，将这个值加到 b中，这样就不需要设置A了
      hB_[i] = stages[i].line_model.B.data();
      hb_[i] = b0_.data();

      nx_[i] = 0;
      nu_[i] = N_U_;
    } else {
      hA_[i] = stages[i].line_model.A.data();
      hB_[i] = stages[i].line_model.B.data();
      hb_[i] = stages[i].line_model.g.data();

      nx_[i] = N_X_;
      nu_[i] = N_U_;
    }
  }
  nx_[N_] = N_X_;
  nu_[N_] = 0;
}

void ThirdOrderSplineSpeedHpipmSolver::SetCost(
    std::vector<ThirdOrderSplineSpeed::Stage> &stages) {
  for (int i = 0; i <= N_; i++) {
    hQ_[i] = stages[i].cost.Q.data();
    hR_[i] = stages[i].cost.R.data();
    hS_[i] = stages[i].cost.S.data();

    hq_[i] = stages[i].cost.q.data();
    hr_[i] = stages[i].cost.r.data();

    if (stages[i].ns != 0) {
      hZl_[i] = stages[i].cost.Z.data();
      hZu_[i] = stages[i].cost.Z.data();
      hzl_[i] = stages[i].cost.z.data();
      hzu_[i] = stages[i].cost.z.data();
    } else {
      hZl_[i] = nullptr;
      hZu_[i] = nullptr;
      hzl_[i] = nullptr;
      hzu_[i] = nullptr;
    }
  }
}

void ThirdOrderSplineSpeedHpipmSolver::SetBounds(
    std::vector<ThirdOrderSplineSpeed::Stage> &stages) {
  nbu_[0] = 0;
  hpipm_bounds_[0].idx_u.resize(0);
  hpipm_bounds_[0].lower_bounds_u.resize(0);
  hpipm_bounds_[0].upper_bounds_u.resize(0);
  for (int j = 0; j < N_U_; j++) {
    if (stages[0].lower_bounds_u(j) > -ThirdOrderSplineSpeed::INF &&
        stages[0].upper_bounds_u(j) < ThirdOrderSplineSpeed::INF) {
      nbu_[0]++;
      hpipm_bounds_[0].idx_u.push_back(j);
      hpipm_bounds_[0].lower_bounds_u.push_back(stages[0].lower_bounds_u(j));
      hpipm_bounds_[0].upper_bounds_u.push_back(stages[0].upper_bounds_u(j));
    }
  }
  nbx_[0] = 0;
  hidxbx_[0] = nullptr;
  hidxbu_[0] = hpipm_bounds_[0].idx_u.data();

  hlbx_[0] = nullptr;
  hubx_[0] = nullptr;
  hlbu_[0] = hpipm_bounds_[0].lower_bounds_u.data();
  hubu_[0] = hpipm_bounds_[0].upper_bounds_u.data();

  for (int i = 1; i <= N_; i++) {
    hpipm_bounds_[i].idx_u.resize(0);
    hpipm_bounds_[i].lower_bounds_u.resize(0);
    hpipm_bounds_[i].upper_bounds_u.resize(0);
    nbu_[i] = 0;
    for (int j = 0; j < N_U_; j++) {
      if (stages[i].lower_bounds_u(j) > -ThirdOrderSplineSpeed::INF &&
          stages[i].upper_bounds_u(j) < ThirdOrderSplineSpeed::INF) {
        nbu_[i]++;
        hpipm_bounds_[i].idx_u.push_back(j);
        hpipm_bounds_[i].lower_bounds_u.push_back(stages[i].lower_bounds_u(j));
        hpipm_bounds_[i].upper_bounds_u.push_back(stages[i].upper_bounds_u(j));
      }
    }

    hpipm_bounds_[i].idx_x.resize(0);
    hpipm_bounds_[i].lower_bounds_x.resize(0);
    hpipm_bounds_[i].upper_bounds_x.resize(0);
    nbx_[i] = 0;
    for (int j = 0; j < N_X_; j++) {
      if (stages[i].lower_bounds_x(j) > -ThirdOrderSplineSpeed::INF &&
          stages[i].upper_bounds_x(j) < ThirdOrderSplineSpeed::INF) {
        nbx_[i]++;
        hpipm_bounds_[i].idx_x.push_back(j);
        hpipm_bounds_[i].lower_bounds_x.push_back(stages[i].lower_bounds_x(j));
        hpipm_bounds_[i].upper_bounds_x.push_back(stages[i].upper_bounds_x(j));
      }
    }

    hidxbx_[i] = hpipm_bounds_[i].idx_x.data();
    hidxbu_[i] = hpipm_bounds_[i].idx_u.data();
    hlbx_[i] = hpipm_bounds_[i].lower_bounds_x.data();
    hubx_[i] = hpipm_bounds_[i].upper_bounds_x.data();
    hlbu_[i] = hpipm_bounds_[i].lower_bounds_u.data();
    hubu_[i] = hpipm_bounds_[i].upper_bounds_u.data();
  }

  nbu_[N_] = 0;
  hidxbu_[N_] = nullptr;
  hlbu_[N_] = nullptr;
  hubu_[N_] = nullptr;
}

void ThirdOrderSplineSpeedHpipmSolver::SetSoftConstraints(
    std::vector<ThirdOrderSplineSpeed::Stage> &stages) {
  // Note: soft constraints support for polytopic constraints
  for (int i = 0; i <= N_; i++) {
    hpipm_bounds_[i].idx_s.resize(0);
    if (stages[i].ns != 0) {
      nsbx_[i] = 0;
      nsbu_[i] = 0;
      nsg_[i] = stages[i].ns;

      for (int j = 0; j < stages[i].ns; j++) {
        hpipm_bounds_[i].idx_s.push_back(j + nbx_[i] + nbu_[i]);
      }

      hidxs_[i] = hpipm_bounds_[i].idx_s.data();
      hlls_[i] = stages[i].lower_bounds_s.data();
      hlus_[i] = stages[i].upper_bounds_s.data();
    } else {
      nsbx_[i] = 0;
      nsbu_[i] = 0;
      nsg_[i] = 0;
      hidxs_[i] = nullptr;
      hlls_[i] = nullptr;
      hlus_[i] = nullptr;
    }
  }
}
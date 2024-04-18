#include "distance_approach_ipopt_fixed_ts_interface.h"

#include "src/planning/common/vehicle_param.h"
namespace neodrive {
namespace planning {

DistanceApproachIPOPTFixedTsInterface::DistanceApproachIPOPTFixedTsInterface(
    const size_t horizon, const double ts, const Eigen::MatrixXd& ego,
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
    const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xf,
    const Eigen::MatrixXd& last_time_u, const std::vector<double>& XYbounds,
    const Eigen::MatrixXi& obstacles_edges_num, const size_t obstacles_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const DistanceApproachConfig& distance_approch_config,
    const ParkingCommonConfig common_config)
    : ts_(ts),
      ego_(ego),
      xWS_(xWS),
      uWS_(uWS),
      l_warm_up_(l_warm_up),
      n_warm_up_(n_warm_up),
      x0_(x0),
      xf_(xf),
      last_time_u_(last_time_u),
      XYbounds_(XYbounds),
      obstacles_edges_num_(obstacles_edges_num),
      obstacles_A_(obstacles_A),
      obstacles_b_(obstacles_b),
      distance_approach_config_(distance_approch_config),
      common_config_(common_config) {
  if (horizon >= std::numeric_limits<int>::max()) {
    LOG_ERROR("Invalid cast on horizon ");
    return;
  }
  horizon_ = static_cast<int>(horizon);
  if (obstacles_num >= std::numeric_limits<int>::max()) {
    LOG_ERROR("Invalid cast on obstacles_num ");
    return;
  }

  obstacles_num_ = static_cast<int>(obstacles_num);
  w_ev_ = ego_(1, 0) + ego_(3, 0);
  l_ev_ = ego_(0, 0) + ego_(2, 0);
  g_ = {l_ev_ / 2, w_ev_ / 2, l_ev_ / 2, w_ev_ / 2};
  offset_ = (ego_(0, 0) + ego_(2, 0)) / 2 - ego_(2, 0);
  obstacles_edges_sum_ = obstacles_edges_num_.sum();
  state_result_ = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  dual_l_result_ = Eigen::MatrixXd::Zero(obstacles_edges_sum_, horizon_ + 1);
  dual_n_result_ = Eigen::MatrixXd::Zero(4 * obstacles_num_, horizon_ + 1);
  control_result_ = Eigen::MatrixXd::Zero(2, horizon_ + 1);
  time_result_ = Eigen::MatrixXd::Zero(1, horizon_ + 1);
  state_start_index_ = 0;
  control_start_index_ = 4 * (horizon_ + 1);
  time_start_index_ = control_start_index_ + 2 * horizon_;
  l_start_index_ = time_start_index_;
  n_start_index_ = l_start_index_ + obstacles_edges_sum_ * (horizon_ + 1);

  weight_state_x_ = distance_approach_config_.weight_x_;
  weight_state_y_ = distance_approach_config_.weight_y_;
  weight_state_phi_ = distance_approach_config_.weight_phi_;
  weight_state_v_ = distance_approach_config_.weight_v_;
  weight_input_steer_ = distance_approach_config_.weight_steer_;
  weight_input_a_ = distance_approach_config_.weight_a_;
  weight_rate_steer_ = distance_approach_config_.weight_steer_rate_;
  weight_rate_a_ = distance_approach_config_.weight_a_rate_;
  weight_stitching_steer_ = distance_approach_config_.weight_steer_stitching_;
  weight_stitching_a_ = distance_approach_config_.weight_a_stitching_;
  weight_first_order_time_ = distance_approach_config_.weight_first_order_time_;
  weight_second_order_time_ =
      distance_approach_config_.weight_second_order_time_;
  min_safety_distance_ = distance_approach_config_.min_safety_distance_;
  max_steer_angle_ = VehicleParam::Instance()->max_steer_angle() /
                     VehicleParam::Instance()->steer_ratio();
  max_speed_forward_ = common_config_.max_forward_v_;
  max_speed_reverse_ = common_config_.max_reverse_v_;
  max_acceleration_forward_ = common_config_.max_forward_acc_;
  max_acceleration_reverse_ = common_config_.max_reverse_acc_;
  min_time_sample_scaling_ = distance_approach_config_.min_time_sample_scaling_;
  max_time_sample_scaling_ = distance_approach_config_.max_time_sample_scaling_;

  max_steer_rate_ = VehicleParam::Instance()->max_steer_angle_rate() /
                    VehicleParam::Instance()->steer_ratio();
  use_fix_time_ = distance_approach_config_.use_fix_time_;
  wheelbase_ = VehicleParam::Instance()->wheel_base();
  enable_constraint_check_ = distance_approach_config_.enable_constraint_check_;
}

bool DistanceApproachIPOPTFixedTsInterface::get_nlp_info(
    int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,
    IndexStyleEnum& index_style) {
  LOG_INFO("get_nlp_info");
  // n1 : states variables, 4 * (N+1)
  int n1 = 4 * (horizon_ + 1);
  LOG_INFO("n1: {},", n1);
  // n2 : control inputs variables
  int n2 = 2 * horizon_;
  LOG_INFO("n2: {}", n2);
  // n4 : dual multiplier associated with obstacle shape
  lambda_horizon_ = obstacles_edges_num_.sum() * (horizon_ + 1);
  LOG_INFO("lambda_horizon_: {}, ", lambda_horizon_);
  // n5 : dual multipier associated with car shape, obstacles_num*4 * (N+1)
  miu_horizon_ = obstacles_num_ * 4 * (horizon_ + 1);
  LOG_INFO("miu_horizon_: {}", miu_horizon_);

  // m1 : dynamics constatins
  int m1 = 4 * horizon_;
  // m2 : control rate constraints (only steering)
  int m2 = horizon_;
  // m4 : obstacle constraints
  int m4 = 4 * obstacles_num_ * (horizon_ + 1);

  num_of_variables_ = n1 + n2 + lambda_horizon_ + miu_horizon_;
  num_of_constraints_ = m1 + m2 + m4 + (num_of_variables_ - (horizon_ + 1) + 2);

  // number of variables
  n = num_of_variables_;
  LOG_INFO("num_of_variables_ {},", num_of_variables_);
  // number of constraints
  m = num_of_constraints_;
  LOG_INFO("num_of_constraints_ {} ", num_of_constraints_);

  GenerateTapes(n, m, &nnz_jac_g, &nnz_h_lag);

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool DistanceApproachIPOPTFixedTsInterface::get_bounds_info(int n, double* x_l,
                                                            double* x_u, int m,
                                                            double* g_l,
                                                            double* g_u) {
  LOG_INFO("get_bounds_info");
  if (XYbounds_.size() != 4) {
    LOG_ERROR("XYbounds_ size is not 4, but {}", XYbounds_.size());
    return false;
  }

  // Variables: includes state, u, sample time and lagrange multipliers
  // 1. state variables, 4 * [0, horizon]
  // start point pose
  int variable_index = 0;
  for (int i = 0; i < 4; ++i) {
    x_l[i] = -2e19;
    x_u[i] = 2e19;
  }
  variable_index += 4;

  // During horizons, 2 ~ N-1
  for (int i = 1; i < horizon_; ++i) {
    // x
    x_l[variable_index] = -2e19;
    x_u[variable_index] = 2e19;

    // y
    x_l[variable_index + 1] = -2e19;
    x_u[variable_index + 1] = 2e19;

    // phi
    x_l[variable_index + 2] = -2e19;
    x_u[variable_index + 2] = 2e19;

    // v
    x_l[variable_index + 3] = -2e19;
    x_u[variable_index + 3] = 2e19;

    variable_index += 4;
  }

  // end point pose
  for (int i = 0; i < 4; ++i) {
    x_l[variable_index + i] = -2e19;
    x_u[variable_index + i] = 2e19;
  }
  variable_index += 4;
  LOG_INFO("variable_index after adding state variables : {}", variable_index);

  // 2. control variables, 2 * [0, horizon_-1]
  for (int i = 0; i < horizon_; ++i) {
    // u1
    x_l[variable_index] = -2e19;
    x_u[variable_index] = 2e19;

    // u2
    x_l[variable_index + 1] = -2e19;
    x_u[variable_index + 1] = 2e19;

    variable_index += 2;
  }
  LOG_INFO("variable_index after adding control variables : {}",
           variable_index);

  // 4. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 2e19;  // nlp_upper_bound_limit
      ++variable_index;
    }
  }
  LOG_INFO("variable_index after adding sample time : {}", variable_index);

  // 5. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 2e19;  // nlp_upper_bound_limit

      ++variable_index;
    }
  }
  LOG_INFO("variable_index after adding lagrange l : {}", variable_index);

  // Constraints: includes four state Euler forward constraints, three
  // Obstacle related constraints

  // 1. dynamics constraints 4 * [0, horizons-1]
  int constraint_index = 0;
  for (int i = 0; i < 4 * horizon_; ++i) {
    g_l[i] = 0.0;
    g_u[i] = 0.0;
  }
  constraint_index += 4 * horizon_;

  LOG_INFO(
      "constraint_index after adding Euler forward dynamics constraints: {}",
      constraint_index);

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  for (int i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = -max_steer_rate_;
    g_u[constraint_index] = max_steer_rate_;
    ++constraint_index;
  }

  LOG_INFO("constraint_index after adding steering rate constraints: {}",
           constraint_index);

  // 4. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      // a. norm(A'*lambda) <= 1
      g_l[constraint_index] = -2e19;
      g_u[constraint_index] = 1.0;

      // b. G'*mu + R'*A*lambda = 0
      g_l[constraint_index + 1] = 0.0;
      g_u[constraint_index + 1] = 0.0;
      g_l[constraint_index + 2] = 0.0;
      g_u[constraint_index + 2] = 0.0;

      // c. -g'*mu + (A*t - b)*lambda > min_safety_distance_
      g_l[constraint_index + 3] = min_safety_distance_;
      g_u[constraint_index + 3] = 2e19;  // nlp_upper_bound_limit
      constraint_index += 4;
    }
  }
  LOG_INFO("constraints_index after adding obstacles related constraints: {}",
           constraint_index);

  // 5. load variable bounds as constraints
  // start configuration
  g_l[constraint_index] = x0_(0, 0);
  g_u[constraint_index] = x0_(0, 0);
  g_l[constraint_index + 1] = x0_(1, 0);
  g_u[constraint_index + 1] = x0_(1, 0);
  g_l[constraint_index + 2] = x0_(2, 0);
  g_u[constraint_index + 2] = x0_(2, 0);
  g_l[constraint_index + 3] = x0_(3, 0);
  g_u[constraint_index + 3] = x0_(3, 0);
  constraint_index += 4;

  for (int i = 1; i < horizon_; ++i) {
    g_l[constraint_index] = XYbounds_[0];
    g_u[constraint_index] = XYbounds_[1];
    g_l[constraint_index + 1] = XYbounds_[2];
    g_u[constraint_index + 1] = XYbounds_[3];
    g_l[constraint_index + 2] = -max_speed_reverse_;
    g_u[constraint_index + 2] = max_speed_forward_;
    constraint_index += 3;
  }

  // end configuration
  g_l[constraint_index] = xf_(0, 0);
  g_u[constraint_index] = xf_(0, 0);
  g_l[constraint_index + 1] = xf_(1, 0);
  g_u[constraint_index + 1] = xf_(1, 0);
  g_l[constraint_index + 2] = xf_(2, 0);
  g_u[constraint_index + 2] = xf_(2, 0);
  g_l[constraint_index + 3] = xf_(3, 0);
  g_u[constraint_index + 3] = xf_(3, 0);
  constraint_index += 4;

  for (int i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = -max_steer_angle_;
    g_u[constraint_index] = max_steer_angle_;
    g_l[constraint_index + 1] = -max_acceleration_reverse_;
    g_u[constraint_index + 1] = max_acceleration_forward_;
    constraint_index += 2;
  }

  for (int i = 0; i < lambda_horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 2e19;
    constraint_index++;
  }

  for (int i = 0; i < miu_horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 2e19;
    constraint_index++;
  }

  LOG_INFO("constraint_index after adding obstacles constraints: {}",
           constraint_index);
  LOG_INFO("get_bounds_info_ out");
  return true;
}

bool DistanceApproachIPOPTFixedTsInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  LOG_INFO("get_starting_point");
  if (init_x == false) {
    LOG_ERROR("Warm start init_x setting failed");
    return false;
  }
  if (horizon_ != uWS_.cols()) {
    LOG_ERROR("horizon_ [{}] != uWS_.cols() [{}]", horizon_, uWS_.cols());
    return false;
  }
  if (horizon_ + 1 != xWS_.cols()) {
    LOG_ERROR("horizon_ + 1 [{}] != xWS_.cols() [{}]", horizon_ + 1,
              xWS_.cols());
    return false;
  }

  // 1. state variables 4 * (horizon_ + 1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    int index = i * 4;
    for (int j = 0; j < 4; ++j) {
      x[index + j] = xWS_(j, i);
    }
  }

  // 2. control variable initialization, 2 * horizon_
  for (int i = 0; i < horizon_; ++i) {
    int index = i * 2;
    x[control_start_index_ + index] = uWS_(0, i);
    x[control_start_index_ + index + 1] = uWS_(1, i);
  }

  // 3. lagrange constraint l, obstacles_edges_sum_ * (horizon_+1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    int index = i * obstacles_edges_sum_;
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x[l_start_index_ + index + j] = l_warm_up_(j, i);
    }
  }

  // 4. lagrange constraint m, 4*obstacles_num * (horizon_+1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    int index = i * 4 * obstacles_num_;
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x[n_start_index_ + index + j] = n_warm_up_(j, i);
    }
  }

  if (enable_constraint_check_) {
    int kM = m;
    double g[kM];
    LOG_INFO("initial points constraint checking");
    EvalConstraints(n, x, m, g);
    CheckG(n, x, m, g);
  }

  LOG_INFO("get_starting_point out");
  return true;
}

bool DistanceApproachIPOPTFixedTsInterface::eval_f(int n, const double* x,
                                                   bool new_x,
                                                   double& obj_value) {
  EvalObj(n, x, &obj_value);
  return true;
}

bool DistanceApproachIPOPTFixedTsInterface::eval_grad_f(int n, const double* x,
                                                        bool new_x,
                                                        double* grad_f) {
  gradient(tag_f, n, x, grad_f);
  return true;
}

bool DistanceApproachIPOPTFixedTsInterface::eval_g(int n, const double* x,
                                                   bool new_x, int m,
                                                   double* g) {
  EvalConstraints(n, x, m, g);
  if (enable_constraint_check_) {
    CheckG(n, x, m, g);
  }
  return true;
}

bool DistanceApproachIPOPTFixedTsInterface::eval_jac_g(int n, const double* x,
                                                       bool new_x, int m,
                                                       int nele_jac, int* iRow,
                                                       int* jCol,
                                                       double* values) {
  if (values == nullptr) {
    // return the structure of the jacobian
    for (int idx = 0; idx < nnz_jac; idx++) {
      iRow[idx] = rind_g[idx];
      jCol[idx] = cind_g[idx];
    }
  } else {
    // return the values of the jacobian of the constraints
    sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval,
               options_g);
    for (int idx = 0; idx < nnz_jac; idx++) {
      values[idx] = jacval[idx];
    }
  }
  return true;
  // return EvalJacGSer(n, x, new_x, m, nele_jac, iRow, jCol, values);
}

bool DistanceApproachIPOPTFixedTsInterface::eval_jac_g_ser(
    int n, const double* x, bool new_x, int m, int nele_jac, int* iRow,
    int* jCol, double* values) {
  LOG_ERROR("NOT VALID NOW");
  return false;
}  // NOLINT

bool DistanceApproachIPOPTFixedTsInterface::EvalH(int n, const double* x,
                                                  bool new_x, double obj_factor,
                                                  int m, const double* lambda,
                                                  bool new_lambda,
                                                  int nele_hess, int* iRow,
                                                  int* jCol, double* values) {
  if (values == nullptr) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    for (int idx = 0; idx < nnz_L; idx++) {
      iRow[idx] = rind_L[idx];
      jCol[idx] = cind_L[idx];
    }
  } else {
    // return the values. This is a symmetric matrix, fill the lower left
    // triangle only

    obj_lam[0] = obj_factor;

    for (int idx = 0; idx < m; idx++) {
      obj_lam[1 + idx] = lambda[idx];
    }

    set_param_vec(tag_L, m + 1, obj_lam);
    sparse_hess(tag_L, n, 1, const_cast<double*>(x), &nnz_L, &rind_L, &cind_L,
                &hessval, options_L);

    for (int idx = 0; idx < nnz_L; idx++) {
      values[idx] = hessval[idx];
    }
  }

  return true;
}

void DistanceApproachIPOPTFixedTsInterface::FinalizeSolution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  int state_index = state_start_index_;
  int control_index = control_start_index_;
  // int time_index = time_start_index_;
  int dual_l_index = l_start_index_;
  int dual_n_index = n_start_index_;

  // enable_constraint_check_: for debug only
  if (enable_constraint_check_) {
    LOG_INFO("final resolution constraint checking");
    CheckG(n, x, m, g);
  }
  // 1. state variables, 4 * [0, horizon]
  // 2. control variables, 2 * [0, horizon_-1]
  // 3. sampling time variables, 1 * [0, horizon_]
  // 4. dual_l obstacles_edges_sum_ * [0, horizon]
  // 5. dual_n obstacles_num * [0, horizon]
  for (int i = 0; i < horizon_; ++i) {
    state_result_(0, i) = x[state_index];
    state_result_(1, i) = x[state_index + 1];
    state_result_(2, i) = x[state_index + 2];
    state_result_(3, i) = x[state_index + 3];
    control_result_(0, i) = x[control_index];
    control_result_(1, i) = x[control_index + 1];
    time_result_(0, i) = ts_;
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      dual_l_result_(j, i) = x[dual_l_index + j];
    }
    for (int k = 0; k < 4 * obstacles_num_; ++k) {
      dual_n_result_(k, i) = x[dual_n_index + k];
    }
    state_index += 4;
    control_index += 2;
    // time_index++;
    dual_l_index += obstacles_edges_sum_;
    dual_n_index += 4 * obstacles_num_;
  }
  state_result_(0, 0) = x0_(0, 0);
  state_result_(1, 0) = x0_(1, 0);
  state_result_(2, 0) = x0_(2, 0);
  state_result_(3, 0) = x0_(3, 0);
  // push back last horizon for state and time variables
  state_result_(0, horizon_) = xf_(0, 0);
  state_result_(1, horizon_) = xf_(1, 0);
  state_result_(2, horizon_) = xf_(2, 0);
  state_result_(3, horizon_) = xf_(3, 0);
  time_result_(0, horizon_) = ts_;
  // time_result_ = ts_ * time_result_;
  for (int j = 0; j < obstacles_edges_sum_; ++j) {
    dual_l_result_(j, horizon_) = x[dual_l_index + j];
  }
  for (int k = 0; k < 4 * obstacles_num_; ++k) {
    dual_n_result_(k, horizon_) = x[dual_n_index + k];
  }
  // memory deallocation of ADOL-C variables
  delete[] obj_lam;
  free(rind_g);
  free(cind_g);
  free(rind_L);
  free(cind_L);
  free(jacval);
  free(hessval);
  return;
}

void DistanceApproachIPOPTFixedTsInterface::GetOptimizationResults(
    Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
    Eigen::MatrixXd* time_result, Eigen::MatrixXd* dual_l_result,
    Eigen::MatrixXd* dual_n_result) const {
  LOG_INFO("GetOptimizationResults");
  *state_result = state_result_;
  *control_result = control_result_;
  *time_result = time_result_;
  *dual_l_result = dual_l_result_;
  *dual_n_result = dual_n_result_;

  if (!distance_approach_config_.enable_initial_final_check_) {
    return;
  }
  if (state_result_.cols() != xWS_.cols()) {
    LOG_ERROR("state_result_.cols() [{}]!= xWS_.cols() [{}]",
              state_result_.cols(), xWS_.cols());
    return;
  }
  if (state_result_.rows() != xWS_.rows()) {
    LOG_ERROR("state_result_.rows() [{}]!= xWS_.rows() [{}]",
              state_result_.rows(), xWS_.rows());
    return;
  }
  double state_diff_max = 0.0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4; ++j) {
      state_diff_max =
          std::max(std::abs(xWS_(j, i) - state_result_(j, i)), state_diff_max);
    }
  }

  // 2. control variable initialization, 2 * horizon_
  if (control_result_.cols() != uWS_.cols()) {
    LOG_ERROR("control_result_.cols() [{}]!= uWS_.cols() [{}]",
              control_result_.cols(), uWS_.cols());
    return;
  }
  if (control_result_.rows() != uWS_.rows()) {
    LOG_ERROR("control_result_.rows() [{}]!= uWS_.rows() [{}]",
              control_result_.rows(), uWS_.rows());
    return;
  }
  double control_diff_max = 0.0;
  for (int i = 0; i < horizon_; ++i) {
    control_diff_max = std::max(std::abs(uWS_(0, i) - control_result_(0, i)),
                                control_diff_max);
    control_diff_max = std::max(std::abs(uWS_(1, i) - control_result_(1, i)),
                                control_diff_max);
  }

  // 3. lagrange constraint l, obstacles_edges_sum_ * (horizon_+1)
  if (dual_l_result_.cols() != l_warm_up_.cols()) {
    LOG_ERROR("dual_l_result_.cols() [{}]!= l_warm_up_.cols() [{}]",
              dual_l_result_.cols(), l_warm_up_.cols());
    return;
  }
  if (dual_l_result_.rows() != l_warm_up_.rows()) {
    LOG_ERROR("dual_l_result_.rows() [{}]!= l_warm_up_.rows() [{}]",
              dual_l_result_.rows(), l_warm_up_.rows());
    return;
  }
  double l_diff_max = 0.0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      l_diff_max = std::max(std::abs(l_warm_up_(j, i) - dual_l_result_(j, i)),
                            l_diff_max);
    }
  }

  // 4. lagrange constraint m, 4*obstacles_num * (horizon_+1)
  if (n_warm_up_.cols() != dual_n_result_.cols()) {
    LOG_ERROR("n_warm_up_.cols() [{}]!= dual_n_result_.cols() [{}]",
              n_warm_up_.cols(), dual_n_result_.cols());
    return;
  }
  if (n_warm_up_.rows() != dual_n_result_.rows()) {
    LOG_ERROR("n_warm_up_.rows() [{}]!= dual_n_result_.rows() [{}]",
              n_warm_up_.rows(), dual_n_result_.rows());
    return;
  }
  double n_diff_max = 0.0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      n_diff_max = std::max(std::abs(n_warm_up_(j, i) - dual_n_result_(j, i)),
                            n_diff_max);
    }
  }

  LOG_INFO("state_diff_max: {:.4f}", state_diff_max);
  LOG_INFO("control_diff_max: {:.4f}", control_diff_max);
  LOG_INFO("dual_l_diff_max: {:.4f}", l_diff_max);
  LOG_INFO("dual_n_diff_max: {:.4f}", n_diff_max);
  return;
}

//***************    start ADOL-C part ***********************************
template <class T>
void DistanceApproachIPOPTFixedTsInterface::EvalObj(int n, const T* x,
                                                    T* obj_value) {
  // Objective is :
  // min control inputs
  // min input rate
  // min time (if the time step is not fixed)
  // regularization wrt warm start trajectory
  if (ts_ < 0.001) {
    LOG_ERROR("ts in distance_approach_ is 0");
    return;
  }
  int control_index = control_start_index_;
  // int time_index = time_start_index_;
  int state_index = state_start_index_;

  // TODO(QiL): Initial implementation towards earlier understanding and debug
  // purpose, later code refine towards improving efficiency

  *obj_value = 0.0;
  // 1. objective to minimize state diff to warm up
  for (int i = 0; i < horizon_ + 1; ++i) {
    T x1_diff = x[state_index] - xWS_(0, i);
    T x2_diff = x[state_index + 1] - xWS_(1, i);
    T x3_diff = x[state_index + 2] - xWS_(2, i);
    T x4_abs = x[state_index + 3];
    *obj_value += weight_state_x_ * x1_diff * x1_diff +
                  weight_state_y_ * x2_diff * x2_diff +
                  weight_state_phi_ * x3_diff * x3_diff +
                  weight_state_v_ * x4_abs * x4_abs;
    state_index += 4;
  }

  // 2. objective to minimize u square
  for (int i = 0; i < horizon_; ++i) {
    *obj_value += weight_input_steer_ * x[control_index] * x[control_index] +
                  weight_input_a_ * x[control_index + 1] * x[control_index + 1];
    control_index += 2;
  }

  // 3. objective to minimize input change rate for first horizon
  control_index = control_start_index_;
  T last_time_steer_rate = (x[control_index] - last_time_u_(0, 0)) / ts_;
  T last_time_a_rate = (x[control_index + 1] - last_time_u_(1, 0)) / ts_;

  *obj_value +=
      weight_stitching_steer_ * last_time_steer_rate * last_time_steer_rate +
      weight_stitching_a_ * last_time_a_rate * last_time_a_rate;

  // 4. objective to minimize input change rates, [0- horizon_ -2]
  for (int i = 0; i < horizon_ - 1; ++i) {
    T steering_rate = (x[control_index + 2] - x[control_index]) / ts_;
    T a_rate = (x[control_index + 3] - x[control_index + 1]) / ts_;
    *obj_value += weight_rate_steer_ * steering_rate * steering_rate +
                  weight_rate_a_ * a_rate * a_rate;
    control_index += 2;
  }
  return;
}

template <class T>
void DistanceApproachIPOPTFixedTsInterface::EvalConstraints(int n, const T* x,
                                                            int m, T* g) {
  // state start index
  int state_index = state_start_index_;

  // control start index.
  int control_index = control_start_index_;

  // time start index
  // int time_index = time_start_index_;

  int constraint_index = 0;

  // 1. state constraints 4 * [0, horizons-1]
  //    commented implementation is linearized model with fixed ts
  for (int i = 0; i < horizon_; ++i) {
    // x1
    /*
    g[constraint_index] =
        x[state_index + 4] -
        ((xWS_(0, i) + ts_ * xWS_(3, i) * cos(xWS_(2, i))) +
         (x[state_index] - xWS_(0, i)) +
         (ts_ * cos(xWS_(2, i))) * (x[state_index + 3] - xWS_(3, i)) +
         (-ts_ * xWS_(3, i) * sin(xWS_(2, i))) *
             (x[state_index + 2] - xWS_(2, i)));
    */

    g[constraint_index] =
        x[state_index + 4] -
        (x[state_index] + ts_ * x[state_index + 3] * cos(x[state_index + 2]));

    // x2
    /*
    g[constraint_index + 1] =
        x[state_index + 5] -
        ((xWS_(1, i) + ts_ * xWS_(3, i) * sin(xWS_(2, i))) +
         (x[state_index + 1] - xWS_(1, i)) +
         (ts_ * sin(xWS_(2, i))) * (x[state_index + 3] - xWS_(3, i)) +
         (ts_ * xWS_(3, i) * cos(xWS_(2, i))) *
             (x[state_index + 2] - xWS_(2, i)));
    */

    g[constraint_index + 1] =
        x[state_index + 5] - (x[state_index + 1] + ts_ * x[state_index + 3] *
                                                       sin(x[state_index + 2]));

    // x3
    /*
    g[constraint_index + 2] =
        x[state_index + 6] -
        ((xWS_(2, i) + ts_ * xWS_(3, i) * tan(uWS_(0, i)) / wheelbase_) +
         (x[state_index + 2] - xWS_(2, i)) +
         (ts_ * tan(uWS_(0, i)) / wheelbase_) *
             (x[state_index + 3] - xWS_(3, i)) +
         (ts_ * xWS_(3, i) / cos(uWS_(0, i)) / cos(uWS_(0, i)) / wheelbase_)
         *
             (x[control_index] - uWS_(0, i)));
    */

    g[constraint_index + 2] =
        x[state_index + 6] -
        (x[state_index + 2] +
         ts_ * x[state_index + 3] * tan(x[control_index]) / wheelbase_);

    // x4
    /*
    g[constraint_index + 3] =
        x[state_index + 7] - ((x[state_index + 3] + ts_ * uWS_(1, i)) +
                              (ts_ * (x[control_index + 1] - uWS_(1, i))));
    */
    g[constraint_index + 3] =
        x[state_index + 7] - (x[state_index + 3] + ts_ * x[control_index + 1]);

    control_index += 2;
    constraint_index += 4;
    state_index += 4;
  }

  LOG_INFO(
      "constraint_index after adding Euler forward dynamics constraints "
      "updated: {}",
      constraint_index);

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  control_index = control_start_index_;

  // First rate is compare first with stitch point
  g[constraint_index] = (x[control_index] - last_time_u_(0, 0)) / ts_;
  control_index += 2;
  constraint_index++;

  for (int i = 1; i < horizon_; ++i) {
    g[constraint_index] = (x[control_index] - x[control_index - 2]) / ts_;
    constraint_index++;
    control_index += 2;
  }

  // 4. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4
  state_index = state_start_index_;
  int l_index = l_start_index_;
  int n_index = n_start_index_;

  for (int i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      Eigen::MatrixXd bj =
          obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

      // norm(A* lambda) <= 1
      T tmp1 = 0.0;
      T tmp2 = 0.0;
      for (int k = 0; k < current_edges_num; ++k) {
        // TODO(QiL) : replace this one directly with x
        tmp1 += Aj(k, 0) * x[l_index + k];
        tmp2 += Aj(k, 1) * x[l_index + k];
      }
      g[constraint_index] = tmp1 * tmp1 + tmp2 * tmp2;

      // G' * mu + R' * lambda == 0
      g[constraint_index + 1] = x[n_index] - x[n_index + 2] +
                                cos(x[state_index + 2]) * tmp1 +
                                sin(x[state_index + 2]) * tmp2;

      g[constraint_index + 2] = x[n_index + 1] - x[n_index + 3] -
                                sin(x[state_index + 2]) * tmp1 +
                                cos(x[state_index + 2]) * tmp2;

      //  -g'*mu + (A*t - b)*lambda > 0
      T tmp3 = 0.0;
      for (int k = 0; k < 4; ++k) {
        tmp3 += -g_[k] * x[n_index + k];
      }

      T tmp4 = 0.0;
      for (int k = 0; k < current_edges_num; ++k) {
        tmp4 += bj(k, 0) * x[l_index + k];
      }

      g[constraint_index + 3] =
          tmp3 + (x[state_index] + cos(x[state_index + 2]) * offset_) * tmp1 +
          (x[state_index + 1] + sin(x[state_index + 2]) * offset_) * tmp2 -
          tmp4;

      // Update index
      edges_counter += current_edges_num;
      l_index += current_edges_num;
      n_index += 4;
      constraint_index += 4;
    }
    state_index += 4;
  }
  LOG_INFO(
      "constraint_index after obstacles avoidance constraints "
      "updated: {}",
      constraint_index);

  // 5. load variable bounds as constraints
  state_index = state_start_index_;
  control_index = control_start_index_;
  // time_index = time_start_index_;
  l_index = l_start_index_;
  n_index = n_start_index_;

  // start configuration
  g[constraint_index] = x[state_index];
  g[constraint_index + 1] = x[state_index + 1];
  g[constraint_index + 2] = x[state_index + 2];
  g[constraint_index + 3] = x[state_index + 3];
  constraint_index += 4;
  state_index += 4;

  // constraints on x,y,v
  for (int i = 1; i < horizon_; ++i) {
    g[constraint_index] = x[state_index];
    g[constraint_index + 1] = x[state_index + 1];
    g[constraint_index + 2] = x[state_index + 3];
    constraint_index += 3;
    state_index += 4;
  }

  // end configuration
  g[constraint_index] = x[state_index];
  g[constraint_index + 1] = x[state_index + 1];
  g[constraint_index + 2] = x[state_index + 2];
  g[constraint_index + 3] = x[state_index + 3];
  constraint_index += 4;
  state_index += 4;

  for (int i = 0; i < horizon_; ++i) {
    g[constraint_index] = x[control_index];
    g[constraint_index + 1] = x[control_index + 1];
    constraint_index += 2;
    control_index += 2;
  }

  for (int i = 0; i < lambda_horizon_; ++i) {
    g[constraint_index] = x[l_index];
    constraint_index++;
    l_index++;
  }

  for (int i = 0; i < miu_horizon_; ++i) {
    g[constraint_index] = x[n_index];
    constraint_index++;
    n_index++;
  }
  return;
}

bool DistanceApproachIPOPTFixedTsInterface::check_g(int n, const double* x,
                                                    int m, const double* g) {
  int kN = n;
  int kM = m;
  double x_u_tmp[kN];
  double x_l_tmp[kN];
  double g_u_tmp[kM];
  double g_l_tmp[kM];

  GetBoundsInfo(n, x_l_tmp, x_u_tmp, m, g_l_tmp, g_u_tmp);

  const double delta_v = 1e-4;
  for (int idx = 0; idx < n; ++idx) {
    x_u_tmp[idx] = x_u_tmp[idx] + delta_v;
    x_l_tmp[idx] = x_l_tmp[idx] - delta_v;
    if (x[idx] > x_u_tmp[idx] || x[idx] < x_l_tmp[idx]) {
      LOG_INFO("x idx unfeasible: {}, x: {:.4f}, lower: {:.4f}, upper: {:.4f}", idx,
               x[idx], x_l_tmp[idx], x_u_tmp[idx]);
    }
  }

  // m1 : dynamics constatins
  int m1 = 4 * horizon_;

  // m2 : control rate constraints (only steering)
  int m2 = m1 + horizon_;

  // m3 : sampling time equality constraints
  int m3 = m2;

  // m4 : obstacle constraints
  int m4 = m3 + 4 * obstacles_num_ * (horizon_ + 1);

  // 5. load variable bounds as constraints
  // start configuration
  int m5 = m4 + 3 + 1;

  // constraints on x,y,v
  int m6 = m5 + 3 * (horizon_ - 1);

  // end configuration
  int m7 = m6 + 3 + 1;

  // control variable bnd
  int m8 = m7 + 2 * horizon_;

  // time interval variable bnd
  int m9 = m8;

  // lambda_horizon_
  int m10 = m9 + lambda_horizon_;

  // miu_horizon_
  int m11 = m10 + miu_horizon_;

  if (m11 != num_of_constraints_) {
    LOG_ERROR("m11 != num_of_constraints_");
    return false;
  }

  LOG_INFO("dynamics constatins to: {}", m1);
  LOG_INFO("control rate constraints (only steering) to: {}", m2);
  LOG_INFO("sampling time equality constraints to: {}", m3);
  LOG_INFO("obstacle constraints to: {}", m4);
  LOG_INFO("start conf constraints to: {}", m5);
  LOG_INFO("constraints on x,y,v to: {}", m6);
  LOG_INFO("end constraints to: {}", m7);
  LOG_INFO("control bnd to: {}", m8);
  LOG_INFO("time interval constraints to: {}", m9);
  LOG_INFO("lambda constraints to: {}", m10);
  LOG_INFO("miu constraints to: {}", m11);
  LOG_INFO("total constraints: {}", num_of_constraints_);

  for (int idx = 0; idx < m; ++idx) {
    if (g[idx] > g_u_tmp[idx] + delta_v || g[idx] < g_l_tmp[idx] - delta_v) {
      LOG_INFO(
          "constrains idx unfeasible: {} , g: {:.4f}, lower: {:.4f}, upper: {:.4f}",
          idx, g[idx], g_l_tmp[idx], g_u_tmp[idx]);
    }
  }
  return true;
}

void DistanceApproachIPOPTFixedTsInterface::GenerateTapes(int n, int m,
                                                          int* nnz_jac_g,
                                                          int* nnz_h_lag) {
  std::vector<double> xp(n);
  std::vector<double> lamp(m);
  std::vector<double> zl(m);
  std::vector<double> zu(m);
  std::vector<adouble> xa(n);
  std::vector<adouble> g(m);
  std::vector<double> lam(m);

  double sig;
  adouble obj_value;
  double dummy = 0.0;
  obj_lam = new double[m + 1];
  GetStartingPoint(n, 1, &xp[0], 0, &zl[0], &zu[0], m, 0, &lamp[0]);

  trace_on(tag_f);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  EvalObj(n, &xa[0], &obj_value);
  obj_value >>= dummy;
  trace_off();

  trace_on(tag_g);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  EvalConstraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    g[idx] >>= dummy;
  }
  trace_off();

  trace_on(tag_L);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  for (int idx = 0; idx < m; idx++) {
    lam[idx] = 1.0;
  }
  sig = 1.0;
  EvalObj(n, &xa[0], &obj_value);
  obj_value *= mkparam(sig);
  EvalConstraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    obj_value += g[idx] * mkparam(lam[idx]);
  }
  obj_value >>= dummy;

  trace_off();

  rind_g = nullptr;
  cind_g = nullptr;
  rind_L = nullptr;
  cind_L = nullptr;
  jacval = nullptr;
  hessval = nullptr;

  options_g[0] = 0; /* sparsity pattern by index domains (default) */
  options_g[1] = 0; /*                         safe mode (default) */
  options_g[2] = 0;
  options_g[3] = 0; /*                column compression (default) */
  sparse_jac(tag_g, m, n, 0, &xp[0], &nnz_jac, &rind_g, &cind_g, &jacval,
             options_g);
  *nnz_jac_g = nnz_jac;

  options_L[0] = 0;
  options_L[1] = 1;
  sparse_hess(tag_L, n, 0, &xp[0], &nnz_L, &rind_L, &cind_L, &hessval,
              options_L);
  *nnz_h_lag = nnz_L;
  return;
}
//***************    end   ADOL-C part ***********************************

}  // namespace planning
}  // namespace neodrive

#include "piecewise_jerk_problem.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

PiecewiseJerkProblem::PiecewiseJerkProblem(
    const std::size_t num_of_knots, const int max_iter, const double delta_s,
    const double time_limit, const std::array<double, 3> &x_init) {
  if (num_of_knots < 2) {
    LOG_ERROR("num_of_knots [{}] < 2, error return", num_of_knots);
    return;
  }
  num_of_knots_ = num_of_knots;

  x_init_ = x_init;

  delta_s_ = delta_s;

  delta_s_vec_ = std::vector<double>(num_of_knots, delta_s);

  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  circles_x_bounds_.resize(num_of_knots_);

  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
  weight_x_obs_dis_vec_ = std::vector<double>(num_of_knots_, 0.0);
  // Define Solver settings_ as default
  // settings_ =
  // reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OrhsSQPSettings)));
  settings_ = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  osqp_set_default_settings(settings_);
  settings_->warm_start = true;
  settings_->polish = true;
  settings_->verbose = false;
  settings_->scaled_termination = true;

  settings_->eps_abs = 0.01;
  settings_->eps_rel = 0.01;

  settings_->max_iter = max_iter;
  settings_->time_limit = time_limit;

  // Populate data_
  // data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data_ = (OSQPData *)c_malloc(sizeof(OSQPData));
}

PiecewiseJerkProblem::~PiecewiseJerkProblem() {
  // data_->A and data_->P are not malloc, do not clean up them
  osqp_cleanup(work_);
  if (data_) c_free(data_);
  if (settings_) c_free(settings_);
  work_ = nullptr;
  data_ = nullptr;
  settings_ = nullptr;
}

void PiecewiseJerkProblem::clean_all_data() {
  osqp_cleanup(work_);
  if (data_) {
    if (data_->A) c_free(data_->A);
    if (data_->P) c_free(data_->P);
    data_->A = nullptr;
    data_->P = nullptr;
    c_free(data_);
  }
  if (settings_) c_free(settings_);
  work_ = nullptr;
  data_ = nullptr;
  settings_ = nullptr;
}

bool PiecewiseJerkProblem::OptimizeWithOsqp(
    const bool use_inaccurate, const std::size_t kernel_dim,
    const std::size_t num_affine_constraint, std::vector<c_float> &P_data,
    std::vector<c_int> &P_indices,                                // NOLINT
    std::vector<c_int> &P_indptr, std::vector<c_float> &A_data,   // NOLINT
    std::vector<c_int> &A_indices, std::vector<c_int> &A_indptr,  // NOLINT
    std::vector<c_float> &lower_bounds,                           // NOLINT
    std::vector<c_float> &upper_bounds,                           // NOLINT
    std::vector<c_float> &q) {
  if (lower_bounds.size() != upper_bounds.size()) {
    LOG_ERROR("lower_bounds.size() != upper_bounds.size()");
    return false;
  }

  data_->n = kernel_dim;
  data_->m = num_affine_constraint;
  data_->P = csc_matrix(data_->n, data_->n, P_data.size(), P_data.data(),
                        P_indices.data(), P_indptr.data());
  data_->q = q.data();
  data_->A = csc_matrix(data_->m, data_->n, A_data.size(), A_data.data(),
                        A_indices.data(), A_indptr.data());
  data_->l = lower_bounds.data();
  data_->u = upper_bounds.data();

  // time_t start_t = clock();

  work_ = osqp_setup(data_, settings_);

  // LOG_DEBUG("osqp_setup cost time {:.3f} ms",
  //           (double)(clock() - start_t) * 1000 / CLOCKS_PER_SEC);
  // start_t = clock();

  // use warm start or not
  // if (primal_warm_start_.size() == data_->n) {
  //   osqp_warm_start_x(work_, primal_warm_start_.data());
  //   LOG_INFO("use warm start, primal_warm_start_.size() {} == data_->n {}",
  //            primal_warm_start_.size(), data_->n);
  // }

  // LOG_DEBUG("osqp_warm_start_x cost time {:.3f} ms",
  //           (double)(clock() - start_t) * 1000 / CLOCKS_PER_SEC);
  time_t start_t = clock();

  // Solve Problem
  osqp_solve(work_);

  LOG_ERROR("osqp_solve cost time {:.3f} ms",
            (double)(clock() - start_t) * 1000 / CLOCKS_PER_SEC);

  auto status = work_->info->status_val;

  // set two drive mode
  if (use_inaccurate) {
    if (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE &&
        status != OSQP_MAX_ITER_REACHED) {
      LOG_ERROR("optimization status: {}", (work_)->info->status);
      clean_all_data();
      return false;
    } else {
      LOG_INFO("optimization status: {}", (work_)->info->status);
    }
  } else {
    if (status != OSQP_SOLVED) {
      LOG_ERROR("optimization status: {}", (work_)->info->status);
      clean_all_data();
      return false;
    } else {
      LOG_INFO("optimization status: {}", (work_)->info->status);
    }
  }

  return true;
}

void PiecewiseJerkProblem::set_delta_s_vec(
    const std::vector<double> &delta_s_vec) {
  if (delta_s_vec.size() != num_of_knots_) {
    LOG_ERROR("delta_s_vec.size({}) != num_of_knots_: {}", delta_s_vec.size());
    return;
  }
  delta_s_vec_ = std::move(delta_s_vec);
}

void PiecewiseJerkProblem::set_x_bounds(
    std::vector<std::pair<double, double>> x_bounds) {
  if (x_bounds.size() != num_of_knots_) {
    LOG_ERROR("x_bounds.size({}) != num_of_knots_: {}", x_bounds.size(),
              num_of_knots_);
    return;
  }
  x_bounds_ = std::move(x_bounds);
}

void PiecewiseJerkProblem::set_circles_x_bounds(
    std::vector<std::vector<std::pair<double, double>>>
        circles_road_obs_boundary) {
  if (circles_road_obs_boundary.size() != num_of_knots_) {
    LOG_ERROR("circles_road_obs_boundary.size({}) != num_of_knots_: {}",
              circles_road_obs_boundary.size(), num_of_knots_);
    return;
  }
  circles_x_bounds_ = std::move(circles_road_obs_boundary);
}

void PiecewiseJerkProblem::set_dx_bounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  if (dx_bounds.size() != num_of_knots_) {
    LOG_ERROR("dx_bounds.size({}) != num_of_knots_: {}", dx_bounds.size(),
              num_of_knots_);
    return;
  }
  dx_bounds_ = std::move(dx_bounds);
}

void PiecewiseJerkProblem::set_ddx_bounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  if (ddx_bounds.size() != num_of_knots_) {
    LOG_ERROR("ddx_bounds.size({}) != num_of_knots_: {}", ddx_bounds.size(),
              num_of_knots_);
    return;
  }
  ddx_bounds_ = std::move(ddx_bounds);
}

void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound,
                                        const double x_upper_bound) {
  for (auto &x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }
}

void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound,
                                         const double dx_upper_bound) {
  for (auto &x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound,
                                          const double ddx_upper_bound) {
  for (auto &x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
}
void PiecewiseJerkProblem::set_dddx_bound(const double dddx_bound) {
  set_dddx_bound(-dddx_bound, dddx_bound);
}
void PiecewiseJerkProblem::set_dddx_bound(const double dddx_lower_bound,
                                          const double dddx_upper_bound) {
  dddx_bound_.first = dddx_lower_bound;
  dddx_bound_.second = dddx_upper_bound;
}

void PiecewiseJerkProblem::set_dddx_bound(
    const std::vector<std::pair<double, double>> &dddx_bounds) {
  dddx_bound_vec_ = std::move(dddx_bounds);
}

void PiecewiseJerkProblem::set_weight_x(const double weight_x) {
  weight_x_ = weight_x;
}

void PiecewiseJerkProblem::set_weight_dx(const double weight_dx) {
  weight_dx_ = weight_dx;
}

void PiecewiseJerkProblem::set_weight_ddx(const double weight_ddx) {
  weight_ddx_ = weight_ddx;
}

void PiecewiseJerkProblem::set_weight_dddx(const double weight_dddx) {
  weight_dddx_ = weight_dddx;
}

void PiecewiseJerkProblem::set_weight_soft_x(const double weight_soft_x) {
  weight_soft_x_ = weight_soft_x;
}

void PiecewiseJerkProblem::set_value_soft_x(const double value_soft_x) {
  value_soft_x_ = value_soft_x;
}

void PiecewiseJerkProblem::set_scale_factor(
    const std::array<double, 3> &scale_factor) {
  scale_factor_ = scale_factor;
}

void PiecewiseJerkProblem::set_x_ref(const double weight_x_ref,
                                     std::vector<double> &x_ref) {
  if (x_ref.size() != num_of_knots_) {
    LOG_ERROR("x_ref.size({}) != num_of_knots_({})", x_ref.size(),
              num_of_knots_);
    return;
  }
  weight_x_ref_ = weight_x_ref;
  // set uniform weighting
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, weight_x_ref);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}
void PiecewiseJerkProblem::set_x_ref(std::vector<double> &weight_x_ref_vec,
                                     std::vector<double> &x_ref) {
  if (x_ref.size() != num_of_knots_) {
    LOG_ERROR("x_ref.size({}) != num_of_knots_({})", x_ref.size(),
              num_of_knots_);
    return;
  }
  if (weight_x_ref_vec.size() != num_of_knots_) {
    LOG_ERROR("weight_x_ref_vec.size({}) != num_of_knots_({})",
              weight_x_ref_vec.size(), num_of_knots_);
    return;
  }
  // set piecewise weighting
  weight_x_ref_vec_ = std::move(weight_x_ref_vec);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

void PiecewiseJerkProblem::set_x_obs_dis(const double weight_x_obs_dis,
                                         std::vector<double> &x_obs_dis) {
  if (x_obs_dis.size() != num_of_knots_) {
    LOG_ERROR("x_obs_dis.size({}) != num_of_knots_({})", x_obs_dis.size(),
              num_of_knots_);
    return;
  }
  weight_x_obs_dis_ = weight_x_obs_dis;
  x_obs_dis_ = std::move(x_obs_dis);
  has_x_obs_dis_ = true;
}

void PiecewiseJerkProblem::set_x_obs_dis(std::vector<double> &weight_x_obs_dis,
                                         std::vector<double> &x_obs_dis) {
  if (weight_x_obs_dis.size() != num_of_knots_ ||
      x_obs_dis.size() != num_of_knots_) {
    LOG_ERROR(
        "weight_x_obs_dis.size({}) or x_obs_dis.size({}) != "
        "num_of_knots_({})",
        weight_x_obs_dis.size(), x_obs_dis.size(), num_of_knots_);
    return;
  }
  weight_x_obs_dis_vec_ = std::move(weight_x_obs_dis);
  x_obs_dis_ = std::move(x_obs_dis);
  has_x_obs_dis_ = true;
}

void PiecewiseJerkProblem::set_end_state_ref(
    const std::array<double, 3> &weight_end_state,
    const std::array<double, 3> &end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

void PiecewiseJerkProblem::set_primal_warm_start(
    const std::vector<c_float> &primal_warm_start) {
  primal_warm_start_ = primal_warm_start;
}

void PiecewiseJerkProblem::set_use_auto_warm_start(
    const bool use_auto_warm_start) {
  use_auto_warm_start_ = use_auto_warm_start;
}

std::vector<c_float> PiecewiseJerkProblem::primal_warm_start() const {
  return primal_warm_start_;
}

std::size_t PiecewiseJerkProblem::num_of_knots() const { return num_of_knots_; }

double PiecewiseJerkProblem::DeltaS() const { return delta_s_; }

double PiecewiseJerkProblem::WeightX() const { return weight_x_; }
double PiecewiseJerkProblem::WeightDx() const { return weight_dx_; }
double PiecewiseJerkProblem::WeightDdx() const { return weight_ddx_; }
double PiecewiseJerkProblem::WeightDddx() const { return weight_dddx_; }

double PiecewiseJerkProblem::WeightSoftX() const { return weight_soft_x_; }
double PiecewiseJerkProblem::ValueSoftX() const { return value_soft_x_; }

const std::vector<double> &PiecewiseJerkProblem::opt_x() const { return x_; }
const std::vector<double> &PiecewiseJerkProblem::opt_dx() const { return dx_; }
const std::vector<double> &PiecewiseJerkProblem::opt_ddx() const {
  return ddx_;
}
const std::vector<double> &PiecewiseJerkProblem::opt_soft_x() const {
  return soft_x_;
}

void PiecewiseJerkProblem::update_x_init(const std::array<double, 3> &x_init) {
  x_init_ = x_init;
}

bool PiecewiseJerkProblem::Optimize(const bool use_inaccurate) {
  if (use_auto_warm_start_) {
    LOG_DEBUG("use auto warm");
    if (!Optimize_with_auto_warm(use_inaccurate)) {
      LOG_ERROR("Optimize_with_auto_warm err");
      return false;
    }
  } else {
    if (!Optimize_without_auto_warm(use_inaccurate)) {
      LOG_DEBUG("donot use auto warm");
      LOG_ERROR("Optimize_without_auto_warm err");
      return false;
    }
  }
  return true;
}

bool PiecewiseJerkProblem::Optimize_with_auto_warm(const bool use_inaccurate) {
  // update x bounds
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  update_bounds(lower_bounds, upper_bounds);
  osqp_update_bounds(work_, lower_bounds.data(), upper_bounds.data());
  osqp_solve(work_);
  auto status = work_->info->status_val;
  // set two drive mode
  if (use_inaccurate) {
    if (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE &&
        status != OSQP_MAX_ITER_REACHED) {
      LOG_ERROR("optimization status: {}", (work_)->info->status);
      clean_all_data();
      return false;
    } else {
      LOG_INFO("optimization status: {}", (work_)->info->status);
    }
  } else {
    if (status != OSQP_SOLVED) {
      LOG_ERROR("optimization status: {}", (work_)->info->status);
      clean_all_data();
      return false;
    } else {
      LOG_INFO("optimization status: {}", (work_)->info->status);
    }
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  soft_x_.resize(num_of_knots_, 0.0);
  for (std::size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = work_->solution->x[i];
    dx_.at(i) = work_->solution->x[i + num_of_knots_];
    ddx_.at(i) = work_->solution->x[i + 2 * num_of_knots_];
    soft_x_.at(i) = work_->solution->x[i + 3 * num_of_knots_];
  }

  return true;
}

bool PiecewiseJerkProblem::Optimize_without_auto_warm(
    const bool use_inaccurate) {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(P_data, P_indices, P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  if (FLAGS_planning_piecewise_use_soft_constraints) {
    CalculateAffineConstraintSoft(A_data, A_indices, A_indptr, lower_bounds,
                                  upper_bounds);
  } else {
    CalculateAffineConstraint(A_data, A_indices, A_indptr, lower_bounds,
                              upper_bounds);
  }

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(q);
  std::size_t kernel_dimensions = 3 * num_of_knots_;
  if (FLAGS_planning_piecewise_use_soft_constraints)
    kernel_dimensions = 4 * num_of_knots_;
  bool res = OptimizeWithOsqp(
      use_inaccurate, kernel_dimensions, lower_bounds.size(), P_data, P_indices,
      P_indptr, A_data, A_indices, A_indptr, lower_bounds, upper_bounds, q);
  if (res == false || work_ == nullptr || work_->solution == nullptr) {
    LOG_INFO("Failed to find solution.");
    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  soft_x_.resize(num_of_knots_, 0.0);
  for (std::size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = work_->solution->x[i];
    dx_.at(i) = work_->solution->x[i + num_of_knots_];
    ddx_.at(i) = work_->solution->x[i + 2 * num_of_knots_];
    if (FLAGS_planning_piecewise_use_soft_constraints)
      soft_x_.at(i) = work_->solution->x[i + 3 * num_of_knots_];
  }

  return true;
}

void PiecewiseJerkProblem::update_bounds(std::vector<c_float> &lower_bounds,
                                         std::vector<c_float> &upper_bounds) {
  const int N = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * N;
  // const int kNumConstraint = kNumParam + 3 * (N - 1) + 3 + circles_num_ * N;
  int kNumConstraint = kNumParam + 3 * (N - 1) + 3;
  if (FLAGS_planning_piecewise_use_four_circles_model) {
    kNumConstraint += circles_num_ * N;
  }
  lower_bounds.resize(kNumConstraint);
  upper_bounds.resize(kNumConstraint);
  int constraint_index = 0;
  // set x, x', x'' bounds
  for (int i = 0; i < kNumParam; ++i) {
    if (i < N) {
      lower_bounds.at(constraint_index) =
          std::get<0>(x_bounds_[i]) * scale_factor_[0];
      upper_bounds.at(constraint_index) =
          std::get<1>(x_bounds_[i]) * scale_factor_[0];
    } else if (i < 2 * N) {
      lower_bounds.at(constraint_index) =
          std::get<0>(dx_bounds_[i - N]) * scale_factor_[1];
      upper_bounds.at(constraint_index) =
          std::get<1>(dx_bounds_[i - N]) * scale_factor_[1];
    } else {
      lower_bounds.at(constraint_index) =
          std::get<0>(ddx_bounds_[i - 2 * N]) * scale_factor_[2];
      upper_bounds.at(constraint_index) =
          std::get<1>(ddx_bounds_[i - 2 * N]) * scale_factor_[2];
    }
    ++constraint_index;
  }

  if (constraint_index != kNumParam) {
    LOG_ERROR("constraint_index != kNumParam");
    return;
  }

  // x'', x', x consistance constraint.
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  // x(i+1)'' - x(i)'' - x(i)''' * delta_s = 0
  for (int i = 0; i + 1 < N; ++i) {
    lower_bounds.at(constraint_index) =
        dddx_bound_.first * delta_s_vec_[i + 1] * scale_factor_[2];
    upper_bounds.at(constraint_index) =
        dddx_bound_.second * delta_s_vec_[i + 1] * scale_factor_[2];
    ++constraint_index;
  }

  // x(i+1)' - x(i)' - 0.5 * delta_s * (x(i+1)'' + x(i)'') = 0
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < N; ++i) {
    lower_bounds.at(constraint_index) = 0.0;
    upper_bounds.at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - x(i)'*delta_s - 1/3*x(i)''*delta_s^2 -
  // 1/6*x(i+1)''*delta_s^2
  for (int i = 0; i + 1 < N; ++i) {
    lower_bounds.at(constraint_index) = 0.0;
    upper_bounds.at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // constrain on x_init
  lower_bounds.at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds.at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  lower_bounds.at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds.at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  lower_bounds.at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds.at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  if (FLAGS_planning_piecewise_use_four_circles_model) {
    // set four circles corresponding bounds
    for (std::size_t i = 0; i < N; ++i) {
      if (circles_x_bounds_[i].size() != circles_num_) {
        LOG_ERROR("circles_x_bounds_[{}].size() {} != {}, err", i,
                  circles_x_bounds_[i].size(), circles_num_);
        return;
      }
      for (std::size_t j = 0; j < circles_num_; j++) {
        lower_bounds.at(constraint_index) =
            std::get<0>(circles_x_bounds_[i][j]) * scale_factor_[0];
        upper_bounds.at(constraint_index) =
            std::get<1>(circles_x_bounds_[i][j]) * scale_factor_[0];
        ++constraint_index;
      }
    }
  }

  if (constraint_index != kNumConstraint) {
    LOG_ERROR("constraint_index {} != kNumConstraint {}", constraint_index,
              kNumConstraint);
    return;
  }
}

void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float> &A_data, std::vector<c_int> &A_indices,
    std::vector<c_int> &A_indptr, std::vector<c_float> &lower_bounds,
    std::vector<c_float> &upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  // 4N params bounds on four circle centers
  const int N = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * N;
  // const int kNumConstraint = kNumParam + 3 * (N - 1) + 3 + circles_num_ * N +
  // 3;
  int kNumConstraint = kNumParam + 3 * (N - 1) + 3 + 3;
  if (FLAGS_planning_piecewise_use_four_circles_model) {
    kNumConstraint += circles_num_ * N;
    LOG_INFO("use four circles model");
  }
  lower_bounds.resize(kNumConstraint);
  upper_bounds.resize(kNumConstraint);

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int constraint_index = 0;
  LOG_DEBUG(
      "num_of_knots_ {}, kNumParam {}, kNumConstraint {}, "
      "columns.size() {}",
      num_of_knots_, kNumParam, kNumConstraint, columns.size());

  // x, x', x'' bounds
  for (int i = 0; i < kNumParam; ++i) {
    columns[i].emplace_back(constraint_index, 1.0);
    if (i < N) {
      lower_bounds.at(constraint_index) = x_bounds_[i].first * scale_factor_[0];
      upper_bounds.at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * N) {
      lower_bounds.at(constraint_index) =
          dx_bounds_[i - N].first * scale_factor_[1];
      upper_bounds.at(constraint_index) =
          dx_bounds_[i - N].second * scale_factor_[1];
    } else {
      lower_bounds.at(constraint_index) =
          ddx_bounds_[i - 2 * N].first * scale_factor_[2];
      upper_bounds.at(constraint_index) =
          ddx_bounds_[i - 2 * N].second * scale_factor_[2];
    }
    ++constraint_index;
  }
  if (constraint_index != kNumParam) {
    LOG_ERROR("constraint_index != kNumParam");
    return;
  }

  // x'', x', x consistance constraint.
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  // x(i+1)'' - x(i)'' - x(i)''' * delta_s = 0
  for (int i = 0; i + 1 < N; ++i) {
    columns[2 * N + i].emplace_back(constraint_index, -1.0);
    columns[2 * N + i + 1].emplace_back(constraint_index, 1.0);
    if (!dddx_bound_vec_.empty()) {
      lower_bounds.at(constraint_index) =
          dddx_bound_vec_[i].first * delta_s_vec_[i + 1] * scale_factor_[2];
      upper_bounds.at(constraint_index) =
          dddx_bound_vec_[i].second * delta_s_vec_[i + 1] * scale_factor_[2];
    } else {
      lower_bounds.at(constraint_index) =
          dddx_bound_.first * delta_s_vec_[i + 1] * scale_factor_[2];
      upper_bounds.at(constraint_index) =
          dddx_bound_.second * delta_s_vec_[i + 1] * scale_factor_[2];
    }
    ++constraint_index;
  }
  LOG_DEBUG("dddx_bound_ ({}, {}) delta_s_ {}", dddx_bound_.first,
            dddx_bound_.second, delta_s_);  // 4, 0.6

  // x(i+1)' - x(i)' - 0.5 * delta_s * (x(i+1)'' + x(i)'') = 0
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < N; ++i) {
    columns[N + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    columns[N + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    columns[2 * N + i].emplace_back(
        constraint_index, -0.5 * delta_s_vec_[i + 1] * scale_factor_[1]);
    columns[2 * N + i + 1].emplace_back(
        constraint_index, -0.5 * delta_s_vec_[i + 1] * scale_factor_[1]);
    lower_bounds.at(constraint_index) = 0.0;
    upper_bounds.at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - x(i)'*delta_s - 1/3*x(i)''*delta_s^2 -
  // 1/6*x(i+1)''*delta_s^2
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < N; ++i) {
    columns[i].emplace_back(constraint_index,
                            -1.0 * scale_factor_[1] * scale_factor_[2]);
    columns[i + 1].emplace_back(constraint_index,
                                1.0 * scale_factor_[1] * scale_factor_[2]);
    columns[N + i].emplace_back(
        constraint_index,
        -delta_s_vec_[i + 1] * scale_factor_[0] * scale_factor_[2]);
    columns[2 * N + i].emplace_back(
        constraint_index, -delta_s_vec_[i + 1] * delta_s_vec_[i + 1] / 3.0 *
                              scale_factor_[0] * scale_factor_[1]);
    columns[2 * N + i + 1].emplace_back(
        constraint_index, -delta_s_vec_[i + 1] * delta_s_vec_[i + 1] / 6.0 *
                              scale_factor_[0] * scale_factor_[1]);

    lower_bounds.at(constraint_index) = 0.0;
    upper_bounds.at(constraint_index) = 0.0;
    ++constraint_index;
  }
  LOG_DEBUG("delta_s_sq_ / 3.0 {}, -delta_s_sq_ / 6.0 {}", delta_s_sq_ / 3.0,
            -delta_s_sq_ / 6.0);

  // constraint on x_init
  columns[0].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds.at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  columns[N].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds.at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  columns[2 * N].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds.at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  // constriant on end_state
  columns[N - 1].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = x_bounds_.back().first;
  upper_bounds.at(constraint_index) = x_bounds_.back().second;
  ++constraint_index;

  columns[2 * N - 1].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = -0.01;
  upper_bounds.at(constraint_index) = 0.01;
  ++constraint_index;

  columns[3 * N - 1].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = ddx_bounds_.back().first;
  upper_bounds.at(constraint_index) = ddx_bounds_.back().second;
  ++constraint_index;

  if (FLAGS_planning_piecewise_use_four_circles_model) {  // set four x
                                                          // corresponding
                                                          // bounds
    // first circle center distance to back edge
    const double circle_distance = FLAGS_planning_piecewise_circle_distance;
    const double back_edge_to_center =
        VehicleParam::Instance()->back_edge_to_center();
    const double front_edge_to_center =
        VehicleParam::Instance()->front_edge_to_center();
    const double first_circle_dis = -(back_edge_to_center - circle_distance);
    const double second_circle_dis =
        -(back_edge_to_center - 3.0 * circle_distance);
    const double third_circle_dis =
        front_edge_to_center - 3.0 * circle_distance;
    const double fourth_circle_dis = front_edge_to_center - circle_distance;
    std::vector<double> circle_dis_vec{first_circle_dis, second_circle_dis,
                                       third_circle_dis, fourth_circle_dis};

    LOG_DEBUG("x_init_({}, {}, {})", x_init_[0], x_init_[1], x_init_[2]);
    for (int i = 0; i < N; ++i) {
      if (circles_x_bounds_[i].size() != circles_num_) {
        LOG_ERROR("circles_x_bounds_[{}].size() {} != {}, err", i,
                  circles_x_bounds_[i].size(), circles_num_);
        return;
      }
      for (int j = 0; j < circles_num_; ++j) {
        columns[i].emplace_back(constraint_index, 1.0);
        columns[N + i].emplace_back(constraint_index, circle_dis_vec[j]);
        lower_bounds.at(constraint_index) =
            circles_x_bounds_[i][j].first * scale_factor_[0];
        upper_bounds.at(constraint_index) =
            circles_x_bounds_[i][j].second * scale_factor_[0];
        ++constraint_index;
        if (i == 0) {
          LOG_DEBUG("circles_x_bounds_[{}][{}] ({}, {}), l+delta*dl = {}", i, j,
                    circles_x_bounds_[i][j].first,
                    circles_x_bounds_[i][j].second,
                    x_init_[0] + x_init_[1] * circle_dis_vec[j]);
        } else {
          LOG_DEBUG("circles_x_bounds_[{}][{}] ({}, {})", i, j,
                    circles_x_bounds_[i][j].first,
                    circles_x_bounds_[i][j].second);
        }
      }
    }
  }

  if (constraint_index != kNumConstraint) {
    LOG_ERROR("constraint_index != kNumConstraint");
    return;
  }

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    A_indptr.push_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      A_data.push_back(row_data_pair.second);
      A_indices.push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  A_indptr.push_back(ind_p);
}

void PiecewiseJerkProblem::CalculateAffineConstraintSoft(
    std::vector<c_float> &A_data, std::vector<c_int> &A_indices,
    std::vector<c_int> &A_indptr, std::vector<c_float> &lower_bounds,
    std::vector<c_float> &upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  // 4N params bounds on four circle centers
  const int N = static_cast<int>(num_of_knots_);
  int kNumParam = 3 * N;
  int kNumConstraint = kNumParam + 3 * (N - 1) + 3;

  if (FLAGS_planning_piecewise_use_soft_constraints) {
    kNumParam = 4 * N;
    kNumConstraint += 2 * N;
  }

  if (FLAGS_planning_piecewise_use_four_circles_model) {
    kNumConstraint += circles_num_ * N;
    LOG_INFO("use four circles model");
  }
  lower_bounds.resize(kNumConstraint);
  upper_bounds.resize(kNumConstraint);

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int constraint_index = 0;
  LOG_DEBUG(
      "num_of_knots_ {}, kNumParam {}, kNumConstraint {}, "
      "columns.size() {}",
      num_of_knots_, kNumParam, kNumConstraint, columns.size());
  // set x, x', x'' bounds
  const double max_range = 100.0;
  for (int i = 0; i < kNumParam; ++i) {
    if (i < 2 * N) {
      columns[i / 2].emplace_back(constraint_index, 1.0);
      if (i % 2 == 0) {
        columns[3 * N + i / 2].emplace_back(constraint_index, -1.0);
        lower_bounds.at(constraint_index) = -max_range;
        upper_bounds.at(constraint_index) =
            std::get<1>(x_bounds_[i / 2]) * scale_factor_[0];
      } else {
        columns[3 * N + i / 2].emplace_back(constraint_index, 1.0);
        lower_bounds.at(constraint_index) =
            std::get<0>(x_bounds_[i / 2]) * scale_factor_[0];
        upper_bounds.at(constraint_index) = max_range;
      }
      // LOG_DEBUG("x_bounds_[{}] ({}, {})", i, x_bounds_[i].first,
      //           x_bounds_[i].second);
    } else if (i < 3 * N) {
      columns[i - N].emplace_back(constraint_index, 1.0);
      lower_bounds.at(constraint_index) =
          std::get<0>(dx_bounds_[i - 2 * N]) * scale_factor_[1];
      upper_bounds.at(constraint_index) =
          std::get<1>(dx_bounds_[i - 2 * N]) * scale_factor_[1];
    } else {
      columns[i - N].emplace_back(constraint_index, 1.0);
      lower_bounds.at(constraint_index) =
          std::get<0>(ddx_bounds_[i - 3 * N]) * scale_factor_[2];
      upper_bounds.at(constraint_index) =
          std::get<1>(ddx_bounds_[i - 3 * N]) * scale_factor_[2];
    }
    ++constraint_index;
  }
  if (constraint_index != kNumParam) {
    LOG_ERROR("constraint_index != kNumParam");
    return;
  }

  // x'', x', x consistance constraint.
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  // x(i+1)'' - x(i)'' - x(i)''' * delta_s = 0
  for (int i = 0; i + 1 < N; ++i) {
    columns[2 * N + i].emplace_back(constraint_index, -1.0);
    columns[2 * N + i + 1].emplace_back(constraint_index, 1.0);
    if (!dddx_bound_vec_.empty()) {
      lower_bounds.at(constraint_index) =
          dddx_bound_vec_[i].first * delta_s_vec_[i + 1] * scale_factor_[2];
      upper_bounds.at(constraint_index) =
          dddx_bound_vec_[i].second * delta_s_vec_[i + 1] * scale_factor_[2];
    } else {
      lower_bounds.at(constraint_index) =
          dddx_bound_.first * delta_s_vec_[i + 1] * scale_factor_[2];
      upper_bounds.at(constraint_index) =
          dddx_bound_.second * delta_s_vec_[i + 1] * scale_factor_[2];
    }
    ++constraint_index;
  }
  LOG_DEBUG("dddx_bound_ ({}, {}) delta_s_ {}", dddx_bound_.first,
            dddx_bound_.second, delta_s_);  // 4, 0.6

  // x(i+1)' - x(i)' - 0.5 * delta_s * (x(i+1)'' + x(i)'') = 0
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < N; ++i) {
    columns[N + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    columns[N + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    columns[2 * N + i].emplace_back(
        constraint_index, -0.5 * delta_s_vec_[i + 1] * scale_factor_[1]);
    columns[2 * N + i + 1].emplace_back(
        constraint_index, -0.5 * delta_s_vec_[i + 1] * scale_factor_[1]);
    lower_bounds.at(constraint_index) = 0.0;
    upper_bounds.at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - x(i)'*delta_s - 1/3*x(i)''*delta_s^2 -
  // 1/6*x(i+1)''*delta_s^2
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < N; ++i) {
    columns[i].emplace_back(constraint_index,
                            -1.0 * scale_factor_[1] * scale_factor_[2]);
    columns[i + 1].emplace_back(constraint_index,
                                1.0 * scale_factor_[1] * scale_factor_[2]);
    columns[N + i].emplace_back(
        constraint_index,
        -delta_s_vec_[i + 1] * scale_factor_[0] * scale_factor_[2]);
    columns[2 * N + i].emplace_back(
        constraint_index, -delta_s_vec_[i + 1] * delta_s_vec_[i + 1] / 3.0 *
                              scale_factor_[0] * scale_factor_[1]);
    columns[2 * N + i + 1].emplace_back(
        constraint_index, -delta_s_vec_[i + 1] * delta_s_vec_[i + 1] / 6.0 *
                              scale_factor_[0] * scale_factor_[1]);

    lower_bounds.at(constraint_index) = 0.0;
    upper_bounds.at(constraint_index) = 0.0;
    ++constraint_index;
  }
  LOG_DEBUG("delta_s_sq_ / 3.0 {}, -delta_s_sq_ / 6.0 {}", delta_s_sq_ / 3.0,
            -delta_s_sq_ / 6.0);
  // constrain on x_init
  columns[0].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds.at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  columns[N].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds.at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  columns[2 * N].emplace_back(constraint_index, 1.0);
  lower_bounds.at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds.at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  const double low_value = 0.0;
  if (FLAGS_planning_piecewise_use_soft_constraints) {
    for (int i = 0; i < N; ++i) {
      columns[3 * N + i].emplace_back(constraint_index, 1.0);
      lower_bounds.at(constraint_index) = 0.0;
      upper_bounds.at(constraint_index) = value_soft_x_;
      ++constraint_index;
    }
  }
  // soft constraints on x
  LOG_DEBUG("soft value {}, {}", low_value, value_soft_x_);

  if (FLAGS_planning_piecewise_use_four_circles_model) {  // set four x
                                                          // corresponding
                                                          // bounds
    // first circle center distance to back edge
    const double circle_distance = FLAGS_planning_piecewise_circle_distance;
    const double back_edge_to_center =
        VehicleParam::Instance()->back_edge_to_center();
    const double front_edge_to_center =
        VehicleParam::Instance()->front_edge_to_center();
    const double first_circle_dis = -(back_edge_to_center - circle_distance);
    const double second_circle_dis =
        -(back_edge_to_center - 3.0 * circle_distance);
    const double third_circle_dis =
        front_edge_to_center - 3.0 * circle_distance;
    const double fourth_circle_dis = front_edge_to_center - circle_distance;
    std::vector<double> circle_dis_vec{first_circle_dis, second_circle_dis,
                                       third_circle_dis, fourth_circle_dis};

    LOG_DEBUG("x_init_({}, {}, {})", x_init_[0], x_init_[1], x_init_[2]);
    for (int i = 0; i < N; ++i) {
      if (circles_x_bounds_[i].size() != circles_num_) {
        LOG_ERROR("circles_x_bounds_[{}].size() {} != {}, err", i,
                  circles_x_bounds_[i].size(), circles_num_);
        return;
      }
      for (int j = 0; j < circles_num_; ++j) {
        columns[i].emplace_back(constraint_index, 1.0);
        columns[N + i].emplace_back(constraint_index, circle_dis_vec[j]);
        lower_bounds.at(constraint_index) =
            std::get<0>(circles_x_bounds_[i][j]) * scale_factor_[0];
        upper_bounds.at(constraint_index) =
            std::get<1>(circles_x_bounds_[i][j]) * scale_factor_[0];
        ++constraint_index;
        if (i == 0) {
          LOG_DEBUG("circles_x_bounds_[{}][{}] ({}, {}), l+delta*dl = {}", i, j,
                    circles_x_bounds_[i][j].first,
                    circles_x_bounds_[i][j].second,
                    x_init_[0] + x_init_[1] * circle_dis_vec[j]);
        } else {
          LOG_DEBUG("circles_x_bounds_[{}][{}] ({}, {})", i, j,
                    circles_x_bounds_[i][j].first,
                    circles_x_bounds_[i][j].second);
        }
      }
    }
  }

  if (constraint_index != kNumConstraint) {
    LOG_ERROR("constraint_index != kNumConstraint");
    return;
  }

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    A_indptr.push_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      A_data.push_back(row_data_pair.second);
      A_indices.push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  A_indptr.push_back(ind_p);
}

OSQPSettings *PiecewiseJerkProblem::SolverDefaultSettings() {
  // Define Solver default settings_
  if (settings_) c_free(settings_);
  // OSQPSettings* settings_ =
  //     reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  settings_ = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  osqp_set_default_settings(settings_);
  settings_->polish = true;
  settings_->verbose = false;
  settings_->scaled_termination = true;
  return settings_;
}

}  // namespace planning
}  // namespace neodrive

#include "third_order_spline_path_mpc.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

double ThirdOrderSplinePathMPC::CalcGoalVarianceGain(
    const std::vector<double>& goal_path,
    const std::vector<double>& delta_s_vector, const double cur_speed) {
  // pre design
  std::vector<double> variance_ladder{0.3, 0.7, 1.2},
      gain_ladder{1.0, 15.0, 30.0, 50.0};
  double consider_time = 8.0;
  double consider_dis = std::max(car_length_ * 4, consider_time * cur_speed);

  // tools
  auto mean = [](std::vector<double> path) {
    double sum = 0;
    for (auto l : path) {
      sum += l;
    }
    return sum / path.size();
  };
  auto variance = [](std::vector<double> path, double mu) {
    double sum = 0;
    for (auto l : path) {
      sum += std::pow(l - mu, 2);
    }
    return sum / path.size();
  };
  auto bin_search = [](std::vector<double> variance_ladder, double variance_l) {
    if (variance_l < variance_ladder.front()) return 0;
    if (variance_l > variance_ladder.back())
      return (int)variance_ladder.size() - 1;
    int m = variance_ladder.size(), l = 0, r = m - 1;
    while (l < r) {
      int mid = l + (r - l) / 2;
      if (variance_ladder[mid] < variance_l + 1e-5) {
        l = mid + 1;
      } else {
        r = mid;
      }
    }
    return l;
  };

  // process
  std::vector<double> accum_s_vector{0.};
  double accum = 0.;
  for (int i = 0; i < delta_s_vector.size(); ++i) {
    accum += delta_s_vector[i];
    accum_s_vector.emplace_back(accum);
  }
  for (int i = 0; i < accum_s_vector.size(); i++) {
    LOG_DEBUG("accum_s_vector[{}] = {}", i, accum_s_vector[i]);
  }

  int consider_index = bin_search(accum_s_vector, consider_dis);
  std::vector<double> consider_goal_path(goal_path.begin(),
                                         goal_path.begin() + consider_index);
  LOG_INFO("consider_dis:{:.4f}, consider_goal_path_size:{}", consider_dis,
           consider_goal_path.size());

  double mean_l = mean(consider_goal_path),
         variance_l = variance(consider_goal_path, mean_l),
         gain = gain_ladder[bin_search(variance_ladder, variance_l)];
  LOG_INFO("consider_goal_path mean:{:.4f}, var:{:.4f}, gain:{:.4f}", mean_l,
           variance_l, gain);

  return gain;
};

ThirdOrderSplinePathMPC::ThirdOrderSplinePathMPC(
    const double car_length, const double cur_speed,
    const std::vector<double>& delta_s_vector,
    const ThirdOrderSplinePath::State& init_state,
    const ThirdOrderSplinePath::Control& init_control,
    const std::vector<double>& upper_l_0_bounds,
    const std::vector<double>& lower_l_0_bounds,
    const std::vector<double>& upper_l_1_bounds,
    const std::vector<double>& lower_l_1_bounds,
    const std::vector<double>& upper_l_2_bounds,
    const std::vector<double>& lower_l_2_bounds,
    const std::vector<double>& upper_dl_0_bounds,
    const std::vector<double>& lower_dl_0_bounds,
    const std::vector<double>& upper_ddl_0_bounds,
    const std::vector<double>& lower_ddl_0_bounds,
    const std::vector<double>& upper_dddl_0_bounds,
    const std::vector<double>& lower_dddl_0_bounds,
    const std::vector<double>& goal_l0, const std::vector<double>& goal_l1,
    const std::vector<double>& goal_l2)
    : car_length_(car_length),
      delta_s_vector_(delta_s_vector),
      init_state_(init_state),
      init_control_(init_control),
      upper_l_0_bounds_(upper_l_0_bounds),
      lower_l_0_bounds_(lower_l_0_bounds),
      upper_l_1_bounds_(upper_l_1_bounds),
      lower_l_1_bounds_(lower_l_1_bounds),
      upper_l_2_bounds_(upper_l_2_bounds),
      lower_l_2_bounds_(lower_l_2_bounds),
      upper_dl_0_bounds_(upper_dl_0_bounds),
      lower_dl_0_bounds_(lower_dl_0_bounds),
      upper_ddl_0_bounds_(upper_ddl_0_bounds),
      lower_ddl_0_bounds_(lower_ddl_0_bounds),
      upper_dddl_0_bounds_(upper_dddl_0_bounds),
      lower_dddl_0_bounds_(lower_dddl_0_bounds),
      goal_l0_(goal_l0),
      goal_l1_(goal_l1),
      goal_l2_(goal_l2) {
  n_ = delta_s_vector.size();
  nx_ = ThirdOrderSplinePath::nx;
  nu_ = ThirdOrderSplinePath::nu;
  npc_ = ThirdOrderSplinePath::npc;
  ns_ = ThirdOrderSplinePath::ns;

  path_model_ = std::make_shared<ThirdOrderSplinePathModel>(
      "third_order_spline_path_model", init_state, init_control, delta_s_vector,
      car_length_);
  mpc_solver_ = std::make_shared<ThirdOrderSplinePathHpipmSolver>(
      "third_order_spline_path_hpipm_solver", n_, nx_, nu_);

  gain_ = CalcGoalVarianceGain(goal_l0_, delta_s_vector, cur_speed);
}

ThirdOrderSplinePathMPC::~ThirdOrderSplinePathMPC() {
  delta_s_vector_.clear();
  upper_l_0_bounds_.clear();
  lower_l_0_bounds_.clear();
  upper_l_1_bounds_.clear();
  lower_l_1_bounds_.clear();
  upper_l_2_bounds_.clear();
  lower_l_2_bounds_.clear();
  goal_l0_.clear();
  goal_l1_.clear();
  goal_l2_.clear();
  mpc_solver_.reset();
  path_model_.reset();
}

bool ThirdOrderSplinePathMPC::Process() {
  LOG_INFO("start run third_order_spline_path_mpc: ");
  if (!path_model_->Process()) {
    LOG_ERROR("third_spline_path_model failed.");
    return false;
  }

  if (!SetMPCProblem()) {
    LOG_ERROR("set mpc problem failed.");
    return false;
  }

  int solver_status = -1;
  optimal_solution_ =
      mpc_solver_->SolveMPC(init_state_, stages_, &solver_status);
  LOG_INFO("third_spline_mpc_hpipm_solver status: {}", solver_status);

  return solver_status == 0;
}

bool ThirdOrderSplinePathMPC::SetMPCProblem() {
  stages_.resize(n_ + 1);
  for (int i = 0; i <= n_; ++i) {
    if (i == n_) {
      stages_[n_] = stages_[n_ - 1];
      continue;
    }
    if (!SetStage(init_state_, init_control_, i)) {
      LOG_ERROR("SetStage failed at index: {}", i);
      return false;
    }
  }
  return true;
}

bool ThirdOrderSplinePathMPC::SetStage(const ThirdOrderSplinePath::State& x_k,
                                       const ThirdOrderSplinePath::Control& u_k,
                                       const int step) {
  stages_[step].nx = nx_;
  stages_[step].nu = nu_;
  if (step == 0) {
    stages_[step].ng = 0;
    stages_[step].ns = 0;
  } else {
    stages_[step].ng = npc_;
    stages_[step].ns = ns_;
  }

  // 1. model
  stages_[step].line_model = path_model_->line_model_matrix()[step];

  // 2. cost
  const auto& config = config::PlanningConfig::Instance()
                           ->planning_research_config()
                           .third_order_spline_path_optimizer_config;
  // state
  ThirdOrderSplinePath::Q Q1 = ThirdOrderSplinePath::Q::Zero();
  ThirdOrderSplinePath::q q1 = ThirdOrderSplinePath::q::Zero();
  Q1(0, 0) = config.weight_l0;
  Q1(1, 1) = config.weight_dl;
  Q1(2, 2) = config.weight_ddl;
  q1(0, 0) = -config.weight_l0 * goal_l0_[step];
  ThirdOrderSplinePath::Q Q2 = ThirdOrderSplinePath::Q::Zero();
  ThirdOrderSplinePath::q q2 = ThirdOrderSplinePath::q::Zero();
  Q2(0, 0) = config.weight_l1 + config.weight_l2;
  Q2(0, 1) =
      (0.5 * car_length_) * config.weight_l1 + car_length_ * config.weight_l2;
  Q2(1, 0) =
      (0.5 * car_length_) * config.weight_l1 + car_length_ * config.weight_l2;
  Q2(1, 1) = (0.5 * car_length_) * (0.5 * car_length_) * config.weight_l1 +
             car_length_ * car_length_ * config.weight_l2;
  q2(0, 0) =
      -config.weight_l1 * goal_l1_[step] - config.weight_l2 * goal_l2_[step];
  q2(1, 0) = -(0.5 * car_length_) * config.weight_l1 * goal_l1_[step] -
             car_length_ * config.weight_l2 * goal_l2_[step];
  stages_[step].cost.Q = gain_ * (Q1 + Q2);
  stages_[step].cost.q = gain_ * (q1 + q2);

  // control
  stages_[step].cost.R = ThirdOrderSplinePath::R::Zero();
  stages_[step].cost.r = ThirdOrderSplinePath::r::Zero();
  stages_[step].cost.R(0, 0) = config.weight_dddl;
  stages_[step].cost.S = ThirdOrderSplinePath::S::Zero();
  // soft
  stages_[step].cost.Z(0, 0) = config.weight_slack_dl;
  stages_[step].cost.z(0, 0) = 0.0;
  stages_[step].cost.Z(1, 1) = config.weight_slack_ddl;
  stages_[step].cost.z(1, 0) = 0.0;
  stages_[step].cost.Z(2, 2) = config.weight_slack_l1;
  stages_[step].cost.z(2, 0) = 0.0;
  stages_[step].cost.Z(3, 3) = config.weight_slack_l2;
  stages_[step].cost.z(3, 0) = 0.0;

  // 3. polytopic constraints
  stages_[step].constraints.C = path_model_->polytopic_constraints()[step].C;
  stages_[step].constraints.D = path_model_->polytopic_constraints()[step].D;
  stages_[step].constraints.du << upper_dl_0_bounds_[step],
      upper_ddl_0_bounds_[step], upper_l_1_bounds_[step],
      upper_l_2_bounds_[step];
  stages_[step].constraints.dl << lower_dl_0_bounds_[step],
      lower_ddl_0_bounds_[step], lower_l_1_bounds_[step],
      lower_l_2_bounds_[step];

  // 4. box constraints
  stages_[step].upper_bounds_x << upper_l_0_bounds_[step],
      2 * ThirdOrderSplinePath::INF, 2 * ThirdOrderSplinePath::INF;
  stages_[step].lower_bounds_x << lower_l_0_bounds_[step],
      -2 * ThirdOrderSplinePath::INF, -2 * ThirdOrderSplinePath::INF;

  stages_[step].upper_bounds_u << upper_dddl_0_bounds_[step];
  stages_[step].lower_bounds_u << lower_dddl_0_bounds_[step];

  stages_[step].upper_bounds_s << 0.1, 0.1, 0.1, 0.1;
  stages_[step].lower_bounds_s << 0.1, 0.1, 0.1, 0.1;

  return true;
}

}  // namespace planning
}  // namespace neodrive

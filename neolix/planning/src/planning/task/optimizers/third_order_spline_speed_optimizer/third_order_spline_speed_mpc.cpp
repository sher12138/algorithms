#include "third_order_spline_speed_mpc.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

ThirdOrderSplineSpeedMPC::ThirdOrderSplineSpeedMPC(
    const double delta_t, const int n, const int n_x, const int n_u,
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
    const std::vector<STGoalAInfo>& goal_a)
    : delta_t_(delta_t),
      n_(n),
      nx_(n_x),
      nu_(n_u),
      npc_(n_p_c),
      ns_(n_s),
      init_state_(init_state),
      init_control_(init_control),
      speed_limit_(speed_limit),
      upper_s_bounds_(upper_s_bounds),
      lower_s_bounds_(lower_s_bounds),
      upper_v_bounds_(upper_v_bounds),
      lower_v_bounds_(lower_v_bounds),
      upper_a_bounds_(upper_a_bounds),
      lower_a_bounds_(lower_a_bounds),
      upper_jerk_bounds_(upper_jerk_bounds),
      lower_jerk_bounds_(lower_jerk_bounds),
      goal_upper_s_(goal_upper_s),
      goal_upper_v_(goal_upper_v),
      goal_lower_s_(goal_lower_s),
      goal_lower_v_(goal_lower_v),
      goal_a_(goal_a) {
  model_ = std::make_shared<ThirdOrderSplineSpeedModel>(
      "third_order_spline_model", init_state, init_control, delta_t);
  mpc_solver_ = std::make_shared<ThirdOrderSplineSpeedHpipmSolver>(
      "third_order_spline_mpc_hpipm_solver", n_, nx_, nu_);
  model_->Process();
}

ThirdOrderSplineSpeedMPC::~ThirdOrderSplineSpeedMPC() {
  upper_s_bounds_.clear();
  lower_s_bounds_.clear();
  upper_v_bounds_.clear();
  lower_v_bounds_.clear();
  upper_a_bounds_.clear();
  lower_a_bounds_.clear();
  upper_jerk_bounds_.clear();
  lower_jerk_bounds_.clear();
  goal_upper_s_.clear();
  goal_upper_v_.clear();
  goal_lower_s_.clear();
  goal_lower_v_.clear();
  goal_a_.clear();
  stages_.clear();
  optimal_solution_.clear();
  model_.reset();
  mpc_solver_.reset();
}

bool ThirdOrderSplineSpeedMPC::Process() {
  LOG_INFO("start run third_order_spline_mpc:");

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

bool ThirdOrderSplineSpeedMPC::RunMPC() { return true; }

bool ThirdOrderSplineSpeedMPC::SetMPCProblem() {
  stages_.resize(n_ + 1);
  double weight_s{}, weight_v{}, weight_a{};
  CalWeight(&weight_s, &weight_v, &weight_a);
  LOG_INFO("Check S weight: {}", weight_s);
  for (int i = 0; i <= n_; ++i) {
    if (!SetStage(init_state_, init_control_, i, weight_s, weight_v,
                  weight_a)) {
      LOG_ERROR("SetStage failed at index: {}", i);
      return false;
    }
  }
  return true;
}

bool ThirdOrderSplineSpeedMPC::SetStage(
    const ThirdOrderSplineSpeed::State& x_k,
    const ThirdOrderSplineSpeed::Control& u_k, const int step,
    const double weight_s, const double weight_v, const double weight_a) {
  // LOG_INFO("Check S weight: {}", weight_s);
  stages_[step].nx = nx_;
  stages_[step].nu = nu_;
  if (step == 0) {
    stages_[step].ng = 0;
    stages_[step].ns = 0;
  } else {
    stages_[step].ng = npc_;
    stages_[step].ns = (init_state_.state_v + 0.01 < speed_limit_) ? 0 : ns_;
  }

  // 1. model
  stages_[step].line_model = model_->line_model_matrix();

  // 2. cost
  auto config = config::PlanningConfig::Instance()
                    ->planning_research_config()
                    .third_order_spline_speed_optimizer_config;
  // s
  double s_upper_weight{0.}, s_lower_weight{0.};

  stages_[step].cost.Q(0, 0) = 1 * weight_s + 1 * weight_s;
  stages_[step].cost.q(0, 0) = -weight_s * goal_upper_s_[step].goal_t_s.s() +
                               -weight_s * goal_lower_s_[step].goal_t_s.s();
  LOG_DEBUG("step: goal_upper/lower_s {:.3f}, {:.3f}",
            goal_upper_s_[step].goal_t_s.s(), goal_lower_s_[step].goal_t_s.s());
  // v
  double v_upper_weight{0.}, v_lower_weight{0.};

  stages_[step].cost.Q(1, 1) = 1 * weight_v + 1 * weight_v;
  stages_[step].cost.q(1, 0) = -weight_v * goal_upper_v_[step].goal_t_v.s() +
                               -weight_v * goal_lower_v_[step].goal_t_v.s();
  LOG_DEBUG("step: goal_upper/lower_v {:.3f}, {:.3f}",
            goal_upper_v_[step].goal_t_v.s(), goal_lower_v_[step].goal_t_v.s());
  // a
  stages_[step].cost.Q(2, 2) = 0.5 * weight_a;
  stages_[step].cost.q(2, 0) = -0.5 * weight_a * goal_a_[step].goal_t_a.s();
  // jerk
  stages_[step].cost.R(0, 0) = 0.5 * config.weight_jerk;

  stages_[step].cost.r = ThirdOrderSplineSpeed::r::Zero();
  stages_[step].cost.S = ThirdOrderSplineSpeed::S::Zero();
  stages_[step].cost.Z << 0.5 * config.weight_slack *
                              ThirdOrderSplineSpeed::Z::Identity();
  stages_[step].cost.z << ThirdOrderSplineSpeed::z::Zero();

  // 3. polytopic constraints: v as the soft constraint
  stages_[step].constraints.C = model_->polytopic_constraints().C;
  stages_[step].constraints.D = model_->polytopic_constraints().D;
  double upper_v_bounds{0.0};
  stages_[step].constraints.du << std::fmax(upper_v_bounds_[step], 0);
  stages_[step].constraints.dl << lower_v_bounds_[step] - 0.01;

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
void ThirdOrderSplineSpeedMPC::CalWeight(double* s_weight, double* v_weight,
                                         double* a_weight) {
  const auto& speed_limits =
      DataCenter::Instance()->behavior_speed_limits().speed_limits();
  const auto& speed_limits_enable =
      DataCenter::Instance()->behavior_speed_limits().speed_limits_enable();
  std::unordered_set<neodrive::global::planning::SpeedLimit::SourceType>
      hard_speed_limit_acc_{
          neodrive::global::planning::SpeedLimit::VEHICLE_MEETING,
          neodrive::global::planning::SpeedLimit::COLLISION_RISK,
          neodrive::global::planning::SpeedLimit::MERGE_IN,
          neodrive::global::planning::SpeedLimit::OBS_TURN_RIGHT_RISK};
  bool has_hard_acc{false};

  for (std::size_t i = 0; i < speed_limits_enable.size(); ++i) {
    if (!speed_limits_enable[i]) {
      continue;
    }
    const auto& speed_limit = speed_limits[i];
    if (speed_limit.constraint_type() == SpeedLimitType::SOFT) {
      continue;
    }
    auto it = hard_speed_limit_acc_.find(speed_limit.source_type());
    if (it != hard_speed_limit_acc_.end()) {
      has_hard_acc = true;
      LOG_INFO("need change opt weight to hard ");
    }
  }
  auto config = config::PlanningConfig::Instance()
                    ->planning_research_config()
                    .third_order_spline_speed_optimizer_config;
  *s_weight = config.weight_cruise_s;
  *v_weight = config.weight_cruise_v;
  *a_weight = config.weight_a;
  if (has_hard_acc) {
    *s_weight = config.weight_hard_brake_s;
    *v_weight = config.weight_hard_brake_v;
    *a_weight = config.weight_hard_brake_a;
  }
}
}  // namespace planning
}  // namespace neodrive

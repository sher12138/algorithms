#include "speed_cilqr_model.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

SpeedCilqrModel::SpeedCilqrModel(const std::string& name,
                                 const double delta_t) {
  name_ = name;
  delta_t_ = delta_t;
  error_model_matrix_.A = CilqrModel::A::Zero();
  error_model_matrix_.B = CilqrModel::B::Zero();
  model_matrix_.A = CilqrModel::A::Zero();
  model_matrix_.B = CilqrModel::B::Zero();
  Process();
}

bool SpeedCilqrModel::Process() {
  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;
  // line model
  error_model_matrix_.A << 1.0, delta_t_, delta_t_ * delta_t_, 0.0, 1.0,
      delta_t_, 0.0, 0.0, 1.0;
  error_model_matrix_.B << 1.0 / 2.0 * delta_t_ * delta_t_ * delta_t_,
      delta_t_ * delta_t_, delta_t_;

  model_matrix_.A << 1, delta_t_, 1 / 2. * delta_t_ * delta_t_, 0, 1, delta_t_,
      0, 0, 1;
  model_matrix_.B << 1.0 / 6.0 * delta_t_ * delta_t_ * delta_t_,
      1.0 / 2.0 * delta_t_ * delta_t_, delta_t_;

  // cost weight matrix
  cost_para_.P_s << 1, 0, 0;
  cost_para_.P_v << 0, 1, 0;
  cost_para_.P_a << 0, 0, 1;
  cost_para_.state_cost_matrix.setZero();
  cost_para_.state_cost_matrix.diagonal() << speed_cilqr_conf.state_s_cost,
      speed_cilqr_conf.state_v_cost, speed_cilqr_conf.state_a_cost;

  return true;
}
}  // namespace planning
}  // namespace neodrive

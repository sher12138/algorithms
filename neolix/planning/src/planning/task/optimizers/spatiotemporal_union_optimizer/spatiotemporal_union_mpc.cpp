#include "spatiotemporal_union_mpc.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

SpatiotemporalUnionMPC::SpatiotemporalUnionMPC(
    const SpatiotemporalUnion::State &init_state, const double speed_limit,
    const SpatiotemporalUnion::TunnelInfos &tunnel_infos,
    const std::vector<double> corridor_flytime)
    : init_state_(init_state),
      speed_limit_(speed_limit),
      tunnel_infos_(tunnel_infos),
      corridor_flytime_(corridor_flytime) {
  n_stage_size_ = corridor_flytime.size();
  nx_ = SpatiotemporalUnion::nx;
  nu_ = SpatiotemporalUnion::nu;
  npc_ = SpatiotemporalUnion::npc;
  ns_ = SpatiotemporalUnion::ns;

  vehicle_model_ = std::make_shared<SpatiotemporalUnionModel>(
      "spatiotemporal_union_model", corridor_flytime_, tunnel_infos_,
      speed_limit_);
  vehicle_model_->Process();

  cost_ = vehicle_model_->cost_matrix();
  box_constraints_ = vehicle_model_->box_constraints();
  constraints_ = vehicle_model_->polytopic_constraints();
  line_model_vector_ = vehicle_model_->line_model_matrix();

  mpc_solver_ = std::make_shared<SpatiotemporalUnionHpipmSolver>(
      "spatiotemporal_union_hpipm_solver", n_stage_size_, nx_, nu_);
}

SpatiotemporalUnionMPC::~SpatiotemporalUnionMPC() {
  stages_.clear();
  optimal_solution_.clear();
  vehicle_model_.reset();
  mpc_solver_.reset();
  corridor_flytime_.clear();
  tunnel_infos_.s_boundary.clear();
  tunnel_infos_.s_boundary.clear();
  line_model_vector_.clear();
  constraints_.clear();
}

bool SpatiotemporalUnionMPC::Process() {
  LOG_INFO("start run spatiotemporal_union_mpc_optimize:");

  if (!SetMPCProblem()) {
    LOG_ERROR("set mpc problem failed.");
    return false;
  }

  int solver_status = -1;
  optimal_solution_ =
      mpc_solver_->SolveMPC(init_state_, stages_, &solver_status);
  LOG_INFO("spatiotemporal_union_mpc_hpipm_solver status: {}", solver_status);

  return solver_status == 0;
}

bool SpatiotemporalUnionMPC::SetMPCProblem() {
  stages_.resize(n_stage_size_ + 1);
  for (int i = 0; i <= n_stage_size_; ++i) {
    if (!SetStage(i)) {
      LOG_ERROR("SetStage failed at index: {}", i);
      return false;
    }
  }
  return true;
}

bool SpatiotemporalUnionMPC::SetStage(size_t step) {
  stages_[step].nx = nx_;
  stages_[step].nu = nu_;
  if (step == 0) {
    stages_[step].ng = 0;
    stages_[step].ns = 0;
  } else {
    stages_[step].ng = npc_;
    stages_[step].ns = ns_;
  }

  stages_[step].lower_bounds_x = box_constraints_.lx;
  stages_[step].upper_bounds_x = box_constraints_.ux;
  stages_[step].lower_bounds_u = box_constraints_.lu;
  stages_[step].upper_bounds_u = box_constraints_.uu;
  stages_[step].lower_bounds_s = -100 * Eigen::MatrixXd::Ones(ns_, 1);
  stages_[step].upper_bounds_s = 100 * Eigen::MatrixXd::Ones(ns_, 1);

  stages_[step].cost = cost_;

  stages_[step].constraints = CalConstraintsAtStep(step);

  stages_[step].line_model = CalLinemodelAtStep(step);

  return true;
}

SpatiotemporalUnion::PolytopicConstraints
SpatiotemporalUnionMPC::CalConstraintsAtStep(const std::size_t step) {
  SpatiotemporalUnion::PolytopicConstraints constraint{};
  constraint.C = constraints_[step].C;
  constraint.D = constraints_[step].D;
  constraint.du = constraints_[step].du;
  constraint.dl = constraints_[step].dl;

  return constraint;
}

SpatiotemporalUnion::LineModelMatrix SpatiotemporalUnionMPC::CalLinemodelAtStep(
    const std::size_t step) {
  SpatiotemporalUnion::LineModelMatrix line_model_in_step{};
  line_model_in_step.A = line_model_vector_[step].A;
  line_model_in_step.B = line_model_vector_[step].B;
  line_model_in_step.g = line_model_vector_[step].g;

  return line_model_in_step;
}

}  // namespace planning
}  // namespace neodrive

#include "third_order_spline_path_model.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

ThirdOrderSplinePathModel::ThirdOrderSplinePathModel(
    const std::string& name, const ThirdOrderSplinePath::State& init_state,
    const ThirdOrderSplinePath::Control& init_control,
    const std::vector<double>& delta_s_vector, const double length)
    : name_(name),
      init_state_(init_state),
      init_control_(init_control),
      delta_s_vector_(delta_s_vector),
      length_(length) {
  stages_size_ = delta_s_vector.size();
}

bool ThirdOrderSplinePathModel::Process() {
  line_model_matrix_.clear();
  polytopic_constraints_.clear();
  if (stages_size_ != delta_s_vector_.size()) {
    LOG_ERROR("stages_size != time_vector size, must be check model.");
    return false;
  }

  // time varying model and polytopic constraints
  line_model_matrix_.resize(stages_size_ + 1);
  polytopic_constraints_.resize(stages_size_ + 1);
  for (std::size_t i = 0; i < stages_size_; ++i) {
    // model
    double stage_delta_s = delta_s_vector_[i];
    ThirdOrderSplinePath::LinModelMatrix stage_model{};
    stage_model.A = ThirdOrderSplinePath::A::Zero();
    stage_model.B = ThirdOrderSplinePath::B::Zero();
    stage_model.g = ThirdOrderSplinePath::g::Zero();
    stage_model.A.row(0) << 1, stage_delta_s,
        1 / 2. * std::pow(stage_delta_s, 2);
    stage_model.A.row(1) << 0, 1, stage_delta_s;
    stage_model.A.row(2) << 0, 0, 1;
    stage_model.B << 1 / 6. * std::pow(stage_delta_s, 3),
        1 / 2. * std::pow(stage_delta_s, 2), stage_delta_s;
    line_model_matrix_[i] = stage_model;

    // polytopic constraints
    ThirdOrderSplinePath::PolytopicConstraints stage_polytopic_constraint{};
    stage_polytopic_constraint.C = ThirdOrderSplinePath::C::Zero();
    stage_polytopic_constraint.D = ThirdOrderSplinePath::D::Zero();
    stage_polytopic_constraint.dl = ThirdOrderSplinePath::d::Zero();
    stage_polytopic_constraint.du = ThirdOrderSplinePath::d::Zero();
    stage_polytopic_constraint.C.row(0) << 0, 1, 0;
    stage_polytopic_constraint.C.row(1) << 0, 0, 1;
    stage_polytopic_constraint.C.row(2) << 1, 1 / 2. * length_, 0;
    stage_polytopic_constraint.C.row(3) << 1, length_, 0;
    polytopic_constraints_[i] = stage_polytopic_constraint;
  }
  // last stage
  ThirdOrderSplinePath::LinModelMatrix last_stage_model{};
  last_stage_model.A = ThirdOrderSplinePath::A::Zero();
  last_stage_model.B = ThirdOrderSplinePath::B::Zero();
  last_stage_model.g = ThirdOrderSplinePath::g::Zero();
  last_stage_model.A = line_model_matrix_[stages_size_ - 1].A;
  last_stage_model.B = line_model_matrix_[stages_size_ - 1].B;
  line_model_matrix_[stages_size_] = last_stage_model;

  ThirdOrderSplinePath::PolytopicConstraints last_stage_polytopic_constraint{};
  last_stage_polytopic_constraint.C = ThirdOrderSplinePath::C::Zero();
  last_stage_polytopic_constraint.D = ThirdOrderSplinePath::D::Zero();
  last_stage_polytopic_constraint.dl = ThirdOrderSplinePath::d::Zero();
  last_stage_polytopic_constraint.du = ThirdOrderSplinePath::d::Zero();
  last_stage_polytopic_constraint.C.row(0) << 0, 1, 0;
  last_stage_polytopic_constraint.C.row(1) << 0, 0, 1;
  last_stage_polytopic_constraint.C.row(2) << 1, 1 / 2. * length_, 0;
  last_stage_polytopic_constraint.C.row(3) << 1, length_, 0;
  polytopic_constraints_[stages_size_] = last_stage_polytopic_constraint;

  return true;
}

}  // namespace planning
}  // namespace neodrive

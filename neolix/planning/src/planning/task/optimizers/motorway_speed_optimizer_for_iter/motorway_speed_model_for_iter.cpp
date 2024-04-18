#include "motorway_speed_model_for_iter.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

MotorwaySpeedModelForIter::MotorwaySpeedModelForIter(
    const std::string& name, const MotorwaySpeedForIter::State& init_state,
    const MotorwaySpeedForIter::Control& init_control, const double delta_t)
    : name_(name),
      init_state_(init_state),
      init_control_(init_control),
      delta_t_(delta_t) {
  line_model_matrix_.A = MotorwaySpeedForIter::A::Zero();
  line_model_matrix_.B = MotorwaySpeedForIter::B::Zero();
  line_model_matrix_.g = MotorwaySpeedForIter::g::Zero();
  polytopic_constraints_.C = MotorwaySpeedForIter::C::Zero();
  polytopic_constraints_.D = MotorwaySpeedForIter::D::Zero();
  polytopic_constraints_.dl = MotorwaySpeedForIter::d::Zero();
  polytopic_constraints_.du = MotorwaySpeedForIter::d::Zero();
}

bool MotorwaySpeedModelForIter::Process() {
  // line model
  line_model_matrix_.A.row(0) << 1, delta_t_, 1 / 2. * delta_t_ * delta_t_;
  line_model_matrix_.A.row(1) << 0, 1, delta_t_;
  line_model_matrix_.A.row(2) << 0, 0, 1;
  line_model_matrix_.B << 1. / 6 * delta_t_ * delta_t_ * delta_t_,
      1. / 2 * delta_t_ * delta_t_, delta_t_;

  polytopic_constraints_.C << 0, 1, 0;

  return true;
}

}  // namespace planning
}  // namespace neodrive
#include "third_order_spline_speed_model.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

ThirdOrderSplineSpeedModel::ThirdOrderSplineSpeedModel(
    const std::string& name, const ThirdOrderSplineSpeed::State& init_state,
    const ThirdOrderSplineSpeed::Control& init_control, const double delta_t)
    : name_(name),
      init_state_(init_state),
      init_control_(init_control),
      delta_t_(delta_t) {
  line_model_matrix_.A = ThirdOrderSplineSpeed::A::Zero();
  line_model_matrix_.B = ThirdOrderSplineSpeed::B::Zero();
  line_model_matrix_.g = ThirdOrderSplineSpeed::g::Zero();
  polytopic_constraints_.C = ThirdOrderSplineSpeed::C::Zero();
  polytopic_constraints_.D = ThirdOrderSplineSpeed::D::Zero();
  polytopic_constraints_.dl = ThirdOrderSplineSpeed::d::Zero();
  polytopic_constraints_.du = ThirdOrderSplineSpeed::d::Zero();
}

bool ThirdOrderSplineSpeedModel::Process() {
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
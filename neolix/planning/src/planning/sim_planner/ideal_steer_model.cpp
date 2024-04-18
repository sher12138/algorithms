#include "ideal_steer_model.h"

#include <boost/numeric/odeint.hpp>

namespace neodrive {
namespace planning {
namespace sim_planner {

namespace odeint = boost::numeric::odeint;

const double kBigEPS = 1e-1;

IdealSteerModel::IdealSteerModel(double wheelbase_len, double max_lon_acc,
                                 double max_lon_dec, double max_lon_acc_jerk,
                                 double max_lon_dec_jerk, double max_lat_acc,
                                 double max_lat_jerk, double max_steering_angle,
                                 double max_steer_rate, double max_curvature)
    : wheelbase_len_(wheelbase_len),
      max_lon_acc_(max_lon_acc),
      max_lon_dec_(max_lon_dec),
      max_lon_acc_jerk_(max_lon_acc_jerk),
      max_lon_dec_jerk_(max_lon_dec_jerk),
      max_lat_acc_(max_lat_acc),
      max_lat_jerk_(max_lat_jerk),
      max_steering_angle_(max_steering_angle),
      max_steer_rate_(max_steer_rate),
      max_curvature_(max_curvature) {
  updateInternalState();
}

IdealSteerModel::~IdealSteerModel() {}

void IdealSteerModel::truncateControl(const double &dt) {
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  double desired_lon_jerk = (desired_lon_acc_ - state_.acceleration) / dt;
  desired_lon_jerk =
      truncate(desired_lon_jerk, -max_lon_dec_jerk_, max_lon_acc_jerk_);
  desired_lon_acc_ = desired_lon_jerk * dt + state_.acceleration;
  desired_lon_acc_ = truncate(desired_lon_acc_, -max_lon_dec_, max_lon_acc_);
  control_.velocity = std::max(state_.velocity + desired_lon_acc_ * dt, 0.0);

  desired_lat_acc_ =
      pow(control_.velocity, 2) * (tan(control_.steer) / wheelbase_len_);
  double lat_acc_ori = pow(state_.velocity, 2) * state_.curvature;
  double lat_jerk_desired = (desired_lat_acc_ - lat_acc_ori) / dt;
  lat_jerk_desired = truncate(lat_jerk_desired, -max_lat_jerk_, max_lat_jerk_);
  desired_lat_acc_ = lat_jerk_desired * dt + lat_acc_ori;
  desired_lat_acc_ = truncate(desired_lat_acc_, -max_lat_acc_, max_lat_acc_);
  control_.steer = atan(desired_lat_acc_ * wheelbase_len_ /
                        std::max(pow(control_.velocity, 2), 0.1 * kBigEPS));
  desired_steer_rate_ = normalizeAngle(control_.steer - state_.steer) / dt;
  desired_steer_rate_ =
      truncate(desired_steer_rate_, -max_steer_rate_, max_steer_rate_);
  control_.steer = normalizeAngle(state_.steer + desired_steer_rate_ * dt);
}

void IdealSteerModel::Step(double dt) {
  max_steer_rate_ = 2.99;
  state_.steer = atan(state_.curvature * wheelbase_len_);
  updateInternalState();
  control_.velocity = std::max(0.0, control_.velocity);
  control_.steer =
      truncate(control_.steer, -max_steering_angle_, max_steering_angle_);
  truncateControl(dt);
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  desired_steer_rate_ = normalizeAngle(control_.steer - state_.steer) / dt;

  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  state_.vec_position = Vec2d(internal_state_[0], internal_state_[1]);
  state_.angle = normalizeAngle(internal_state_[2]);
  state_.velocity = internal_state_[3];
  state_.steer = normalizeAngle(internal_state_[4]);
  state_.curvature = tan(state_.steer) * 1.0 / wheelbase_len_;
  state_.acceleration = desired_lon_acc_;
  updateInternalState();
}

void IdealSteerModel::operator()(const InternalState &x, InternalState &dxdt,
                                 const double /* t */) {
  State cur_state;
  cur_state.vec_position = Vec2d(x[0], x[1]);
  cur_state.angle = x[2];
  cur_state.velocity = x[3];
  cur_state.steer = x[4];

  dxdt[0] = cos(cur_state.angle) * cur_state.velocity;
  dxdt[1] = sin(cur_state.angle) * cur_state.velocity;
  dxdt[2] = tan(cur_state.steer) * cur_state.velocity / wheelbase_len_;
  dxdt[3] = desired_lon_acc_;
  dxdt[4] = desired_steer_rate_;
}

void IdealSteerModel::setControl(const Control &control) { control_ = control; }

const IdealSteerModel::State &IdealSteerModel::state(void) const {
  return state_;
}

void IdealSteerModel::setState(const IdealSteerModel::State &state) {
  state_ = state;
  updateInternalState();
}

void IdealSteerModel::updateInternalState(void) {
  internal_state_[0] = state_.vec_position.x();
  internal_state_[1] = state_.vec_position.y();
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.velocity;
  internal_state_[4] = state_.steer;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

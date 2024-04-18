#include "idm_model.h"

#include <boost/numeric/odeint.hpp>
#include <ctime>

#include "src/planning/common/math/integral.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

namespace odeint = boost::numeric::odeint;

IntelligentDriverModel::IntelligentDriverModel() { updateInternalState(); }

IntelligentDriverModel::IntelligentDriverModel(const Param &parm)
    : param_(parm) {
  updateInternalState();
}

IntelligentDriverModel::~IntelligentDriverModel() {}

void IntelligentDriverModel::Step(double dt) {
  InternalState x_out;
  Linear(internal_state_, dt, &x_out);
  // odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  internal_state_[0] = x_out[0];
  internal_state_[1] = x_out[1];
  internal_state_[2] = x_out[2];
  internal_state_[3] = x_out[3];
  // state_.s = internal_state_[0];
  // state_.v = internal_state_[1];
  // state_.s_front = internal_state_[2];
  // state_.v_front = internal_state_[3];
  state_.s = x_out[0];
  state_.v = x_out[1];
  state_.s_front = x_out[2];
  state_.v_front = x_out[3];
  updateInternalState();
}

void IntelligentDriverModel::Linear(const InternalState &x, const double dt,
                                    InternalState *x_out) {
  State cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];

  double acc;
  neodrive::planning::sim_planner::control::IntelligentDriverModel::
      getIIdmDesiredAcceleration(param_, cur_state, &acc);
  acc = std::max(acc,
                 -std::min(param_.kHardBrakingDeceleration, cur_state.v / dt));

  (*x_out)[0] = x[0] + cur_state.v * dt + 0.5 * acc * dt * dt;
  (*x_out)[1] = cur_state.v + acc * dt;
  (*x_out)[2] = x[2] + x[3] * dt;
  (*x_out)[3] = x[3];
}

void IntelligentDriverModel::operator()(const InternalState &x,
                                        InternalState &dxdt, const double dt) {
  State cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];

  double acc;
  neodrive::planning::sim_planner::control::IntelligentDriverModel::
      getAccDesiredAcceleration(param_, cur_state, &acc);
  acc = std::max(acc,
                 -std::min(param_.kHardBrakingDeceleration, cur_state.v / dt));
  dxdt[0] = cur_state.v;
  dxdt[1] = acc;
  dxdt[2] = cur_state.v_front;
  dxdt[3] = 0.0;  // assume other vehicle keep the current velocity
}

const IntelligentDriverModel::State &IntelligentDriverModel::state(void) const {
  return state_;
}

void IntelligentDriverModel::setState(
    const IntelligentDriverModel::State &state) {
  state_ = state;
  updateInternalState();
}

void IntelligentDriverModel::updateInternalState(void) {
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

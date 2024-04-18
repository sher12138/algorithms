#include "ctx_idm_model.h"

#include <boost/numeric/odeint.hpp>

#include "calculations.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

namespace odeint = boost::numeric::odeint;

ContextIntelligentDriverModel::ContextIntelligentDriverModel() {
  updateInternalState();
}

ContextIntelligentDriverModel::ContextIntelligentDriverModel(
    const IdmParam &idm_parm, const CtxParam &ctx_param)
    : idm_param_(idm_parm), ctx_param_(ctx_param) {
  updateInternalState();
}

ContextIntelligentDriverModel::~ContextIntelligentDriverModel() {}

void ContextIntelligentDriverModel::Step(double dt) {
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  state_.s = internal_state_[0];
  state_.v = internal_state_[1];
  state_.s_front = internal_state_[2];
  state_.v_front = internal_state_[3];
  state_.s_target = internal_state_[4];
  state_.v_target = internal_state_[5];
  updateInternalState();
}

void ContextIntelligentDriverModel::operator()(const InternalState &x,
                                               InternalState &dxdt,
                                               const double dt) {
  CtxIdmState cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];
  cur_state.s_target = x[4];
  cur_state.v_target = x[5];

  IdmState idm_state;

  double acc_idm;
  control::IntelligentDriverModel::getAccDesiredAcceleration(
      idm_param_, idm_state, &acc_idm);
  acc_idm = std::max(acc_idm, -std::min(idm_param_.kHardBrakingDeceleration,
                                        cur_state.v / dt));

  double v_ref =
      cur_state.v_target + ctx_param_.k_s * (cur_state.s_target - cur_state.s);
  double acc_track = ctx_param_.k_v * (v_ref - cur_state.v);

  acc_track = std::min(std::max(acc_track, -1.0), 1.0);

  double acc = acc_track;

  dxdt[0] = cur_state.v;
  dxdt[1] = acc;
  dxdt[2] = cur_state.v_front;
  dxdt[3] = 0.0;  // assume other vehicle keep the current velocity
  dxdt[4] = cur_state.v_target;
  dxdt[5] = 0.0;  // assume constant velocity for target state
}

const ContextIntelligentDriverModel::CtxIdmState &
ContextIntelligentDriverModel::state(void) const {
  return state_;
}

void ContextIntelligentDriverModel::setState(
    const ContextIntelligentDriverModel::CtxIdmState &state) {
  state_ = state;
  updateInternalState();
}

void ContextIntelligentDriverModel::updateInternalState(void) {
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
  internal_state_[4] = state_.s_target;
  internal_state_[5] = state_.v_target;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

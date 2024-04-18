#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include "calculations.h"
#include "intelligent_driver_model.h"
#include "state.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

class ContextIntelligentDriverModel {
 public:
  using IdmParam = control::IntelligentDriverModel::Param;
  using IdmState = control::IntelligentDriverModel::State;

  struct CtxParam {
    double k_s = 0.5;
    double k_v = 2.0 * k_s;

    CtxParam() = default;
    CtxParam(const double &_k_s, const double &_k_v) : k_s(_k_s), k_v(_k_v) {}
  };

  struct CtxIdmState {
    double s{0.0};        // longitudinal distance
    double v{0.0};        // longitudinal speed
    double s_front{0.0};  // leading vehicle
    double v_front{0.0};
    double s_target{0.0};
    double v_target{0.0};
  };

  ContextIntelligentDriverModel();

  ContextIntelligentDriverModel(const IdmParam &idm_parm,
                                const CtxParam &ctx_param);

  ~ContextIntelligentDriverModel();

  const CtxIdmState &state(void) const;

  void setState(const CtxIdmState &state);

  void Step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 6> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  void updateInternalState(void);

  InternalState internal_state_;

  IdmParam idm_param_;
  CtxParam ctx_param_;

  CtxIdmState state_;
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

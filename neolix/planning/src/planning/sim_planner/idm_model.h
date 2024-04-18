#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include "calculations.h"
#include "intelligent_driver_model.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

class IntelligentDriverModel {
 public:
  using Param = control::IntelligentDriverModel::Param;
  using State = control::IntelligentDriverModel::State;

  IntelligentDriverModel();

  IntelligentDriverModel(const Param &parm);

  ~IntelligentDriverModel();

  const State &state(void) const;

  void setState(const State &state);

  void Step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 4> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  void updateInternalState(void);

  void Linear(const InternalState &x, const double dt, InternalState *x_out);

  InternalState internal_state_;

  Param param_;
  State state_;
};
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

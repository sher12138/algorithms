#pragma once

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include "calculations.h"
#include "state.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

class IdealSteerModel {
 public:
  using State = neodrive::planning::sim_planner::State;
  struct Control {
    double steer{0.0};     // steer
    double velocity{0.0};  // body velocity
    Control() {}
    Control(const double s, const double v) : steer(s), velocity(v) {}
  };

  IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec,
                  double max_lon_acc_jerk, double max_lon_dec_jerk,
                  double max_lat_acc, double max_lat_jerk,
                  double max_steering_angle, double max_steer_rate,
                  double max_curvature);

  ~IdealSteerModel();

  const State &state(void) const;

  void setState(const State &state);

  void setControl(const Control &control);

  void Step(double dt);

  void truncateControl(const double &dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 5> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  void updateInternalState(void);
  State state_;
  Control control_;
  double desired_steer_rate_;
  double desired_lon_acc_;
  double desired_lat_acc_;
  InternalState internal_state_;
  double wheelbase_len_;
  double max_lon_acc_;
  double max_lon_dec_;
  double max_lon_acc_jerk_;
  double max_lon_dec_jerk_;
  double max_lat_acc_;
  double max_lat_jerk_;
  double max_steering_angle_;
  double max_steer_rate_;
  double max_curvature_;
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

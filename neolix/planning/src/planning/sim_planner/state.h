#pragma once

#include <math.h>

#include <Eigen/Eigen>

#include "sim_map.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

struct State {
  double time_stamp{0.0};
  Vec2d vec_position = Vec2d(0.0, 0.0);
  double angle{0.0};
  double curvature{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double steer{0.0};
};

class VehicleParam {
 public:
  inline double width() const { return width_; }
  inline double length() const { return length_; }
  inline double wheel_base() const { return wheel_base_; }
  inline double front_suspension() const { return front_suspension_; }
  inline double rear_suspension() const { return rear_suspension_; }
  inline double max_steering_angle() const { return max_steering_angle_; }
  inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
  inline double max_lateral_acc() const { return max_lateral_acc_; }
  inline double d_cr() const { return d_cr_; }

  inline void set_width(const double val) { width_ = val; }
  inline void set_length(const double val) { length_ = val; }
  inline void set_wheel_base(const double val) { wheel_base_ = val; }
  inline void set_front_suspension(const double val) {
    front_suspension_ = val;
  }
  inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
  inline void set_max_steering_angle(const double val) {
    max_steering_angle_ = val;
  }
  inline void set_max_longitudinal_acc(const double val) {
    max_longitudinal_acc_ = val;
  }
  inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
  inline void set_d_cr(const double val) { d_cr_ = val; }

 private:
  double width_ = 1.091;
  double length_ = 2.69;
  double wheel_base_ = 1.73;
  double front_suspension_ = 2.29;
  double rear_suspension_ = 0.4;
  double max_steering_angle_ = 6.6323;

  double max_longitudinal_acc_ = 2.0;
  double max_lateral_acc_ = 0.5;

  double d_cr_ = 0.945;  // length between geometry center and rear axle
};

class Vehicle {
 public:
  Vehicle() = default;
  Vehicle(const VehicleParam &param, const State &state)
      : id_(-1), param_(param), state_(state) {}
  Vehicle(const int &id, const VehicleParam &param, const State &state)
      : id_(id), param_(param), state_(state) {}

  inline int id() const { return id_; }
  inline VehicleParam param() const { return param_; }
  inline State state() const { return state_; }

  inline void set_id(const int &id) { id_ = id; }
  inline void set_param(const VehicleParam &in) { param_ = in; }
  inline void setState(const State &in) { state_ = in; }

 private:
  int id_{-1};
  VehicleParam param_;
  State state_;
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

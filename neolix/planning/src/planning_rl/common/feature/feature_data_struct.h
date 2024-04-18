#pragma once

namespace neodrive {
namespace planning_rl {

struct PredState {
 public:
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double acc = 0.0;
  double curvature = 0.0;

  PredState() {}
};

struct TrajPoint {
 public:
  int arg = 0;
  double x = 0.0;
  double x_der = 0.0;
  double x_dder = 0.0;
  double x_ddder = 0.0;
  TrajPoint() {}
};

// class CurvePoint
// {
// public:
//     double s = 0.0;
//     double x = 0.0;
//     double y = 0.0;
//     double theta = 0.0;
//     double kappa = 0.0;
//     double kappa_prime = 0.0;
//     double v = 0.0;
//     double t = 0.0;
//     CurvePoint() {}
// };

struct RefPoint {
 public:
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double theta = 0.0;
  double curvature = 0.0;
  RefPoint() {}
};

}  // namespace planning_rl
}  // namespace neodrive
#pragma once

#include <Eigen/Dense>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "src/planning/common/planning_macros.h"

namespace neodrive {
namespace planning {

namespace ThirdOrderSplineSpeed {
constexpr int nx = 3;   // state: [s, v, a]'T
constexpr int nu = 1;   // control: [jerk]
constexpr int nd = 0;   // disturb: none
constexpr int no = 3;   // observe: [s, v, a]'T
constexpr int npc = 1;  // polytopic constraints: v
constexpr int ns = 1;   // soft polytopic constraints: v

constexpr double INF = 10000.;

using StateVector = Eigen::Matrix<double, nx, 1>;
using ControlVector = Eigen::Matrix<double, nu, 1>;
using DisturbVector = Eigen::Matrix<double, nd, 1>;
using ObserveVector = Eigen::Matrix<double, no, 1>;

using A = Eigen::Matrix<double, nx, nx>;
using B = Eigen::Matrix<double, nx, nu>;
using g = Eigen::Matrix<double, nx, 1>;
using Q = Eigen::Matrix<double, nx, nx>;
using R = Eigen::Matrix<double, nu, nu>;
using S = Eigen::Matrix<double, nx, nu>;
using q = Eigen::Matrix<double, nx, 1>;
using r = Eigen::Matrix<double, nu, 1>;
using C = Eigen::Matrix<double, npc, nx>;
using D = Eigen::Matrix<double, npc, nu>;
using d = Eigen::Matrix<double, npc, 1>;
using Z = Eigen::Matrix<double, ns, ns>;
using z = Eigen::Matrix<double, ns, 1>;

using Bounds_x = Eigen::Matrix<double, nx, 1>;
using Bounds_u = Eigen::Matrix<double, nu, 1>;
using Bounds_s = Eigen::Matrix<double, ns, 1>;

struct State {
  double state_s{0.};
  double state_v{0.};
  double state_a{0.};
  void SetZero() {
    state_s = 0.;
    state_v = 0.;
    state_a = 0.;
  }
};
struct Control {
  double control_jerk{0.};
  void SetZero() { control_jerk = 0.; }
};
struct Disturb {};
struct Observe {
  double observe_s{0.};
  double observe_v{0.};
  double observe_a{0.};
};

struct LinModelMatrix {
  ThirdOrderSplineSpeed::A A{};
  ThirdOrderSplineSpeed::B B{};
  ThirdOrderSplineSpeed::g g{};
};
struct CostMatrix {
  ThirdOrderSplineSpeed::Q Q{};
  ThirdOrderSplineSpeed::R R{};
  ThirdOrderSplineSpeed::S S{};
  ThirdOrderSplineSpeed::q q{};
  ThirdOrderSplineSpeed::r r{};
  ThirdOrderSplineSpeed::Z Z{};
  ThirdOrderSplineSpeed::z z{};
};
struct PolytopicConstraints {
  ThirdOrderSplineSpeed::C C{};
  ThirdOrderSplineSpeed::D D{};
  ThirdOrderSplineSpeed::d dl{};
  ThirdOrderSplineSpeed::d du{};
};
struct BoxConstraints {
  ThirdOrderSplineSpeed::Bounds_x ux{};
  ThirdOrderSplineSpeed::Bounds_x lx{};
  ThirdOrderSplineSpeed::Bounds_u uu{};
  ThirdOrderSplineSpeed::Bounds_u lu{};
  ThirdOrderSplineSpeed::Bounds_s us{};
  ThirdOrderSplineSpeed::Bounds_s ls{};
};

struct OptVariables {
  State xk{};
  Control uk{};
};
struct Stage {
  LinModelMatrix line_model{};
  CostMatrix cost{};
  PolytopicConstraints constraints{};

  Bounds_x upper_bounds_x{};
  Bounds_x lower_bounds_x{};

  Bounds_u upper_bounds_u{};
  Bounds_u lower_bounds_u{};

  Bounds_s upper_bounds_s{};
  Bounds_s lower_bounds_s{};

  // nx    -> number of states
  // nu    -> number of inputs
  // nbx   -> number of bounds on x
  // nbu   -> number of bounds on u
  // ng    -> number of polytopic constratins
  // ns   -> number of soft constraints
  int nx{}, nu{}, nbx{}, nbu{}, ng{}, ns{};
};
struct HpipmBound {
  std::vector<int> idx_u{};
  std::vector<int> idx_x{};
  std::vector<int> idx_s{};
  std::vector<double> lower_bounds_u{};
  std::vector<double> upper_bounds_u{};
  std::vector<double> lower_bounds_x{};
  std::vector<double> upper_bounds_x{};
};

static StateVector StateToVector(const State &x) {
  return {x.state_s, x.state_v, x.state_a};
}
static ControlVector ControlToVector(const Control &u) {
  ControlVector uk;
  uk(0) = u.control_jerk;
  return uk;
}
static ObserveVector ObserveToVector(const Observe &o) {
  return {o.observe_s, o.observe_v, o.observe_a};
}
static State VectorToState(const StateVector &x) { return {x(0), x(1), x(2)}; }
static Control VectorToControl(const ControlVector &u) { return {u(0)}; }
static Observe VectorToObserve(const ObserveVector &o) {
  return {o(0), o(1), o(2)};
}
static State ArrayToState(double *x) { return {x[0], x[1], x[2]}; }
static Control ArrayToControl(double *u) { return {u[0]}; }
static Observe ArrayToObserve(double *o) { return {o[0], o[1], o[2]}; }

}  // namespace ThirdOrderSplineSpeed

class ThirdOrderSplineSpeedModel {
 public:
  ThirdOrderSplineSpeedModel(const std::string &name,
                             const ThirdOrderSplineSpeed::State &init_state,
                             const ThirdOrderSplineSpeed::Control &init_control,
                             const double delta_t);

  ~ThirdOrderSplineSpeedModel() = default;

  bool Process();

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::string, name)
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, delta_t)
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      ThirdOrderSplineSpeed::LinModelMatrix, line_model_matrix)
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      ThirdOrderSplineSpeed::PolytopicConstraints, polytopic_constraints)

 private:
  std::string name_{""};
  ThirdOrderSplineSpeed::State init_state_{};
  ThirdOrderSplineSpeed::Control init_control_{};
  double delta_t_{0.1};

  ThirdOrderSplineSpeed::LinModelMatrix line_model_matrix_{};
  ThirdOrderSplineSpeed::PolytopicConstraints polytopic_constraints_{};
};

}  // namespace planning
}  // namespace neodrive

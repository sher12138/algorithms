#pragma once

#include <Eigen/Dense>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "src/planning/common/planning_macros.h"

namespace neodrive {
namespace planning {

namespace ThirdOrderSplinePath {
constexpr int nx = 3;   // state: [l0, dl, ddl]'T
constexpr int nu = 1;   // control: [dddl]
constexpr int nd = 0;   // disturb: none
constexpr int no = 3;   // observe: [l0, dl, ddl]'T
constexpr int npc = 4;  // polytopic constraints: dl, ddl, l1, l2
constexpr int ns = 4;   // soft polytopic constraints: dl, ddl, l1, l2

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
  double state_l0{0.};
  double state_dl{0.};
  double state_ddl{0.};
  void SetZero() {
    state_l0 = 0.;
    state_dl = 0.;
    state_ddl = 0.;
  }
};
struct Control {
  double control_dddl{0.};
  void SetZero() { control_dddl = 0.; }
};
struct Disturb {};
struct Observe {
  double observe_l0{0.};
  double observe_dl{0.};
  double observe_ddl{0.};
};

struct LinModelMatrix {
  ThirdOrderSplinePath::A A{};
  ThirdOrderSplinePath::B B{};
  ThirdOrderSplinePath::g g{};
};
struct CostMatrix {
  ThirdOrderSplinePath::Q Q{};
  ThirdOrderSplinePath::R R{};
  ThirdOrderSplinePath::S S{};
  ThirdOrderSplinePath::q q{};
  ThirdOrderSplinePath::r r{};
  ThirdOrderSplinePath::Z Z{};
  ThirdOrderSplinePath::z z{};
};
struct PolytopicConstraints {
  ThirdOrderSplinePath::C C{};
  ThirdOrderSplinePath::D D{};
  ThirdOrderSplinePath::d dl{};
  ThirdOrderSplinePath::d du{};
};
struct BoxConstraints {
  ThirdOrderSplinePath::Bounds_x ux{};
  ThirdOrderSplinePath::Bounds_x lx{};
  ThirdOrderSplinePath::Bounds_u uu{};
  ThirdOrderSplinePath::Bounds_u lu{};
  ThirdOrderSplinePath::Bounds_s us{};
  ThirdOrderSplinePath::Bounds_s ls{};
};
struct SoftConstraints {
  ThirdOrderSplinePath::Bounds_s Zx{};
  ThirdOrderSplinePath::Bounds_s Zu{};
  ThirdOrderSplinePath::Bounds_s zx{};
  ThirdOrderSplinePath::Bounds_s zu{};
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
  std::vector<double> lower_bounds_s{};
  std::vector<double> upper_bounds_s{};
};

static StateVector StateToVector(const State &x) {
  StateVector state_vector{};
  state_vector(0) = x.state_l0;
  state_vector(1) = x.state_dl;
  state_vector(2) = x.state_ddl;
  return state_vector;
}
static ControlVector ControlToVector(const Control &u) {
  ControlVector uk;
  uk(0) = u.control_dddl;
  return uk;
}
static ObserveVector ObserveToVector(const Observe &o) {
  ObserveVector observe_vector{};
  observe_vector(0) = o.observe_l0;
  observe_vector(1) = o.observe_dl;
  observe_vector(2) = o.observe_ddl;
  return observe_vector;
}
static State VectorToState(const StateVector &x) { return {x(0), x(1), x(2)}; }
static Control VectorToControl(const ControlVector &u) { return {u(0)}; }
static Observe VectorToObserve(const ObserveVector &o) {
  return {o(0), o(1), o(2)};
}
static State ArrayToState(double *x) { return {x[0], x[1], x[2]}; }
static Control ArrayToControl(double *u) { return {u[0]}; }
static Observe ArrayToObserve(double *o) { return {o[0], o[1], o[2]}; }

}  // namespace ThirdOrderSplinePath

class ThirdOrderSplinePathModel {
 public:
  ThirdOrderSplinePathModel(const std::string &name,
                            const ThirdOrderSplinePath::State &init_state,
                            const ThirdOrderSplinePath::Control &init_control,
                            const std::vector<double> &delta_s_vector,
                            const double length);

  ~ThirdOrderSplinePathModel() = default;

  bool Process();

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::string, name)
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::size_t, stages_size)
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<double>,
                                             delta_s_vector)
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<ThirdOrderSplinePath::LinModelMatrix>, line_model_matrix)
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<ThirdOrderSplinePath::PolytopicConstraints>,
      polytopic_constraints)

 private:
  std::string name_{""};
  std::size_t stages_size_{};
  ThirdOrderSplinePath::State init_state_{};
  ThirdOrderSplinePath::Control init_control_{};
  std::vector<double> delta_s_vector_{};
  double length_{};

  std::vector<ThirdOrderSplinePath::LinModelMatrix> line_model_matrix_{};
  std::vector<ThirdOrderSplinePath::PolytopicConstraints>
      polytopic_constraints_{};
};

}  // namespace planning
}  // namespace neodrive
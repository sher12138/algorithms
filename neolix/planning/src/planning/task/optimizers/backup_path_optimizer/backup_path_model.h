#pragma once

#include <Eigen/Dense>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "src/planning/common/planning_macros.h"

namespace neodrive {
namespace planning {

namespace BackupPath {
constexpr int nx = 3;   // state: [l, dl, ddl]'T
constexpr int nu = 1;   // control: [dddl]
constexpr int nd = 0;   // disturb: none
constexpr int no = 3;   // observe: [l, dl, ddl]'T
constexpr int npc = 0;  // polytopic constraints: none
constexpr int ns = 0;   // soft polytopic constraints: none

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
  double state_l{0.};
  double state_dl{0.};
  double state_ddl{0.};
  void SetZero() {
    state_l = 0.;
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
  double observe_l{0.};
  double observe_dl{0.};
  double observe_ddl{0.};
};

struct LinModelMatrix {
  BackupPath::A A{};
  BackupPath::B B{};
  BackupPath::g g{};
};
struct CostMatrix {
  BackupPath::Q Q{};
  BackupPath::R R{};
  BackupPath::S S{};
  BackupPath::q q{};
  BackupPath::r r{};
  BackupPath::Z Z{};
  BackupPath::z z{};
};
struct PolytopicConstraints {
  BackupPath::C C{};
  BackupPath::D D{};
  BackupPath::d dl{};
  BackupPath::d du{};
};
struct BoxConstraints {
  BackupPath::Bounds_x ux{};
  BackupPath::Bounds_x lx{};
  BackupPath::Bounds_u uu{};
  BackupPath::Bounds_u lu{};
  BackupPath::Bounds_s us{};
  BackupPath::Bounds_s ls{};
};
struct SoftConstraints {
  BackupPath::Bounds_s Zx{};
  BackupPath::Bounds_s Zu{};
  BackupPath::Bounds_s zx{};
  BackupPath::Bounds_s zu{};
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
  return {x.state_l, x.state_dl, x.state_ddl};
}
static ControlVector ControlToVector(const Control &u) {
  ControlVector uk;
  uk(0) = u.control_dddl;
  return uk;
}
static ObserveVector ObserveToVector(const Observe &o) {
  return {o.observe_l, o.observe_dl, o.observe_ddl};
}
static State VectorToState(const StateVector &x) { return {x(0), x(1), x(2)}; }
static Control VectorToControl(const ControlVector &u) { return {u(0)}; }
static Observe VectorToObserve(const ObserveVector &o) {
  return {o(0), o(1), o(2)};
}
static State ArrayToState(double *x) { return {x[0], x[1], x[2]}; }
static Control ArrayToControl(double *u) { return {u[0]}; }
static Observe ArrayToObserve(double *o) { return {o[0], o[1], o[2]}; }

}  // namespace BackupPath

class BackupPathModel {
 public:
  BackupPathModel(const std::string &name, const BackupPath::State &init_state,
                  const BackupPath::Control &init_control,
                  const double delta_s);

  ~BackupPathModel() = default;

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(BackupPath::LinModelMatrix,
                                             line_model_matrix);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(BackupPath::PolytopicConstraints,
                                             polytopic_constraint);

 private:
  std::string name_{""};
  BackupPath::State init_state_{};
  BackupPath::Control init_control_{};
  double delta_s_{};

  BackupPath::LinModelMatrix line_model_matrix_{};
  BackupPath::PolytopicConstraints polytopic_constraint_{};
};

}  // namespace planning
}  // namespace neodrive
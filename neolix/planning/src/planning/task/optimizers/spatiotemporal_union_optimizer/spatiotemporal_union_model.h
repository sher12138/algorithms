#pragma once

#include <Eigen/Dense>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "src/planning/common/planning_macros.h"

namespace neodrive {
namespace planning {

namespace SpatiotemporalUnion {
constexpr int nx = 8;    // state: 2*[p, v, a,j]'T
constexpr int nu = 2;    // control: 2*[jerk_d]
constexpr int nd = 0;    // disturb: none
constexpr int no = 8;    // observe: 2*[p, v, a,j]'T
constexpr int npc = 28;  // polytopic constraints
constexpr int ns = 0;    // soft polytopic constraints is none

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
using dl = Eigen::Matrix<double, npc, 1>;
using du = Eigen::Matrix<double, npc, 1>;

using Bounds_x = Eigen::Matrix<double, nx, 1>;
using Bounds_u = Eigen::Matrix<double, nu, 1>;
using Bounds_s = Eigen::Matrix<double, ns, 1>;

struct TunnelInfos {
  std::vector<std::pair<double, double>> s_boundary{};
  std::vector<std::pair<double, double>> l_boundary{};
  // std::vector<std::pair<double, double>> left_boundary{};
  // std::vector<std::pair<double, double>> center_line{};
  // std::vector<std::pair<double, double>> righe_boundary{};
};

struct State {
  double state_p_s{0.};
  double state_v_s{0.};
  double state_a_s{0.};
  double state_j_s{0.};
  double state_p_l{0.};
  double state_v_l{0.};
  double state_a_l{0.};
  double state_j_l{0.};
  void SetZero() {
    state_p_s = 0.;
    state_v_s = 0.;
    state_a_s = 0.;
    state_j_s = 0.;
    state_p_l = 0.;
    state_v_l = 0.;
    state_a_l = 0.;
    state_j_l = 0.;
  }
};
struct Control {
  double control_jerk_d_s{0.};
  double control_jerk_d_l{0.};
  void SetZero() {
    control_jerk_d_s = 0.;
    control_jerk_d_l = 0.;
  }
};
struct Disturb {};
struct Observe {
  double observe_p_s{0.};
  double observe_v_s{0.};
  double observe_a_s{0.};
  double observe_j_s{0.};
  double observe_p_l{0.};
  double observe_v_l{0.};
  double observe_a_l{0.};
  double observe_j_l{0.};
};
struct SLTPoint {
  double t{0.};
  double s{0.};
  double v_s{0.};
  double a_s{0.};
  double l{0.};
  double v_l{0.};
  double a_l{0.};
};

struct LineModelMatrix {
  SpatiotemporalUnion::A A{};
  SpatiotemporalUnion::B B{};
  SpatiotemporalUnion::g g{};
};
struct CostMatrix {
  SpatiotemporalUnion::Q Q{};
  SpatiotemporalUnion::R R{};
  SpatiotemporalUnion::S S{};
  SpatiotemporalUnion::q q{};
  SpatiotemporalUnion::r r{};
  SpatiotemporalUnion::Z Z{};
  SpatiotemporalUnion::z z{};
};
struct PolytopicConstraints {
  SpatiotemporalUnion::C C{};
  SpatiotemporalUnion::D D{};
  SpatiotemporalUnion::d dl{};
  SpatiotemporalUnion::d du{};
};
struct BoxConstraints {
  SpatiotemporalUnion::Bounds_x ux{};
  SpatiotemporalUnion::Bounds_x lx{};
  SpatiotemporalUnion::Bounds_u uu{};
  SpatiotemporalUnion::Bounds_u lu{};
  SpatiotemporalUnion::Bounds_s us{};
  SpatiotemporalUnion::Bounds_s ls{};
};

struct OptVariables {
  State xk{};
  Control uk{};
};
struct Stage {
  LineModelMatrix line_model{};
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
  StateVector xk;
  xk(0) = x.state_p_s;
  xk(1) = x.state_v_s;
  xk(2) = x.state_a_s;
  xk(3) = x.state_j_s;
  xk(4) = x.state_p_l;
  xk(5) = x.state_v_l;
  xk(6) = x.state_a_l;
  xk(7) = x.state_j_l;
  return xk;
  // return {x.state_p_s, x.state_v_s, x.state_a_s, x.state_j_s,
  //         x.state_p_l, x.state_v_l, x.state_a_l, x.state_j_l};
}
static ControlVector ControlToVector(const Control &u) {
  ControlVector uk;
  uk(0) = u.control_jerk_d_s;
  uk(1) = u.control_jerk_d_l;
  return uk;
}

static ObserveVector ObserveToVector(const Observe &o) {
  ObserveVector ok;
  ok(0) = o.observe_p_s;
  ok(1) = o.observe_v_s;
  ok(2) = o.observe_a_s;
  ok(3) = o.observe_j_s;
  ok(4) = o.observe_p_l;
  ok(5) = o.observe_v_l;
  ok(6) = o.observe_a_l;
  ok(7) = o.observe_j_l;
  return ok;
  // return {o.observe_p_s, o.observe_v_s, o.observe_a_s, o.observe_j_s,
  //         o.observe_p_l, o.observe_v_l, o.observe_a_l, o.observe_j_l};
}
static State VectorToState(const StateVector &x) {
  return {x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7)};
}
static Control VectorToControl(const ControlVector &u) { return {u(0), u(1)}; }
static Observe VectorToObserve(const ObserveVector &o) {
  return {o(0), o(1), o(2), o(3), o(4), o(5), o(6), o(7)};
}
static State ArrayToState(double *x) {
  return {x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]};
}
static Control ArrayToControl(double *u) { return {u[0], u[1]}; }
static Observe ArrayToObserve(double *o) {
  return {o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7]};
}

}  // namespace SpatiotemporalUnion

namespace SpatiotemporalUnionConfig {

class SpatiotemporalUnionModelConfig {
 public:
  SpatiotemporalUnionModelConfig() = default;
  ~SpatiotemporalUnionModelConfig() = default;

  const double MaxV() const { return max_v_; }
  const double MaxA() const { return max_a_; }
  const double MaxJ() const { return max_j_; }
  const double MinV() const { return min_v_; }
  const double MinA() const { return min_a_; }
  const double MinJ() const { return min_j_; }
  const double WeightA() const { return w_a_; }
  const double Weightj() const { return w_j_; }

  const double BoundP() const { return bound_p_; }
  const double BoundV() const { return bound_v_; }
  const double BoundA() const { return bound_a_; }
  const double BoundJ() const { return bound_j_; }
  const double BoundU() const { return bound_u_; }

 private:
  double max_v_{200};
  double max_a_{400};
  double max_j_{1000000};

  double min_v_{-200};
  double min_a_{-400};
  double min_j_{-1000000};

  double w_a_{1.0};
  double w_j_{1.0};
  // 这几个参数暂定
  double bound_p_{100};
  double bound_v_{100};
  double bound_a_{100};
  double bound_j_{100};
  double bound_u_{100};
};

}  // namespace SpatiotemporalUnionConfig

class SpatiotemporalUnionModel {
 public:
  SpatiotemporalUnionModel(const std::string &name,
                           const std::vector<double> &flytime,
                           const SpatiotemporalUnion::TunnelInfos &TunnelInfos,
                           const double speed_limit);

  ~SpatiotemporalUnionModel() = default;

  bool Process();

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<SpatiotemporalUnion::LineModelMatrix>, line_model_matrix);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<SpatiotemporalUnion::PolytopicConstraints>,
      polytopic_constraints);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(SpatiotemporalUnion::CostMatrix,
                                             cost_matrix);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      SpatiotemporalUnion::BoxConstraints, box_constraints);

 private:
  std::string name_{""};
  // SpatiotemporalUnion::State init_state_{};
  std::vector<double> flytime_{};
  SpatiotemporalUnion::TunnelInfos TunnelInfos_;
  double speed_limit_;

  std::vector<SpatiotemporalUnion::LineModelMatrix> line_model_matrix_{};
  std::vector<SpatiotemporalUnion::PolytopicConstraints>
      polytopic_constraints_{};
  SpatiotemporalUnion::CostMatrix cost_matrix_{};
  SpatiotemporalUnion::BoxConstraints box_constraints_{};
};

}  // namespace planning
}  // namespace neodrive
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/planning_macros.h"

namespace neodrive {
namespace planning {
// 先定正无穷吧
constexpr double MAXCOST = std::numeric_limits<double>::infinity();
namespace CilqrModel {
constexpr int nx = 3;
constexpr int nu = 1;
using A = Eigen::Matrix<double, nx, nx>;
using B = Eigen::Matrix<double, nx, nu>;
// using K = Eigen::Matrix<double, nx, nx>;
// using k = Eigen::Matrix<double, nx, nu>;
using X_state = Eigen::Matrix<double, nx, 1>;

using l_x = Eigen::Matrix<double, nx, 1>;
using l_xx = Eigen::Matrix<double, nx, nx>;
using V_x = Eigen::Matrix<double, nx, 1>;
using V_xx = Eigen::Matrix<double, nx, nx>;
using l_u = Eigen::Matrix<double, nu, 1>;
using l_uu = Eigen::Matrix<double, nu, nu>;
using l_ux = Eigen::Matrix<double, nu, nx>;
using k = Eigen::Matrix<double, nu, 1>;
using K = Eigen::Matrix<double, nu, nx>;
using Q_x = Eigen::Matrix<double, nx, 1>;
using Q_xx = Eigen::Matrix<double, nx, nx>;
using Q_u = Eigen::Matrix<double, nu, 1>;
using Q_uu = Eigen::Matrix<double, nu, nu>;
using Q_ux = Eigen::Matrix<double, nu, nx>;

struct CilqrMatrix {
  CilqrModel::A A{};
  CilqrModel::B B{};
};

struct CilqrCostPara {
  CilqrModel::B P_s;
  CilqrModel::B P_v;
  CilqrModel::B P_a;
  CilqrModel::A state_cost_matrix;
};

struct CilqrState {
  double state_s{0.};
  double state_v{0.};
  double state_a{0.};
  void SetZero() {
    state_s = 0.;
    state_v = 0.;
    state_a = 0.;
  }
};
struct CilqrControl {
  double control_jerk{0.};
  void SetZero() { control_jerk = 0.; }
};

}  // namespace CilqrModel
class SpeedCilqrModel {
 public:
  SpeedCilqrModel(const std::string& name, const double delta_t);
  ~SpeedCilqrModel() = default;

  bool Process();

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(CilqrModel::CilqrMatrix,
                                             error_model_matrix)
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(CilqrModel::CilqrMatrix,
                                             model_matrix)
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(CilqrModel::CilqrCostPara,
                                             cost_para)

 private:
  std::string name_{""};
  double delta_t_;
  CilqrModel::CilqrMatrix error_model_matrix_{};
  CilqrModel::CilqrMatrix model_matrix_{};
  CilqrModel::CilqrCostPara cost_para_{};
};

}  // namespace planning
}  // namespace neodrive
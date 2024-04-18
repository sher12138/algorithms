#pragma once

#include <Eigen/Eigen>

#include "src/planning/common/vehicle_param.h"
#include "src/planning/math/common/kdtree.h"
#include "src/planning/math/common/occupy_map.h"
#include "src/planning/math/lbfgs/lbfgs.h"
#include "src/planning/math/minco/minco.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

class MincoGenerator {
 public:
  MincoGenerator() = delete;
  MincoGenerator(
      const Eigen::MatrixXd &ini_state, const Eigen::MatrixXd &fin_state,
      const Eigen::MatrixXd &inner_pts, const Eigen::VectorXd &duration_time,
      const std::vector<Eigen::Vector3d> &state_list,
      const std::vector<Eigen::MatrixXd> &origin_h_polys,
      const std::vector<Eigen::MatrixXd> &shrink_h_polys,
      const std::vector<std::array<double, 2>> &origin_freespace_data,
      const std::vector<std::array<double, 2>> &shrink_freespace_data,
      const int traj_resolution, const int destraj_resolution,
      OccupyMap &&origin_om, OccupyMap &&shrink_om);

  bool OptimizeTrajectory();

  minco::Trajectory MincoPath() { return min_jerk_opt_.getTraj(1); }

 private:
  bool BuildOptProblem();

  bool Solve();

  static double CostFunctionCallback(void *func_data, const Eigen::VectorXd &x,
                                     Eigen::VectorXd &grad);

  double AddPVAGradCost2CT(Eigen::VectorXd &gdTs, minco::MinJerkOpt &traj);

 private:
  Eigen::MatrixXd ini_state_;
  Eigen::MatrixXd fin_state_;
  Eigen::MatrixXd inner_pts_;
  Eigen::VectorXd duration_time_;
  std::vector<Eigen::Vector3d> state_list_{};
  std::vector<Eigen::MatrixXd> origin_h_polys_{};
  std::vector<Eigen::MatrixXd> shrink_h_polys_{};
  std::vector<std::array<double, 2>> origin_freespace_data_{};
  std::vector<std::array<double, 2>> shrink_freespace_data_{};
  int traj_resolution_;
  int destraj_resolution_;

  OccupyMap &origin_om_;
  OccupyMap &shrink_om_;

  Eigen::VectorXd opt_x_;

  int piece_nums_{0};
  int variable_num_{0};
  std::vector<Eigen::Vector2d> veh_le_{};
  double final_cost_{0.};

  std::size_t iter_cnt_{0};
  minco::MinJerkOpt min_jerk_opt_;
  double delta_t_{0.2};

  double wei_obs_{0.5};
  double wei_shrink_{0.5};
  double wei_match_{0.001};

  double wei_vel_{0.01};
  double wei_acc_{0.01};
  double wei_latacc_{0.01};
  double wei_cur_{1.0};
  double wei_phidot_{1.0};

  double wei_time_{0.01};

  double max_vel_{10.0};
  double max_acc_{2.0};
  double max_latacc_{0.0};
  double max_cur_{0.0};
  double max_phidot_{0.0};

  std::vector<std::vector<math::AD2>> car_pts_{};
};

}  // namespace planning
}  // namespace neodrive

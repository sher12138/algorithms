#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "src/planning/common/planning_code_define.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/trajectory/discretized_trajectory.h"
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/math/interpolation.h"
#include "src/planning/parking/hybridAstar/hybrid_a_star.h"
#include "src/planning/parking/parking_config.h"
//#include "src/planning/parking/parking_smoother/distance_approach_problem.h"
//#include
//"src/planning/parking/parking_smoother/dual_variable_warm_start_problem.h"
#include "src/planning/parking/parking_smoother/iterative_anchoring_smoother.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class ParkingTrajectoryOptimizer {
 public:
  ParkingTrajectoryOptimizer(const ParkingConfig& config);

  ~ParkingTrajectoryOptimizer() = default;

  bool Generate(const TrajectoryPoint& trajectory_init_point,
                const std::vector<double>& end_pose,
                const std::vector<double>& XYbounds, double rotate_angle,
                const Vec2d& translate_origin,
                const Eigen::MatrixXi& obstacles_edges_num,
                const Eigen::MatrixXd& obstacles_A,
                const Eigen::MatrixXd& obstacles_b,
                const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
                const ReferencePoint& anchor_point);

  std::vector<TrajectoryPoint> GetOptimizedTrajectory();

  bool IterativeSmoothing(
      const TrajectoryPoint& trajectory_init_point,
      std::vector<TrajectoryPoint>& input,
      std::vector<HybridAStartResult>& partition_trajectories,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      std::vector<TrajectoryPoint>& output);

  //  bool distance_approach_smoothing(
  //      const TrajectoryPoint& trajectory_init_point,
  //      std::vector<TrajectoryPoint>& input, const std::vector<double>&
  //      XYbounds, const Eigen::MatrixXi& obstacles_edges_num, const
  //      Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
  //      std::vector<HybridAStartResult>& partition_trajectories,
  //      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
  //      std::vector<TrajectoryPoint>& output);

  // ***discard
  //  ErrorCode Plan(const std::vector<TrajectoryPoint>& stitching_trajectory,
  //                 const std::vector<double>& end_pose,
  //                 const std::vector<double>& XYbounds, double rotate_angle,
  //                 const Vec2d& translate_origin,
  //                 const Eigen::MatrixXi& obstacles_edges_num,
  //                 const Eigen::MatrixXd& obstacles_A,
  //                 const Eigen::MatrixXd& obstacles_b,
  //                 const std::vector<std::vector<Vec2d>>&
  //                 obstacles_vertices_vec, const ReferencePoint&
  //                 anchor_point);

  //  void GetStitchingTrajectory(
  //      std::vector<TrajectoryPoint>* stitching_trajectory);
  //  void GetOptimizedTrajectory(DiscretizedTrajectory* optimized_trajectory);

  //  void LoadTrajectory(const Eigen::MatrixXd& state_result_ds,
  //                      const Eigen::MatrixXd& control_result_ds,
  //                      const Eigen::MatrixXd& time_result_ds);

  //  void ReTransformOnAnchor(const ReferencePoint& anchor_point);

 private:
  bool IsInitPointNearDestination(const TrajectoryPoint& init_pt,
                                  const std::vector<double>& end_pose,
                                  double rotate_angle,
                                  const Vec2d& translate_origin);

  void PathPointNormalizing(double rotate_angle, const Vec2d& translate_origin,
                            double* x, double* y, double* theta);

  void PathPointDeNormalizing(double rotate_angle,
                              const Vec2d& translate_origin, double* x,
                              double* y, double* theta);

  void PathPointDeNormalizing(double rotate_angle,
                              const Vec2d& translate_origin,
                              std::vector<TrajectoryPoint>& traj);

  void LoadHybridAstarResultInEigen(HybridAStartResult* result,
                                    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS);

  bool GenerateDecoupledTraj(
      const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
      Eigen::MatrixXd* time_result_dc);

  bool GenerateDistanceApproachTraj(
      const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
      const std::vector<double>& XYbounds,
      const Eigen::MatrixXi& obstacles_edges_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
      const Eigen::MatrixXd& last_time_u, const double init_v,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
      Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
      Eigen::MatrixXd* dual_n_result_ds);

  // use draft path as result; smoother failed
  void UseWarmStartAsResult(
      const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
      const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* dual_l_result_ds,
      Eigen::MatrixXd* dual_n_result_ds);

  void LoadResult(const DiscretizedTrajectory& discretized_trajectory,
                  Eigen::MatrixXd* state_result_dc,
                  Eigen::MatrixXd* control_result_dc,
                  Eigen::MatrixXd* time_result_dc);

  void CombineTrajectories(
      const std::vector<Eigen::MatrixXd>& xWS_vec,
      const std::vector<Eigen::MatrixXd>& uWS_vec,
      const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& l_warm_up_vec,
      const std::vector<Eigen::MatrixXd>& n_warm_up_vec,
      const std::vector<Eigen::MatrixXd>& dual_l_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& dual_n_result_ds_vec,
      Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
      Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
      Eigen::MatrixXd* dual_n_result_ds);

  void ReTransformOnAnchor(const ReferencePoint& anchor_point,
                           std::vector<TrajectoryPoint>& output);

  bool InterpolatePoints(std::vector<TrajectoryPoint>& output);

  void GetSmoothTrajectoryDirection(std::vector<TrajectoryPoint>& output);

  void LogHybriad(HybridAStartResult& result,
                  const ReferencePoint& anchor_point, double rotate_angle,
                  const Vec2d& translate_origin);

  void LogOutput(std::vector<TrajectoryPoint>& output);

  void LogTraj(DiscretizedTrajectory& traj);

 private:
  ParkingConfig config_;
  ParkingTrajectoryOptimizerConfig tra_opt_config_;

  std::unique_ptr<HybridAStar> hybrid_astar_;
  //   std::unique_ptr<DistanceApproachProblem> distance_approach_;
  //   std::unique_ptr<DualVariableWarmStartProblem> dual_variable_warm_start_;
  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;

  std::vector<TrajectoryPoint> optimized_pts_;

  // ***discard
  std::vector<TrajectoryPoint> stitching_trajectory_;
  DiscretizedTrajectory optimized_trajectory_;
};

}  // namespace planning
}  // namespace neodrive

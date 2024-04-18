#pragma once

#include <algorithm>
#include <limits>
#include <random>
#include <vector>

#include "src/planning/common/path/discretized_path.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/speed/speed_data.h"
#include "src/planning/common/trajectory/discretized_trajectory.h"
#include "common/math/box2d.h"
#include "common/math/math_utils.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"
#include "src/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "src/planning/math/fempos_smoothing_spline/fem_pos_smoother.h"
#include "src/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "common/math/discrete_points_math.h"
#include "common/math/util.h"
#include "src/planning/parking/parking_config.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class IterativeAnchoringSmoother {
 public:
  IterativeAnchoringSmoother(
      const ParkingIterativeAnchoringConfig& iterative_config,
      const ParkingCommonConfig common_config);

  ~IterativeAnchoringSmoother() = default;

  bool Smooth(const std::vector<TrajectoryPoint>& raw_tra,
              const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
              DiscretizedTrajectory* discretized_trajectory);

 private:
  // use vector<TrajectoryPoint>
  bool CheckGear(const std::vector<TrajectoryPoint>& raw_tra);

  void AdjustStartEndHeading(
      const std::vector<TrajectoryPoint>& raw_tra,
      std::vector<std::pair<double, double>>* const point2d);

  bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
                      DiscretizedPath* raw_path_points);

  bool GenerateInitialBounds(const DiscretizedPath& path_points,
                             std::vector<double>* initial_bounds);

  bool CheckCollisionAvoidance(const DiscretizedPath& path_points,
                               std::vector<std::size_t>* colliding_point_index);

  bool SmoothPath(const DiscretizedPath& raw_path_points,
                  const std::vector<double>& bounds,
                  DiscretizedPath* smoothed_path_points);

  void AdjustPathBounds(const std::vector<std::size_t>& colliding_point_index,
                        std::vector<double>* bounds);

  bool FakeCombinePathAndSpeed(const DiscretizedPath& path_points,
                               DiscretizedTrajectory* discretized_trajectory);

  void AdjustPathAndSpeedByGear(DiscretizedTrajectory* discretized_trajectory);

  // havenot used
  // function on discrete point heading adjustment
  bool ReAnchoring(const std::vector<std::size_t>& colliding_point_index,
                   DiscretizedPath* path_points);

  double CalcHeadings(const DiscretizedPath& path_points,
                      const std::size_t index);

  // *** discard
  bool Smooth(const Eigen::MatrixXd& xWS, const double init_a,
              const double init_v,
              const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
              DiscretizedTrajectory* discretized_trajectory);

  // use eigen
  bool CheckGear(const Eigen::MatrixXd& xWS);

  void AdjustStartEndHeading(
      const Eigen::MatrixXd& xWS,
      std::vector<std::pair<double, double>>* const point2d);

  // speed smoothing
  bool SmoothSpeed(const double init_a, const double init_v,
                   const double path_length, SpeedData* smoothed_speeds);

  bool GenerateStopProfileFromPolynomial(const double init_acc,
                                         const double init_speed,
                                         const double stop_distance,
                                         SpeedData* smoothed_speeds);

  bool CombinePathAndSpeed(const DiscretizedPath& path_points,
                           const SpeedData& speed_points,
                           DiscretizedTrajectory* discretized_trajectory);

  bool IsValidPolynomialProfile(const QuinticPolynomialCurve1d& curve);

 private:
  // vehicle_param
  double ego_length_ = 0.0;
  double ego_width_ = 0.0;
  double center_shift_distance_ = 0.0;

  std::vector<std::vector<Segment2d>> obstacles_linesegments_vec_;

  std::vector<std::size_t> input_colliding_point_index_;

  bool enforce_initial_kappa_ = true;

  // gear DRIVE as true and gear REVERSE as false
  bool gear_ = false;

  DiscretizedTrajectory path_smooth_data_;

  ParkingIterativeAnchoringConfig iterative_config_;
  ParkingCommonConfig common_config_;
};

}  // namespace planning
}  // namespace neodrive

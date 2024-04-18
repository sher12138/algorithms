#pragma once

#include <cmath>
#include <set>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/double.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "reference_line/reference_line.h"
#include "src/planning/common/boundary.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/speed/st_point.h"

namespace neodrive {
namespace planning {
// TODO(wwl) : using Google naming rule
class Utility {
 public:
  Utility() = delete;
  ~Utility() = delete;
  static double gaussian(const double u, const double std, const double x);
  static double sigmoid(const double x);
  static void uniform_slice(double start, double end, uint32_t num,
                            std::vector<double>* sliced);
  template <typename T>
  static double get_area(const std::vector<T>& points);
  template <typename T>
  static bool check_area(const std::vector<T>& points);
  static bool check_area(const Box2d& box);

  static Polygon2d get_trajectory_point_polygon(
      const Vec2d& obstacle_center_point, const Vec2d& trajectory_point,
      const double obstacle_heading, const double trajectory_point_heading,
      const Polygon2d& obstacle_polygon);

  static bool CalcBoundary(const ReferenceLinePtr& reference_line,
                           const Polygon2d& polygon, const Box2d& bounding_box,
                           Boundary* const boundary);

  static bool CalcBoundary(const ReferenceLinePtr& reference_line,
                           const Box2d& bounding_box, Boundary* const boundary);

  static bool CalcBoundary(const ReferencePoint& reference_point,
                           const Box2d& bounding_box, const SLPoint& center_sl,
                           Boundary* const boundary);

  static double look_forward_distance(double v);
  static void calc_plan_length(const double init_s, const double ref_length,
                               const double v, double* backword_length,
                               double* forward_length);

  static bool ExtendPathFrontLastPoint(ReferencePointVec1d& ref_pts,
                                       const std::size_t& num,
                                       const double& distance,
                                       const bool& is_forward);

  static bool CalcLeftRightSpace(
      std::vector<ObstacleBoundary>& obs_valid_boundary,
      const double& left_bound, const double& right_bound,
      std::vector<std::pair<double, double>>& spaces,
      const double L_THRESHOLD = 1.99, const double S_THRESHOLD = 3.99);

  static void NewtonSecLaw(const double acc, const double x0, const double v0,
                           const double delta_t, double* v, double* x);

  static void FittingQuartic(const Eigen::VectorXd& xs,
                             const Eigen::VectorXd& ys, Eigen::VectorXd* coeff);

  static bool SolveQuadraticEquation(const double a, const double b,
                                     const double c, double* root);

 private:
  static constexpr double kMinArea = 0.001;
};

template <typename T>
double Utility::get_area(const std::vector<T>& points) {
  double area = 0.0;
  if (points.size() < 3) {
    return area;
  }
  for (std::size_t i = 2; i < points.size(); ++i) {
    area += cross_prod(points[0], points[i - 1], points[i]);
  }
  area = std::fabs(area) / 2.0;
  return area;
}

template <typename T>
bool Utility::check_area(const std::vector<T>& points) {
  double area = get_area(points);
  return !Double::is_nan(area) && area > kMinArea;
}
}  // namespace planning
}  // namespace neodrive

#pragma once

#include <mutex>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"
#include "fem_pos_config.h"
#include "fem_pos_osqp_solver.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/reference_line/reference_point.h"

namespace neodrive {
namespace planning {

class FemPosSmoother {
 public:
  bool Smooth(const std::vector<EvaluatedPoint>& evaluated_points,
              const double delta_s, const double dense_s);

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<ReferencePoint>,
                                             generate_points);

 private:
  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
             const std::vector<double>& bounds, std::vector<double>* opt_x,
             std::vector<double>* opt_y);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool GenerateRefPoint(const std::vector<EvaluatedPoint>& evaluated_points,
                        std::vector<std::pair<double, double>>& xy_points,
                        const double delta_s, const double dense_s);

  bool ComputePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);

 private:
  double delta_s_{0.};
  double dense_s_{0.};
  double start_s_{0.};

  FemPosConfig config_;
  double zero_x_ = 0.0;
  double zero_y_ = 0.0;
  std::vector<std::pair<double, double>> need_opt_pts_{};
  std::vector<double> need_opt_bounds_{};
  std::vector<size_t> seg_idxs_{};
  std::vector<ReferencePoint> generate_points_;
};
}  // namespace planning
}  // namespace neodrive

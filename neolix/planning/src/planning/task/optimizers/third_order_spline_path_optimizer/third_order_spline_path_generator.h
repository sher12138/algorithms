#pragma once

#include <string>
#include <vector>

#include "math/curve1d/spline.h"
#include "src/planning/common/path/frenet_frame_point.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/math/frame_conversion/sl_analytic_transformation.h"
#include "src/planning/util/path_planner_common.h"
#include "third_order_spline_path_model.h"
#include "third_order_spline_path_mpc.h"

namespace neodrive {
namespace planning {

class ThirdOrderSplinePathGenerator {
 public:
  explicit ThirdOrderSplinePathGenerator(const std::string& name)
      : name_(name) {}

  ~ThirdOrderSplinePathGenerator();

  ErrorCode Optimize(const ReferenceLinePtr& reference_line,
                     const InsidePlannerData& inside_data,
                     OutsidePlannerData* const outside_data);

 private:
  ErrorCode Init(const ReferenceLinePtr& reference_line,
                 const InsidePlannerData& inside_data,
                 OutsidePlannerData* const outside_data);

  bool LateralBoundariesPreProcess(const InsidePlannerData& inside_data,
                                   OutsidePlannerData* const outside_data);

  bool Process(const ReferenceLinePtr& reference_line,
               const InsidePlannerData& inside_data,
               const std::vector<PieceBoundary>& lat_boundaries,
               OutsidePlannerData* const outside_data);

  void ClearPrivateMember();

  bool SampleBoundaries(const std::vector<PieceBoundary>& lat_boundaries);

  std::vector<std::pair<std::size_t, std::size_t>> ObsNeedSamplePairs(
      const std::vector<PieceBoundary>& lat_boundaries) const;

  std::vector<std::pair<std::size_t, std::size_t>> RoadNeedSamplePairs(
      const std::vector<std::pair<std::size_t, std::size_t>>&
          obs_need_sample_pairs,
      const std::vector<PieceBoundary>& lat_boundaries) const;

  bool SampleBoundaries(const std::vector<PieceBoundary>& lat_boundaries,
                        const std::vector<std::pair<std::size_t, std::size_t>>&
                            obs_need_sample_pairs,
                        const std::vector<std::pair<std::size_t, std::size_t>>&
                            road_need_sample_pairs);

  std::vector<PieceBoundary> ObsSampleBoundaries(
      const tk::spline& left_bound_spine, const tk::spline& right_bound_spine,
      const std::vector<std::pair<std::size_t, std::size_t>>& obs_sample_pairs,
      const std::vector<PieceBoundary>& lat_boundaries) const;

  std::vector<PieceBoundary> RoadSampleBoundaries(
      const tk::spline& left_bound_spine, const tk::spline& right_bound_spine,
      const std::vector<std::pair<std::size_t, std::size_t>>& road_sample_pairs,
      const std::vector<PieceBoundary>& lat_boundaries) const;

  bool GenerateOptProblem(const ReferenceLinePtr& reference_line,
                          const InsidePlannerData& inside_data,
                          OutsidePlannerData* const outside_data);

  std::vector<double> DeltaSForKnots() const;

  void PathGoalL(const InsidePlannerData& inside_data,
                 OutsidePlannerData* const outside_data);

  bool PathBounds(const ReferenceLinePtr& reference_line,
                  const InsidePlannerData& inside_data,
                  OutsidePlannerData* const motin_info);

  bool UseAdaptiveBounds(const InsidePlannerData& inside_data,
                         const std::array<double, 3>& init_state,
                         double* const used_dl, double* const used_ddl,
                         double* const used_dddl);

  bool UseDynamicDddxBounds(const InsidePlannerData& inside_data,
                            double* const dl_ptr, double* const ddl_ptr,
                            double* const dddl_ptr);

  bool OptimizePath(const ReferenceLinePtr& reference_line);

  bool DenseThirdOrderSplinePath(
      const ReferenceLinePtr& reference_line, const double& path_output_length,
      const std::vector<ThirdOrderSplinePath::OptVariables>& opt_path_variables,
      std::vector<FrenetFramePoint>& opt_frenet_frame_points);

  bool FillPathData(const ReferenceLinePtr& reference_line,
                    const PathObstacleContext& path_obstacle_context,
                    const InsidePlannerData& inside_data,
                    OutsidePlannerData* const outside_data,
                    const std::vector<FrenetFramePoint>& frenet_path,
                    PathData* const path_data);

 private:
  std::string name_{""};

  double path_output_length_{0.0};
  std::array<double, 3> init_s_state_{};
  std::array<double, 3> init_l_state_{};

  std::vector<PieceBoundary> lateral_boundaries_with_obs_{};
  std::vector<PieceBoundary> lateral_boundaries_without_obs_{};
  std::vector<PieceBoundary> sample_boundaries_{};
  tk::spline upper_l_spline_{};
  tk::spline lower_l_spline_{};
  double speed_enlarge_buffer_{0.2};

  ThirdOrderSplinePath::State init_state_{};
  ThirdOrderSplinePath::Control init_control_{};
  std::vector<double> knots_delta_s_{};
  std::vector<double> upper_l_0_bounds_{};
  std::vector<double> lower_l_0_bounds_{};
  std::vector<double> upper_l_1_bounds_{};
  std::vector<double> lower_l_1_bounds_{};
  std::vector<double> upper_l_2_bounds_{};
  std::vector<double> lower_l_2_bounds_{};
  std::vector<double> upper_dl_0_bounds_{};
  std::vector<double> lower_dl_0_bounds_{};
  std::vector<double> upper_ddl_0_bounds_{};
  std::vector<double> lower_ddl_0_bounds_{};
  std::vector<double> upper_dddl_0_bounds_{};
  std::vector<double> lower_dddl_0_bounds_{};
  double dl_bound_{};
  double ddl_bound_{};
  double dddl_bound_{};
  std::vector<double> goal_l0_{};
  std::vector<double> goal_l1_{};
  std::vector<double> goal_l2_{};

  double dense_delta_s_{0.2};

  std::vector<FrenetFramePoint> opt_frenet_frame_points_{};
  std::vector<ThirdOrderSplinePath::OptVariables> opt_path_variables_{};
  std::shared_ptr<ThirdOrderSplinePathMPC> third_order_spline_path_solver_{};
};

}  // namespace planning
}  // namespace neodrive

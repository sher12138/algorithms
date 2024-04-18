#pragma once

#include "src/planning/config/auto_planning_research_config.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
class PathBoundShrinkerDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathBoundShrinkerDecider);

 public:
  virtual ~PathBoundShrinkerDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  void ShrinkByCurbLane(TaskInfo& task_info);

  bool ShrinkByEgoVehicleHalfWidth(TaskInfo& task_info);

  void ShrinkByKappa(TaskInfo& task_info);

  void ShrinkByHeading(TaskInfo& task_info);

  void ShrinkByFullSpeed(TaskInfo& task_info);

  void ShrinkByEgoVehicle(TaskInfo& task_info);

  bool ShrinkDoubleLane(const double origin_left_bound,
                        const double origin_right_bound, double* left_bound,
                        double* right_bound) const;

  bool ShrinkSingleLane(TaskInfo& task_info, const double origin_left_bound,
                        const double origin_right_bound, const int left_id,
                        const int right_id,
                        const PathRegion::Bound::BoundType& upper_type,
                        const PathRegion::Bound::BoundType& lower_type,
                        double* left_bound, double* right_bound) const;

  bool ShrinkZeroLane(TaskInfo& task_info, const double origin_left_bound,
                      const double origin_right_bound,
                      const PathRegion::Bound::BoundType& upper_type,
                      const PathRegion::Bound::BoundType& lower_type,
                      double* left_bound, double* right_bound) const;
  void VisShrink(const neodrive::planning::ReferenceLinePtr reference_line,
                 const std::vector<neodrive::planning::PathRegion::Bound>&
                     shrink_bounds_info,
                 const std::string& name);
  void VisShrink(const std::vector<Vec2d>& left,
                 const std::vector<Vec2d>& right, const std::string& name);
  void VisShrink(
      const neodrive::planning::ReferenceLinePtr reference_line,
      const std::vector<neodrive::global::perception::camera::CameraLaneLine>&
          curb_lines,
      const std::string& name);
  void SetCurbLines(const neodrive::planning::ReferenceLinePtr reference_line,
                    TaskInfo& task_info);
  void ShrinkFinalCheck(TaskInfo& task_info);

 private:
  double half_car_width_{0.56};
  double longi_dis_thresh_{-1.0};
  static constexpr double kObsMeanDis = 0.50;            // m
  static constexpr double kObsMinDis = 0.40;             // m
  static constexpr double kLaneDis = 0.20;               // m
  static constexpr double kObsOffsetCoef = 0.50;         // ratio
  static constexpr double kDoubleLaneOffsetCoef = 0.40;  // ratio
  static constexpr double kSingleLaneOffsetCoef = 0.35;  // ratio
  static constexpr double kDeltL = 0.3;                  // m
  std::vector<std::pair<Vec2d, PathRegion::Bound::BoundType>>
      vis_shrink_right_xy_boundary_{};
  std::vector<std::pair<Vec2d, PathRegion::Bound::BoundType>>
      vis_shrink_left_xy_boundary_{};
  std::vector<PathRegion::Bound> shrink_bounds_info_{};
  std::vector<SLPoint> left_curb_lane_{};
  std::vector<SLPoint> right_curb_lane_{};
  std::vector<neodrive::planning::PathRegion::Bound> path_boundary_{};

  config::AutoPlanningResearchConfig::PathRoadGraphConfig::PathShrink
      path_shrink_config_{};
};

REGISTER_SCENARIO_TASK(PathBoundShrinkerDecider);

}  // namespace planning
}  // namespace neodrive

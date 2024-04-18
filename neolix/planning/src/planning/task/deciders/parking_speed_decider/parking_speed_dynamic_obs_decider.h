#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ParkingSpeedDynamicObsDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ParkingSpeedDynamicObsDecider);

 public:
  virtual ~ParkingSpeedDynamicObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool DynamicObstaclePreDecision(TaskInfo& task_info) const;
  bool IsDynamicObsNeedIgnore(const InsidePlannerData& inside_data,
                              const Obstacle& obstacle, bool& is_ignore) const;
  bool CollisionCheckObstacleWithTrajectory(TaskInfo& task_info,
                                            const Obstacle& obstacle) const;
  void CollisionCheckWithPredictionTrajectory(
      TaskInfo& task_info, const bool adc_on_reference_line_flag,
      const Obstacle& obstacle) const;
  bool IsAdcOnReferenceLine(const ReferenceLinePtr& reference_line,
                            const InsidePlannerData& inside_data) const;
  void GetAdcFirstCollideCornerPoint(
      const Polygon2d& obs_polygon, const std::vector<Vec2d>& adc_corners,
      AdcCollideCornerPoint& adc_first_collide_corner_point) const;
  void FindHighAndLowWithPolygon(const std::vector<Box2d>& adc_bounding_boxes,
                                 const Polygon2d& obstacle_box, bool* find_high,
                                 bool* find_low, std::size_t* high_index,
                                 std::size_t* low_index) const;
  bool ComputePathAccumulatedS(const PathData& path_data,
                               std::vector<double>* accumulated_s) const;
  bool BuildPathAdcBoundingBoxesAndAABoxes(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const ReferenceLinePtr ref_ptr, const PathData& path_data,
      std::vector<Box2d>* path_adc_boxes,
      std::vector<Boundary>* path_adc_sl_boundaries) const;
  bool BuildPathAdcBoundingBoxes(const InsidePlannerData& inside_data,
                                 const PathData& path_data,
                                 const double& en_large_buffer,
                                 const double& front_en_large_buffer,
                                 const double& back_en_large_buffer,
                                 std::vector<Box2d>* path_adc_boxes) const;
  bool BuildPathAdcAABoxes(const ReferenceLinePtr ref_ptr,
                           const InsidePlannerData& inside_data,
                           const PathData& path_data,
                           const double& en_large_buffer,
                           const double& front_en_large_buffer,
                           const double& back_en_large_buffer,
                           std::vector<Boundary>* adc_sl_boundaries) const;
  bool BuildPathAdcPreBoundingBoxes(
      const InsidePlannerData& inside_data, const PathData& path_data,
      std::vector<Box2d>* path_adc_pre_boxes,
      std::vector<PathPoint>* pre_path_points) const;
};

REGISTER_SCENARIO_TASK(ParkingSpeedDynamicObsDecider);

}  // namespace planning
}  // namespace neodrive
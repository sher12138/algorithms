#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

struct UnprotectedTurnCheckInfo {
  Vec2d center_point{};
  double radius{0.0};
  double begin_edge_heading{0.0};
  double end_edge_heading{0.0};
  double original_start_s{0.0};
  double original_end_s{0.0};
  double extended_start_s{0.0};
  double extended_end_s{0.0};
  size_t last_routing_seq_num{UINT_MAX};
  Polygon2d check_area_polygon{};
  bool check_area_on_left_side{false};
  bool check_area_on_right_side{false};
  bool is_valid{false};
  bool is_extend_end_s{false};  // true:need to extend end_s; false:no need.
};

class SpeedDynamicObsPreDecisionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedDynamicObsPreDecisionDecider);

 public:
  virtual ~SpeedDynamicObsPreDecisionDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);

  bool IsTheSameLeftRoutingLine(const ReferenceLinePtr& reference_line);

 private:
  void InitUnprotectedTurnRightCheckArea(TaskInfo& task_info);
  void InitUnprotectedTurnLeftCheckArea(TaskInfo& task_info);
  void InitUnprotectedTurnLeftPolygon();
  bool IsTheSameRightRoutingLine(const ReferenceLinePtr& reference_line);
  void InitUnprotectedTurnRightPolygon();
  void ExtendTurnLeftAreaEndS(const ReferenceLinePtr& reference_line);
  void ExtendTurnRightAreaEndS(const ReferenceLinePtr& reference_line);
  bool ComputePathAccumulatedS(const PathData& path_data,
                               std::vector<double>* accumulated_s) const;
  bool BuildPathAdcBoundingBoxes(const InsidePlannerData& inside_data,
                                 const PathData& path_data,
                                 const double& en_large_buffer,
                                 const double& front_en_large_buffer,
                                 const double& back_en_large_buffer,
                                 std::vector<Box2d>* path_adc_boxes) const;
  bool BuildPathAdcPreBoundingBoxes(
      const InsidePlannerData& inside_data, const PathData& path_data,
      std::vector<Box2d>* path_adc_pre_boxes,
      std::vector<PathPoint>* pre_path_points) const;

  bool BuildPathAdcBoundingBoxesAndAABoxes(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const ReferenceLinePtr ref_ptr, const PathData& path_data,
      std::vector<Box2d>* path_adc_boxes,
      std::vector<Boundary>* adc_aaboxes) const;

  bool BuildPathAdcAABoxes(const ReferenceLinePtr ref_ptr,
                           const InsidePlannerData& inside_data,
                           const PathData& path_data,
                           const double& en_large_buffer,
                           const double& front_en_large_buffer,
                           const double& back_en_large_buffer,
                           std::vector<Boundary>* adc_aaboxes) const;

  ErrorCode DynamicObstaclePreDecision(TaskInfo& task_info) const;

  bool ObsIsInLeftTurnCheckArea(const Obstacle& obstacle) const;
  bool AdcIsInValidLeftTurnSRange() const;
  bool AdcIsInValidRightTurnSRange() const;
  bool ObsIsInRightTurnCheckArea(const Obstacle& obstacle) const;
  bool IsDynamicObsNeedIgnore(const InsidePlannerData& inside_data,
                              const Obstacle& obstacle, bool& is_valid,
                              bool& is_in_left_turn_area,
                              bool& is_in_right_turn_area) const;

  ErrorCode CollisionCheckObstacleWithTrajectory(
      TaskInfo& task_info, const Obstacle& obstacle,
      const bool& is_in_left_turn_area,
      const bool& is_in_right_turn_area) const;

  bool IsAdcOnReferenceLine(const ReferenceLinePtr& reference_line,
                            const InsidePlannerData& inside_data) const;

  void CollisionCheckWithPredictionTrajectory(
      TaskInfo& task_info, const bool adc_on_reference_line_flag,
      const Obstacle& obstacle, const bool& is_in_left_turn_area,
      const bool& is_in_right_turn_area) const;

  void FindHighAndLowWithPolygon(const std::vector<Box2d>& adc_bounding_boxes,
                                 const Polygon2d& obstacle_box, bool* find_high,
                                 bool* find_low, std::size_t* high_index,
                                 std::size_t* low_index) const;

  void DynamicContextInfo(OutsidePlannerData* const outside_data);
  void VisUnprotectedLeftTurnCheckArea();
  void VisUnprotectedRightTurnCheckArea();

 private:
  UnprotectedTurnCheckInfo unprotected_turn_left_check_info_{},
      unprotected_turn_right_check_info_{};
  double adc_current_s_{0.0};
  double adc_current_v_{0.0};

 private:
  const double kEpsilon{0.1};

  Polygon2d path_max_polygon_;
  std::vector<Polygon2d> path_small_polygons_;
};

REGISTER_SCENARIO_TASK(SpeedDynamicObsPreDecisionDecider);

}  // namespace planning
}  // namespace neodrive

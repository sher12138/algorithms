#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

struct UnprotectedTurnLeftCheckInfo {
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

class MotorwaySpeedIterCipvDynamicObsPreDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedIterCipvDynamicObsPreDecider);

 public:
  virtual ~MotorwaySpeedIterCipvDynamicObsPreDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);

 private:
  void InitUnprotectedTurnLeftCheckArea(TaskInfo& task_info);
  void InitUnprotectedTurnRightCheckArea(TaskInfo& task_info);
  bool IsTheSameRightRoutingLine(const ReferenceLinePtr& reference_line);
  bool IsTheSameLeftRoutingLine(const ReferenceLinePtr& reference_line);
  void InitUnprotectedTurnLeftPolygon();
  void InitUnprotectedTurnRightPolygon();
  void ExtendTurnLeftAreaEndS(const ReferenceLinePtr& reference_line);
  void ExtendTurnRightAreaEndS(const ReferenceLinePtr& reference_line);

  ErrorCode DynamicObstaclePreDecision(
      const ReferenceLinePtr reference_line,
      const InsidePlannerData& inside_data,
      const std::vector<Obstacle*>& dynamic_obs_vec,
      OutsidePlannerData* const outside_data) const;

  bool ObsIsInLeftTurnCheckArea(const Obstacle& obstacle) const;
  bool ObsIsInRightTurnCheckArea(const Obstacle& obstacle) const;
  bool AdcIsInValidLeftTurnSRange() const;
  bool EgoIsInProtectedTurnIntersection() const;
  bool AdcIsInValidRightTurnSRange() const;
  bool IsDynamicObsNeedIgnore(const InsidePlannerData& inside_data,
                              const Obstacle& obstacle, bool& is_valid,
                              bool& is_in_left_turn_area,
                              bool& is_in_right_turn_area) const;

  ErrorCode CollisionCheckObstacleWithTrajectory(
      const ReferenceLinePtr reference_line,
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      const bool& is_in_left_turn_area, const bool& is_in_right_turn_area,
      const bool& adc_on_reference_line_flag,
      OutsidePlannerData* const outside_data) const;

  bool IsAdcOnReferenceLine(const ReferenceLinePtr& reference_line,
                            const InsidePlannerData& inside_data) const;

  void CollisionCheckWithPredictionTrajectory(
      const ReferenceLinePtr reference_line,
      const std::vector<Box2d>& adc_bounding_boxes,
      const std::vector<PathPoint>& path_points,
      const bool adc_on_reference_line_flag, const double adc_front_theta,
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      const bool& is_in_left_turn_area, const bool& is_in_right_turn_area,
      OutsidePlannerData* const outside_data) const;

  void FindHighAndLowWithPolygon(const std::vector<Box2d>& adc_bounding_boxes,
                                 const Polygon2d& obstacle_box, bool* find_high,
                                 bool* find_low, std::size_t* high_index,
                                 std::size_t* low_index) const;

  void DynamicContextInfo(OutsidePlannerData* const outside_data);
  void VisUnprotectedLeftTurnCheckArea();
  void VisUnprotectedRightTurnCheckArea();

 private:
  UnprotectedTurnLeftCheckInfo unprotected_turn_left_check_info_{},
      unprotected_turn_right_check_info_{};
  double adc_current_s_{0.0};
  double adc_current_v_{0.0};

 private:
  const double kEpsilon{0.1};

  Polygon2d path_max_polygon_;
  std::vector<Polygon2d> path_small_polygons_;
};

REGISTER_SCENARIO_TASK(MotorwaySpeedIterCipvDynamicObsPreDecider);

}  // namespace planning
}  // namespace neodrive

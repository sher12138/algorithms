#pragma once

#include "math/curve1d/spline.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/util/util.h"
namespace neodrive {
namespace planning {

namespace {
struct TurnRegionInfo {
  Vec2d start_vec2d{};
  Vec2d end_vec2d{};
  double start_s{};
  double end_s{};
  double max_curva{};
  int turn_direction{};  // 0: straight， -1：turn left， 1:turn right
};

struct CalAfterCollisionInfo {
  Vec2d current_point_xy{};
  Vec2d collision_point_xy{};
  SLPoint collision_point_sl{};
  SLPoint current_point_sl{};
  double attain_time{};
  double speed{};
  double speed_heading{};
  double width{};
  double length{};

  std::string Print() {
    std::string str = "";
    str += "current_point_xy: " + DoubleFormat(current_point_xy.x(), 1) + ", " +
           DoubleFormat(current_point_xy.y(), 1) + "\t";
    str += "collision_point_xy: " + DoubleFormat(collision_point_xy.x(), 1) +
           ", " + DoubleFormat(collision_point_xy.y(), 1) + "\t";
    str += "collision_point_sl: " + DoubleFormat(collision_point_sl.s(), 1) +
           ", " + DoubleFormat(collision_point_sl.l(), 1) + "\t";
    str += "current_point_sl: " + DoubleFormat(current_point_sl.s(), 1) + ", " +
           DoubleFormat(current_point_sl.l(), 1) + "\t";
    str += "attain_time: " + DoubleFormat(attain_time, 1) + "\t";
    str += "speed: " + DoubleFormat(speed, 1) + "\t";
    str += "speed_heading: " + DoubleFormat(speed_heading, 1) + "\t";
    str += "width: " + DoubleFormat(width, 1) + "\t";
    str += "length: " + DoubleFormat(length, 1) + "\t";
    return std::move(str);
  }
};
}  // namespace

class MotorwaySpeedUnprotectedTurnDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedUnprotectedTurnDecider);

 public:
  virtual ~MotorwaySpeedUnprotectedTurnDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset(){};

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);

  bool ObsIsInEgoTurnDirection(const Obstacle& obs);
  void ReMapBaseRefLine(const ReferenceLinePtr& ref_line);

  bool GenerateDecision(const ReferenceLinePtr& ref_line, const Obstacle& obs);
  bool VehicleCoordinationBehind(const Obstacle& obs);
  double GetHeadingDiffRatio(double max_heading_diff, const double collision_s,
                             const SLPoint& start_sl, const double obs_speed);

 private:
  double adc_current_s_{};
  double adc_current_x_{};
  double adc_current_y_{};
  double ego_vel_width_{};
  double adc_heading_{};
  double adc_current_l_{};
  double adc_current_v_{};
  double ego_start_s_{};
  double ego_end_s_{};
  double ego_to_head_{};
  double ego_to_tail_{};
  tk::spline obs_curva_line_{};

  //
  bool update_limited_speed_ = false;
  double limited_speed_ = 0.0;

  // ref line
  double last_ref_start_s_{-1.};

  tk::spline ego_curva_line_{};
  int adc_add_x_dir_{0};
  TurnRegionInfo turn_region_info_{};
  bool weather_turn_region_info_available_{false};
  //
  const config::AutoPlanningResearchConfig::SpeedUnprotectedTurnDeciderConfig*
      config_ = &config::PlanningConfig::Instance()
                     ->planning_research_config()
                     .speed_unprotected_turn_decider_config;
};
REGISTER_SCENARIO_TASK(MotorwaySpeedUnprotectedTurnDecider);
}  // namespace planning
}  // namespace neodrive
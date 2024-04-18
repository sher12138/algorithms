#include "motorway_speed_limit_turn_right.h"

#include "planning.pb.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/util/speed_limit_trans.h"

namespace neodrive {
namespace planning {

using MotorwayIntersectionStageState =
    neodrive::global::planning::MotorwayIntersectionStageState;

MotorwaySpeedLimitTurnRight::MotorwaySpeedLimitTurnRight()
    : MotorwaySpeedLimitInterface("motorway_speed_limit_turn_right") {}

void MotorwaySpeedLimitTurnRight::ComputeSpeedLimit(TaskInfo& task_info) {
  const auto& turn_right_config =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .motorway_speed_confilict_decider_config.turn_right_scenario_config;
  if (!turn_right_config.enable_turn_speed_limit) {
    return;
  }

  /// compute speed limit
  const auto& init_point =
      task_info.current_frame()->inside_planner_data().init_point;
  const auto& curr_s = task_info.curr_sl().s();
  std::array<double, 3> init_state{curr_s, init_point.velocity(), 0.0};
  std::array<double, 3> end_state{0.0, 0.0, 0.0};

  const auto consider_length = std::max(5.0, init_point.velocity() * 8.0);
  bool consider_right_turn_limit = false;
  const auto& ref_pts = task_info.reference_line()->ref_points();
  for (const auto& pt : ref_pts) {
    if (pt.s() < curr_s) {
      continue;
    }
    if (pt.s() > curr_s + consider_length) {
      break;
    }
    if (pt.is_right_signal()) {
      end_state[0] = std::max(pt.s(), curr_s) + 0.1;
      end_state[1] = turn_right_config.turn_right_speed_limit;
      consider_right_turn_limit = true;
      break;
    }
  }
  if (!consider_right_turn_limit) return;

  double turn_right_speed_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "turn_right_speed_limit", init_state, end_state, max_speed(), 3.0);
  LOG_INFO("turn_right_speed_limit: {:.3f}", turn_right_speed_limit);

  if (turn_right_speed_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::TURN_RIGHT, SpeedLimitType::SOFT,
                   turn_right_speed_limit, 0.0);
  }
}

}  // namespace planning
}  // namespace neodrive

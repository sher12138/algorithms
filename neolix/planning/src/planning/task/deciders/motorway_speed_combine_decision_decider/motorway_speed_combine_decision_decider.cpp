#include "motorway_speed_combine_decision_decider.h"

namespace neodrive {
namespace planning {

MotorwaySpeedCombineDecisionDecider::MotorwaySpeedCombineDecisionDecider() {
  name_ = "MotorwaySpeedCombineDecisionDecider";
}

MotorwaySpeedCombineDecisionDecider::~MotorwaySpeedCombineDecisionDecider() {
  Reset();
}

ErrorCode MotorwaySpeedCombineDecisionDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  auto outside_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  auto inside_data = task_info.current_frame()->inside_planner_data();
  if (outside_data_ptr == nullptr) return ErrorCode::PLANNING_ERROR_FAILED;

  double sample_t{0.1};
  auto only_for_single_cipv = config::PlanningConfig::Instance()
                                  ->plan_config()
                                  .common.only_for_single_cipv;
  const auto& enable_back_cipv =
      config::PlanningConfig::Instance()->plan_config().common.enable_back_cipv;
  auto& final_goal_s = outside_data_ptr->motorway_speed_context.final_goal_s;
  auto& final_goal_v = outside_data_ptr->motorway_speed_context.final_goal_v;
  auto& final_goal_a = outside_data_ptr->motorway_speed_context.final_goal_a;
  final_goal_s.clear(), final_goal_v.clear(), final_goal_a.clear();
  MotorwaySequenceDecisionData final_decision_data{
      .source = MotorwaySequenceDecisionType::NONE};

  /// Only for single cipv
  if (only_for_single_cipv) {
    auto& tunnel_decision =
        outside_data_ptr->motorway_speed_context.tunnel_decision;
    tunnel_decision.Reset();
    for (const auto& decision :
         outside_data_ptr->motorway_speed_context.goal_decision) {
      if (decision.source == MotorwaySequenceDecisionType::ONE_CIPV) {
        final_decision_data = decision;
        final_decision_data.source = MotorwaySequenceDecisionType::ONE_CIPV;
        break;
      }
    }
    if (final_decision_data.source == MotorwaySequenceDecisionType::NONE) {
      LOG_ERROR("invalid final_decision_data.");
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
    if (final_decision_data.source == MotorwaySequenceDecisionType::ONE_CIPV) {
      double speed_limit = DataCenter::Instance()
                               ->mutable_behavior_speed_limits()
                               ->speed_limit();
      for (double t = 0.; t < final_decision_data.goal_s.back().t();
           t += sample_t) {
        tunnel_decision.upper_s_tunnel.emplace_back(
            STPoint(10.0 + std::max(speed_limit, inside_data.vel_v) * t, t));
        tunnel_decision.lower_s_tunnel.emplace_back(STPoint(-10.0, t));
        tunnel_decision.upper_v_tunnel.emplace_back(STPoint(speed_limit, t));
        tunnel_decision.lower_v_tunnel.emplace_back(STPoint(0.0, t));
      }
      final_goal_s = std::move(final_decision_data.goal_s);
      final_goal_v = std::move(final_decision_data.goal_v);
      final_goal_a = std::move(final_decision_data.goal_a);
    }

    return ErrorCode::PLANNING_OK;
  }

  // Only for multi cipv
  for (const auto& decision :
       outside_data_ptr->motorway_speed_context.goal_decision) {
    if (decision.source == MotorwaySequenceDecisionType::MULTI_CIPV) {
      final_decision_data = decision;
      final_decision_data.source = MotorwaySequenceDecisionType::MULTI_CIPV;
      break;
    }
  }
  if (final_decision_data.source == MotorwaySequenceDecisionType::MULTI_CIPV) {
    final_goal_s = std::move(final_decision_data.goal_s);
    final_goal_v = std::move(final_decision_data.goal_v);
    final_goal_a = std::move(final_decision_data.goal_a);
  }

  // back cipv speed decider
  if (enable_back_cipv) {
    for (const auto& decision :
         outside_data_ptr->motorway_speed_context.goal_decision) {
      if (decision.source == MotorwaySequenceDecisionType::BACK_CIPV) {
        final_decision_data = decision;
        final_decision_data.source = MotorwaySequenceDecisionType::BACK_CIPV;
        break;
      }
    }
    if (final_decision_data.source == MotorwaySequenceDecisionType::BACK_CIPV) {
      final_goal_s = std::move(final_decision_data.goal_s);
      final_goal_v = std::move(final_decision_data.goal_v);
      final_goal_a = std::move(final_decision_data.goal_a);
    }
  }

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive

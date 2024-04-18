#include "stop_sign_law.h"

#include "src/planning/common/data_center/data_center.h"

namespace neodrive {
namespace planning {

StopSignLaw::StopSignLaw() : TrafficLaw("StopSignLaw") {}

ErrorCode StopSignLaw::Apply(TaskInfo& task_info,
                             const InsidePlannerData& inside_data,
                             const OutsidePlannerData& outside_data,
                             const Boundary& adc_boundary,
                             DecisionData* const decision_data,
                             TrafficLawContext* const traffic_law_context) {
  ErrorCode ret = ErrorCode::PLANNING_OK;
  auto stop_sign_law_context = traffic_law_context->MutableStopsignlawcontext();
  current_stop_sign_ = stop_sign_law_context->StopSign();
  stop_sign_state_ = stop_sign_law_context->StopsignState();
  stop_line_s_ = stop_sign_law_context->StopLineS();
  last_stop_time_ = stop_sign_law_context->LastStopTime();

  const ReferenceLinePtr reference_line = task_info.reference_line();

  switch (stop_sign_state_) {
    case StopSignState::NORMAL_CRUISE:
      ret = NormalCruiseManager(reference_line, adc_boundary);
      break;
    case StopSignState::PRE_STOP:
      ret = PreStopManager(task_info, adc_boundary);
      break;
    case StopSignState::STOP:
      ret = StopManager();
      break;
    case StopSignState::FINISH:
      ret = FinishManager();
      break;
    default:
      break;
  }

  // update stop sign context
  stop_sign_law_context->SetStopSign(current_stop_sign_);
  stop_sign_law_context->SetStopSignState(stop_sign_state_);
  stop_sign_law_context->SetStopLineS(stop_line_s_);
  stop_sign_law_context->SetLastStopTime(last_stop_time_);
  // update_stop_sign_context(traffic_law_context);

  if (need_stop_) {
    LOG_INFO("virtual_obstacle_route_s:{:.4f}", stop_line_s_);
    int32_t obstacle_id = 0;
    ret = decision_data->create_virtual_obstacle(
        stop_line_s_, VirtualObstacle::CROSSWALK, &obstacle_id);
    if (ret != ErrorCode::PLANNING_OK) {
      LOG_ERROR("Failed to create virtual obstacle.");
      return ret;
    }
    LOG_INFO("Created virtual obstacle id:{} for stop sign.", obstacle_id);
  }

  return ret;
}

ErrorCode StopSignLaw::NormalCruiseManager(const ReferenceLinePtr& refer_line,
                                           const Boundary& adc_boundary) {
  std::vector<RouteStopSign> stop_signs;
  FindFrontStopSign(refer_line, adc_boundary, &stop_signs);

  if (stop_signs.empty()) {
    LOG_INFO("front no stop signs");
    return ErrorCode::PLANNING_OK;
  }

  for (const auto& stop_sign : stop_signs) {
    LOG_DEBUG("stop sign : start_s {:.2f} end_s {:.2f}",
              stop_sign.StartRouteS(), stop_sign.EndRouteS());
    if (IsNearStopSign(adc_boundary, stop_sign)) {
      LOG_INFO("near front stop sign");
      current_stop_sign_ = stop_sign;
      need_stop_ = true;
      stop_line_s_ = stop_sign.StartRouteS();
      stop_sign_state_ = StopSignState::PRE_STOP;

      return ErrorCode::PLANNING_OK;
    }
  }

  LOG_DEBUG("front {} stop signs too far", stop_signs.size());
  return ErrorCode::PLANNING_OK;
}

ErrorCode StopSignLaw::PreStopManager(const TaskInfo& task_info,
                                      const Boundary& adc_boundary) {
  double vel = DataCenter::Instance()->vehicle_state_proxy().LinearVelocity();
  vel = std::abs(vel);
  double stop_dis_before_stop_line = 10.0;
  if (vel < FLAGS_planning_adc_stop_velocity_threshold &&
      adc_boundary.end_s() > stop_line_s_ - stop_dis_before_stop_line) {
    LOG_INFO("adc reach stop line.");
    last_stop_time_ = common::util::TimeLogger::GetCurrentTimeseocnd();
    stop_sign_state_ = StopSignState::STOP;
    return ErrorCode::PLANNING_OK;
  }
  need_stop_ = true;

  LOG_INFO("pre stop : approach to stop line");
  return ErrorCode::PLANNING_OK;
}

ErrorCode StopSignLaw::StopManager() {
  double stop_sign_wait_time = 5.0;
  double stop_time =
      common::util::TimeLogger::GetCurrentTimeseocnd() - last_stop_time_;
  if (stop_time > stop_sign_wait_time) {
    LOG_INFO("stop for stop sign : {:.2f} s", stop_time);
    last_stop_time_ = 0.0;
    stop_sign_state_ = StopSignState::FINISH;
    return ErrorCode::PLANNING_OK;
  }
  need_stop_ = true;

  return ErrorCode::PLANNING_OK;
}

ErrorCode StopSignLaw::FinishManager() {
  RouteStopSign empty_stop_sign;
  current_stop_sign_ = empty_stop_sign;
  last_stop_time_ = 0.0;

  LOG_INFO("finish reset stop sign context info.");
  stop_sign_state_ = StopSignState::NORMAL_CRUISE;

  return ErrorCode::PLANNING_OK;
}

void StopSignLaw::FindFrontStopSign(const ReferenceLinePtr& reference_line,
                                    const Boundary& adc_boundary,
                                    std::vector<RouteStopSign>* stop_sign) {
  for (const auto& stop_sign_overlap : reference_line->stop_sign_overlaps()) {
    if (stop_sign_overlap.start_s > adc_boundary.end_s()) {
      stop_sign->emplace_back(stop_sign_overlap.object_id,
                              stop_sign_overlap.start_s,
                              stop_sign_overlap.end_s);
    }
  }
}

bool StopSignLaw::IsNearStopSign(const Boundary& adc_boundary,
                                 const RouteStopSign& stop_sign) {
  double distance_to_stop_sign =
      std::fabs(stop_sign.StartRouteS() - adc_boundary.end_s());
  return distance_to_stop_sign <
             FLAGS_planning_stop_sign_look_forward_distance &&
         distance_to_stop_sign > 8.0;  // TODO : temp value to invalid stop sign
}

}  // namespace planning
}  // namespace neodrive

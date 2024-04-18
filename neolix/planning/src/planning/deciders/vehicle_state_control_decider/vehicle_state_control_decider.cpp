#include "vehicle_state_control_decider.h"

namespace neodrive {
namespace planning {

VehicleStateControlDecider::VehicleStateControlDecider()
    : last_signal_(TurnLightSignal::NO_TURN) {
  name_ = "VehicleStateControlDecider";
  last_signal_start_time_ = common::util::TimeLogger::GetCurrentTimeseocnd();
}

VehicleStateControlDecider::~VehicleStateControlDecider() {}

void VehicleStateControlDecider::Reset() {
  prev_intention_update_timestamp_ = 0.0;
}

ErrorCode VehicleStateControlDecider::Execute(TaskInfo& task_info) {
  auto new_intention = GetNewIntention();
  // filter intention.
  FilterIntention(new_intention);
  // write light signal into trajectory.
  UpdateIntentionTurnLight();
  return ErrorCode::PLANNING_OK;
}

void VehicleStateControlDecider::SaveTaskResults(TaskInfo& task_info) {
  auto master_info = data_center_->mutable_master_info();
  master_info->set_curr_intention(curr_intention_);
  master_info->set_adc_signal(adc_signal_);
  master_info->set_clean_adc_signal(clean_adc_signal_);
}

void VehicleStateControlDecider::FilterIntention(
    const AutoPilotIntention new_intention) {
  static constexpr double KIntentionFilterInterval = 2.0;  // second.
  auto master_info = data_center_->mutable_master_info();
  double now_timestamp = common::util::TimeLogger::GetCurrentTimeseocnd();
  if (new_intention != master_info->curr_intention() &&
      now_timestamp - prev_intention_update_timestamp_ >
          KIntentionFilterInterval) {
    curr_intention_ = new_intention;
    prev_intention_update_timestamp_ = now_timestamp;
  }
}

AutoPilotIntention VehicleStateControlDecider::GetNewIntention() {
  auto master_info = data_center_->mutable_master_info();
  auto& task_info = data_center_->task_info_list().front();
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  // intention only in cruise.
  if (false == data_center_->global_state_proxy().is_cruise()) {
    return AutoPilotIntention::FINISH;
  }
  // check start go and station stop.
  // station end.
  if (master_info->distance_to_end() <
      FLAGS_planning_turn_lights_pull_over_distance_threshold) {
    if (FLAGS_planning_default_left_right_side) {
      return AutoPilotIntention::PULL_OVER_RIGHT;
    } else {
      return AutoPilotIntention::PULL_OVER_LEFT;
    }
  }

  // start go.
  if (task_info.curr_sl().s() <
          FLAGS_planning_turn_lights_start_to_move_distance_threshold +
              FLAGS_planning_reference_line_extend_length + 20.0 &&
      task_info.adc_point().velocity() <= 1.0) {
    if (FLAGS_planning_default_left_right_side) {
      return AutoPilotIntention::START_GO_LEFT;
    } else {
      return AutoPilotIntention::START_GO_RIGHT;
    }
  }

  // check scenario.
  auto& lane_borrow_context = master_info->lane_borrow_context();
  switch (master_info->curr_scenario()) {
    // DETOUR scenario.
    case ScenarioState::DETOUR: {
      switch (lane_borrow_context.stage) {
        case DetourStageState::PREPARE: {
          if (lane_borrow_context.borrow_side ==
              LaneBorrowContext::BorrowSide::Left) {
            return AutoPilotIntention::LANE_BORROW_LEFT;
          } else if (lane_borrow_context.borrow_side ==
                     LaneBorrowContext::BorrowSide::Right) {
            return AutoPilotIntention::LANE_BORROW_RIGHT;
          }
        } break;
        case DetourStageState::BORROWING: {
          return AutoPilotIntention::LANE_BORROW_KEEP;
        } break;
        case DetourStageState::EXIT: {
          if (lane_borrow_context.borrow_side ==
                  LaneBorrowContext::BorrowSide::Left &&
              false == lane_borrow_context.is_adc_on_refer_lane) {
            return AutoPilotIntention::LANE_BORROW_RETURN_RIGHT;
          } else if (lane_borrow_context.borrow_side ==
                         LaneBorrowContext::BorrowSide::Right &&
                     false == lane_borrow_context.is_adc_on_refer_lane) {
            return AutoPilotIntention::LANE_BORROW_RETURN_LEFT;
          }
        } break;
        default:
          break;
      }
    } break;
    // other scenario.
    default:
      break;
  }

  // check nudge obstacle.

  // check path and control.
  auto reference_line = task_info.reference_line();
  for (const auto& pt : reference_line->ref_points()) {
    if (pt.s() > task_info.curr_sl().s() +
                     plan_config.intention.turn_intention_preview_distance)
      break;
    if (pt.s() <= task_info.curr_sl().s()) {
      continue;
    }
    switch (pt.signal_type()) {
      case neodrive::planning::TurnSignalType::TST_LEFT:
        LOG_INFO("front lane turn left.");
        return AutoPilotIntention::TURN_LEFT;
      case neodrive::planning::TurnSignalType::TST_RIGHT:
        LOG_INFO("front lane turn right.");
        return AutoPilotIntention::TURN_RIGHT;
      default:
        break;
    }
  }

  return AutoPilotIntention::FINISH;
}

void VehicleStateControlDecider::UpdateIntentionTurnLight() {
  switch (curr_intention_) {
    case AutoPilotIntention::TURN_LEFT:
    case AutoPilotIntention::LANE_BORROW_LEFT:
    case AutoPilotIntention::LANE_BORROW_RETURN_LEFT:
    case AutoPilotIntention::LANE_CHANGE_LEFT:
    case AutoPilotIntention::PASS_BY_LEFT:
    case AutoPilotIntention::START_GO_LEFT:
    case AutoPilotIntention::PULL_OVER_LEFT: {
      clean_adc_signal_ = false;
      adc_signal_ = ADCSignals::LEFT_TURN;
    } break;
    case AutoPilotIntention::TURN_RIGHT:
    case AutoPilotIntention::LANE_BORROW_RIGHT:
    case AutoPilotIntention::LANE_BORROW_RETURN_RIGHT:
    case AutoPilotIntention::LANE_CHANGE_RIGHT:
    case AutoPilotIntention::PASS_BY_RIGHT:
    case AutoPilotIntention::START_GO_RIGHT:
    case AutoPilotIntention::PULL_OVER_RIGHT: {
      clean_adc_signal_ = false;
      adc_signal_ = ADCSignals::RIGHT_TURN;
    } break;
    case AutoPilotIntention::LANE_BORROW_KEEP: {
      clean_adc_signal_ = false;
      adc_signal_ = ADCSignals::EMERGENCY_LIGHT;
    } break;
    default: { clean_adc_signal_ = true; } break;
  }
}

}  // namespace planning
}  // namespace neodrive

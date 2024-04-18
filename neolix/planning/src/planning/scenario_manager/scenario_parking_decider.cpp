#include "scenario_parking_decider.h"

namespace neodrive {
namespace planning {

ScenarioParkingDecider::ScenarioParkingDecider() {
  name_ = "ScenarioParkingDecider";
}

bool ScenarioParkingDecider::Init() {
  if (initialized_) {
    return true;
  }

  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_parking_stage";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }
  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioParkingDecider init.");
  return true;
}

bool ScenarioParkingDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioParkingDecider.");
  return true;
}

ErrorCode ScenarioParkingDecider::RunOnce() {
  LOG_INFO("ScenarioBackOutDecider::RunOnce");
  if (OnHandleAllStates() == false) {
    auto it = state_machine_function_map_.find(curr_state_);
    if (it != state_machine_function_map_.end()) {
      auto func = it->second;
      (this->*func)();
    } else {
      // missing handle state function
      LOG_ERROR("missing handle state function {}", curr_state_str_);
    }
  }
  if (!UpdateStateMachine()) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool ScenarioParkingDecider::ResetStateMachine() {
  curr_state_ = ParkingStageState::INIT;
  curr_state_str_ = ParkingStageState::State_Name(curr_state_);
  prev_decision_ = ParkingStageChangeFlag::T_NONE_INIT;
  curr_decision_ = ParkingStageChangeFlag::T_NONE_INIT;

  if (!state_machine_.SetInitializeState(curr_state_str_)) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }
  return true;
}

void ScenarioParkingDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<ParkingStageState::State, void (ScenarioParkingDecider::*)()>(
          ParkingStageState::INIT, &ScenarioParkingDecider::OnHandleStateInit));
  state_machine_function_map_.insert(
      std::pair<ParkingStageState::State, void (ScenarioParkingDecider::*)()>(
          ParkingStageState::PARKING_IN,
          &ScenarioParkingDecider::OnHandleStateParkingIn));
  state_machine_function_map_.insert(
      std::pair<ParkingStageState::State, void (ScenarioParkingDecider::*)()>(
          ParkingStageState::PARKING_OUT,
          &ScenarioParkingDecider::OnHandleStateParkingOut));
  state_machine_function_map_.insert(
      std::pair<ParkingStageState::State, void (ScenarioParkingDecider::*)()>(
          ParkingStageState::FINISH,
          &ScenarioParkingDecider::OnHandleStateFinish));
}

bool ScenarioParkingDecider::OnHandleAllStates() { return false; }

void ScenarioParkingDecider::OnHandleStateInit() {
  auto &task_info = data_center_->task_info_list().front();
  auto path_data = task_info.last_frame()->outside_planner_data().path_data;
  auto parking_ptr = data_center_->parking_ptr();
  if (parking_ptr != nullptr && !parking_ptr->ParkPath().empty()) {
    is_state_change_ = true;
    curr_decision_ = parking_ptr->is_park_in()
                         ? ParkingStageChangeFlag::T_INIT_PARKING_IN
                         : ParkingStageChangeFlag::T_INIT_PARKING_OUT;
  }
}

void ScenarioParkingDecider::OnHandleStateParkingIn() {
  // TODO: parking in finish
  bool parking_in_finish{false};
  if (parking_in_finish) {
    is_state_change_ = true;
    curr_decision_ = ParkingStageChangeFlag::T_PARKING_IN_FINISH;
  }
}

void ScenarioParkingDecider::OnHandleStateParkingOut() {
  // TODO: parking out finish
  bool parking_out_finish{false};
  if (!data_center_->parking_ptr() ||
      !data_center_->parking_ptr()->OriginPark())
    return;
  if (data_center_->parking_ptr()->OriginPark()->Type() !=
      global::hdmap::ParkingSpace_ParkingType_HORIZONTAL)
    return;
  auto lane_ptr = data_center_->parking_ptr()->OverlapLane();
  if (!lane_ptr) return;
  common::math::Vec2d ego_utm{data_center_->vehicle_state_utm().X(),
                              data_center_->vehicle_state_utm().Y()};
  if (!lane_ptr->IsOnLane(ego_utm)) return;
  // TODO: to json
  if (std::abs(lane_ptr->Heading(ego_utm) -
               data_center_->vehicle_state_utm().Heading()) <
      config::PlanningConfig::Instance()
          ->plan_config()
          .parking.horizontal_parking_space.finish_heading_threshold) {
    parking_out_finish = true;
  }

  if (parking_out_finish) {
    is_state_change_ = true;
    curr_decision_ = ParkingStageChangeFlag::T_PARKING_OUT_FINISH;
  }
}

void ScenarioParkingDecider::OnHandleStateFinish() { ; }

bool ScenarioParkingDecider::UpdateStateMachine() {
  if (prev_decision_ == curr_decision_) {
    return true;
  }
  auto prev_state = curr_state_str_;
  if (!state_machine_.ChangeState(curr_decision_)) {
    LOG_ERROR("{}: state change fail. state: {}, changeflag: {}", name_,
              curr_state_str_,
              ParkingStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    curr_state_ =
        static_cast<ParkingStageState::State>(state_machine_.GetCurrentState());
    prev_decision_ = curr_decision_;
    LOG_INFO("{}: update current_decision: ({}) {} {}", Name(),
             state_machine_.GetChangeFlagStr(curr_decision_), prev_state,
             curr_state_str_);
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive

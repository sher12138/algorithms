#include "scenario_motorway_detour_decider.h"

namespace neodrive {
namespace planning {

constexpr int ScenarioMotorwayDetourDecider::kFilterFrameThreshold;

ScenarioMotorwayDetourDecider::ScenarioMotorwayDetourDecider() {
  name_ = "ScenarioMotorwayDetourDecider";
}

bool ScenarioMotorwayDetourDecider::Init() {
  if (initialized_) {
    return true;
  }

  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_motorway_detour_stage";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }
  plan_config_ptr_ = &config::PlanningConfig::Instance()->plan_config();

  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioMotorwayDetourDecider init.");
  return true;
}

bool ScenarioMotorwayDetourDecider::ResetStateMachine() {
  // considering the state transform filter, reset state to PREPARE instead of
  // INIT.
  if (!state_machine_.SetInitializeState("INIT")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }

  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(static_cast<MotorwayDetourStageState::State>(
      state_machine_.GetCurrentState()));
  prev_decision_ = MotorwayDetourStageChangeFlag::T_NONE_INIT;
  curr_decision_ = MotorwayDetourStageChangeFlag::T_NONE_INIT;
  lane_bound_weight_ = 1.0;
  return true;
}

bool ScenarioMotorwayDetourDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioMotorwayDetourDecider.");
  return true;
}

ErrorCode ScenarioMotorwayDetourDecider::RunOnce() {
  LOG_INFO("ScenarioMotorwayDetourDecider::RunOnce");
  // change stage.
  is_state_change_ = false;
  if (!OnHandleAllStates()) {
    auto it = state_machine_function_map_.find(curr_state());
    if (it != state_machine_function_map_.end()) {
      auto func = it->second;
      (this->*func)();
    } else {
      // missing handle state function
      LOG_ERROR("missing handle state function {}", curr_state_str_);
    }
  }
  if (is_state_change_) {
    if (!UpdateStateMachine()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }

  // stage process.
  // decision logic business here.
  if (curr_state() == MotorwayDetourStageState::PREPARE) {
    if (!ProcessStagePrepare()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  } else if (curr_state() == MotorwayDetourStageState::BORROWING) {
    if (!ProcessStageBorrowing()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  } else if (curr_state() == MotorwayDetourStageState::REVERSE_LANE_BORROWING) {
    if (!ProcessStageReverseLaneBorrowing()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  } else if (curr_state() == MotorwayDetourStageState::EXIT) {
    if (!ProcessStageExit()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }
  // process all stage.
  if (!ProcessAllStage()) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  SaveMonitorMessage();
  return ErrorCode::PLANNING_OK;
}

bool ScenarioMotorwayDetourDecider::ProcessStagePrepare() { return true; }

bool ScenarioMotorwayDetourDecider::ProcessStageBorrowing() { return true; }

bool ScenarioMotorwayDetourDecider::ProcessStageReverseLaneBorrowing() {
  return true;
}

bool ScenarioMotorwayDetourDecider::ProcessStageExit() { return true; }

bool ScenarioMotorwayDetourDecider::ProcessAllStage() {
  SetLaneBorrowTurnType();
  SetLaneBorrowLaneExtendRatio();
  auto& task_info = data_center_->mutable_task_info_list()->front();
  scenario_common::ComputeRefL(task_info, motorway_config_.enable_bias_drive);
  scenario_common::RefLFilter(task_info);
  return true;
}

void ScenarioMotorwayDetourDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<MotorwayDetourStageState::State,
                void (ScenarioMotorwayDetourDecider::*)()>(
          MotorwayDetourStageState::INIT,
          &ScenarioMotorwayDetourDecider::OnHandleStateInit));
  state_machine_function_map_.insert(
      std::pair<MotorwayDetourStageState::State,
                void (ScenarioMotorwayDetourDecider::*)()>(
          MotorwayDetourStageState::PREPARE,
          &ScenarioMotorwayDetourDecider::OnHandleStatePrepare));
  state_machine_function_map_.insert(
      std::pair<MotorwayDetourStageState::State,
                void (ScenarioMotorwayDetourDecider::*)()>(
          MotorwayDetourStageState::BORROWING,
          &ScenarioMotorwayDetourDecider::OnHandleStateBorrowing));
  state_machine_function_map_.insert(
      std::pair<MotorwayDetourStageState::State,
                void (ScenarioMotorwayDetourDecider::*)()>(
          MotorwayDetourStageState::REVERSE_LANE_BORROWING,
          &ScenarioMotorwayDetourDecider::OnHandleStateReverseLaneBorrowing));
  state_machine_function_map_.insert(
      std::pair<MotorwayDetourStageState::State,
                void (ScenarioMotorwayDetourDecider::*)()>(
          MotorwayDetourStageState::EXIT,
          &ScenarioMotorwayDetourDecider::OnHandleStateExit));
}

bool ScenarioMotorwayDetourDecider::UpdateStateMachine() {
  // filter decision
  static int decision_filter_count = 0;
  if (prev_decision_ == curr_decision_) {
    // reset count.
    decision_filter_count = 0;
    return true;
  }
  if (curr_decision_ == filter_decision_) {
    ++decision_filter_count;
  } else if (curr_state() == MotorwayDetourStageState::INIT) {
    filter_decision_ = curr_decision_;
    decision_filter_count = kFilterFrameThreshold;
  } else {
    // new decision, update filter_decision and count 1.
    filter_decision_ = curr_decision_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO("filter decision: {}",
             MotorwayDetourStageChangeFlag::ChangeFlag_Name(curr_decision_));
  }
  if (decision_filter_count < kFilterFrameThreshold) {
    // hold state
    return true;
  }
  // update state.
  auto prev_state = curr_state_str_;
  if (!state_machine_.ChangeState(curr_decision_)) {
    LOG_ERROR("{}: state change fail. state: {}, changeflag: {}", name_,
              curr_state_str_,
              MotorwayDetourStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    set_curr_state(static_cast<MotorwayDetourStageState::State>(
        state_machine_.GetCurrentState()));
    prev_decision_ = curr_decision_;
    if (curr_state_str_ != prev_state) {
      prev_state_str_ = prev_state;
      LOG_INFO("{}: update detour stage: ({}) {} {}", Name(),
               state_machine_.GetChangeFlagStr(curr_decision_), prev_state,
               curr_state_str_);
    }
  }
  return true;
}

bool ScenarioMotorwayDetourDecider::OnHandleAllStates() { return false; }

void ScenarioMotorwayDetourDecider::OnHandleStateInit() {
  auto& motorway_lane_borrow_context =
      data_center_->mutable_master_info()->motorway_lane_borrow_context();
  curr_decision_ = MotorwayDetourStageChangeFlag::T_NONE_INIT;
  if (!motorway_lane_borrow_context.is_refer_lane_static_obs_clear &&
      !motorway_lane_borrow_context.outside_finish_signal) {
    is_state_change_ = true;
    curr_decision_ = MotorwayDetourStageChangeFlag::T_INIT_PREPARE;
    auto& task_info = data_center_->task_info_list().front();
    start_l_ = std::fabs(task_info.curr_sl().l());
  }
}

void ScenarioMotorwayDetourDecider::OnHandleStatePrepare() {
  auto& motorway_lane_borrow_context =
      data_center_->mutable_master_info()->motorway_lane_borrow_context();
  auto& reverse_lane_detour_context =
      data_center_->mutable_master_info()->reverse_lane_detour_context();

  do {
    if (motorway_lane_borrow_context.is_refer_lane_static_obs_clear ||
        motorway_lane_borrow_context.outside_finish_signal) {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ + kDeltWeightShrink, 0.0, 1.0);
      if (IsAdcInLaneWithoutRoadBoundSpeedLimit() &&
          lane_bound_weight_ > kWeightThresh) {
        is_state_change_ = true;
        curr_decision_ = MotorwayDetourStageChangeFlag::T_PREPARE_EXIT;
        break;
      }
    } else {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ - kDeltWeightExtend, 0.0, 1.0);
    }

    if (!motorway_lane_borrow_context.is_adc_on_refer_lane) {
      is_state_change_ = true;
      LOG_INFO("adc is not in refer lane!");
      auto& task_info = data_center_->task_info_list().front();
      if (IsReverseLaneDetour(task_info.curr_referline_pt())) {
        curr_decision_ =
            MotorwayDetourStageChangeFlag::T_PREPARE_REVERSE_LANE_BORROWING;
      } else {
        curr_decision_ = MotorwayDetourStageChangeFlag::T_PREPARE_BORROWING;
      }
      break;
    }
    // keep prepare.
  } while (0);
  return;
}

void ScenarioMotorwayDetourDecider::OnHandleStateBorrowing() {
  auto& motorway_lane_borrow_context =
      data_center_->mutable_master_info()->motorway_lane_borrow_context();
  do {
    if (motorway_lane_borrow_context.is_refer_lane_static_obs_clear ||
        motorway_lane_borrow_context.outside_finish_signal) {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ + kDeltWeightShrink, 0.0, 1.0);
      auto& task_info = data_center_->task_info_list().front();
      double width_threshold = VehicleParam::Instance()->width() * 0.5;
      double end_l = std::fabs(task_info.curr_sl().l());
      if (IsAdcInLaneWithoutRoadBoundSpeedLimit() &&
          (lane_bound_weight_ > kWeightThresh) &&
          (motorway_lane_borrow_context.is_adc_on_refer_lane ||
           end_l <= width_threshold)) {
        is_state_change_ = true;
        curr_decision_ = MotorwayDetourStageChangeFlag::T_BORROWING_EXIT;
        break;
      }
    } else {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ - kDeltWeightExtend, 0.0, 1.0);
    }
    // keep borrowing. do nothing.
  } while (0);
  return;
}

void ScenarioMotorwayDetourDecider::OnHandleStateReverseLaneBorrowing() {
  auto& motorway_lane_borrow_context =
      data_center_->mutable_master_info()->motorway_lane_borrow_context();
  do {
    if (motorway_lane_borrow_context.is_refer_lane_static_obs_clear ||
        motorway_lane_borrow_context.outside_finish_signal) {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ + kDeltWeightShrink, 0.0, 1.0);
      auto& task_info = data_center_->task_info_list().front();
      double width_threshold = VehicleParam::Instance()->width() * 0.5;
      double end_l = std::fabs(task_info.curr_sl().l());
      if (IsAdcInLaneWithoutRoadBoundSpeedLimit() &&
          (lane_bound_weight_ > kWeightThresh) &&
          (end_l <= start_l_ + 0.5 || end_l <= width_threshold)) {
        is_state_change_ = true;
        curr_decision_ =
            MotorwayDetourStageChangeFlag::T_REVERSE_LANE_BORROWING_EXIT;
        break;
      }
    } else {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ - kDeltWeightExtend, 0.0, 1.0);
    }
    // keep borrowing. do nothing.
  } while (0);
  return;
}

void ScenarioMotorwayDetourDecider::OnHandleStateExit() {
  auto& motorway_lane_borrow_context =
      data_center_->mutable_master_info()->motorway_lane_borrow_context();
  do {
    if (motorway_lane_borrow_context.is_refer_lane_static_obs_clear ||
        motorway_lane_borrow_context.outside_finish_signal) {
      is_state_change_ = true;
      curr_decision_ = MotorwayDetourStageChangeFlag::T_NONE_INIT;
      break;
    } else if (motorway_lane_borrow_context.is_adc_on_refer_lane) {
      is_state_change_ = true;
      curr_decision_ = MotorwayDetourStageChangeFlag::T_EXIT_PREPARE;
      break;
    }
  } while (0);
  return;
}

void ScenarioMotorwayDetourDecider::SetLaneBorrowTurnType() {
  auto& motorway_lane_borrow_context =
      data_center_->mutable_master_info()->motorway_lane_borrow_context();
  int turn_direction = motorway_lane_borrow_context.borrow_side ==
                               MotorwayLaneBorrowContext::BorrowSide::Left
                           ? 1
                           : 2;
  if (curr_state() == MotorwayDetourStageState::PREPARE) {
    data_center_->mutable_master_info()->set_lane_borrow_type(turn_direction);
  } else if (curr_state() == MotorwayDetourStageState::EXIT &&
             !motorway_lane_borrow_context.is_adc_on_refer_lane) {
    data_center_->mutable_master_info()->set_lane_borrow_type(
        turn_direction == 1 ? 2 : 1);
  } else if (curr_state() == MotorwayDetourStageState::BORROWING) {
    data_center_->mutable_master_info()->set_lane_borrow_type(3);
  } else if (curr_state() == MotorwayDetourStageState::REVERSE_LANE_BORROWING) {
    data_center_->mutable_master_info()->set_lane_borrow_type(3);
  } else {
    data_center_->mutable_master_info()->set_lane_borrow_type(0);
  }
}

void ScenarioMotorwayDetourDecider::SetLaneBorrowLaneExtendRatio() {
  LOG_INFO("set lane_bound_weight:{:.4f} in {}", lane_bound_weight_, Name());
  data_center_->mutable_master_info()
      ->mutable_motorway_lane_borrow_context()
      ->lane_borrow_extend_ratio = lane_bound_weight_;
}

void ScenarioMotorwayDetourDecider::SaveMonitorMessage() {
  LOG_INFO("Motorway Detour Running: {}",
           scenario_common::PrintDetourInfo(true, true, true));
}

bool ScenarioMotorwayDetourDecider::IsAdcInLaneWithoutRoadBoundSpeedLimit() {
  auto& task_info = data_center_->task_info_list().front();
  ReferencePoint pt = task_info.curr_referline_pt();
  double adc_right_l = task_info.adc_boundary().start_l();
  double adc_left_l = task_info.adc_boundary().end_l();

  LOG_INFO(
      "adc_left_l:{:.4f}, adc_right_l:{:.4f}, left lane/road/bound:{:.4f}, "
      "{:.4f}, {:.4f},  right lane/road/bound:{:.4f}, {:.4f}, {:.4f}",
      adc_left_l, adc_right_l, pt.left_lane_bound(), pt.left_road_bound(),
      pt.left_bound(), -pt.right_lane_bound(), -pt.right_road_bound(),
      -pt.right_bound());

  return adc_left_l <
             pt.left_lane_bound() - FLAGS_planning_speed_border_shrink_dis &&
         adc_right_l >
             -pt.right_lane_bound() + FLAGS_planning_speed_border_shrink_dis;
}

bool ScenarioMotorwayDetourDecider::IsReverseLaneDetour(
    const ReferencePoint& ref_pt) {
  /// 1.Read the information of the reference point
  BoundaryEdgeType left_boundary_edge_type = ref_pt.left_boundary_edge_type();
  std::vector<DividerFeature> left_divider_feature =
      ref_pt.left_divider_feature();
  double left_road_bound = ref_pt.left_road_bound(),
         left_reverse_road_bound = ref_pt.left_reverse_road_bound();
  LOG_INFO(
      "ref_pt infos x:{:.4f}, y:{:.4f}, is_left_lane:{}, "
      "left_boundary_edge_type:{}, left_road_bound:{:.4f}, "
      "left_reverse_road_bound:{:.4f}",
      ref_pt.x(), ref_pt.y(), ref_pt.is_left_lane(), left_boundary_edge_type,
      left_road_bound, left_reverse_road_bound);

  /// 2.Information updates
  if (!ref_pt.is_left_lane()) return false;
  if (left_boundary_edge_type != BoundaryEdgeType::MARKING) return false;
  int cross_cnt = 0;
  for (int i = 0; i < left_divider_feature.size(); ++i) {
    DividerFeature divider_feature = left_divider_feature[i];
    if (scenario_common::CanCrossLane(divider_feature.divider_type_,
                                      divider_feature.divider_color_))
      cross_cnt++;
  }
  if (cross_cnt != left_divider_feature.size()) return false;
  if (std::abs(left_road_bound - left_reverse_road_bound) < 1e-5) return false;

  return true;
}

}  // namespace planning
}  // namespace neodrive
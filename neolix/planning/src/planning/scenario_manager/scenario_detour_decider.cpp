#include "scenario_detour_decider.h"

namespace neodrive {
namespace planning {

constexpr int ScenarioDetourDecider::kFilterFrameThreshold;

ScenarioDetourDecider::ScenarioDetourDecider() {
  name_ = "ScenarioDetourDecider";
}

bool ScenarioDetourDecider::Init() {
  if (initialized_) {
    return true;
  }
  // init state machine.
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_detour_stage";
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
  LOG_INFO("ScenarioDetourDecider init.");
  return true;
}

bool ScenarioDetourDecider::ResetStateMachine() {
  // considering the state transform filter, reset state to PREPARE instead of
  // INIT.
  if (!state_machine_.SetInitializeState("INIT")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }

  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(
      static_cast<DetourStageState::State>(state_machine_.GetCurrentState()));
  prev_decision_ = DetourStageChangeFlag::T0_NONE_INIT;
  curr_decision_ = DetourStageChangeFlag::T0_NONE_INIT;
  lane_bound_weight_ = 1.0;
  return true;
}

bool ScenarioDetourDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioDetourDecider.");
  return true;
}

ErrorCode ScenarioDetourDecider::RunOnce() {
  LOG_INFO("ScenarioDetourDecider::RunOnce");
  // change stage.
  is_state_change_ = false;
  if (OnHandleAllStates() == false) {
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
  if (curr_state() == DetourStageState::PREPARE) {
    if (!ProcessStagePrepare()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  } else if (curr_state() == DetourStageState::BORROWING) {
    if (!ProcessStageBorrowing()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  } else if (curr_state() == DetourStageState::REVERSE_LANE_BORROWING) {
    if (!ProcessStageReverseLaneBorrowing()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  } else if (curr_state() == DetourStageState::EXIT) {
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

bool ScenarioDetourDecider::ProcessStagePrepare() { return true; }

bool ScenarioDetourDecider::ProcessStageBorrowing() { return true; }

bool ScenarioDetourDecider::ProcessStageReverseLaneBorrowing() { return true; }

bool ScenarioDetourDecider::ProcessStageExit() { return true; }

bool ScenarioDetourDecider::ProcessAllStage() {
  SetLaneBorrowTurnType();
  SetLaneBorrowLaneExtendRatio();

  return true;
}

void ScenarioDetourDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<DetourStageState::State, void (ScenarioDetourDecider::*)()>(
          DetourStageState::INIT, &ScenarioDetourDecider::OnHandleStateInit));
  state_machine_function_map_.insert(
      std::pair<DetourStageState::State, void (ScenarioDetourDecider::*)()>(
          DetourStageState::PREPARE,
          &ScenarioDetourDecider::OnHandleStatePrepare));
  state_machine_function_map_.insert(
      std::pair<DetourStageState::State, void (ScenarioDetourDecider::*)()>(
          DetourStageState::BORROWING,
          &ScenarioDetourDecider::OnHandleStateBorrowing));
  state_machine_function_map_.insert(
      std::pair<DetourStageState::State, void (ScenarioDetourDecider::*)()>(
          DetourStageState::REVERSE_LANE_BORROWING,
          &ScenarioDetourDecider::OnHandleStateReverseLaneBorrowing));
  state_machine_function_map_.insert(
      std::pair<DetourStageState::State, void (ScenarioDetourDecider::*)()>(
          DetourStageState::EXIT, &ScenarioDetourDecider::OnHandleStateExit));
}

bool ScenarioDetourDecider::UpdateStateMachine() {
  // filter decision.
  static int decision_filter_count = 0;
  if (prev_decision_ == curr_decision_) {
    // reset count.
    decision_filter_count = 0;
    return true;
  }
  if (curr_decision_ == filter_decision_) {
    ++decision_filter_count;
  } else if (curr_state() == DetourStageState::INIT) {
    filter_decision_ = curr_decision_;
    decision_filter_count = kFilterFrameThreshold;
  } else {
    // new decision, update filter_decision and count 1.
    filter_decision_ = curr_decision_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO("filter decision: {}",
             DetourStageChangeFlag::ChangeFlag_Name(curr_decision_));
  }
  if (decision_filter_count < kFilterFrameThreshold) {
    // hold state.
    return true;
  }
  // update state.
  auto prev_state = curr_state_str_;
  if (!state_machine_.ChangeState(curr_decision_)) {
    LOG_ERROR("{}: state change fail. state: {}, changeflag: {}", name_,
              curr_state_str_,
              DetourStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    set_curr_state(
        static_cast<DetourStageState::State>(state_machine_.GetCurrentState()));
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

bool ScenarioDetourDecider::OnHandleAllStates() { return false; }

void ScenarioDetourDecider::OnHandleStateInit() {
  auto& lane_borrow_context =
      data_center_->mutable_master_info()->lane_borrow_context();
  curr_decision_ = DetourStageChangeFlag::T0_NONE_INIT;
  if (!lane_borrow_context.is_refer_lane_static_obs_clear &&
      !lane_borrow_context.outside_finish_signal) {
    is_state_change_ = true;
    curr_decision_ = DetourStageChangeFlag::T1_INIT_PREPARE;
    auto& task_info = data_center_->task_info_list().front();
    start_l_ = std::fabs(task_info.curr_sl().l());
  }
}

void ScenarioDetourDecider::OnHandleStatePrepare() {
  auto& lane_borrow_context =
      data_center_->mutable_master_info()->lane_borrow_context();
  auto& reverse_lane_detour_context =
      data_center_->mutable_master_info()->reverse_lane_detour_context();

  do {
    if (lane_borrow_context.is_refer_lane_static_obs_clear ||
        lane_borrow_context.outside_finish_signal) {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ + kDeltWeightShrink, 0.0, 1.0);
      if (IsAdcInLaneWithoutRoadBoundSpeedLimit() &&
          lane_bound_weight_ > kWeightThresh) {
        is_state_change_ = true;
        curr_decision_ = DetourStageChangeFlag::T3_PREPARE_EXIT;
        break;
      }
    } else {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ - kDeltWeightExtend, 0.0, 1.0);
    }

    if (!lane_borrow_context.is_adc_on_refer_lane) {
      is_state_change_ = true;
      LOG_INFO("adc is not in refer lane!");
      auto& task_info = data_center_->task_info_list().front();
      if (IsReverseLaneDetour(task_info.curr_referline_pt())) {
        curr_decision_ =
            DetourStageChangeFlag::T8_PREPARE_REVERSE_LANE_BORROWING;
      } else {
        curr_decision_ = DetourStageChangeFlag::T2_PREPARE_BORROWING;
      }
      break;
    }
    // keep prepare.
  } while (0);
  return;
}
void ScenarioDetourDecider::OnHandleStateBorrowing() {
  auto& lane_borrow_context =
      data_center_->mutable_master_info()->lane_borrow_context();
  do {
    if (lane_borrow_context.is_refer_lane_static_obs_clear ||
        lane_borrow_context.outside_finish_signal) {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ + kDeltWeightShrink, 0.0, 1.0);
      auto& task_info = data_center_->task_info_list().front();
      double width_threshold = VehicleParam::Instance()->width() * 0.5;
      double end_l = std::fabs(task_info.curr_sl().l());
      if (IsAdcInLaneWithoutRoadBoundSpeedLimit() &&
          (lane_bound_weight_ > kWeightThresh) &&
          (end_l <= start_l_ + 0.5 || end_l <= width_threshold)) {
        is_state_change_ = true;
        curr_decision_ = DetourStageChangeFlag::T5_BORROWING_EXIT;
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

void ScenarioDetourDecider::OnHandleStateReverseLaneBorrowing() {
  auto& lane_borrow_context =
      data_center_->mutable_master_info()->lane_borrow_context();
  do {
    if (lane_borrow_context.is_refer_lane_static_obs_clear ||
        lane_borrow_context.outside_finish_signal) {
      lane_bound_weight_ =
          std::clamp(lane_bound_weight_ + kDeltWeightShrink, 0.0, 1.0);
      auto& task_info = data_center_->task_info_list().front();
      double width_threshold = VehicleParam::Instance()->width() * 0.5;
      double end_l = std::fabs(task_info.curr_sl().l());
      if (IsAdcInLaneWithoutRoadBoundSpeedLimit() &&
          (lane_bound_weight_ > kWeightThresh) &&
          (lane_borrow_context.is_adc_on_refer_lane ||
           end_l <= width_threshold)) {
        is_state_change_ = true;
        curr_decision_ = DetourStageChangeFlag::T9_REVERSE_LANE_BORROWING_EXIT;
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

void ScenarioDetourDecider::OnHandleStateExit() {
  auto& lane_borrow_context =
      data_center_->mutable_master_info()->lane_borrow_context();
  do {
    if (lane_borrow_context.is_refer_lane_static_obs_clear ||
        lane_borrow_context.outside_finish_signal) {
      is_state_change_ = true;
      curr_decision_ = DetourStageChangeFlag::T0_NONE_INIT;
      break;
    } else if (lane_borrow_context.is_adc_on_refer_lane) {
      is_state_change_ = true;
      curr_decision_ = DetourStageChangeFlag::T6_EXIT_PREPARE;
      break;
    }
  } while (0);
  return;
}

void ScenarioDetourDecider::SetLaneBorrowTurnType() {
  // do nothing. use default judgement of turn.
  auto& lane_borrow_context =
      data_center_->mutable_master_info()->lane_borrow_context();
  int turn_direction =
      lane_borrow_context.borrow_side == LaneBorrowContext::BorrowSide::Left
          ? 1
          : 2;
  if (curr_state() == DetourStageState::PREPARE) {
    data_center_->mutable_master_info()->set_lane_borrow_type(turn_direction);
  } else if (curr_state() == DetourStageState::EXIT &&
             lane_borrow_context.is_adc_on_refer_lane == false) {
    data_center_->mutable_master_info()->set_lane_borrow_type(
        turn_direction == 1 ? 2 : 1);
  } else if (curr_state() == DetourStageState::BORROWING) {
    data_center_->mutable_master_info()->set_lane_borrow_type(
        3);  // double light.
  } else if (curr_state() == DetourStageState::REVERSE_LANE_BORROWING) {
    data_center_->mutable_master_info()->set_lane_borrow_type(
        3);  // double light.
  } else {
    data_center_->mutable_master_info()->set_lane_borrow_type(0);
  }
}

void ScenarioDetourDecider::SetLaneBorrowLaneExtendRatio() {
  LOG_INFO("set lane_bound_weight:{:.4f} in {}", lane_bound_weight_, Name());
  data_center_->mutable_master_info()
      ->mutable_lane_borrow_context()
      ->lane_borrow_extend_ratio = lane_bound_weight_;
}

void ScenarioDetourDecider::SaveMonitorMessage() {
  LOG_INFO("Detour Running: {}",
           scenario_common::PrintDetourInfo(true, true, false));
}

bool ScenarioDetourDecider::IsAdcInLaneWithoutRoadBoundSpeedLimit() {
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

bool ScenarioDetourDecider::IsReverseLaneDetour(const ReferencePoint& ref_pt) {
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

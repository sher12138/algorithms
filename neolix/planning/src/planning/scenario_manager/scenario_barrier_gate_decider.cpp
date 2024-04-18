#include "scenario_barrier_gate_decider.h"

namespace neodrive {
namespace planning {

void ScenarioBarrierGateDecider::VisBarrierGate() {
  if (!FLAGS_planning_enable_vis_event) return;

  auto barrier_gate =
      data_center_->master_info().barrier_gate_context().barrier_gate;
  double start_s = barrier_gate.start_s;
  double end_s = barrier_gate.end_s;

  auto &task_info = data_center_->task_info_list().front();
  auto reference_line = task_info.reference_line();

  ReferencePoint start_pt{}, end_pt{};
  if (!reference_line->GetNearestRefPoint(start_s, &start_pt)) {
    LOG_INFO("fail to get nearest ref point of barrier gate start_s!");
    return;
  }
  if (!reference_line->GetNearestRefPoint(end_s, &start_pt)) {
    LOG_INFO("fail to get nearest ref point of barrier gate end_s!");
    return;
  }
  double start_left_l = start_pt.left_bound();
  double start_right_l = -start_pt.right_bound();
  double end_left_l = end_pt.left_bound();
  double end_right_l = -end_pt.right_bound();
  std::vector<SLPoint> barrier_gate_box_sl{
      SLPoint(start_s, start_right_l), SLPoint(start_s, start_left_l),
      SLPoint(end_s, end_left_l), SLPoint(end_s, end_right_l)};
  std::vector<Vec2d> barrier_gate_box_xy;
  for (int i = 0; i < barrier_gate_box_sl.size(); ++i) {
    SLPoint sl = barrier_gate_box_sl[i];
    Vec2d xy;
    if (!reference_line->GetPointInCartesianFrame(sl, &xy)) {
      LOG_INFO("fail to get cartesian point of barrier gate box!");
      return;
    }
    barrier_gate_box_xy.emplace_back(xy);
  }

  auto event = vis::EventSender::Instance()->GetEvent("barrier gate");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  auto polygon = event->mutable_polygon()->Add();
  for (auto &xy : barrier_gate_box_xy) {
    set_pt(polygon->add_point(), xy);
  }
}

constexpr int ScenarioBarrierGateDecider::kFilterFrameThreshold;

ScenarioBarrierGateDecider::ScenarioBarrierGateDecider() {
  name_ = "ScenarioBarrierGateDecider";
}

bool ScenarioBarrierGateDecider::Init() {
  if (initialized_) {
    return true;
  }

  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_barrier_gate_stage";
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
  LOG_INFO("ScenarioBarrierGateDecider init.");
  return true;
}

bool ScenarioBarrierGateDecider::Reset() {
  if (!ResetStateMachine()) return false;
  LOG_INFO("Reset ScenarioBarrierGateDecider.");
  return true;
}

ErrorCode ScenarioBarrierGateDecider::RunOnce() {
  LOG_INFO("ScenarioBarrierGateDecider::RunOnce");

  VisBarrierGate();

  // change stage.
  is_state_change_ = false;
  auto it = state_machine_function_map_.find(curr_state());
  if (it != state_machine_function_map_.end()) {
    auto func = it->second;
    (this->*func)();
  } else {
    // missing handle state function
    LOG_ERROR("missing handle state function {}", curr_state_str_);
  }
  if (is_state_change_) {
    if (!UpdateStateMachine()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }
  return ErrorCode::PLANNING_OK;
}

bool ScenarioBarrierGateDecider::ResetStateMachine() {
  if (!state_machine_.SetInitializeState("INIT")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }

  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(static_cast<BarrierGateStageState::State>(
      state_machine_.GetCurrentState()));
  prev_decision_ = BarrierGateStageChangeFlag::T_NONE_INIT;
  curr_decision_ = BarrierGateStageChangeFlag::T_NONE_INIT;
  return true;
}

void ScenarioBarrierGateDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<BarrierGateStageState::State,
                void (ScenarioBarrierGateDecider::*)()>(
          BarrierGateStageState::INIT,
          &ScenarioBarrierGateDecider::OnHandleStateInit));
  state_machine_function_map_.insert(
      std::pair<BarrierGateStageState::State,
                void (ScenarioBarrierGateDecider::*)()>(
          BarrierGateStageState::STOP,
          &ScenarioBarrierGateDecider::OnHandleStateStop));
  state_machine_function_map_.insert(
      std::pair<BarrierGateStageState::State,
                void (ScenarioBarrierGateDecider::*)()>(
          BarrierGateStageState::WAIT,
          &ScenarioBarrierGateDecider::OnHandleStateWait));
  state_machine_function_map_.insert(
      std::pair<BarrierGateStageState::State,
                void (ScenarioBarrierGateDecider::*)()>(
          BarrierGateStageState::MOVE,
          &ScenarioBarrierGateDecider::OnHandleStateMove));
  state_machine_function_map_.insert(
      std::pair<BarrierGateStageState::State,
                void (ScenarioBarrierGateDecider::*)()>(
          BarrierGateStageState::EXIT,
          &ScenarioBarrierGateDecider::OnHandleStateExit));
}

bool ScenarioBarrierGateDecider::UpdateStateMachine() {
  // filter decision.
  static int decision_filter_count = 0;
  if (prev_decision_ == curr_decision_) {
    // reset count.
    decision_filter_count = 0;
    return true;
  }
  if (curr_decision_ == filter_decision_) {
    ++decision_filter_count;
  } else if (curr_state() == BarrierGateStageState::INIT ||
             curr_state() == BarrierGateStageState::STOP ||
             curr_state() == BarrierGateStageState::WAIT ||
             curr_state() == BarrierGateStageState::MOVE ||
             curr_state() == BarrierGateStageState::EXIT) {
    filter_decision_ = curr_decision_;
    decision_filter_count = kFilterFrameThreshold;
  } else {
    // new decision, update filter_decision and count 1.
    filter_decision_ = curr_decision_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO("filter decision: {}",
             BarrierGateStageChangeFlag::ChangeFlag_Name(curr_decision_));
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
              BarrierGateStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    set_curr_state(static_cast<BarrierGateStageState::State>(
        state_machine_.GetCurrentState()));
    prev_decision_ = curr_decision_;
    if (curr_state_str_ != prev_state) {
      prev_state_str_ = prev_state;
      LOG_INFO("{}: update barrier_gate stage: ({}) {} {}", Name(),
               state_machine_.GetChangeFlagStr(curr_decision_), prev_state,
               curr_state_str_);
    }
  }
  return true;
}

void ScenarioBarrierGateDecider::OnHandleStateInit() {
  print_context();

  curr_decision_ = BarrierGateStageChangeFlag::T_NONE_INIT;
  if (CheckExitFinished()) {
    is_state_change_ = false;
  } else if (CheckExitTriggered()) {
    is_state_change_ = true;
    curr_decision_ = BarrierGateStageChangeFlag::T_INIT_EXIT;
  } else {
    if (CheckInitFinished()) {
      is_state_change_ = true;
      curr_decision_ = BarrierGateStageChangeFlag::T_INIT_STOP;
    } else {
      is_state_change_ = false;
    }
  }

  print_context();
}

void ScenarioBarrierGateDecider::OnHandleStateStop() {
  print_context();

  set_consider_barrier_gate(true);
  set_is_stop_stage(true);

  if (CheckExitTriggered()) {
    is_state_change_ = true;
    curr_decision_ = BarrierGateStageChangeFlag::T_STOP_EXIT;
  } else {
    if (CheckStopFinished()) {
      is_state_change_ = true;
      curr_decision_ = BarrierGateStageChangeFlag::T_STOP_WAIT;
    } else {
      is_state_change_ = false;
    }
  }

  print_context();
}

void ScenarioBarrierGateDecider::OnHandleStateWait() {
  print_context();

  set_is_wait_stage(true);

  // log monitor
  // data_center_->mutable_event_report_proxy()->SetEvent(
  //     EventType::MEET_BARRIER_GATE);
  data_center_->mutable_event_report_proxy()->SetEvent(
      EventType::PREPROCESS_FAIL);  // Temporary planning failure due to
                                    // cumbersome cabin end modifications

  if (CheckExitTriggered()) {
    is_state_change_ = true;
    curr_decision_ = BarrierGateStageChangeFlag::T_WAIT_EXIT;
  } else {
    if (CheckWaitFinished()) {
      is_state_change_ = true;
      curr_decision_ = BarrierGateStageChangeFlag::T_WAIT_MOVE;
    } else {
      is_state_change_ = false;
    }
  }

  print_context();
}

void ScenarioBarrierGateDecider::OnHandleStateMove() {
  print_context();

  set_consider_barrier_gate(false);
  set_is_move_stage(true);

  if (CheckExitTriggered()) {
    is_state_change_ = true;
    curr_decision_ = BarrierGateStageChangeFlag::T_MOVE_EXIT;
  } else {
    if (CheckMoveFinished()) {
      is_state_change_ = true;
      curr_decision_ = BarrierGateStageChangeFlag::T_MOVE_EXIT;
    } else {
      is_state_change_ = false;
    }
  }

  print_context();
}

void ScenarioBarrierGateDecider::OnHandleStateExit() {
  print_context();

  set_is_barrier_gate_finished(true);

  print_context();
}

bool ScenarioBarrierGateDecider::CheckInitFinished() {
  reset_context();  // clear global vars ,a new start

  bool signal_from_other{true};
  if (signal_from_other) {
    LOG_INFO("INIT finished");
    return true;
  }
  return false;
}

bool ScenarioBarrierGateDecider::CheckStopFinished() {
  auto &ego_v = data_center_->last_frame()->inside_planner_data().vel_v;
  auto &control_command = (*data_center_->control_command_msg.ptr);

  if (ego_v < FLAGS_planning_adc_stop_velocity_threshold + kMathEpsilon) {
    set_is_stop_stage(false);
    LOG_INFO("STOP finished");
    return true;
  }
  return false;
}

bool ScenarioBarrierGateDecider::CheckWaitFinished() {
  bool flag_test = plan_config_ptr_->barrier_gate_scenario.flag_test;
  bool flag_complete_auto_drive =
      plan_config_ptr_->barrier_gate_scenario.flag_complete_auto_drive;

  // flag_test for dv test
  if (flag_test) {
    if (!flag_complete_auto_drive) {
      set_is_wait_stage(false);
      LOG_INFO("WAIT finished");
      return true;
    }
  } else {
    const auto &mode = data_center_->vehicle_state_proxy().DrivingMode();
    if (mode != neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
      set_is_wait_stage(false);
      LOG_INFO("WAIT finished");
      return true;
    }
  }
  return false;
}

bool ScenarioBarrierGateDecider::CheckMoveFinished() {
  auto &barrier_gate =
      data_center_->master_info().barrier_gate_context().barrier_gate;
  auto &init_sl =
      data_center_->last_frame()->inside_planner_data().init_sl_point;

  if (init_sl.s() - VehicleParam::Instance()->back_edge_to_center() >
      barrier_gate.end_s) {
    set_is_move_stage(false);
    LOG_INFO("MOVE finished");
    return true;
  }
  return false;
}

bool ScenarioBarrierGateDecider::CheckExitFinished() {
  auto &cnt =
      data_center_->master_info().barrier_gate_context().scenario_filter_cnt;
  LOG_INFO("scenario_filter_cnt:{}", cnt);
  if (cnt > 0) {
    LOG_INFO("EXIT finished");
    return true;
  }
  return false;
}

bool ScenarioBarrierGateDecider::CheckExitTriggered() {
  // take over signal
  if (data_center_->vehicle_state_proxy().DrivingMode() !=
      neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
    reset_context();  // clear global vars
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace neodrive

#include "scenario_manager.h"

#include "src/common/util/hash_util.h"

namespace neodrive {
namespace planning {
using neodrive::global::data_recorder::EventOfInterest;
using neodrive::global::planning::PilotState;
using neodrive::global::planning::ScenarioChangeFlag;
using neodrive::global::planning::ScenarioState;
using LaneType = neodrive::global::hdmap::Lane_LaneType;
constexpr int ScenarioManager::kFilterFrameThreshold;

ScenarioManager::ScenarioManager() {}

ScenarioManager::~ScenarioManager() {
  state_machine_function_map_.clear();
  all_deciders_.clear();
  scenario_decider_map_.clear();
  scenario_task_pipeline_map_.clear();
}

bool ScenarioManager::Init() {
  if (initialized_) {
    return true;
  }
  // init config.
  plan_config_ptr_ = &config::PlanningConfig::Instance()->plan_config();
  // init state machine.
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_manager";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }
  curr_state_ = ScenarioState::INIT;
  prev_state_str_ = "";
  curr_state_str_ = ScenarioState::State_Name(curr_state_);

  if (!state_machine_.SetInitializeState(curr_state_str_)) {
    LOG_WARN("set state machine {} initialize state failed!",
             state_machine_file);
    return false;
  }
  data_center_->set_prev_state(ScenarioState::INIT);
  curr_change_flag_ = ScenarioChangeFlag::T_INIT_CRUISE;
  RegisterStateMachineResponseFunctions();
  if (!LoadScenarioConfig()) {
    return false;
  }
  auto iter = scenario_decider_map_.find(curr_state_str_);
  if (iter != scenario_decider_map_.end()) {
    curr_stage_decider_ = iter->second;
    LOG_INFO("init curr_stage_decider_: {} {}", curr_state_str_,
             curr_stage_decider_->Name());
  } else {
    LOG_ERROR("find stage decider error: {}", curr_state_str_);
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioManager::Init");
  return true;
}

bool ScenarioManager::LoadScenarioConfig() {
  std::string planning_scenario_config_file =
      "/home/caros/cyberrt/conf/planning_scenario_config.json";
  std::ifstream json_file(planning_scenario_config_file);
  Json::Value planning_scenario_config;
  Json::Reader reader;
  if (!reader.parse(json_file, planning_scenario_config)) {
    LOG_ERROR("Error parsing: {}, can not find file!",
              planning_scenario_config_file);
    return false;
  }

  auto common_tasks_list_names =
      planning_scenario_config["common_tasks_list"].getMemberNames();
  if (common_tasks_list_names.size() != 0) {
    for (auto &common_list_name : common_tasks_list_names) {
      auto &task_list_config =
          planning_scenario_config["common_tasks_list"][common_list_name];
      std::vector<std::string> task_list;
      for (auto &task_name : task_list_config) {
        task_list.push_back(task_name.asString());
      }
      common_tasks_map_.insert(std::pair<std::string, std::vector<std::string>>(
          common_list_name, task_list));
    }
  }

  auto scenarios_names = planning_scenario_config["scenario"].getMemberNames();
  if (scenarios_names.size() != 0) {
    for (auto &scenario_name : scenarios_names) {
      auto &scenario_config =
          planning_scenario_config["scenario"][scenario_name];
      auto decider_name = scenario_config["decider"].asString();
      auto decider_ptr =
          ScenarioStageDeciderFactory::Instance()->Instantiate(decider_name);
      if (decider_ptr == nullptr) {
        LOG_ERROR("ScenarioStageDecider: {} not found", decider_name);
        return false;
      } else {
        decider_ptr->Init();
        auto ret = scenario_decider_map_.insert(
            std::pair<std::string, ScenarioStageDeciderInterfacePtr>(
                scenario_name, decider_ptr));
        if (ret.second) {
          LOG_INFO("add scenario: {} decider: {}", scenario_name, decider_name);
        } else {
          LOG_ERROR("failed add scenario: {} decider: {}", scenario_name,
                    decider_name);
          return false;
        }
      }

      auto &stage_config = scenario_config["stage"];
      auto stage_names = stage_config.getMemberNames();
      if (stage_names.size() != 0) {
        for (auto &stage_name : stage_names) {
          auto &behavior_tasks_config =
              stage_config[stage_name]["behavior_tasks"];
          // for every scenario:behavior_tasks_config.
          std::string map_key =
              scenario_name + '_' + stage_name + "_behavior_tasks";
          std::vector<std::string> task_name_list;
          for (auto &task_name : behavior_tasks_config) {
            task_name_list.push_back(task_name.asString());
          }
          auto behavior_tasks_pipeline_ptr =
              std::make_shared<ScenarioTaskPipeline>();
          if (behavior_tasks_pipeline_ptr == nullptr ||
              !behavior_tasks_pipeline_ptr->Init(map_key, task_name_list,
                                                 common_tasks_map_)) {
            LOG_ERROR("init ScenarioTaskPipeline {} error", map_key);
            return false;
          }
          auto ret = scenario_task_pipeline_map_.insert(
              std::pair<std::string, ScenarioTaskPipelineShrPtr>(
                  map_key, behavior_tasks_pipeline_ptr));
          if (ret.second) {
            LOG_INFO("add behavior_tasks: {} {}", stage_name, map_key);
          } else {
            LOG_ERROR("add behavior_tasks failed: {} {}", stage_name, map_key);
            return false;
          }

          auto &motion_tasks_config = stage_config[stage_name]["motion_tasks"];
          // for every scenario:motion_tasks_config.
          map_key = scenario_name + '_' + stage_name + "_motion_tasks";
          task_name_list.clear();
          for (auto &task_name : motion_tasks_config) {
            task_name_list.push_back(task_name.asString());
          }
          auto motion_tasks_pipeline_ptr =
              std::make_shared<ScenarioTaskPipeline>();
          if (motion_tasks_pipeline_ptr == nullptr ||
              !motion_tasks_pipeline_ptr->Init(map_key, task_name_list,
                                               common_tasks_map_)) {
            LOG_ERROR("init ScenarioTaskPipeline {} error", map_key);
            return false;
          }
          ret = scenario_task_pipeline_map_.insert(
              std::pair<std::string, ScenarioTaskPipelineShrPtr>(
                  map_key, motion_tasks_pipeline_ptr));
          if (ret.second) {
            LOG_INFO("add motion_tasks: {} {}", stage_name, map_key);
          } else {
            LOG_ERROR("add motion_tasks failed: {} {}", stage_name, map_key);
            return false;
          }
        }
      } else {
        LOG_ERROR("scenario: {} is empty!", scenario_name);
      }
    }
  } else {
    LOG_ERROR("{} has no scenario!", planning_scenario_config_file);
    return false;
  }
  return true;
}

ErrorCode ScenarioManager::ExecuteBehaviorTasks(TaskInfo &task_info) {
  SetStaticDetourVariables();
  UpdateLightTurn(task_info);
  data_center_->set_no_abnormal_stop_check(
      scenario_common::IsFrontHasTrafficLight(
          task_info,
          plan_config_ptr_->human_interface.no_abnormal_stop_check_dist));
  auto ret = UpdateScenarioState();
  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Execute UpdateScenarioState failed!");
    return ret;
  }
  ret = UpdateScenarioStage();
  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Execute UpdateScenarioStage failed!");
    return ret;
  }
  ret = ExecuteScenarioStageBehaviorTasks(task_info);
  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Execute ExecuteScenarioStageBehaviorTasks failed!");
    return ret;
  }
  SetLanechangeSpeed();
  scenario_common::UpdateRecordEvents(curr_state_);
  scenario_common::UpdateLaneType(task_info, lane_type_);
  LOG_INFO("test navigator_request:{}",
           data_center_->navigation_result().navigator_request);
  SaveMonitorMessage();
  return ErrorCode::PLANNING_OK;
}

void ScenarioManager::SetStaticDetourVariables() {
  data_center_->mutable_master_info()->set_enable_static_detour(
      (common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .non_motorway.enable_static_obs_detour &&
       !data_center_->task_info_list()
            .front()
            .curr_referline_pt()
            .lane_type_is_indoor_lane())
          ? true
          : false);
  LOG_INFO("enable static detour: [{}], drive strategy: [{}], lane type: [{}]",
           data_center_->master_info().enable_static_detour(),
           common::config::CommonConfig::Instance()
               ->drive_strategy_config()
               .non_motorway.enable_static_obs_detour,
           data_center_->task_info_list()
               .front()
               .curr_referline_pt()
               .lane_type_is_indoor_lane());
}

void ScenarioManager::SetLanechangeSpeed() {
  if (curr_state_ == ScenarioState::MOTORWAY_LANE_CHANGE) {
    return;
  }
  if (data_center_->navigation_result().preview_navigator_request ==
      NO_REQUEST) {
    return;
  }
  auto &lane_change_end_point =
      data_center_->navigation_result().lane_change_end_point;
  LOG_INFO("lane_change_end_point x: {:.4f}, y: {:.4f}",
           lane_change_end_point.x(), lane_change_end_point.y());
  if (std::abs(lane_change_end_point.x()) < 10.0 ||
      std::abs(lane_change_end_point.y()) < 10.0) {
    return;
  }

  ReferencePoint utm_lane_change_end_point;
  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (!task_info.reference_line_raw()->GetNearestRefPoint(
          Vec2d{lane_change_end_point.x(), lane_change_end_point.y()},
          &utm_lane_change_end_point)) {
    return;
  }
  double dis_to_ref = std::sqrt(
      std::pow(lane_change_end_point.x() - utm_lane_change_end_point.x(), 2) +
      std::pow(lane_change_end_point.y() - utm_lane_change_end_point.y(), 2));
  if (dis_to_ref > 1.0) {
    LOG_INFO("lane_change_end_point not on reference line");
    return;
  }

  double dis_to_lanechange_end_point =
      utm_lane_change_end_point.s() - task_info.curr_sl().s();
  double speed_limit = std::max(dis_to_lanechange_end_point / 10.0, 2.0);
  LOG_INFO("dis_to_lanechange_end_point : {:.4f}, speed_limit : {:.4f}",
           dis_to_lanechange_end_point, speed_limit);

  double max_cruise_speed = common::config::CommonConfig::Instance()
                                ->drive_strategy_config()
                                .motor_way.max_cruise_speed;
  if (speed_limit < max_cruise_speed) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::LANE_CHANGE);
    internal_speed_limit.add_upper_bounds(speed_limit);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

void ScenarioManager::UpdateLightTurn(TaskInfo &task_info) {
  if (task_info.last_frame() == nullptr) return;
  double curr_l = task_info.curr_sl().l();
  double last_ref_l = task_info.last_frame()
                          ->outside_planner_data()
                          .path_observe_ref_l_info.observe_ref_l;
  auto &observe_ref_info = task_info.current_frame()
                                      ->mutable_outside_planner_data()
                                      ->path_observe_ref_l_info;
  if (last_ref_l - curr_l > 0.3) {
    observe_ref_info.light_turn = HornLightsCmd::LEFT_TURN;
    LOG_INFO("ego car is going to turn left, curr_l:{:2f}, last_observe_l:{:2f}",
             curr_l, last_ref_l);
  } else if (last_ref_l - curr_l < -0.3) {
    observe_ref_info.light_turn = HornLightsCmd::RIGHT_TURN;
    LOG_INFO(
        "ego car is going to turn right, curr_l: {:2f}, last_observe_l: {:2f}",
        curr_l, last_ref_l);
  } else {
    observe_ref_info.light_turn = HornLightsCmd::NO_TURN;
    LOG_INFO("reset light turn");
  }
}

ErrorCode ScenarioManager::UpdateScenarioState() {
  auto master_info = data_center_->mutable_master_info();
  if (data_center_->global_state_proxy().scenario_manager_need_reset()) {
    curr_state_ = ScenarioState::INIT;
    prev_state_str_ = "";
    curr_state_str_ = ScenarioState::State_Name(curr_state_);
    if (!state_machine_.SetInitializeState(curr_state_str_)) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
    data_center_->set_prev_state(ScenarioState::INIT);
    curr_change_flag_ = ScenarioChangeFlag::T_INIT_CRUISE;
    data_center_->mutable_global_state_proxy()->set_scenario_manager_need_reset(
        false);
  }
  // make decision and update curr_state_str_
  MakeDecision();
  return ErrorCode::PLANNING_OK;
}

ErrorCode ScenarioManager::UpdateScenarioStage() {
  // run scenario decider to decide what stage is
  auto iter = scenario_decider_map_.find(curr_state_str_);
  if (iter != scenario_decider_map_.end()) {
    curr_stage_decider_ = iter->second;
    LOG_INFO("{} {}::RunOnce", curr_state_str_, curr_stage_decider_->Name());
    // when scenario change, scenario stage decider should reset.
    if (is_state_change_ && state_change_finish_) {
      if (!curr_stage_decider_->Reset()) {
        data_center_->mutable_master_info()->set_curr_stage("DEFAULT");
        return ErrorCode::PLANNING_ERROR_FAILED;
      }
    }
    auto ret = curr_stage_decider_->RunOnce();
    data_center_->mutable_master_info()->set_curr_stage(
        curr_stage_decider_->curr_state_str());
    if (ret != ErrorCode::PLANNING_OK) {
      LOG_ERROR("{} {}::RunOnce PLANNING_ERROR_FAILED", curr_state_str_,
                curr_stage_decider_->Name());
      return ret;
    }
  } else {
    LOG_ERROR("scenario decider: {} not found", curr_state_str_);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode ScenarioManager::ExecuteScenarioStageBehaviorTasks(
    TaskInfo &task_info) {
  std::string stage_name = curr_stage_decider_->curr_state_str();
  std::string map_key = curr_state_str_ + '_' + stage_name + "_behavior_tasks";
  auto iter = scenario_task_pipeline_map_.find(map_key);
  if (iter != scenario_task_pipeline_map_.end()) {
    // pipeline runonce.
    LOG_INFO("Scenario Stage Pipeline: {}::ExecuteScenarioStageBehaviorTasks",
             map_key);
    auto ret = iter->second->ExecuteTasks(task_info);
    if (ret != ErrorCode::PLANNING_OK) {
      LOG_ERROR("{}::RunOnce PLANNING_ERROR_FAILED", map_key);
      return ret;
    }
  } else {
    LOG_ERROR("Pipeline: {} not found", map_key);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode ScenarioManager::ExecuteScenarioStageMotionTasks(
    TaskInfo &task_info) {
  std::string stage_name = curr_stage_decider_->curr_state_str();
  std::string map_key = curr_state_str_ + '_' + stage_name + "_motion_tasks";
  auto iter = scenario_task_pipeline_map_.find(map_key);
  if (iter != scenario_task_pipeline_map_.end()) {
    // pipeline runonce.
    LOG_INFO("Scenario Stage Pipeline: {}::ExecuteScenarioStageMotionTasks",
             map_key);
    auto ret = iter->second->ExecuteTasks(task_info);
    if (ret != ErrorCode::PLANNING_OK) {
      freespace_fail_cnt_ = map_key == "BACK_OUT_DEFAULT_motion_tasks"
                                ? freespace_fail_cnt_ + 1
                                : 0;
      LOG_ERROR("{}::RunOnce PLANNING_ERROR_FAILED. BACK_OUT count: {}",
                map_key, freespace_fail_cnt_);
      return ret;
    }
  } else {
    freespace_fail_cnt_ = 0;
    LOG_ERROR("Pipeline: {} not found", map_key);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  freespace_fail_cnt_ = 0;
  return ErrorCode::PLANNING_OK;
}

ScenarioTaskInterfacePtr ScenarioManager::GetDecider(
    const std::string &decider_name) {
  auto iter = all_deciders_.find(decider_name);
  if (iter == all_deciders_.end()) {
    auto scenario_task_factory = ScenarioTaskFactory::Instance();
    auto task_ptr = scenario_task_factory->Instantiate(decider_name);
    if (task_ptr != nullptr) {
      all_deciders_[decider_name] = task_ptr;
      return task_ptr;
    } else {
      LOG_ERROR("failed to instantiate task: {}", decider_name);
      return nullptr;
    }
  } else {
    return iter->second;
  }
}

void ScenarioManager::ResetDeciders() {
  for (auto &decider : all_deciders_) {
    decider.second->Reset();
  }
}

void ScenarioManager::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::INIT, &ScenarioManager::OnHandleStateInit));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::CRUISE,
          &ScenarioManager::OnHandleStateCruiseFollowing));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::MOTORWAY_LANE_CHANGE,
          &ScenarioManager::OnHandleMotorwayStateLaneChange));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::MOTORWAY_INTERSECTION,
          &ScenarioManager::OnHandleMotorwayStateIntersection));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::MOTORWAY_CRUISE,
          &ScenarioManager::OnHandleMotorwayStateCruiseFollowing));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::MOTORWAY_DETOUR,
          &ScenarioManager::OnHandleMotorwayStateDetour));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::DETOUR, &ScenarioManager::OnHandleStateDetour));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::NARROW_RAOD,
          &ScenarioManager::OnHandleStateNarrowRoadPass));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::INTERSECTION,
          &ScenarioManager::OnHandleStateIntersectionPass));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::BACK_OUT, &ScenarioManager::OnHandleStateBackOutPass));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::BARRIER_GATE,
          &ScenarioManager::OnHandleStateBarrierGate));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::SIDE_WAY_INTERSECTION,
          &ScenarioManager::OnHandleStateSideWayIntersectionPass));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::INDOOR_CRUISE,
          &ScenarioManager::OnHandleStateIndoorCruise));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::PARK_CRUISE,
          &ScenarioManager::OnHandleStateParkCruise));
  state_machine_function_map_.insert(
      std::pair<ScenarioState::State, void (ScenarioManager::*)()>(
          ScenarioState::PARKING, &ScenarioManager::OnHandleStateParking));
}

void ScenarioManager::MakeDecision() {
  is_state_change_ = false;
  auto iter = state_machine_function_map_.find(curr_state_);
  if (iter != state_machine_function_map_.end()) {
    auto func_ptr = iter->second;
    (this->*func_ptr)();
  } else {
    LOG_ERROR("State: {} not found!", curr_state_str_);
  }
  UpdateStateMachine();
}

void ScenarioManager::UpdateStateMachine() {
  static int decision_filter_count = 0;
  state_change_finish_ = false;
  if (!is_state_change_) {
    // reset filter.
    decision_filter_count = 0;
    return;
  }
  if (curr_change_flag_ == filter_change_flag_) {
    ++decision_filter_count;
  } else if (curr_state_ == ScenarioState::MOTORWAY_LANE_CHANGE) {
    filter_change_flag_ = curr_change_flag_;
    decision_filter_count = kFilterFrameThreshold;
  } else {
    // new decision, update filter_change_flag and count 1.
    filter_change_flag_ = curr_change_flag_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO("filter decision: {}",
             ScenarioChangeFlag::ChangeFlag_Name(curr_change_flag_));
  }
  if (decision_filter_count < kFilterFrameThreshold) {
    // hold state.
    return;
  }
  if (!state_machine_.ChangeState(curr_change_flag_)) {
    LOG_ERROR("ChangeState error: {} {}", curr_state_str_,
              ScenarioChangeFlag::ChangeFlag_Name(curr_change_flag_));
    return;
  }
  state_change_finish_ = true;
  auto tmp_state = curr_state_;
  auto tmp_state_str = curr_state_str_;
  curr_state_str_ = state_machine_.GetCurrentStateString();
  curr_state_ =
      static_cast<ScenarioState::State>(state_machine_.GetCurrentState());
  if (tmp_state != curr_state_) {
    data_center_->set_prev_state(tmp_state);
    prev_state_str_ = tmp_state_str;
    LOG_INFO("state transform: {}, {} -> {} {}",
             ScenarioChangeFlag::ChangeFlag_Name(curr_change_flag_),
             ScenarioState::State_Name(tmp_state),
             ScenarioState::State_Name(curr_state_), curr_state_str_);
    // update scenario in datacenter.
    data_center_->mutable_master_info()->set_curr_scenario(curr_state_);
  }
}

void ScenarioManager::OnHandleStateInit() {
  LOG_INFO("OnHandleStateInit");
  is_state_change_ = true;
  motorway_cruise_triggered_ = CheckMotorwayCruiseTriggered();
  indoor_cruise_detected_ = DetectIndoorCruise();
  park_cruise_detected_ = DetectParkCruise();
  parking_detected_ = DetectParking();
  data_center_->mutable_global_state_proxy()->set_cruise();
  if (parking_detected_) {
    curr_change_flag_ = ScenarioChangeFlag::T_INIT_PARKING;
  } else if (indoor_cruise_detected_) {
    curr_change_flag_ = ScenarioChangeFlag::T_INIT_INDOOR_CRUISE;
  } else if (park_cruise_detected_) {
    curr_change_flag_ = ScenarioChangeFlag::T_INIT_PARK_CRUISE;
  } else if (motorway_cruise_triggered_) {
    curr_change_flag_ = ScenarioChangeFlag::T_INIT_MOTORWAY_CRUISE;
  } else {
    curr_change_flag_ = ScenarioChangeFlag::T_INIT_CRUISE;
  }
}

void ScenarioManager::OnHandleStateCruiseFollowing() {
  LOG_INFO("OnHandleStateCruiseFollowing");
  do {
    indoor_cruise_detected_ = DetectIndoorCruise();
    if (indoor_cruise_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_INDOOR_CRUISE;
      break;
    }
    park_cruise_detected_ = DetectParkCruise();
    if (park_cruise_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_PARK_CRUISE;
      break;
    }
    motorway_cruise_triggered_ = CheckMotorwayCruiseTriggered();
    if (motorway_cruise_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_MOTORWAY_CRUISE;
      break;
    }

    if (config::PlanningConfig::Instance()
            ->plan_config()
            .barrier_gate_scenario.enable_barrier_gate) {
      barrier_gate_triggered_ = CheckBarrierGateTriggered();
      if (barrier_gate_triggered_) {
        barrier_gate_condition_satisfied_ = CheckBarrierGateCondition();
        if (barrier_gate_condition_satisfied_) {
          is_state_change_ = true;
          curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_BARRIER_GATE;
          break;
        }
      }
    }

    PrepareDetourCondition();
    narrow_road_detected_ = DetectNarrowRoad();
    if (narrow_road_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_NARROW_RAOD;
      break;
    }
    intersection_detected_ = DetectIntersection();
    if (intersection_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_INTERSECTION;
      break;
    }
    side_way_intersection_detected_ = DetectSideWayIntersection();
    if (side_way_intersection_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_SIDE_WAY_INTERSECTION;
      break;
    }
    detour_triggered_ = CheckDetourTriggered();
    if (detour_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_DETOUR;
      break;
    }
    if (back_out_detected_ = DetectBackOut()) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_BACK_OUT;
      break;
    }
    PrepareMotorwayLaneChangeCondition();
    motorway_lane_change_triggered_ = CheckMotorwayLaneChangeTriggered();
    if (motorway_lane_change_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_CRUISE_MOTORWAY_LANE_CHANGE;
      break;
    }
  } while (false);
}

void ScenarioManager::OnHandleStateBarrierGate() {
  LOG_INFO("OnHandleStateBarrierGate");
  do {
    barrier_gate_finished_ = CheckBarrierGateFinished();
    if (barrier_gate_finished_) {
      is_state_change_ = true;
      auto &task_info = data_center_->task_info_list().front();
      curr_change_flag_ =
          task_info.curr_referline_pt().lane_type_is_pure_city_driving()
              ? ScenarioChangeFlag::T_BARRIER_GATE_MOTORWAY_CRUISE
              : ScenarioChangeFlag::T_BARRIER_GATE_CRUISE;
      break;
    }
  } while (false);
}

void ScenarioManager::OnHandleStateDetour() {
  do {
    PrepareDetourCondition();
    detour_finished_ = CheckDetourFinished();
    if (detour_finished_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_DETOUR_CRUISE;
      break;
    }
    if (back_out_detected_ = DetectBackOut()) {
      LOG_INFO("XXX get to BACK_OUT");
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_DETOUR_BACK_OUT;
      break;
    }
    CheckReroutingTriggered();
    PrepareMotorwayLaneChangeCondition();
    motorway_lane_change_triggered_ = CheckMotorwayLaneChangeTriggered();
    if (motorway_lane_change_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_DETOUR_MOTORWAY_LANE_CHANGE;
      break;
    }
  } while (false);
}

void ScenarioManager::OnHandleStateNarrowRoadPass() {
  do {
    indoor_cruise_detected_ = DetectIndoorCruise();
    if (indoor_cruise_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_NARROW_RAOD_INDOOR_CRUISE;
      break;
    }
    park_cruise_detected_ = DetectParkCruise();
    if (park_cruise_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_NARROW_RAOD_PARK_CRUISE;
      break;
    }
    intersection_detected_ = DetectIntersection();
    if (intersection_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_NARROW_RAOD_INTERSECTION;
      break;
    }
    side_way_intersection_detected_ = DetectSideWayIntersection();
    if (side_way_intersection_detected_) {
      is_state_change_ = true;
      curr_change_flag_ =
          ScenarioChangeFlag::T_NARROW_RAOD_SIDE_WAY_INTERSECTION;
      break;
    }
    PrepareDetourCondition();
    detour_triggered_ = CheckDetourTriggered();
    if (detour_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_NARROW_RAOD_DETOUR;
      break;
    }
    narrow_road_pass_finished_ = CheckNarrowRoadPassFinished();
    if (narrow_road_pass_finished_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_NARROW_RAOD_CRUISE;
      break;
    }
  } while (false);  // break here.
}

void ScenarioManager::OnHandleStateIndoorCruise() {
  do {
    indoor_cruise_finished_ = CheckIndoorCruiseFinished();
    park_cruise_detected_ = DetectParkCruise();
    parking_detected_ = DetectParking();
    if (parking_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_INDOOR_CRUISE_PARKING;
      break;
    }
    if (indoor_cruise_finished_) {
      is_state_change_ = true;
      curr_change_flag_ = park_cruise_detected_
                              ? ScenarioChangeFlag::T_INDOOR_CRUISE_PARK_CRUISE
                              : ScenarioChangeFlag::T_INDOOR_CRUISE_CRUISE;
      break;
    }
  } while (false);  // break here.
}

void ScenarioManager::OnHandleStateParkCruise() {
  do {
    indoor_cruise_detected_ = DetectIndoorCruise();
    park_cruise_finished_ = CheckParkCruiseFinished();
    parking_detected_ = DetectParking();
    if (parking_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_PARK_CRUISE_PARKING;
      break;
    }
    if (indoor_cruise_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_PARK_CRUISE_INDOOR_CRUISE;
      break;
    }
    if (park_cruise_finished_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_PARK_CRUISE_CRUISE;
      break;
    }
  } while (false);  // break here.
}

void ScenarioManager::OnHandleStateParking() {
  do {
    if (!CheckParkingFinished()) return;
    auto park_ptr = data_center_->parking_ptr();
    if (park_ptr == nullptr) {
      LOG_INFO("park finished but park ptr is nullptr");
    }
    if (park_ptr && park_ptr->is_park_in()) {
      LOG_INFO("park finished but not park in");
    }
    if (park_ptr && park_ptr->is_park_in()) {
      data_center_->set_planning_clear_task(true);
      data_center_->set_have_task(false);
    }
    is_state_change_ = true;
    data_center_->mutable_global_state_proxy()->SetFinish(true);
    data_center_->set_parking_ptr(nullptr);
    data_center_->GetNavigationSwapContext().parking_space_ptr.Set(nullptr);
    curr_change_flag_ = ScenarioChangeFlag::T_PARKING_CRUISE;
  } while (false);
}

void ScenarioManager::OnHandleStateIntersectionPass() {
  LOG_INFO("OnHandleStateIntersectionPass");
  do {
    // intersection export finished -> detour
    intersection_export_finished_ = CheckIntersectionExportFinished();
    if (intersection_export_finished_) {
      PrepareDetourCondition();
      detour_triggered_ = CheckDetourTriggered();
      if (detour_triggered_) {
        is_state_change_ = true;
        curr_change_flag_ = ScenarioChangeFlag::T_INTERSECTION_DETOUR;
      }
      break;
    }
    // intersection finished -> narrow road
    intersection_pass_finished_ = CheckIntersectionPassFinished();
    if (intersection_pass_finished_) {
      indoor_cruise_detected_ = DetectIndoorCruise();
      park_cruise_detected_ = DetectParkCruise();
      narrow_road_detected_ = DetectNarrowRoad();
      if (indoor_cruise_detected_) {
        is_state_change_ = true;
        curr_change_flag_ = ScenarioChangeFlag::T_INTERSECTION_INDOOR_CRUISE;
      } else if (park_cruise_detected_) {
        is_state_change_ = true;
        curr_change_flag_ = ScenarioChangeFlag::T_INTERSECTION_PARK_CRUISE;
        break;
      } else if (narrow_road_detected_) {
        is_state_change_ = true;
        curr_change_flag_ = ScenarioChangeFlag::T_INTERSECTION_NARROW_RAOD;
      } else {
        is_state_change_ = true;
        curr_change_flag_ = ScenarioChangeFlag::T_INTERSECTION_CRUISE;
      }
      break;
    }
  } while (false);
}

void ScenarioManager::OnHandleStateSideWayIntersectionPass() {
  do {
    PrepareDetourCondition();
    detour_triggered_ = CheckDetourTriggered();
    if (detour_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_SIDE_WAY_INTERSECTION_DETOUR;
      LOG_INFO("change mode from side way intersection to detour");
      break;
    }
    side_way_intersection_finished_ = CheckSideWayIntersectionPassFinished();
    if (side_way_intersection_finished_) {
      indoor_cruise_detected_ = DetectIndoorCruise();
      park_cruise_detected_ = DetectParkCruise();
      narrow_road_detected_ = DetectNarrowRoad();
      if (indoor_cruise_detected_) {
        is_state_change_ = true;
        curr_change_flag_ =
            ScenarioChangeFlag::T_SIDE_WAY_INTERSECTION_INDOOR_CRUISE;
      } else if (park_cruise_detected_) {
        is_state_change_ = true;
        curr_change_flag_ =
            ScenarioChangeFlag::T_SIDE_WAY_INTERSECTION_PARK_CRUISE;
        break;
      } else if (narrow_road_detected_) {
        is_state_change_ = true;
        curr_change_flag_ =
            ScenarioChangeFlag::T_SIDE_WAY_INTERSECTION_NARROW_RAOD;
      } else {
        is_state_change_ = true;
        curr_change_flag_ = ScenarioChangeFlag::T_SIDE_WAY_INTERSECTION_CRUISE;
        LOG_INFO("change mode from side way intersection to cruise");
      }
    }
    break;
  } while (false);
}

void ScenarioManager::OnHandleStateBackOutPass() {
  LOG_INFO("OnHandleStateBackOutPass");
  if (freespace_fail_cnt_ > 50) {
    LOG_INFO("XXX freespace_fail_cnt_: {}. freespace -> cruise",
             freespace_fail_cnt_);
    path_fail_cnt_ = 0;
    speed_fail_cnt_ = 0;
    is_state_change_ = true;
    curr_change_flag_ = ScenarioChangeFlag::T_BACK_OUT_CRUISE;
    return;
  }
  back_out_finished_ = CheckBackOutFinished();
  if (!back_out_finished_) return;

  data_center_->set_motorway_back_out(false);
  ++count_back_out_;
  last_back_out_point_ = {data_center_->vehicle_state_utm().X(),
                          data_center_->vehicle_state_utm().Y()};
  auto detour_choose =
      config::PlanningConfig::Instance()->plan_config().back_out.detour_choose;
  if (detour_choose) {
    LOG_INFO("XXX back_out -> Detour");
    is_state_change_ = true;
    curr_change_flag_ = ScenarioChangeFlag::T_BACK_OUT_DETOUR;
  } else {
    LOG_INFO("XXX back_out -> cruise");
    is_state_change_ = true;
    curr_change_flag_ = ScenarioChangeFlag::T_BACK_OUT_CRUISE;
  }
}

void ScenarioManager::OnHandleMotorwayStateCruiseFollowing() {
  LOG_INFO("OnHandleMotorwayStateCruiseFollowing");
  auto &task_info = data_center_->task_info_list().front();
  do {
    motorway_cruise_finished_ = CheckMotorwayCruiseFinished();
    if (motorway_cruise_finished_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_MOTORWAY_CRUISE_CRUISE;
      break;
    }

    if (config::PlanningConfig::Instance()
            ->plan_config()
            .barrier_gate_scenario.enable_barrier_gate) {
      barrier_gate_triggered_ = CheckBarrierGateTriggered();
      if (barrier_gate_triggered_) {
        barrier_gate_condition_satisfied_ = CheckBarrierGateCondition();
        if (barrier_gate_condition_satisfied_) {
          is_state_change_ = true;
          curr_change_flag_ =
              ScenarioChangeFlag::T_MOTORWAY_CRUISE_BARRIER_GATE;
          break;
        }
      }
    }

    // TODO: add a binary code representing the current state to build a
    // decision tree, which shall resolve the priority of states instead
    PrepareMotorwayLaneChangeCondition();
    motorway_lane_change_triggered_ = CheckMotorwayLaneChangeTriggered();
    if (motorway_lane_change_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ =
          ScenarioChangeFlag::T_MOTORWAY_CRUISE_MOTORWAY_LANE_CHANGE;
      break;
    }

    PrepareMotorwayDetourCondition();
    motorway_detour_triggered_ = CheckMotorwayDetourTriggered();
    if (motorway_detour_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_MOTORWAY_CRUISE_MOTORWAY_DETOUR;
      break;
    }

    auto motorway_lane_borrow_context =
        data_center_->master_info().motorway_lane_borrow_context();
    motorway_intersection_triggered_ = CheckMotorwayIntersectionTriggered();
    if (motorway_intersection_triggered_) {
      data_center_->mutable_master_info()->set_enable_static_detour(false);
      LOG_INFO(
          "motorway_intersection is entering, static detour is not enabled.");
      is_state_change_ = true;
      curr_change_flag_ =
          ScenarioChangeFlag::T_MOTORWAY_CRUISE_MOTORWAY_INTERSECTION;
      break;
    }

    back_out_detected_ = DetectBackOut();
    if (back_out_detected_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_MOTORWAY_CRUISE_BACK_OUT;
      break;
    }

    if ((motorway_lane_borrow_context.is_front_has_traffic_light ||
         motorway_lane_borrow_context.is_front_has_cross_road) &&
        scenario_common::IsCloseStaticDetour(task_info)) {
      data_center_->mutable_master_info()->set_enable_static_detour(false);
      LOG_INFO(
          "front is traffic light/cross road, static detour is not "
          "enabled.");
    }
  } while (false);
}

void ScenarioManager::OnHandleMotorwayStateLaneChange() {
  LOG_INFO("OnHandleMotorwayStateLaneChange");
  PrepareMotorwayLaneChangeCondition();
  motorway_lane_change_finished_ = CheckMotorwayLaneChangeFinished();
  if (motorway_lane_change_finished_) {
    is_state_change_ = true;
    auto &task_info = data_center_->task_info_list().front();

    curr_change_flag_ =
        task_info.curr_referline_pt().lane_type_is_pure_city_driving()
            ? ScenarioChangeFlag::T_MOTORWAY_LANE_CHANGE_MOTORWAY_CRUISE
            : ScenarioChangeFlag::T_MOTORWAY_LANE_CHANGE_CRUISE;
    return;
  }
}

void ScenarioManager::OnHandleMotorwayStateDetour() {
  do {
    PrepareMotorwayDetourCondition();
    motorway_detour_finished_ = CheckMotorwayDetourFinished();
    if (motorway_detour_finished_) {
      is_state_change_ = true;
      curr_change_flag_ = ScenarioChangeFlag::T_MOTORWAY_DETOUR_MOTORWAY_CRUISE;
      break;
    }
    CheckMotorwayReroutingTriggered();
    PrepareMotorwayLaneChangeCondition();
    motorway_lane_change_triggered_ = CheckMotorwayLaneChangeTriggered();
    if (motorway_lane_change_triggered_) {
      is_state_change_ = true;
      curr_change_flag_ =
          ScenarioChangeFlag::T_MOTORWAY_DETOUR_MOTORWAY_LANE_CHANGE;
      break;
    }
  } while (false);
}

bool ScenarioManager::CheckMotorwayCruiseTriggered() {
  if (!common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway) {
    return false;
  }

  auto &task_info = data_center_->task_info_list().front();
  return task_info.curr_referline_pt().lane_type_is_pure_city_driving();
}

bool ScenarioManager::CheckMotorwayLaneChangeTriggered() {
  if (!common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway) {
    return false;
  }
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  if (data_center_->navigation_result().navigator_request != NO_REQUEST &&
      motorway_lane_change_context->is_target_lane_adjacent &&
      data_center_->target_odom_ref() != nullptr) {
    return true;
  }
  return false;
}

bool ScenarioManager::CheckMotorwayCruiseFinished() {
  if (!common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway) {
    return true;
  }

  auto &task_info = data_center_->task_info_list().front();
  return !task_info.curr_referline_pt().lane_type_is_pure_city_driving();
}

void ScenarioManager::OnHandleMotorwayStateIntersection() {
  LOG_INFO("OnHandleMotorwayStateIntersection");
  do {
    motorway_intersection_export_finished_ = CheckIntersectionExportFinished();
    if (motorway_intersection_export_finished_) {
      PrepareMotorwayDetourCondition();
      motorway_detour_triggered_ = CheckMotorwayDetourTriggered();
      if (motorway_detour_triggered_) {
        is_state_change_ = true;
        curr_change_flag_ =
            ScenarioChangeFlag::T_MOTORWAY_INTERSECTION_MOTORWAY_DETOUR;
      }
      break;
    }
    motorway_intersection_finished_ = CheckMotorwayIntersectionFinished();
    if (motorway_intersection_finished_) {
      is_state_change_ = true;
      curr_change_flag_ =
          ScenarioChangeFlag::T_MOTORWAY_INTERSECTION_MOTORWAY_CRUISE;
      break;
    }
    auto &task_info = data_center_->task_info_list().front();
    if (scenario_common::IsCloseStaticDetour(task_info)) {
      data_center_->mutable_master_info()->set_enable_static_detour(false);
      LOG_INFO("motorway_intersection, static detour is not enabled.");
    }
  } while (false);
}

bool ScenarioManager::PrepareMotorwayDetourCondition() {
  // prepare data
  auto &task_info = data_center_->task_info_list().front();
  auto &decision_data = task_info.decision_data();
  auto motorway_lane_borrow_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_borrow_context();
  const auto &ref_points = task_info.reference_line()->ref_points();
  double detour_preview_distance =
      plan_config_ptr_->motorway_detour_scenario.preview_distance;
  if (task_info.last_frame() != nullptr) {
    detour_preview_distance =
        fmax(plan_config_ptr_->motorway_detour_scenario.preview_distance,
             task_info.last_frame()->inside_planner_data().vel_v * 5.0);
  }
  scenario_common::DetourScenarioInfo detour_conf;
  scenario_common::UpdateDetourInfo(detour_conf, true);

  // obs related check prepare
  double &front_left_min_lane_bound =
      motorway_lane_borrow_context->front_left_min_lane_bound;
  double &front_right_min_lane_bound =
      motorway_lane_borrow_context->front_right_min_lane_bound;
  double &left_min_lane_bound =
      motorway_lane_borrow_context->left_min_lane_bound;
  double &right_min_lane_bound =
      motorway_lane_borrow_context->right_min_lane_bound;
  scenario_common::FrontMinLaneBound(
      ref_points, detour_preview_distance, task_info.curr_sl(),
      front_left_min_lane_bound, front_right_min_lane_bound);
  scenario_common::NearbyMinLaneBound(
      ref_points,
      plan_config_ptr_->motorway_detour_scenario.preview_front_distance,
      plan_config_ptr_->motorway_detour_scenario.preview_back_distance,
      task_info.curr_sl(), left_min_lane_bound, right_min_lane_bound);
  scenario_common::TargetLaneCheckInfo target_lane_check_info{
      .driving_direction =
          plan_config_ptr_->motorway_detour_scenario.driving_direction,
      .preview_front_distance =
          plan_config_ptr_->motorway_detour_scenario.preview_front_distance,
      .preview_back_distance =
          plan_config_ptr_->motorway_detour_scenario.preview_back_distance,
      .near_front_distance =
          plan_config_ptr_->motorway_detour_scenario.near_front_distance,
      .near_back_distance =
          plan_config_ptr_->motorway_detour_scenario.near_back_distance,
      .preview_time = plan_config_ptr_->motorway_detour_scenario.preview_time};
  motorway_lane_borrow_context->is_refer_lane_static_obs_clear =
      scenario_common::IsFrontStaticObsClear(
          task_info, front_left_min_lane_bound, front_right_min_lane_bound,
          task_info.curr_sl().s() + detour_preview_distance, detour_obs_end_s_,
          motorway_lane_borrow_context->refer_lane_block_static_obs_id,
          front_detour_obs_);
  motorway_lane_borrow_context->refer_lane_block_static_obs_height =
      front_detour_obs_.empty() ? 0.0 : front_detour_obs_.back()->height();

  double preview_distance =
      motorway_lane_borrow_context->is_refer_lane_static_obs_clear
          ? detour_preview_distance
          : detour_obs_end_s_ - task_info.curr_sl().s();
  // basic check
  double traffic_light_preview_distance =
      detour_conf.traffic_light_preview_distance;
  double crossroad_preview_distance = detour_conf.crossroad_preview_distance;
  std::vector<Obstacle *> front_blocked_obs;
  motorway_lane_borrow_context->is_road_queued =
      !plan_config_ptr_->motorway_detour_scenario.queued_crossroad_check ||
      scenario_common::IsFrontRoadQueued(
          task_info, task_info.curr_sl().s() + detour_preview_distance,
          front_left_min_lane_bound, front_right_min_lane_bound);
  if (motorway_lane_borrow_context->is_road_queued) {
    traffic_light_preview_distance =
        detour_conf.queued_traffic_light_preview_distance;
    crossroad_preview_distance = detour_conf.queued_crossroad_preview_distance;
  }
  motorway_lane_borrow_context->is_front_has_traffic_light =
      scenario_common::IsFrontHasTrafficLight(
          task_info, preview_distance + traffic_light_preview_distance) ||
      scenario_common::IsRightTurnHasTrafficLight(
          task_info, preview_distance + traffic_light_preview_distance,
          motorway_lane_borrow_context->is_road_queued);
  motorway_lane_borrow_context->is_front_has_cross_road =
      scenario_common::IsFrontHasCrossRoad(
          task_info, preview_distance + crossroad_preview_distance,
          plan_config_ptr_->motorway_detour_scenario
              .preview_crossroad_close_ignore_distance);
  motorway_lane_borrow_context->is_left_front_has_road_boundary =
      scenario_common::IsLeftFrontHasRoadBoundary(
          task_info, preview_distance + detour_conf.road_bound_preview_distance,
          plan_config_ptr_->motorway_detour_scenario
              .preview_road_bound_close_ignore_distance);
  motorway_lane_borrow_context->is_right_front_has_road_boundary =
      scenario_common::IsRightFrontHasRoadBoundary(
          task_info, preview_distance + detour_conf.road_bound_preview_distance,
          plan_config_ptr_->motorway_detour_scenario
              .preview_road_bound_close_ignore_distance);
  motorway_lane_borrow_context->is_in_right_first_line =
      scenario_common::IsRightFirstLane(
          task_info,
          plan_config_ptr_->motorway_detour_scenario
              .right_first_line_preview_distance,
          0.0);
  motorway_lane_borrow_context->is_front_has_lane_turn =
      scenario_common::IsFrontHasLaneTurn(
          task_info, preview_distance + detour_conf.lane_turn_preview_distance,
          plan_config_ptr_->motorway_detour_scenario
              .preview_lane_turn_close_ignore_distance,
          true);
  motorway_lane_borrow_context->is_adc_on_refer_lane =
      scenario_common::IsAdcOnRefLine(task_info);

  // reverse borrowing considerations
  auto reverse_lane_detour_context =
      data_center_->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  reverse_lane_detour_context->is_allowed_detour_in_reverse_lane =
      scenario_common::IsAllowedDetourInReverseLane(
          task_info, preview_distance + detour_conf.road_bound_preview_distance,
          plan_config_ptr_->motorway_detour_scenario
              .preview_road_bound_close_ignore_distance);
  scenario_common::AdcPositionDiscussion(task_info);
  reverse_lane_detour_context->is_left_front_reverse_obs_exist =
      scenario_common::IsLeftFrontReverseObsExist(
          task_info, left_front_reverse_obs_, front_reverse_obs_);

  // obs related check
  Vec2d utm_pt{};
  task_info.reference_line_raw()->GetPointInCartesianFrame(
      {task_info.curr_sl().s(), 0.0}, &utm_pt);
  uint64_t left_lane_id, right_lane_id;
  double preview_s =
      preview_distance + plan_config_ptr_->motorway_detour_scenario
                             .adjacent_lanes_preview_distance;
  double &region_left_bound = motorway_lane_borrow_context->region_left_bound;
  double &region_right_bound = motorway_lane_borrow_context->region_right_bound;
  // left side check
  if (reverse_lane_detour_context->is_allowed_detour_in_reverse_lane) {
    region_left_bound = task_info.curr_referline_pt().left_reverse_road_bound();
  } else {
    if (PlanningMap::Instance()->GetNearestLeftLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, left_lane_id)) {
      double lane_s, lane_l;
      if (PlanningMap::Instance()->GetSLWithLane(left_lane_id, utm_pt.x(),
                                                 utm_pt.y(), lane_s, lane_l)) {
        std::pair<double, double> left_lane_width{0.0, 0.0};
        left_lane_width =
            PlanningMap::Instance()->GetLaneDistanceWidth(left_lane_id, lane_s);
        region_left_bound = task_info.curr_referline_pt().left_lane_bound() +
                            left_lane_width.first + left_lane_width.second;
      }
    } else {
      LOG_INFO("get nearest left lane fail!");
      region_left_bound =
          std::max(front_left_min_lane_bound, left_min_lane_bound) + 3.5;
    }
  }
  std::vector<int> left_dynamic_obs_ids{}, right_dynamic_obs_ids{};
  motorway_lane_borrow_context->is_left_front_static_obs_clear =
      scenario_common::IsFrontStaticObsClear(
          task_info, region_left_bound, front_left_min_lane_bound,
          task_info.curr_sl().s() + preview_s, detour_obs_end_s_,
          motorway_lane_borrow_context->left_front_block_static_obs_id,
          left_front_detour_obs_);
  motorway_lane_borrow_context->is_left_dynamic_obs_danger =
      !scenario_common::IsDynamicObsClear(
          task_info, target_lane_check_info, region_left_bound,
          left_min_lane_bound, left_dynamic_obs_ids, left_front_reverse_obs_);
  // right side check
  if (PlanningMap::Instance()->GetNearestRightLane(
          task_info.curr_referline_pt().hd_map_lane_id(),
          {utm_pt.x(), utm_pt.y()}, right_lane_id)) {
    double lane_s, lane_l;
    if (PlanningMap::Instance()->GetSLWithLane(right_lane_id, utm_pt.x(),
                                               utm_pt.y(), lane_s, lane_l)) {
      std::pair<double, double> right_lane_width{0.0, 0.0};
      right_lane_width =
          PlanningMap::Instance()->GetLaneDistanceWidth(right_lane_id, lane_s);
      region_right_bound = -task_info.curr_referline_pt().right_lane_bound() -
                           right_lane_width.first - right_lane_width.second;
    }
  } else {
    LOG_INFO("get nearest right lane fail!");
    region_right_bound =
        std::min(front_right_min_lane_bound, right_min_lane_bound) - 3.5;
  }
  motorway_lane_borrow_context->is_right_front_static_obs_clear =
      scenario_common::IsFrontStaticObsClear(
          task_info, front_right_min_lane_bound, region_right_bound,
          task_info.curr_sl().s() + preview_s, detour_obs_end_s_,
          motorway_lane_borrow_context->right_front_block_static_obs_id,
          right_front_detour_obs_);
  motorway_lane_borrow_context->is_right_dynamic_obs_danger =
      !scenario_common::IsDynamicObsClear(
          task_info, target_lane_check_info, right_min_lane_bound,
          region_right_bound, right_dynamic_obs_ids, right_front_reverse_obs_);

  motorway_lane_borrow_context->dynamic_obs_ids.clear();
  motorway_lane_borrow_context->dynamic_obs_ids.insert(
      motorway_lane_borrow_context->dynamic_obs_ids.end(),
      left_dynamic_obs_ids.begin(), left_dynamic_obs_ids.end());
  motorway_lane_borrow_context->dynamic_obs_ids.insert(
      motorway_lane_borrow_context->dynamic_obs_ids.end(),
      right_dynamic_obs_ids.begin(), right_dynamic_obs_ids.end());
  return true;
}

bool ScenarioManager::CheckMotorwayIntersectionTriggered() {
  if (!CheckMotorwayCruiseTriggered()) {
    return false;
  }
  auto &task_info = data_center_->task_info_list().front();
  double detect_distance =
      plan_config_ptr_->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr_->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }
  if (scenario_common::IsFrontHasUTurn(task_info, detect_distance)) {
    LOG_INFO("u-turn is found");
    return true;
  }
  // junction
  for (const auto &[junction_ptr, overlap] :
       task_info.reference_line()->junctions()) {
    if (overlap.object_id != 0 &&
        (task_info.curr_sl().s() + detect_distance > overlap.start_s &&
         task_info.curr_sl().s() < overlap.end_s)) {
      if (junction_ptr->Type() ==
          static_cast<uint32_t>(
              autobot::cyberverse::Junction::JunctionType::CROSS_ROAD)) {
        LOG_INFO("cross road junction is found");
        junction_id_ = overlap.object_id;
        return true;
      } else if (junction_ptr->Type() ==
                     static_cast<uint32_t>(autobot::cyberverse::Junction::
                                               JunctionType::T_CROSS_ROAD) ||
                 junction_ptr->Type() ==
                     static_cast<uint32_t>(autobot::cyberverse::Junction::
                                               JunctionType::Y_CROSS_ROAD)) {
        if (scenario_common::IsFrontHasLaneTurn(task_info, detect_distance)) {
          LOG_INFO("have turn in T_CROSS_ROAD junction");
          // have turn in T cross road, maybe no traffic light
          junction_id_ = overlap.object_id;
          return true;
        } else {
          LOG_INFO("front is straight");
          // straight with traffic light
          bool is_traffic_light_front = scenario_common::IsFrontHasTrafficLight(
              task_info, detect_distance);
          if (is_traffic_light_front) {
            LOG_INFO("T_CROSS_ROAD has traffic light");
            junction_id_ = overlap.object_id;
            return true;
          } else {
            LOG_INFO("T/Y_CROSS_ROAD has no traffic light");
            return false;
          }
        }
      }
      break;
    }
  }
  return false;
}

bool ScenarioManager::CheckMotorwayIntersectionFinished() {
  auto &task_info = data_center_->task_info_list().front();
  double detect_distance =
      plan_config_ptr_->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr_->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }
  if (scenario_common::IsFrontHasUTurn(task_info, detect_distance)) {
    LOG_INFO("front has uturn");
    return false;
  }
  if (scenario_common::IsCertainJunction(task_info, junction_id_,
                                         detect_distance)) {
    LOG_INFO("adc in junction");
    return false;
  }
  return true;
}

bool ScenarioManager::CheckMotorwayDetourTriggered() {
  if (!data_center_->master_info().enable_static_detour() ||
      !common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway) {
    return false;
  }
  auto reverse_lane_detour_context =
      data_center_->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  auto motorway_lane_borrow_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_borrow_context();
  auto lane_borrow_context =
      data_center_->mutable_master_info()->mutable_lane_borrow_context();
  auto &task_info = data_center_->task_info_list().front();
  const auto &vehicle_mode =
      data_center_->vehicle_state_proxy().chassis().driving_mode();

  bool res = false;
  bool enable_detour = false;
  MotorwayLaneBorrowContext::BorrowSide borrow_side;
  do {
    // lane borrow enable check.
    enable_detour = FLAGS_planning_lane_borrow_enable_flag &&
                    FLAGS_planning_lane_borrow_obs_trigger_flag;
    if (!enable_detour) {
      LOG_INFO("lane borrow and obs_trigger is not enabled, skip.");
      break;
    }

    // check pull over destination
    double distance_to_obs_end = detour_obs_end_s_ - task_info.curr_sl().s();
    if (data_center_->master_info().pull_over_distance_to_goal() <
        FLAGS_planning_station_enable_detour_min_dis) {
      LOG_INFO(
          "obs near stop station,distance is :{:2f}, detour is not enable.",
          distance_to_obs_end);
      break;
    }

    // check [traffic light]
    if (motorway_lane_borrow_context->is_front_has_traffic_light ||
        motorway_lane_borrow_context->is_front_has_cross_road ||
        motorway_lane_borrow_context->is_front_has_lane_turn ||
        !motorway_lane_borrow_context->is_in_right_first_line) {
      LOG_INFO(
          "front has traffic lights, cross_road, lane turn, or not in right "
          "first lane,  should not trigger borrow scenario.");
      break;
    }

    // check [refer lane static obstacle]
    if (motorway_lane_borrow_context->is_refer_lane_static_obs_clear) {
      LOG_INFO(
          "curr lane, front static obs clear, should not trigger borrow "
          "scenario.");
      break;
    }

    // check road bound or reverse lane detour
    bool borrow_left{false}, borrow_right{false};
    if (reverse_lane_detour_context->is_allowed_detour_in_reverse_lane) {
      borrow_left =
          !motorway_lane_borrow_context->is_left_dynamic_obs_danger &&
          motorway_lane_borrow_context->is_left_front_static_obs_clear;
    } else {
      borrow_left =
          !motorway_lane_borrow_context->is_left_front_has_road_boundary &&
          !motorway_lane_borrow_context->is_left_dynamic_obs_danger &&
          motorway_lane_borrow_context->is_left_front_static_obs_clear;
    }
    borrow_right =
        !motorway_lane_borrow_context->is_right_front_has_road_boundary &&
        !motorway_lane_borrow_context->is_right_dynamic_obs_danger &&
        motorway_lane_borrow_context->is_right_front_static_obs_clear;
    if (!borrow_left) {
      LOG_INFO(
          "borrow lane, left side should not "
          "trigger borrow scenario");
      break;
    }

    // all check condition satisfied.
    res = true;
    if (borrow_left) {
      borrow_side = MotorwayLaneBorrowContext::BorrowSide::Left;
    }
  } while (false);  // break here.

  // print
  LOG_INFO("Motorway Detour Triggered: {}",
           scenario_common::PrintDetourInfo(enable_detour, res, true));

  if (res) {
    // set lane borrow trigger info.
    motorway_lane_borrow_context->Reset();
    motorway_lane_borrow_context->borrow_side = borrow_side;
    lane_borrow_context->borrow_side =
        borrow_side == MotorwayLaneBorrowContext::BorrowSide::Left
            ? LaneBorrowContext::BorrowSide::Left
            : LaneBorrowContext::BorrowSide::Right;
  } else {
    bool is_dynamic_obs_danger =
        motorway_lane_borrow_context->is_left_dynamic_obs_danger;
    if (!motorway_lane_borrow_context->is_refer_lane_static_obs_clear &&
        !motorway_lane_borrow_context->is_front_has_traffic_light &&
        !motorway_lane_borrow_context->is_front_has_cross_road &&
        !is_dynamic_obs_danger &&
        data_center_->vehicle_state_odometry().LinearVelocity() < 0.1 &&
        vehicle_mode ==
            neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
      LOG_INFO("detour failed");
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::DETOUR_FAIL);
    } else {
      data_center_->mutable_event_report_proxy()->EventReset(
          EventType::DETOUR_FAIL);
    }
  }
  return res;
}

bool ScenarioManager::CheckMotorwayDetourFinished() const {
  if (!data_center_->master_info().enable_static_detour() ||
      !common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway) {
    return true;
  }

  auto motorway_lane_borrow_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_borrow_context();
  auto reverse_lane_detour_context =
      data_center_->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  const auto &vehicle_mode =
      data_center_->vehicle_state_proxy().chassis().driving_mode();

  auto &task_info = data_center_->task_info_list().front();
  bool is_near_traffic_light = scenario_common::IsNearTrafficJunction(
      task_info.reference_line(),
      plan_config_ptr_->motorway_detour_scenario.preview_distance,
      task_info.curr_sl().s());

  bool is_borrow_side_has_boundary{true};
  if (reverse_lane_detour_context->is_allowed_detour_in_reverse_lane) {
    is_borrow_side_has_boundary = false;
  } else {
    is_borrow_side_has_boundary =
        motorway_lane_borrow_context->borrow_side ==
                MotorwayLaneBorrowContext::BorrowSide::Left
            ? motorway_lane_borrow_context->is_left_front_has_road_boundary
            : motorway_lane_borrow_context->is_right_front_has_road_boundary;
  }

  motorway_lane_borrow_context->outside_finish_signal = false;
  if (motorway_lane_borrow_context->is_front_has_traffic_light ||
      motorway_lane_borrow_context->is_front_has_cross_road ||
      motorway_lane_borrow_context->is_front_has_lane_turn ||
      is_borrow_side_has_boundary) {
    if (curr_stage_decider_->curr_state_str() == "INIT" ||
        curr_stage_decider_->curr_state_str() == "PREPARE" ||
        curr_stage_decider_->curr_state_str() == "EXIT") {
      LOG_INFO(
          "Motorway_Detour_finished: front has "
          "traffic_light[{}]/cross_road[{}]/lane_turn[{}]/bound_limit[{}], "
          "curr_stage is {}",
          motorway_lane_borrow_context->is_front_has_traffic_light,
          motorway_lane_borrow_context->is_front_has_cross_road,
          motorway_lane_borrow_context->is_front_has_lane_turn,
          is_borrow_side_has_boundary, curr_stage_decider_->curr_state_str());
      motorway_lane_borrow_context->outside_finish_signal = true;
      if (curr_stage_decider_->curr_state_str() == "INIT") return true;
    }
    if (curr_stage_decider_->curr_state_str() == "BORROWING" ||
        curr_stage_decider_->curr_state_str() == "REVERSE_LANE_BORROWING") {
      if (motorway_lane_borrow_context->is_adc_on_refer_lane) {
        motorway_lane_borrow_context->outside_finish_signal = true;
      } else {
        if (!motorway_lane_borrow_context->is_refer_lane_static_obs_clear &&
            scenario_common::IsCloseStaticDetour(task_info)) {
          motorway_lane_borrow_context->outside_finish_signal = true;
          data_center_->mutable_master_info()->set_enable_static_detour(false);
        }
      }
    }

    if ((curr_stage_decider_->curr_state_str() == "BORROWING" ||
         curr_stage_decider_->curr_state_str() == "REVERSE_LANE_BORROWING") &&
        is_near_traffic_light &&
        data_center_->vehicle_state_odometry().LinearVelocity() < 0.01 &&
        vehicle_mode ==
            neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
      LOG_INFO("unable to back to the original lane in borrowing stage");
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::DETOUR_FAIL);
    } else {
      data_center_->mutable_event_report_proxy()->EventReset(
          EventType::DETOUR_FAIL);
    }
  }

  LOG_INFO("outside finish signal:{}",
           motorway_lane_borrow_context->outside_finish_signal);
  if (curr_stage_decider_->curr_state_str() == "INIT") {
    if (motorway_lane_borrow_context->is_refer_lane_static_obs_clear &&
        motorway_lane_borrow_context->is_adc_on_refer_lane) {
      LOG_INFO(
          "Motorway_Detour_finished: refer_lane_obs_clear and "
          "adc_on_refer_lane.");
      return true;
    }
  }

  LOG_INFO("Motorway_Detour_finished: false");
  return false;
}

bool ScenarioManager::CheckMotorwayLaneChangeFinished() {
  if (!common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway) {
    return false;
  }
  // Internal cancel or finish
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();

  // check lane is_target_lane_not_adjacent
  if (!motorway_lane_change_context->is_target_lane_adjacent ||
      data_center_->navigation_result().navigator_request == NO_REQUEST) {
    LOG_INFO("target lane not adjacent, motorway lane change is not enable.");
    motorway_lane_change_context->stage =
        global::planning::MotorwayLaneChangeStageState::PREPARE;
    return true;
  }
  return false;
}

bool ScenarioManager::CheckReroutingTriggered() {
  auto &task_info = data_center_->task_info_list().front();
  bool white_line = false;
  auto &left_divider_feature =
      task_info.curr_referline_pt().left_divider_feature();
  for (auto divider_feature : left_divider_feature) {
    if (divider_feature.divider_type_ ==
            DividerFeature::DividerType::SINGLE_SOLID_LINE &&
        divider_feature.divider_color_ == DividerFeature::DividerColor::WHITE) {
      white_line = true;
      break;
    }
  }
  LOG_INFO("white_line: {}", white_line);
  auto lane_borrow_context = data_center_->master_info().lane_borrow_context();
  if (white_line && lane_borrow_context.is_front_has_traffic_light &&
      !lane_borrow_context.is_adc_on_refer_lane &&
      !lane_borrow_context.is_refer_lane_static_obs_clear) {
    Vec2d utm_pt{};
    task_info.reference_line_raw()->GetPointInCartesianFrame(
        {task_info.curr_sl().s(), 0.0}, &utm_pt);
    uint64_t left_lane_id;
    if (PlanningMap::Instance()->GetNearestLeftLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, left_lane_id)) {
      data_center_->set_lane_change_target_id(left_lane_id);
      LOG_INFO("left_lane_id: {}", left_lane_id);
      return true;
    } else {
      LOG_INFO("not find left lane");
    }
  }

  return false;
}

bool ScenarioManager::CheckMotorwayReroutingTriggered() {
  auto &task_info = data_center_->task_info_list().front();
  bool white_line = false;
  auto &left_divider_feature =
      task_info.curr_referline_pt().left_divider_feature();
  for (auto divider_feature : left_divider_feature) {
    if (divider_feature.divider_type_ ==
            DividerFeature::DividerType::SINGLE_SOLID_LINE &&
        divider_feature.divider_color_ == DividerFeature::DividerColor::WHITE) {
      white_line = true;
      break;
    }
  }
  LOG_INFO("white_line: {}", white_line);
  auto motorway_lane_borrow_context =
      data_center_->master_info().motorway_lane_borrow_context();
  if (white_line && motorway_lane_borrow_context.is_front_has_traffic_light &&
      !motorway_lane_borrow_context.is_adc_on_refer_lane &&
      !motorway_lane_borrow_context.is_refer_lane_static_obs_clear) {
    Vec2d utm_pt{};
    task_info.reference_line_raw()->GetPointInCartesianFrame(
        {task_info.curr_sl().s(), 0.0}, &utm_pt);
    uint64_t left_lane_id;
    if (PlanningMap::Instance()->GetNearestLeftLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, left_lane_id)) {
      data_center_->set_lane_change_target_id(left_lane_id);
      LOG_INFO("left_lane_id: {}", left_lane_id);
      return true;
    } else {
      LOG_INFO("not find left lane");
    }
  }

  return false;
}

bool ScenarioManager::PrepareMotorwayLaneChangeCondition() {
  // prepare data
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();

  // update is_target_lane_adjacent
  motorway_lane_change_context->is_target_lane_adjacent =
      scenario_common::IsTargetLineAdjacent();

  // update change_side
  // data_center_->set_navigator_request(static_cast<NavigatorLaneChangeRequest>(
  //     config::PlanningConfig::Instance()
  //         ->plan_config()
  //         .motorway_lane_change_scenario.motorway_lane_change_request));
  LOG_INFO(
      "MotorwayLaneChange init info: [traffic_light: {}, "
      "road_boundary: {},  lane_turn: {}, adc_current/target_lane: {} "
      "{},  is_target_lane_adjacent: {}, change_side: {}]",
      motorway_lane_change_context->is_front_has_traffic_light,
      motorway_lane_change_context->is_front_has_road_boundary,
      motorway_lane_change_context->is_front_has_lane_turn,
      motorway_lane_change_context->is_adc_on_current_lane,
      motorway_lane_change_context->is_adc_on_target_lane,
      motorway_lane_change_context->is_target_lane_adjacent,
      data_center_->navigation_result().navigator_request);

  return true;
}

bool ScenarioManager::CheckBarrierGateFinished() {
  auto &cnt = data_center_->mutable_master_info()
                  ->mutable_barrier_gate_context()
                  ->scenario_filter_cnt;
  auto &is_barrier_gate_finished = data_center_->mutable_master_info()
                                       ->mutable_barrier_gate_context()
                                       ->is_barrier_gate_finished;
  LOG_INFO("check:  is_barrier_gate_finished: {}, scenario_filter_cnt: {}",
           is_barrier_gate_finished, cnt);
  if (is_barrier_gate_finished) {
    cnt++;
    return true;
  }
  return false;
}

bool ScenarioManager::CheckBarrierGateTriggered() {
  const auto &reference_line =
      data_center_->task_info_list().front().reference_line();
  const auto &refer_pts = reference_line->ref_points();
  const auto &inside_data = data_center_->last_frame()->inside_planner_data();
  const auto &outside_data = data_center_->last_frame()->outside_planner_data();
  SLPoint adc_sl_point = inside_data.init_sl_point;

  double lower_dis = 15.0;
  double upper_dis = std::max(inside_data.distance_to_end, lower_dis + 1.0);
  double sim_dis = std::max(inside_data.vel_v, 2.0) * 10.0;
  double forward_valid_s = std::clamp(sim_dis, lower_dis, upper_dis);
  double backward_valid_s = 0.0;
  LOG_INFO("forward_valid_s: {:.4f}, backward_valid_s: {:.4f}", forward_valid_s,
           backward_valid_s);

  double cut_start_s = std::max(adc_sl_point.s() - backward_valid_s,
                                reference_line->ref_points().front().s());
  std::size_t start_index{0}, end_index{0};
  if (!reference_line->GetStartEndIndexBySLength(
          cut_start_s, forward_valid_s + backward_valid_s, &start_index,
          &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return false;
  }
  Segment2d cut_ref_seg({refer_pts[start_index].s(), 0.},
                        {refer_pts[end_index].s(), 0.});

  for (auto overlap : reference_line->barrier_gate_overlaps()) {
    Segment2d barrier_gate_seg({overlap.start_s, 0.}, {overlap.end_s, 0.});
    if (cut_ref_seg.has_intersect(barrier_gate_seg)) {
      LOG_INFO("There're barrier gates nearby, {:.3f}, {:.3f}, {:.3f}",
               adc_sl_point.s(), overlap.start_s, overlap.end_s);
      data_center_->mutable_master_info()
          ->mutable_barrier_gate_context()
          ->barrier_gate = overlap;

      double adc_end_s =
          outside_data.path_obstacle_context.adc_boundary.end_s();
      data_center_->mutable_master_info()
          ->mutable_barrier_gate_context()
          ->barrier_gate_first_check_dis =
          std::max(0.0, overlap.start_s - adc_end_s);
      LOG_INFO("barrier_gate_first_check_dis:{:.4f}",
               data_center_->mutable_master_info()
                   ->mutable_barrier_gate_context()
                   ->barrier_gate_first_check_dis);
      return true;
    }
  }
  LOG_INFO("no barrier gate!");

  return false;
}

bool ScenarioManager::CheckBarrierGateCondition() {
  auto &cnt = data_center_->mutable_master_info()
                  ->mutable_barrier_gate_context()
                  ->scenario_filter_cnt;
  LOG_INFO("scenario_filter_cnt:{} threshold:{}", cnt, kFilterFrameThreshold);
  if (cnt >= kFilterFrameThreshold) {
    data_center_->mutable_master_info()
        ->mutable_barrier_gate_context()
        ->Reset();
  }
  return true;
}

bool ScenarioManager::CheckLaneChangeTriggered() { return false; }

bool ScenarioManager::CheckLaneChangeCondition() { return false; }

bool ScenarioManager::CheckLaneChangeCanceled() { return false; }

bool ScenarioManager::CheckLaneChangeFinished() { return false; }

bool ScenarioManager::PrepareDetourCondition() {
  // prepare data
  auto &task_info = data_center_->task_info_list().front();
  auto &decision_data = task_info.decision_data();
  auto lane_borrow_context =
      data_center_->mutable_master_info()->mutable_lane_borrow_context();
  const auto &ref_points = task_info.reference_line()->ref_points();
  double detour_preview_distance =
      plan_config_ptr_->detour_scenario.preview_distance;
  if (task_info.last_frame() != nullptr) {
    detour_preview_distance =
        fmax(plan_config_ptr_->detour_scenario.preview_distance,
             task_info.last_frame()->inside_planner_data().vel_v * 5.0);
  }
  scenario_common::DetourScenarioInfo detour_conf;
  scenario_common::UpdateDetourInfo(detour_conf);

  // obs related check prepare
  double &front_left_min_lane_bound =
      lane_borrow_context->front_left_min_lane_bound;
  double &front_right_min_lane_bound =
      lane_borrow_context->front_right_min_lane_bound;
  double &left_min_lane_bound = lane_borrow_context->left_min_lane_bound;
  double &right_min_lane_bound = lane_borrow_context->right_min_lane_bound;
  scenario_common::FrontMinLaneBound(
      ref_points, detour_preview_distance, task_info.curr_sl(),
      front_left_min_lane_bound, front_right_min_lane_bound);
  scenario_common::NearbyMinLaneBound(
      ref_points, plan_config_ptr_->detour_scenario.preview_front_distance,
      plan_config_ptr_->detour_scenario.preview_back_distance,
      task_info.curr_sl(), left_min_lane_bound, right_min_lane_bound);
  scenario_common::TargetLaneCheckInfo target_lane_check_info{
      .driving_direction = plan_config_ptr_->detour_scenario.driving_direction,
      .preview_front_distance =
          plan_config_ptr_->detour_scenario.preview_front_distance,
      .preview_back_distance =
          plan_config_ptr_->detour_scenario.preview_back_distance,
      .near_front_distance =
          plan_config_ptr_->detour_scenario.near_front_distance,
      .near_back_distance =
          plan_config_ptr_->detour_scenario.near_back_distance,
      .preview_time = plan_config_ptr_->detour_scenario.preview_time};
  lane_borrow_context->is_refer_lane_static_obs_clear =
      scenario_common::IsFrontStaticObsClear(
          task_info, front_left_min_lane_bound, front_right_min_lane_bound,
          task_info.curr_sl().s() + detour_preview_distance, detour_obs_end_s_,
          lane_borrow_context->refer_lane_block_static_obs_id,
          front_detour_obs_);
  lane_borrow_context->refer_lane_block_static_obs_height =
      front_detour_obs_.empty() ? 0.0 : front_detour_obs_.back()->height();

  // basic check
  double preview_distance = lane_borrow_context->is_refer_lane_static_obs_clear
                                ? detour_preview_distance
                                : detour_obs_end_s_ - task_info.curr_sl().s();
  double traffic_light_preview_distance =
      detour_conf.traffic_light_preview_distance;
  double crossroad_preview_distance = detour_conf.crossroad_preview_distance;
  std::vector<Obstacle *> front_blocked_obs;
  lane_borrow_context->is_road_queued =
      !plan_config_ptr_->detour_scenario.queued_crossroad_check ||
      scenario_common::IsFrontRoadQueued(
          task_info, task_info.curr_sl().s() + detour_preview_distance,
          front_left_min_lane_bound, front_right_min_lane_bound);
  if (lane_borrow_context->is_road_queued) {
    traffic_light_preview_distance =
        detour_conf.queued_traffic_light_preview_distance;
    crossroad_preview_distance = detour_conf.queued_crossroad_preview_distance;
  }
  lane_borrow_context->is_front_has_traffic_light =
      scenario_common::IsFrontHasTrafficLight(
          task_info, preview_distance + traffic_light_preview_distance);
  lane_borrow_context->is_front_has_cross_road =
      scenario_common::IsFrontHasCrossRoad(
          task_info, preview_distance + crossroad_preview_distance,
          plan_config_ptr_->detour_scenario
              .preview_crossroad_close_ignore_distance);
  lane_borrow_context->is_front_has_road_boundary =
      scenario_common::IsFrontHasRoadBoundary(
          task_info, preview_distance + detour_conf.road_bound_preview_distance,
          plan_config_ptr_->detour_scenario
              .preview_road_bound_close_ignore_distance);

  lane_borrow_context->is_adc_on_refer_lane =
      scenario_common::IsAdcOnRefLine(task_info);

  // reverse borrowing considerations
  auto reverse_lane_detour_context =
      data_center_->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  reverse_lane_detour_context->is_allowed_detour_in_reverse_lane =
      scenario_common::IsAllowedDetourInReverseLane(
          task_info, preview_distance + detour_conf.road_bound_preview_distance,
          plan_config_ptr_->detour_scenario
              .preview_road_bound_close_ignore_distance);
  scenario_common::AdcPositionDiscussion(task_info);
  reverse_lane_detour_context->is_left_front_reverse_obs_exist =
      scenario_common::IsLeftFrontReverseObsExist(
          task_info, left_front_reverse_obs_, front_reverse_obs_);

  // obs related check
  Vec2d utm_pt{};
  task_info.reference_line_raw()->GetPointInCartesianFrame(
      {task_info.curr_sl().s(), 0.0}, &utm_pt);
  uint64_t left_lane_id;
  double &region_left_bound = lane_borrow_context->region_left_bound;
  double &region_right_bound = lane_borrow_context->region_right_bound;
  if (reverse_lane_detour_context->is_allowed_detour_in_reverse_lane) {
    region_left_bound = task_info.curr_referline_pt().left_reverse_road_bound();
  } else {
    if (PlanningMap::Instance()->GetNearestLeftLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, left_lane_id)) {
      double lane_s, lane_l;
      if (PlanningMap::Instance()->GetSLWithLane(left_lane_id, utm_pt.x(),
                                                 utm_pt.y(), lane_s, lane_l)) {
        std::pair<double, double> left_lane_width{0.0, 0.0};
        left_lane_width =
            PlanningMap::Instance()->GetLaneDistanceWidth(left_lane_id, lane_s);
        region_left_bound = task_info.curr_referline_pt().left_lane_bound() +
                            left_lane_width.first + left_lane_width.second;
      }
    } else {
      region_left_bound = left_min_lane_bound + 3.5;
    }
  }
  lane_borrow_context->is_left_dynamic_obs_danger =
      !scenario_common::IsDynamicObsClear(
          task_info, target_lane_check_info, region_left_bound,
          left_min_lane_bound, lane_borrow_context->dynamic_obs_ids,
          front_reverse_obs_);

  return true;
}

bool ScenarioManager::CheckDetourTriggered() {
  auto &task_info = data_center_->task_info_list().front();
  if (!data_center_->master_info().enable_static_detour()) {
    return false;
  }
  auto lane_borrow_context =
      data_center_->mutable_master_info()->mutable_lane_borrow_context();
  auto reverse_lane_detour_context =
      data_center_->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  auto &decision_data = task_info.decision_data();
  double preview_distance = plan_config_ptr_->detour_scenario.preview_distance;
  const auto &vehicle_mode =
      data_center_->vehicle_state_proxy().chassis().driving_mode();

  bool res = false;
  bool is_extend_lane_static_clear = false;
  bool enable_detour = false;
  LaneBorrowContext::BorrowSide borrow_side;
  do {
    // lane borrow enable check.
    enable_detour = FLAGS_planning_lane_borrow_enable_flag &&
                    FLAGS_planning_lane_borrow_obs_trigger_flag;
    if (!enable_detour) {
      LOG_INFO("lane borrow and obs_trigger is not enabled, skip.");
      break;
    }

    // check pull over destination
    double distance_to_obs_end = detour_obs_end_s_ - task_info.curr_sl().s();
    if (data_center_->master_info().pull_over_distance_to_goal() <
        FLAGS_planning_station_enable_detour_min_dis) {
      LOG_INFO(
          "obs near stop station,distance is :{:2f}, detour is not enable.",
          distance_to_obs_end);
      break;
    }

    // check [traffic light], [road boundary]
    if (lane_borrow_context->is_front_has_traffic_light ||
        lane_borrow_context->is_front_has_cross_road) {
      LOG_INFO(
          "front has traffic lights, cross road should not "
          "trigger borrow scenario.");
      break;
    }

    // check [refer lane static obstacle]
    if (lane_borrow_context->is_refer_lane_static_obs_clear) {
      LOG_INFO(
          "curr lane, front static obs clear, should not trigger borrow "
          "scenario.");
      break;
    }
    if (lane_borrow_context->is_left_dynamic_obs_danger) {
      LOG_INFO(
          "Nearby lane has dynamic obs, should not trigger detour scenario");
      break;
    }

    // check road bound or reverse lane detour
    if (lane_borrow_context->is_front_has_road_boundary &&
        !reverse_lane_detour_context->is_allowed_detour_in_reverse_lane) {
      LOG_INFO("front has road boundary should not trigger borrow scenario.");
      break;
    }

    // all check condition satisfied.
    res = true;
    if (FLAGS_planning_default_left_right_side) {
      borrow_side = LaneBorrowContext::BorrowSide::Left;
    }
  } while (false);  // break here.

  // print
  LOG_INFO("Detour Triggered: {}",
           scenario_common::PrintDetourInfo(enable_detour, res, false));

  if (res) {
    // set lane borrow trigger info.
    lane_borrow_context->Reset();
    lane_borrow_context->borrow_side = borrow_side;
  } else {
    if (!lane_borrow_context->is_refer_lane_static_obs_clear &&
        !lane_borrow_context->is_front_has_cross_road &&
        !lane_borrow_context->is_front_has_traffic_light &&
        !lane_borrow_context->is_left_dynamic_obs_danger &&
        vehicle_mode ==
            neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE &&
        data_center_->vehicle_state_odometry().LinearVelocity() < 0.01) {
      LOG_INFO("detour failed");
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::DETOUR_FAIL);
    } else {
      data_center_->mutable_event_report_proxy()->EventReset(
          EventType::DETOUR_FAIL);
    }
  }
  return res;
}

bool ScenarioManager::CheckDetourFinished() const {
  auto &task_info = data_center_->task_info_list().front();
  if (!data_center_->master_info().enable_static_detour()) {
    return true;
  }
  auto lane_borrow_context =
      data_center_->mutable_master_info()->mutable_lane_borrow_context();
  auto reverse_lane_detour_context =
      data_center_->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  const auto &vehicle_mode =
      data_center_->vehicle_state_proxy().chassis().driving_mode();

  bool is_borrow_side_has_boundary =
      reverse_lane_detour_context->is_allowed_detour_in_reverse_lane
          ? false
          : lane_borrow_context->is_front_has_road_boundary;

  lane_borrow_context->outside_finish_signal = false;
  if (lane_borrow_context->is_front_has_traffic_light ||
      lane_borrow_context->is_front_has_cross_road ||
      is_borrow_side_has_boundary) {
    if (curr_stage_decider_->curr_state_str() == "INIT" ||
        curr_stage_decider_->curr_state_str() == "PREPARE" ||
        curr_stage_decider_->curr_state_str() == "EXIT") {
      LOG_INFO(
          "Detour_finished: front has "
          "traffic_light[{}]/cross_road[{}]/bound_limit[{}], curr_stage is {}",
          lane_borrow_context->is_front_has_traffic_light,
          lane_borrow_context->is_front_has_cross_road,
          is_borrow_side_has_boundary, curr_stage_decider_->curr_state_str());
      lane_borrow_context->outside_finish_signal = true;
      if (curr_stage_decider_->curr_state_str() == "INIT") return true;
    }
    if ((curr_stage_decider_->curr_state_str() == "BORROWING" ||
         curr_stage_decider_->curr_state_str() == "REVERSE_LANE_BORROWING") &&
        lane_borrow_context->is_adc_on_refer_lane) {
      lane_borrow_context->outside_finish_signal = true;
    }

    if ((curr_stage_decider_->curr_state_str() == "BORROWING" ||
         curr_stage_decider_->curr_state_str() == "REVERSE_LANE_BORROWING") &&
        data_center_->vehicle_state_odometry().LinearVelocity() < 0.01 &&
        vehicle_mode ==
            neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
      LOG_INFO("unable to back to the original lane in borrowing stage");
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::DETOUR_FAIL);
    } else {
      data_center_->mutable_event_report_proxy()->EventReset(
          EventType::DETOUR_FAIL);
    }
  }

  LOG_INFO("outside finish signal:{}",
           lane_borrow_context->outside_finish_signal);
  if (curr_stage_decider_->curr_state_str() == "INIT") {
    if (lane_borrow_context->is_refer_lane_static_obs_clear &&
        lane_borrow_context->is_adc_on_refer_lane) {
      LOG_INFO("Detour_finished: refer_lane_obs_clear and adc_on_refer_lane.");

      // dengganglin add
      if (data_center_->master_info().distance_to_end() < 12) {
        LOG_INFO("detour exit, near station.");
        data_center_->mutable_master_info()
            ->mutable_cruise_context()
            ->is_finish_detour_and_near_station = true;
      }
      return true;
    }
  }

  LOG_INFO("Detour_finished: false");
  return false;
}

bool ScenarioManager::DetectIntersection() {
  // traffic light.
  auto &task_info = data_center_->task_info_list().front();
  double detect_distance =
      plan_config_ptr_->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr_->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }
  return scenario_common::IsFrontHasTrafficLight(task_info, detect_distance) &&
         scenario_common::IsJunction(task_info, junction_id_, detect_distance);
}

bool ScenarioManager::DetectSideWayIntersection() {
  if (!config::PlanningConfig::Instance()
           ->plan_config()
           .side_way_intersection_scenario.enable_side_way_intersection) {
    return false;
  }
  LOG_INFO("detect side way junction");
  // traffic light.
  auto &task_info = data_center_->task_info_list().front();
  double detect_distance =
      plan_config_ptr_->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr_->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }
  bool is_traffic_light_front =
      scenario_common::IsFrontHasTrafficLight(task_info, detect_distance);
  bool is_T_cross_road_junction = scenario_common::IsTCrossRoadJunctionScenario(
      task_info, plan_config_ptr_->side_way_intersection_scenario
                         .crossroad_preview_distance +
                     detect_distance);
  if (!is_T_cross_road_junction) {
    LOG_INFO("can not find T_cross_road junction.");
    return false;
  }
  bool have_lane_turn = scenario_common::IsFrontHasLaneTurn(
      task_info,
      plan_config_ptr_->motorway_intersection_scenario
              .lanetype_detect_distance +
          detect_distance,
      0.0);
  if (!is_traffic_light_front && is_T_cross_road_junction && !have_lane_turn) {
    LOG_INFO("side way junction found");
    return true;
  }
  return false;
}

bool ScenarioManager::DetectBackOut() {
  LOG_INFO("DetectBackOut");
  static int stop_cnt = 0;
  const auto &task_info = data_center_->task_info_list().front();
  const auto &chassis = data_center_->vehicle_state_proxy().chassis();
  const auto &back_out_config = plan_config_ptr_->back_out;
  const auto &pose = data_center_->vehicle_state_utm();

  LOG_INFO("pilot_state: {}", data_center_->pilot_state().state());
  switch (data_center_->pilot_state().state()) {
    case PilotState::ESCALATION: {
      LOG_INFO("back_out count reset.");
      stop_cnt = 0;
      path_fail_cnt_ = 0;
      speed_fail_cnt_ = 0;
      return false;
    }
    default:
      break;
  }
  if (!data_center_->is_auto_driving()) {
    LOG_WARN("Skip BACK_OUT: not auto pilot");
    return false;
  }

  if (scenario_common::IsFrontHasTrafficLight(
          task_info, back_out_config.preview_distance) ||
      scenario_common::IsNearTrafficJunction(task_info.reference_line(),
                                             back_out_config.preview_distance,
                                             task_info.curr_sl().s())) {
    LOG_INFO("Skip BACK_OUT: close to traffic junction");
    return false;
  }

  LOG_INFO("pose({:.4f}, {:.4f})", pose.X(), pose.Y());
  if (!scenario_common::CheckMotorwayBackOut(task_info.reference_line(),
                                             task_info.curr_sl().s(), pose))
    return false;

  auto dis = (pose.X() - last_back_out_point_.x()) *
                 (pose.X() - last_back_out_point_.x()) +
             (pose.Y() - last_back_out_point_.y()) *
                 (pose.Y() - last_back_out_point_.y());
  LOG_INFO("dis: {:.4f}, target x: {:.4f}, target y: {:.4f}.", dis,
           last_back_out_point_.x(), last_back_out_point_.y());

  if (dis > 20) {
    count_back_out_ = 0;
  }
  if (count_back_out_ >= 3) {
    LOG_INFO("Skip BACK_OUT: BACK_OUT has finished {} times", count_back_out_);
    return false;
  }

  auto &data_out = data_center_->last_frame()->outside_planner_data();
  auto velocity = data_center_->vehicle_state_odometry().LinearVelocity();
  double plan_vel = 0.0;
  if (data_out.speed_data != nullptr &&
      !data_out.speed_data->speed_vector().empty())
    plan_vel = data_out.speed_data->speed_vector().back().v();
  stop_cnt = std::abs(velocity) < 0.1 ? stop_cnt + 1 : 0;

  path_fail_cnt_ = data_out.path_fail_tasks > 0 ? path_fail_cnt_ + 1 : 0;

  speed_fail_cnt_ = data_out.speed_fail_tasks > 0 ? speed_fail_cnt_ + 1 : 0;

  if (back_out_finished_) {
    back_out_finished_ = false;
    stop_cnt = 0;
    path_fail_cnt_ = 0;
    speed_fail_cnt_ = 0;
  }
  LOG_INFO("XXXX BACK_OUT in vel {}, stop_cnt {}, path {}, speed {}, stat {}",
           velocity, stop_cnt, path_fail_cnt_, speed_fail_cnt_,
           curr_state_str_);
  // TODO: Need to check the trigger condition of BACK_OUT
  return plan_config_ptr_->back_out.enable_back_out &&
         (stop_cnt > 50 && (std::abs(plan_vel) < 0.1 || speed_fail_cnt_ > 50));
  // return plan_config_ptr_->back_out.enable_back_out && stop_cnt > 50;
}

/// Add Side Road Intersection
bool ScenarioManager::CheckSideWayIntersectionPassFinished() {
  if (!config::PlanningConfig::Instance()
           ->plan_config()
           .side_way_intersection_scenario.enable_side_way_intersection) {
    return true;
  }
  auto &task_info = data_center_->task_info_list().front();
  double detect_distance =
      plan_config_ptr_->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr_->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }
  bool is_traffic_light_front =
      scenario_common::IsFrontHasTrafficLight(task_info, detect_distance);
  bool is_T_cross_road_junction = scenario_common::IsTCrossRoadJunctionScenario(
      task_info, plan_config_ptr_->side_way_intersection_scenario
                         .crossroad_preview_distance +
                     detect_distance);
  if (!is_T_cross_road_junction || is_traffic_light_front) {
    return true;
  }
  return false;
}

bool ScenarioManager::CheckIntersectionPassFinished() {
  auto &task_info = data_center_->task_info_list().front();
  double detect_distance =
      plan_config_ptr_->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr_->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }
  if (scenario_common::IsCertainJunction(task_info, junction_id_,
                                         detect_distance)) {
    LOG_INFO("adc in junction");
    return false;
  }
  return true;
}

bool ScenarioManager::CheckIntersectionExportFinished() {
  const auto &task_info = data_center_->task_info_list().front();
  if (task_info.curr_referline_pt().is_right_signal()) {
    LOG_INFO("intersection turn right signal.");
    return true;
  }
  for (const auto &junction_overlap :
       task_info.reference_line()->junction_overlaps()) {
    if (junction_overlap.object_id == junction_id_ &&
        task_info.curr_sl().s() > junction_overlap.start_s &&
        task_info.curr_sl().s() < junction_overlap.end_s) {
      std::vector<std::pair<double, double>> crosswalk_overlap_s_pair{};
      for (const auto &crosswalk_overlap :
           task_info.reference_line()->crosswalk_overlaps()) {
        if (crosswalk_overlap.object_id != 0 &&
            crosswalk_overlap.end_s < junction_overlap.end_s &&
            crosswalk_overlap.start_s > junction_overlap.start_s) {
          crosswalk_overlap_s_pair.push_back(
              {crosswalk_overlap.start_s, crosswalk_overlap.end_s});
        }
      }
      if (crosswalk_overlap_s_pair.empty()) {
        return true;
      }
      if (!crosswalk_overlap_s_pair.empty()) {
        std::sort(
            crosswalk_overlap_s_pair.begin(), crosswalk_overlap_s_pair.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; });
        if (crosswalk_overlap_s_pair.back().first <
            task_info.adc_boundary().end_s()) {
          return true;
        }
      }
      break;
    }
  }
  return false;
}

bool ScenarioManager::CheckBackOutFinished() {
  const auto &task_info = data_center_->task_info_list().front();
  auto data = data_center_->last_frame()->inside_planner_data();
  const auto &chassis = data_center_->vehicle_state_proxy().chassis();
  auto velocity = data_center_->vehicle_state_odometry().LinearVelocity();
  const auto &back_out_config =
      config::PlanningConfig::Instance()->plan_config().back_out;
  auto tar_s_bias = back_out_config.tar_s_bias;
  auto stop_cnt_threshold = back_out_config.stop_cnt_threshold;
  static int stop_cnt = 0;
  stop_cnt = std::abs(velocity) < 0.1 ? stop_cnt + 1 : 0;

  if (curr_state_str_ != "BACK_OUT") {
    stop_cnt = 0;
  }
  LOG_INFO("XXXX cur s {:.4f}, end s {:.4f}, stop_cnt {}, state {}",
           task_info.curr_sl().s(), data.back_out_end_s, stop_cnt,
           curr_state_str_);
  LOG_INFO("data.init_point({:.4f}, {:.4f}), data.target_point({:.4f}, {:.4f})",
           data.init_point.x(), data.init_point.y(), data.target_point.x(),
           data.target_point.y());

  if (!data_center_->is_auto_driving()) {
    LOG_WARN("Quit BACK_OUT: not auto pilot");
    stop_cnt = 0;
    return true;
  }

  if (task_info.curr_sl().s() > data.back_out_end_s ||
      std::hypot(task_info.adc_point().x() - data.target_point.x(),
                 task_info.adc_point().y() - data.target_point.y()) < 1.5) {
    stop_cnt = 0;
    return true;
  }
  if (stop_cnt > stop_cnt_threshold) {
    if (stop_cnt > stop_cnt_threshold + kFilterFrameThreshold) stop_cnt = 0;
    return true;
  }
  return false;
}

bool ScenarioManager::DetectNarrowRoad() {
  const auto &task_info = data_center_->task_info_list().front();
  double road_width = 0;
  if (!scenario_common::ComputePreviewRoadWidth(
          task_info, plan_config_ptr_->narrow_road_scenario.preview_distance,
          road_width)) {
    LOG_ERROR("compute road width failed! ");
    return false;
  }

  if (road_width <
      plan_config_ptr_->narrow_road_scenario.narrow_road_width_threshold) {
    return true;
  }
  return false;
}

bool ScenarioManager::CheckNarrowRoadPassFinished() {
  const auto &task_info = data_center_->task_info_list().front();
  // compute road average width.
  double road_width = 0;
  double preview_distance = 5.0;
  if (!scenario_common::ComputePreviewRoadWidth(task_info, preview_distance,
                                                road_width)) {
    LOG_ERROR("compute road width failed! ");
    // keep NarrowRoad Scenario.
    return false;
  }

  double closest_pt_width = task_info.curr_referline_pt().left_lane_bound() +
                            task_info.curr_referline_pt().right_lane_bound();

  auto &narrow_road_scenario_config = plan_config_ptr_->narrow_road_scenario;

  if (closest_pt_width >
          narrow_road_scenario_config.narrow_road_width_threshold &&
      road_width >
          narrow_road_scenario_config.narrow_road_width_threshold +
              narrow_road_scenario_config.exit_narrow_road_width_buffer) {
    return true;
  }
  return false;
}

bool ScenarioManager::DetectParking() {
  const auto parking_ptr = data_center_->parking_ptr();
  if (!parking_ptr) {
    return false;
  }
  const auto parking_space_ptr = parking_ptr->OriginPark();
  if (!parking_space_ptr) {
    LOG_INFO("No parking space ptr");
    return false;
  }
  if (parking_space_ptr->Type() ==
      global::hdmap::ParkingSpace_ParkingType_UNKNOWN) {
    return false;
  }
  const auto &parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  auto hdmap = cyberverse::HDMap::Instance();
  std::string parking_id = hdmap->GetIdHashString(parking_space_ptr->Id());
  LOG_INFO("parking space id = {}", parking_id);
  const auto &utm_pose = data_center_->vehicle_state_utm();
  bool is_ego_car_in_park = parking_ptr->IsInParkingSpace();
  if (parking_ptr->is_park_in() && is_ego_car_in_park) {
    LOG_ERROR(
        "Got a parking in task, but ego is in target parking space already, "
        "clear task!");
    data_center_->set_planning_clear_task(true);
    data_center_->set_have_task(false);
    data_center_->mutable_event_report_proxy()->SetEvent(
        EventType::PARK_IN_FINISH, data_center_->parking_ptr()->GetParkId());
    data_center_->set_parking_ptr(nullptr);
    data_center_->GetNavigationSwapContext().parking_space_ptr.Set(nullptr);
    return false;
  }
  // if (!parking_ptr->is_park_in() &&
  //     parking_ptr->OriginPark()->Type() ==
  //         global::hdmap::ParkingSpace_ParkingType_ONLANE &&
  //     is_ego_car_in_park) {
  //   data_center_->mutable_event_report_proxy()->SetEvent(
  //       EventType::PARK_OUT_FINISH,
  //       data_center_->parking_ptr()->GetParkId());
  // }
  if (!parking_ptr->is_park_in() &&
      // parking_ptr->OriginPark()->Type() !=
      //     global::hdmap::ParkingSpace_ParkingType_ONLANE &&
      is_ego_car_in_park) {
    auto &task_info = data_center_->mutable_task_info_list()->front();
    auto data = task_info.current_frame()->mutable_inside_planner_data();
    data->is_replan = true;
    LOG_INFO("Detect parking out");
    return true;
  }
  const double parking_in_detect_max_dis =
      config::PlanningConfig::Instance()
          ->plan_config()
          .parking.parking_in_detect_max_dis;
  if (parking_space_ptr->Polygon().distance_to({utm_pose.X(), utm_pose.Y()}) >=
      parking_in_detect_max_dis) {
    LOG_INFO("Ego is too far from parking space");
    return false;
  }

  // for test
  for (std::size_t i = 0; i < 4; ++i) {
    LOG_INFO("TEST parking space point x = {:.2f}, y = {:.2f}",
             parking_space_ptr->Points()[i].x,
             parking_space_ptr->Points()[i].y);
  }
  LOG_INFO("TEST parking space heading = {:.2f}", parking_space_ptr->Heading());

  const auto &task_info = data_center_->mutable_task_info_list()->front();
  Vec3d parking_space_utm{parking_space_ptr->Points()[0].x,
                          parking_space_ptr->Points()[0].y, 0.};
  if (parking_space_ptr->Type() ==
      global::hdmap::ParkingSpace_ParkingType_ONLANE) {
    parking_space_utm.set_x(parking_space_ptr->Points()[3].x);
    parking_space_utm.set_y(parking_space_ptr->Points()[3].y);
  }
  Vec3d parking_space_rel, parking_space_odom;
  common::ConvertToRelativeCoordinate(
      parking_space_utm, {utm_pose.X(), utm_pose.Y(), utm_pose.Heading()},
      parking_space_rel);
  common::ConvertToWorldCoordinate(
      parking_space_rel,
      {data_center_->vehicle_state_proxy().X(),
       data_center_->vehicle_state_proxy().Y(),
       data_center_->vehicle_state_proxy().Heading()},
      parking_space_odom);
  SLPoint parking_space_sl;
  if (!task_info.reference_line()->GetPointInFrenetFrame(
          {parking_space_odom.x(), parking_space_odom.y()},
          &parking_space_sl)) {
    LOG_ERROR("Parking space to frenet failed.");
    return false;
  }

  double distance_threshold{-4.};
  auto &parking_thresholds = config::PlanningConfig::Instance()
                                 ->scene_special_config()
                                 .parking_thresholds;
  for (const auto &pair : parking_thresholds) {
    if (common::HashString(pair.first) == parking_space_ptr->Id()) {
      distance_threshold = -pair.second.dis;
      LOG_INFO("distance_threshold = {:.2f}, parking_id = {:.2f}",
               distance_threshold, pair.first);
      break;
    }
  }

  bool is_motorway = data_center_->master_info().curr_scenario() ==
                         ScenarioState::MOTORWAY_CRUISE ||
                     data_center_->master_info().curr_scenario() ==
                         ScenarioState::MOTORWAY_INTERSECTION;

  if (parking_space_ptr->Type() ==
          global::hdmap::ParkingSpace_ParkingType_HORIZONTAL ||
      parking_space_ptr->Type() ==
          global::hdmap::ParkingSpace_ParkingType_ONLANE ||
      is_motorway) {
    if (task_info.curr_sl().s() - parking_space_sl.s() > -8.) {
      neodrive::global::planning::SpeedLimit new_limit{};
      new_limit.set_source_type(SpeedLimitType::REF_LINE);
      new_limit.add_upper_bounds(0.83);
      new_limit.set_constraint_type(SpeedLimitType::HARD);
      new_limit.set_acceleration(0.);
      LOG_INFO("Near parking space, parking speed limit to {:.2f}",
               new_limit.upper_bounds().at(0) * 3.6);
      data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(new_limit);
    }
  }

  if (task_info.curr_sl().s() - parking_space_sl.s() < distance_threshold) {
    LOG_INFO("curr s = {:.2f}, parking_s = {:.2f}", task_info.curr_sl().s(),
             parking_space_sl.s());
    return false;
  }
  if (!parking_ptr->is_park_in()) {
    return false;
  }
  LOG_INFO("Detect parking");
  return true;
}

bool ScenarioManager::CheckParkingFinished() {
  const auto &vehicle_mode =
      data_center_->vehicle_state_proxy().chassis().driving_mode();
  const auto &open_api_cmd = *(data_center_->planning_interface_msg.ptr);
  auto parking_ptr = data_center_->parking_ptr();
  if (vehicle_mode !=
      neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
    if (parking_ptr == nullptr) {
      LOG_ERROR("parking ptr is nullptr");
      return true;
    }
    if (!parking_ptr->ParkPath().empty()) {
      parking_ptr->ParkPath().clear();
    }
    if (!parking_ptr->IsInParkingSpace() &&
        !parking_ptr->CheckOnParkingLane()) {
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::PARK_FAIL, data_center_->parking_ptr()->GetParkId());
      parking_ptr->set_is_finished(true);
      data_center_->set_parking_ptr(nullptr);
      LOG_INFO("not on lane, parking finish");
      return true;
    }
    // else {
    //   double lane_heading =
    //       parking_ptr->OverlapLane()->Heading(vehicle_position);
    //   double ego_heading = vehicle_state_utm.Heading();
    //   double heading_diff = std::fabs(ego_heading - lane_heading);
    //   if (heading_diff > M_PI / 6.) {
    //     data_center_->mutable_event_report_proxy()->SetEvent(
    //         EventType::PARK_FAIL);
    //     parking_ptr->set_is_finished(true);
    //     data_center_->set_parking_ptr(nullptr);
    //     LOG_INFO("heading diff , parking finish");
    //     return true;
    //   }
    // }
  }
  if (data_center_->planning_interface_msg.is_updated &&
      open_api_cmd.state() == global::status::FINISH) {
    data_center_->set_parking_ptr(nullptr);
    data_center_->GetNavigationSwapContext().parking_space_ptr.Set(nullptr);
    LOG_INFO("openapi clear parking task");
    return true;
  }
  // if (parking_ptr->OriginPark()->Type() ==
  //         global::hdmap::ParkingSpace_ParkingType_ONLANE &&
  //     !parking_ptr->is_park_in()) {
  //   LOG_INFO("Onlane parking space parking out, quit parking");
  //   return true;
  // }
  LOG_INFO("parking ptr is finished: {}", parking_ptr->is_finished());
  return parking_ptr->is_finished();
}

bool ScenarioManager::DetectIndoorCruise() {
  const auto &task_info = data_center_->task_info_list().front();
  if (task_info.last_frame() != nullptr &&
      task_info.last_frame()->inside_planner_data().is_indoor) {
    return true;
  }
  return false;
}

bool ScenarioManager::CheckIndoorCruiseFinished() {
  const auto &task_info = data_center_->task_info_list().front();
  if (!task_info.last_frame()->inside_planner_data().is_indoor) {
    return true;
  }
  return false;
}

bool ScenarioManager::DetectParkCruise() {
  const auto &task_info = data_center_->task_info_list().front();
  if (task_info.last_frame() != nullptr &&
      task_info.last_frame()->inside_planner_data().is_in_the_park) {
    return true;
  }
  return false;
}

bool ScenarioManager::CheckParkCruiseFinished() {
  const auto &task_info = data_center_->task_info_list().front();
  if (!task_info.last_frame()->inside_planner_data().is_in_the_park) {
    return true;
  }
  return false;
}

void ScenarioManager::SaveMonitorMessage() {
  auto &task_info_list = *data_center_->mutable_task_info_list();
  auto &task_info = task_info_list.front();
  double observe_ref_l = (task_info.last_frame())
                             ? task_info.last_frame()
                                   ->outside_planner_data()
                                   .path_observe_ref_l_info.observe_ref_l
                             : 0.0;
  static char str_buffer[512];
  sprintf(str_buffer,
          "[SCENARIO][%s->%s stage:%s->%s] [LANETYPE %s] [DETOUR %d %d %d] "
          "[NARROW %d %d] "
          "[INTERSECTION %d %d %d] [Freespace %d] [BARRIER_GATE %d %d %d] "
          "[NAVIGATOR_REQUEST: %d] [Enable Static Detour: %d] [Replan: %d] "
          "[Ref L: %.3f]",
          prev_state_str_.c_str(), curr_state_str_.c_str(),
          curr_stage_decider_ == nullptr
              ? " "
              : curr_stage_decider_->prev_state_str().c_str(),
          curr_stage_decider_ == nullptr
              ? " "
              : curr_stage_decider_->curr_state_str().c_str(),
          lane_type_.c_str(), detour_triggered_, detour_condition_satisfied_,
          detour_finished_, narrow_road_detected_, narrow_road_pass_finished_,
          intersection_detected_, intersection_pass_finished_,
          intersection_export_finished_, back_out_detected_,
          barrier_gate_triggered_, barrier_gate_condition_satisfied_,
          barrier_gate_finished_,
          data_center_->navigation_result().navigator_request,
          data_center_->master_info().enable_static_detour(),
          DataCenter::Instance()->master_info().is_use_position_stitch(),
          observe_ref_l);
  DataCenter::Instance()->SetMonitorString(str_buffer,
                                           MonitorItemSource::SCENARIO_STATE);
}

}  // namespace planning
}  // namespace neodrive

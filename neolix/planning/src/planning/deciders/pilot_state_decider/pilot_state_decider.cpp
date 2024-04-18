#include "deciders/pilot_state_decider/pilot_state_decider.h"

#include "common/get_cyberrt_version.h"
#include "common/get_machine_type.h"
#include "common_config/config/get_drive_strategy.h"
#include "src/planning/common/planning_code_define.h"

namespace neodrive {
namespace planning {
using neodrive::global::localization::SensorErrorCode;
using neodrive::global::localization::SensorErrorCode_Name;
using neodrive::global::patrol::DisCode;
using neodrive::global::patrol::ErrStat_Name;
using neodrive::global::planning::AebState;
using neodrive::global::planning::PilotChangeFlag;
using neodrive::global::planning::PilotState;
using neodrive::global::status::GearPosition_Name;
using neodrive::global::status::VcuControlMode_Name;

PilotStateDecider::PilotStateDecider() {
  name_ = "PilotStateDecider";
  Init();
}

PilotStateDecider::~PilotStateDecider() {}

bool PilotStateDecider::Init() {
  if (initialized_) {
    return true;
  }
  sys_start_t_ = cyber::Time::Now().ToSecond();
  auto& topics_config = neodrive::common::config::CommonConfig::Instance()
                            ->topics_config()
                            .topics;
  pilot_state_pub_ = data_center_->node()->CreateWriter<PilotState>(
      topics_config.at("pilot_state").topic);
  if (pilot_state_pub_ == nullptr) {
    LOG_ERROR("writer pilot_state_pub_ failed!");
    return false;
  }
  plan_config_ptr_ = &config::PlanningConfig::Instance()->plan_config();
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/pilot_state";
  if (!state_machine_->LoadStateMachine(state_machine_file)) {
    return false;
  }
  state_machine_->SetInitializeState("ACTIVE");
  prev_state_ = "";
  current_state_ = state_machine_->GetCurrentStateString();
  data_center_->mutable_pilot_state()->set_state(
      static_cast<PilotState::State>(state_machine_->GetCurrentState()));
  current_decision_ = PilotChangeFlag::T_ACTIVE_STANDBY;
  is_state_changed_ = true;
  RegisterStateMachineResponseFunctions();
  machine_type_ = neodrive::cyber::common::StrMachineType(
      neodrive::cyber::common::GetMachineType());
  drive_strategy_ = neodrive::common::config::GetDriveStrategy();
  initialized_ = true;
  block_start_timestamp_ = cyber::Time::Now().ToSecond();
  LOG_INFO("PilotStateDecider Init.");
  return true;
}

void PilotStateDecider::Reset() {
  state_machine_->SetInitializeState("ACTIVE");
  prev_state_ = "";
  current_state_ = state_machine_->GetCurrentStateString();
  current_decision_ = PilotChangeFlag::T_ACTIVE_STANDBY;
  is_state_changed_ = true;
  data_center_->mutable_pilot_state()->set_state(
      static_cast<PilotState::State>(state_machine_->GetCurrentState()));
  is_need_takeover_ = false;
  block_start_timestamp_ = cyber::Time::Now().ToSecond();
  data_center_->mutable_pilot_state()->set_is_need_takeover(is_need_takeover_);
}

void PilotStateDecider::PackagePilotStateMsg() {
  auto pilot_state_msg_ptr = data_center_->mutable_pilot_state();
  pilot_state_msg_ptr->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  pilot_state_msg_ptr->mutable_header()->set_sequence_num(++sequence_num_);
  pilot_state_msg_ptr->mutable_header()->set_module_name("planning");
  pilot_state_msg_ptr->set_state(
      static_cast<PilotState::State>(state_machine_->GetCurrentState()));
  pilot_state_msg_ptr->set_is_need_takeover(is_need_takeover_);
}

void PilotStateDecider::AddTakeoverEvent() {
  auto& vehicle_state_proxy = data_center_->environment().vehicle_state_proxy();
  if (prev_is_auto_driving_ && !data_center_->is_auto_driving()) {
    data_center_->AddRecordEvent(
        EventOfInterest::TAKE_OVER,
        plan_config_ptr_->record_event.take_over_before_time_s,
        plan_config_ptr_->record_event.take_over_after_time_s);
    LOG_INFO("manual Take over, record even signal.");
  }
  prev_is_auto_driving_ = data_center_->is_auto_driving();
}

void PilotStateDecider::UpdatePilotState() {
  LOG_DEBUG("PilotStateDecider::UpdatePilotState");
  CheckAndSetFlag();
  // stage transition.
  if (OnHandleAllStates() == false) {
    auto it = handle_state_function_map_->find(current_state_);
    if (it != handle_state_function_map_->end()) {
      auto func = it->second;
      (this->*func)();
    } else {
      // missing handle state function
      LOG_ERROR("missing handle state function {}", current_state_);
    }
  }
  UpdateStateMachine();
  PackagePilotStateMsg();
  AddTakeoverEvent();
  SaveMonitorMessage();
  pilot_state_pub_->Write(data_center_->pilot_state());
  is_state_changed_ = false;
}

ErrorCode PilotStateDecider::Execute(TaskInfo& task_info) {
  update_limited_speed_ = false;
  LOG_INFO("PilotStateDecider::MakeDecision: pilot_state: {}",
           PilotState::State_Name(data_center_->pilot_state().state()));
  if (data_center_->pilot_state().state() == PilotState::DEGRADATION) {
    SmoothSpeedLimitAction();
  }
  LOG_INFO("PilotStateDecider::MakeDecision finished.");
  return ErrorCode::PLANNING_OK;
}

void PilotStateDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::DEGRADATION);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "DEGRADATION {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

void PilotStateDecider::UpdateStateMachine() {
  if (!is_state_changed_) return;
  prev_state_ = current_state_;
  if (!state_machine_->ChangeState(current_decision_)) {
    LOG_ERROR("ChangeState failed. curr_state: {}, curr_decision: {}",
              current_state_,
              state_machine_->GetChangeFlagStr(current_decision_))
    return;
  }
  current_state_ = state_machine_->GetCurrentStateString();
  LOG_INFO("pilot state change from {} to {}", prev_state_, current_state_);
}

void PilotStateDecider::CheckAndSetFlag() {
  auto& chassis = data_center_->vehicle_state_proxy().chassis();
  auto patrol_status = data_center_->patrol_status_msg.ptr;

  // update: pilot_error_level_, is_permanent_error_
  do {
    // check vehicle_state_valid
    vehicle_state_valid_ = true;
    is_permanent_error_ = false;
    is_localization_pos_jump_ = false;
    pilot_error_level_ = PilotErrorLevel::NORMAL;
    if (data_center_->is_sim()) {
      break;
    }
    if (!data_center_->environment().vehicle_state_proxy().IsValid() ||
        !data_center_->environment().vehicle_state_odometry_proxy().IsValid()) {
      LOG_ERROR("vehicle_state invalid");
      reason_ = "VechileStateInvalid";
      vehicle_state_valid_ = false;
      pilot_error_level_ = PilotErrorLevel::SERIOUS;
      break;
    }

    // perception arbitration: roi sector nan raio speed limit
    if (data_center_->lidar_non_point_severity_lvel() !=
        SeverityLevel::SEVERITY_NORMAL) {
      pilot_error_level_ = (data_center_->lidar_non_point_severity_lvel() ==
                            SeverityLevel::SEVERITY_EMERGENCY)
                               ? PilotErrorLevel::SERIOUS
                               : PilotErrorLevel::ALARM;
      reason_ = "LidarPerceptionNanRatioHigh: level" +
                std::to_string(static_cast<int>(
                    data_center_->lidar_non_point_severity_lvel()));
      LOG_INFO(
          "set pilot_error_level to {}, roi sector error, severity level {}",
          static_cast<int>(pilot_error_level_),
          static_cast<int>(data_center_->lidar_non_point_severity_lvel()));
    }

    // perception arbitration: error code direct escalation
    if (data_center_->lidar_perception_error_code() ==
            PerceptionErrorCodeInLidar::POINTCLOUD_DATA_SERIOUS ||
        data_center_->lidar_perception_error_code() ==
            PerceptionErrorCodeInLidar::POINTCLOUD_ODO_SERIOUS ||
        data_center_->lidar_perception_error_code() ==
            PerceptionErrorCodeInLidar::POINTCLOUD_MISS) {
      reason_ = "LidarPerceptionDetectionError";
      pilot_error_level_ = PilotErrorLevel::SERIOUS;
      LOG_INFO(
          "set pilot_error_level to SERIOUS, lidar perception error code {}",
          static_cast<int>(data_center_->lidar_perception_error_code()));
      break;
    }

    double now_t = cyber::Time::Now().ToSecond();
    if (data_center_->lidar_freespace_msg.is_updated &&
        data_center_->lidar_freespace_msg.ptr->freespace_size() <= 2)
      lidar_freespace_empty_q_.emplace(now_t);
    while (!lidar_freespace_empty_q_.empty()) {
      if (now_t - lidar_freespace_empty_q_.front() > 1.0)
        lidar_freespace_empty_q_.pop();
      else
        break;
    }
    if (lidar_freespace_empty_q_.size() >=
        plan_config_ptr_->pilot_state.lidar_freespace_empty_threshold) {
      reason_ = "LidarFreespaceEmpty";
      pilot_error_level_ = PilotErrorLevel::SERIOUS;
      LOG_INFO("lidar freespace empty too match:{}",
               lidar_freespace_empty_q_.size());
    }

    auto status_msg = data_center_->localization_status_msg.ptr;
    double utm_imu_delay =
        status_msg->utm_measurement_time() - status_msg->utm_raw_imu_time();
    double utm_veh_delay =
        status_msg->utm_measurement_time() - status_msg->utm_raw_veh_time();
    bool utm_sensor_delay =
        (utm_imu_delay >
         plan_config_ptr_->pilot_state.utm_sensor_delay_threshold) ||
        (utm_veh_delay >
         plan_config_ptr_->pilot_state.utm_sensor_delay_threshold);
    bool utm_invalid = status_msg->msf_status().has_sensor_error_code() &&
                       ((status_msg->msf_status().sensor_error_code() ==
                         SensorErrorCode::WHEEL_INVALID) ||
                        (status_msg->msf_status().sensor_error_code() ==
                         SensorErrorCode::IMU_INVALID) ||
                        (status_msg->msf_status().sensor_error_code() ==
                         SensorErrorCode::WHEEL_ERROR));

    double odom_imu_delay = status_msg->odometry_measurement_time() -
                            status_msg->odometry_raw_imu_time();
    double odom_veh_delay = status_msg->odometry_measurement_time() -
                            status_msg->odometry_raw_veh_time();
    bool odom_sensor_delay =
        (odom_imu_delay >
         plan_config_ptr_->pilot_state.odom_sensor_delay_threshold) ||
        (odom_veh_delay >
         plan_config_ptr_->pilot_state.odom_sensor_delay_threshold);

    if (status_msg->has_odometry_sensor_error_code() &&
        (status_msg->odometry_sensor_error_code() ==
         SensorErrorCode::WHEEL_ERROR)) {
      if (++wheel_error_cnt_ >=
          plan_config_ptr_->pilot_state.odom_wheel_error_cnt_threshold) {
        is_wheel_error_ = true;
      }
      LOG_WARN("Has SensorErrorCode::WHEEL_ERROR, wheel_error_cnt: {}",
               wheel_error_cnt_);
    } else {
      wheel_error_cnt_ = 0;
      is_wheel_error_ = false;
    }

    bool odom_invalid = status_msg->has_odometry_sensor_error_code() &&
                        ((status_msg->odometry_sensor_error_code() ==
                          SensorErrorCode::WHEEL_INVALID) ||
                         (status_msg->odometry_sensor_error_code() ==
                          SensorErrorCode::IMU_INVALID) ||
                         is_wheel_error_);

    if (utm_invalid || utm_sensor_delay || odom_invalid || odom_sensor_delay) {
      reason_ =
          SensorErrorCode_Name(status_msg->msf_status().sensor_error_code()) +
          "|" + SensorErrorCode_Name(status_msg->odometry_sensor_error_code()) +
          ". utm_invalid: " + std::to_string(utm_invalid) +
          ", utm_delay: " + std::to_string(utm_sensor_delay) +
          ", [utm_imu_delay: " + std::to_string(utm_imu_delay) +
          ", utm_veh_delay: " + std::to_string(utm_veh_delay) +
          "], odom_invalid: " + std::to_string(odom_invalid) +
          ", odom_delay: " + std::to_string(odom_sensor_delay) +
          ", [odom_imu_delay: " + std::to_string(odom_imu_delay) +
          ", odom_veh_delay: " + std::to_string(odom_veh_delay) + "]";
      pilot_error_level_ = PilotErrorLevel::SERIOUS;
      LOG_ERROR(
          "utm_invalid: {} {} utm_delay:{} "
          "[utm_imu_delay:{},utm_veh_delay:{}], odom_invalid: {} {} "
          "odom_delay:{} "
          "[odom_imu_delay:{},odom_veh_delay:{}]",
          utm_invalid,
          SensorErrorCode_Name(status_msg->msf_status().sensor_error_code()),
          utm_sensor_delay, utm_imu_delay, utm_veh_delay, odom_invalid,
          SensorErrorCode_Name(status_msg->odometry_sensor_error_code()),
          odom_sensor_delay, odom_imu_delay, odom_veh_delay);
    }
    auto& utm_msg = data_center_->pose_base_link_in_utm_msg;
    if (utm_msg.ptr != nullptr && utm_msg.is_updated) {
      auto curr_utm_msg_vec =
          utm_msg.reader->GetNewObservedMessages(prev_utm_msg_);
      for (auto item = curr_utm_msg_vec.rbegin();
           item != curr_utm_msg_vec.rend(); ++item) {
        if (nullptr == prev_utm_msg_) {
          prev_utm_msg_ = *item;
        }
        curr_utm_msg_ = *item;
        if (1 == curr_utm_msg_->header().sequence_num() -
                     prev_utm_msg_->header().sequence_num()) {
          common::math::Vec2d prev_utm_pos{
              prev_utm_msg_->pose().position().x(),
              prev_utm_msg_->pose().position().y()};

          common::math::Vec2d curr_utm_pos{
              curr_utm_msg_->pose().position().x(),
              curr_utm_msg_->pose().position().y()};
          double localization_position_jump_dis =
              prev_utm_pos.distance_to(curr_utm_pos);
          if (localization_position_jump_dis >=
              plan_config_ptr_->pilot_state
                  .localization_position_jump_threshold) {
            reason_ = "LocalizationPositionJump";
            pilot_error_level_ = PilotErrorLevel::SERIOUS;
            is_localization_pos_jump_ = true;
            LOG_INFO("LocalizationPositionJump dis:{}",
                     localization_position_jump_dis);
          }
        }
        prev_utm_msg_ = curr_utm_msg_;
      }
      if (is_localization_pos_jump_) {
        break;
      }
    }

    if (data_center_->distance_to_zone() < 0.0) {
      reason_ = "NotInOdd";
      pilot_error_level_ = PilotErrorLevel::SERIOUS;
      LOG_INFO("not in odd");
    }
    double perception_delay_t =
        data_center_->perception_obstacles_msg.reader->GetDelaySec();
    double localization_delay_t =
        data_center_->twist_base_link_msg.reader->GetDelaySec();
    if (perception_delay_t >
            plan_config_ptr_->pilot_state.perception_msg_delay_threshold ||
        localization_delay_t >
            plan_config_ptr_->pilot_state.localization_msg_delay_threshold) {
      reason_ = "MsgDelay,per:" + std::to_string(perception_delay_t) +
                "|loc:" + std::to_string(localization_delay_t);
      pilot_error_level_ = PilotErrorLevel::SERIOUS;
      LOG_INFO("key msg delay:perception:{},localization:{}",
               perception_delay_t, localization_delay_t);
    }

    // no patrol error.
    if (0 == patrol_status->err_items_size()) {
      break;
    }
    // patrol error.
    bool is_system_start =
        now_t - sys_start_t_ > plan_config_ptr_->pilot_state.system_start_time;
    for (auto& erritem : patrol_status->err_items()) {
      // check hardware error.
      if (is_system_start && true == erritem.is_hardware_error()) {
        is_permanent_error_ = true;
        reason_ = "PatrolErrorCode:" + erritem.err_msg();
        pilot_error_level_ = PilotErrorLevel::SERIOUS;
        LOG_INFO("set pilot_error_level to SERIOUS, {}", erritem.err_msg());
        break;
      }
      auto error_level = GetPilotErrorLevelFromErrItem(erritem);
      if (error_level > pilot_error_level_) {
        pilot_error_level_ = error_level;
        LOG_INFO("set pilot_error_level to {}, {}", int(pilot_error_level_),
                 erritem.err_msg());
      }
      if (pilot_error_level_ == PilotErrorLevel::SERIOUS) {
        break;
      }
    }
  } while (false);
  // update: latest_patrol_error_time_
  if (pilot_error_level_ != PilotErrorLevel::NORMAL) {
    latest_patrol_error_time_ =
        common::util::TimeLogger::GetCurrentTimeseocnd();
  }
  if (!is_need_takeover_) {
    double now_timestamp = cyber::Time::Now().ToSecond();
    if (now_timestamp - block_start_timestamp_ >
        plan_config_ptr_->pilot_state.system_start_time) {
      is_need_takeover_ = true;
    }
  }
  LOG_INFO("is_permanent_error_: {}, PilotErrorLevel: {}", is_permanent_error_,
           int(pilot_error_level_));
}

void PilotStateDecider::RegisterStateMachineResponseFunctions() {
  handle_state_function_map_->insert(
      std::pair<std::string, void (PilotStateDecider::*)()>(
          "STANDBY", &PilotStateDecider::OnHandleStateStandby));
  handle_state_function_map_->insert(
      std::pair<std::string, void (PilotStateDecider::*)()>(
          "REJECT", &PilotStateDecider::OnHandleStateReject));
  handle_state_function_map_->insert(
      std::pair<std::string, void (PilotStateDecider::*)()>(
          "ACTIVE", &PilotStateDecider::OnHandleStateActive));
  handle_state_function_map_->insert(
      std::pair<std::string, void (PilotStateDecider::*)()>(
          "DEGRADATION", &PilotStateDecider::OnHandleStateDegradation));
  handle_state_function_map_->insert(
      std::pair<std::string, void (PilotStateDecider::*)()>(
          "ESCALATION", &PilotStateDecider::OnHandleStateEscalation));
}

bool PilotStateDecider::OnHandleAllStates() { return false; }

void PilotStateDecider::OnHandleStateStandby() {
  double now_timestamp = cyber::Time::Now().ToSecond();
  do {
    // check escalation.
    if (PilotErrorLevel::SERIOUS == pilot_error_level_) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_STANDBY_ESCALATION;
      last_degradation_time_ = now_timestamp;
      break;
    }
    // check degradation.
    if (PilotErrorLevel::ALARM == pilot_error_level_) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_STANDBY_DEGRADATION;
      last_degradation_time_ = now_timestamp;
      break;
    }
    // to REJECT
    if (!data_center_->is_auto_driving()) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_STANDBY_REJECT;
      break;
    }
    // to ACTIVE.
    if (data_center_->global_state_proxy().HaveTask()) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_STANDBY_ACTIVE;
      break;
    }
  } while (0);  // break here.
  if (!reason_.empty()) reason_.clear();
}

void PilotStateDecider::OnHandleStateReject() {
  double now_timestamp = cyber::Time::Now().ToSecond();
  do {
    // check escalation.
    if (PilotErrorLevel::SERIOUS == pilot_error_level_) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_REJECT_ESCALATION;
      last_degradation_time_ = now_timestamp;
      break;
    }
    // check degradation.
    if (PilotErrorLevel::ALARM == pilot_error_level_) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_REJECT_DEGRADATION;
      last_degradation_time_ = now_timestamp;
      break;
    }
    // to ACTIVE or STANDBY.
    if (data_center_->is_auto_driving()) {
      is_state_changed_ = true;
      current_decision_ = data_center_->global_state_proxy().HaveTask()
                              ? PilotChangeFlag::T_REJECT_ACTIVE
                              : PilotChangeFlag::T_REJECT_STANDBY;
      break;
    }
  } while (0);  // break here.
  if (!reason_.empty()) reason_.clear();
}

void PilotStateDecider::OnHandleStateActive() {
  double now_timestamp = cyber::Time::Now().ToSecond();
  do {
    // check escalation.
    if (PilotErrorLevel::SERIOUS == pilot_error_level_) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_ACTIVE_ESCALATION;
      last_degradation_time_ = now_timestamp;
      break;
    }
    // check degradation.
    if (PilotErrorLevel::ALARM == pilot_error_level_) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_ACTIVE_DEGRADATION;
      last_degradation_time_ = now_timestamp;
      break;
    }
    // to REJECT
    if (!data_center_->is_auto_driving()) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_ACTIVE_REJECT;
      break;
    }
    // to STANDBY
    if (!data_center_->global_state_proxy().HaveTask()) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_ACTIVE_STANDBY;
      break;
    }
  } while (0);  // break here.
  if (!reason_.empty()) reason_.clear();
}

PilotChangeFlag::ChangeFlag PilotStateDecider::BackToNomalState(
    bool from_escalation) {
  reason_.clear();
  if (!data_center_->is_auto_driving()) {
    return from_escalation ? PilotChangeFlag::T_ESCALATION_REJECT
                           : PilotChangeFlag::T_DEGRADATION_REJECT;
  }
  if (data_center_->global_state_proxy().HaveTask()) {
    return from_escalation ? PilotChangeFlag::T_ESCALATION_ACTIVE
                           : PilotChangeFlag::T_DEGRADATION_ACTIVE;
  }
  return from_escalation ? PilotChangeFlag::T_ESCALATION_STANDBY
                         : PilotChangeFlag::T_DEGRADATION_STANDBY;
}

void PilotStateDecider::OnHandleStateDegradation() {
  double now_timestamp = cyber::Time::Now().ToSecond();
  do {
    // check escalaiton.
    // serious error || alarm last too long.
    if (PilotErrorLevel::SERIOUS == pilot_error_level_) {
      is_state_changed_ = true;
      current_decision_ = PilotChangeFlag::T_DEGRADATION_ESCALATION;
      last_degradation_time_ = now_timestamp;
      break;
    }
    // check upgradation.
    if (PilotErrorLevel::NORMAL == pilot_error_level_) {
      if (now_timestamp - last_upgradation_time_ <
          plan_config_ptr_->pilot_state.frequent_degradation_interval) {
        // frequent into degradation. strict upgrade condition.
        // long time no error, upgrade.
        if (now_timestamp - latest_patrol_error_time_ >
            plan_config_ptr_->pilot_state.normal_upgradation_interval) {
          is_state_changed_ = true;
          current_decision_ = BackToNomalState(false);
          last_upgradation_time_ = now_timestamp;
          break;
        }
        // keep degradation. reset degradation timing.
        last_degradation_time_ = now_timestamp;
        last_upgradation_time_ = now_timestamp;
      } else {
        // loose upgrade condition.
        is_state_changed_ = true;
        current_decision_ = BackToNomalState(false);
        last_upgradation_time_ = now_timestamp;
        break;
      }
    }
    // keep degradation state.
  } while (0);  // break here.
}

void PilotStateDecider::OnHandleStateEscalation() {
  double now_timestamp = cyber::Time::Now().ToSecond();
  do {
    // check upgradation.
    if (PilotErrorLevel::NORMAL == pilot_error_level_) {
      // try updradation.
      if (now_timestamp - last_upgradation_time_ <
              plan_config_ptr_->pilot_state.frequent_degradation_interval ||
          now_timestamp - last_degradation_time_ >
              plan_config_ptr_->pilot_state.escalation_hold_interval) {
        // frequent degradation or escalation hold.
        // strict upgradation.
        // long time no error, upgrade.
        if (now_timestamp - latest_patrol_error_time_ >
            plan_config_ptr_->pilot_state.normal_upgradation_interval) {
          is_state_changed_ = true;
          current_decision_ = BackToNomalState(true);
          last_upgradation_time_ = now_timestamp;
          break;
        }
        // keep escalation. reset degradation timing.
        last_degradation_time_ = now_timestamp;
        last_upgradation_time_ = now_timestamp;
      } else {
        is_state_changed_ = true;
        current_decision_ = BackToNomalState(true);
        last_upgradation_time_ = now_timestamp;
        break;
      }
    }
    // keep escalation.
  } while (0);  // break here.
}

void PilotStateDecider::SaveMonitorMessage() {
  static char str_buffer[512];
  auto patrol_status = data_center_->patrol_status_msg.ptr;

  if (patrol_status->err_items_size() > 0) {
    prev_error_status_ = " ";
    for (auto& err_item : patrol_status->err_items()) {
      prev_error_status_ += "[" + patrol_status->err_items(0).err_msg() + " " +
                            std::to_string(patrol_status->err_level()) + "]";
    }
    prev_error_status_time_ = data_center_->init_frame_time();
  } else if (data_center_->init_frame_time() - prev_error_status_time_ > 2.0) {
    prev_error_status_ = "";
    prev_error_status_time_ = data_center_->init_frame_time();
  }
  auto& aeb_cmd = *(data_center_->aeb_cmd_msg.ptr);
  auto aeb_state_str = AebState::State_Name(aeb_cmd.aeb_state().state());
  auto aeb_brake = aeb_cmd.brake();
  auto global_state_str = neodrive::global::status::State_Name(
      data_center_->global_state_proxy().global_state().state());
  auto& vehicle_state_proxy = data_center_->environment().vehicle_state_proxy();
  auto gear_str = GearPosition_Name(vehicle_state_proxy.Gear());
  auto vcu_mode_str =
      VcuControlMode_Name(vehicle_state_proxy.chassis().current_vcu_mode());
  auto cyberrt_version_str = neodrive::cyber::common::GetCyberrtVersion();

  snprintf(str_buffer, sizeof(str_buffer),
           "[PATROL][%s][strategy:%s][%s][%s]%s", machine_type_.c_str(),
           drive_strategy_.c_str(), cyberrt_version_str.c_str(),
           ErrStat_Name(patrol_status->err_stat()).c_str(),
           prev_error_status_.c_str());
  data_center_->SetMonitorString(str_buffer, MonitorItemSource::DISCODE);

  snprintf(str_buffer, sizeof(str_buffer),
           "[PILOT][%s %s %s][%s->%s][auto: %d task: %d][vehicle_state: "
           "%d][aeb: %s %.1lf][reason: %s]",
           global_state_str.c_str(), gear_str.c_str(), vcu_mode_str.c_str(),
           prev_state_.c_str(), current_state_.c_str(),
           data_center_->is_auto_driving(),
           data_center_->global_state_proxy().HaveTask(), vehicle_state_valid_,
           aeb_state_str.c_str(), aeb_brake, reason_.c_str());
  data_center_->SetMonitorString(str_buffer, MonitorItemSource::PILOT_STATE);
}

void PilotStateDecider::SmoothSpeedLimitAction() {
  // set low speed limit to decelerate.
  update_limited_speed_ = true;
  auto& pilot_state_config = plan_config_ptr_->pilot_state;
  switch (data_center_->lidar_non_point_severity_lvel()) {
    case SeverityLevel::SEVERITY_LOW:
      limited_speed_ = pilot_state_config.lidar_non_point_low_speed_limit;
      break;
    case SeverityLevel::SEVERITY_MEDIUM:
      limited_speed_ = pilot_state_config.lidar_non_point_medium_speed_limit;
      break;
    case SeverityLevel::SEVERITY_HIGH:
      limited_speed_ = pilot_state_config.lidar_non_point_high_speed_limit;
      break;
    default:
      limited_speed_ = pilot_state_config.degradation_speed_limit;
      break;
  }
}

}  // namespace planning
}  // namespace neodrive

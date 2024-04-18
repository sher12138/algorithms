#include "human_interface_decider.h"

#include "common/coordinate/coordinate_math.h"
namespace neodrive {
namespace planning {
using neodrive::global::status::GlobalState;
using neodrive::global::status::StopReason;

HumanInterfaceDecider::HumanInterfaceDecider() {
  name_ = "HumanInterfaceDecider";
  not_sop_.Init(0, FLAGS_planning_human_interface_trigger_conts);
  stop_from_planning_.Init(0, FLAGS_planning_human_interface_trigger_conts);
  vehicle_collision_.Init(0, FLAGS_planning_human_interface_trigger_conts);
  invalid_pos_.Init(0, FLAGS_planning_human_interface_trigger_conts);
  pre_utm_ = {0.0, 0.0, 0.0};
}

HumanInterfaceDecider::~HumanInterfaceDecider() {
  not_sop_.Reset();
  stop_from_planning_.Reset();
  vehicle_collision_.Reset();
  invalid_pos_.Reset();
}

void HumanInterfaceDecider::Reset() {}

ErrorCode HumanInterfaceDecider::Execute(TaskInfo &task_info) {
  // reserved
  return ErrorCode::PLANNING_OK;
}

bool HumanInterfaceDecider::UpdateStopReason(
    const MasterInfo &master_info, const GlobalState &curr_global_state) {
  bool ask_for_take_over = false;
  auto last_stop_reason = curr_global_state.stop_reason();
  LOG_INFO("last_stop_reason: {}", last_stop_reason);

  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  if (planning_ask_take_over_enable_) {
    LOG_INFO("ask take over time check");
    double delta_time = common::util::TimeLogger::GetCurrentTimeseocnd() -
                        planning_ask_take_over_start_;
    LOG_INFO("ask take over time check [{:.4f}]", delta_time);
    if (delta_time > plan_config.human_interface.block_time_ask_for_takeover &&
        delta_time < plan_config.human_interface.ask_for_take_over_last_time) {
      // ask for take over for 10 mins
      ask_for_take_over = true;
    } else {
      ask_for_take_over = false;
    }
  } else {
    ask_for_take_over = false;
  }

  StopReason upt_reason = StopReason::NOT_STOP;
  bool last_planning_ask_take_over_enable = planning_ask_take_over_enable_;
  planning_ask_take_over_enable_ = false;
  if (master_info.human_take_over()) {
    if (last_stop_reason == StopReason::HUMAN_TAKE_OVER) {
      return true;
    }
    upt_reason = StopReason::HUMAN_TAKE_OVER;
  } else if (master_info.is_trigger_stop()) {
    if (last_stop_reason == StopReason::PARK_IOV) {
      return true;
    }
    upt_reason = StopReason::PARK_IOV;
  } else if (master_info.is_station_stop()) {
    if (last_stop_reason == StopReason::REACH_STATION) {
      return true;
    }
    upt_reason = StopReason::REACH_STATION;
  } else if (ask_for_take_over) {
    if (last_stop_reason == StopReason::INVALID_HILL_START) {
      return true;
    }
    upt_reason = StopReason::INVALID_HILL_START;
  } else if (master_info.stop_due_to_invalid_pos()) {
    planning_ask_take_over_enable_ = true;
    if (last_stop_reason == StopReason::INVALID_POSITION) {
      return true;
    }
    upt_reason = StopReason::INVALID_POSITION;
  } else if (master_info.stop_due_to_collision()) {
    planning_ask_take_over_enable_ = true;
    if (last_stop_reason == StopReason::VEHILCE_COLLISION) {
      return true;
    }
    upt_reason = StopReason::VEHILCE_COLLISION;
  } else if (master_info.stop_due_to_obs()) {
    planning_ask_take_over_enable_ = true;
    if (last_stop_reason == StopReason::ESTOP_FROM_PLANNING) {
      return true;
    }
    upt_reason = StopReason::ESTOP_FROM_PLANNING;
  } else if (master_info.is_ask_for_takeover()) {
    if (master_info.stop_due_to_junction_left_turn()) {
      if (last_stop_reason == StopReason::JUNCTION_LEFT_TURN) {
        return true;
      }
      upt_reason = StopReason::JUNCTION_LEFT_TURN;
    } else {
      if (last_stop_reason == StopReason::HAVE_TO_TAKEOVER) {
        return true;
      }
      upt_reason = StopReason::HAVE_TO_TAKEOVER;
    }
  } else if (master_info.restricted_area()) {
    planning_ask_take_over_enable_ = true;
    if (last_stop_reason == StopReason::NEAR_BLOCKED_LANE) {
      return true;
    }
    upt_reason = StopReason::NEAR_BLOCKED_LANE;
  } else if (master_info.barrier_gate_context().is_wait_stage) {
    planning_ask_take_over_enable_ = true;
    if (last_stop_reason == StopReason::HAVE_TO_TAKEOVER) {
      return true;
    }
    upt_reason = StopReason::HAVE_TO_TAKEOVER;
  } else {
    if (last_stop_reason == StopReason::NOT_STOP) {
      return true;
    }
    upt_reason = StopReason::NOT_STOP;
  }

  if (planning_ask_take_over_enable_ == true &&
      last_planning_ask_take_over_enable == false) {
    planning_ask_take_over_start_ =
        common::util::TimeLogger::GetCurrentTimeseocnd();
  } else if (planning_ask_take_over_enable_ == false &&
             last_planning_ask_take_over_enable == false) {
    planning_ask_take_over_start_ = 0.0;
  } else if (planning_ask_take_over_enable_ == false &&
             last_planning_ask_take_over_enable == true) {
    planning_ask_take_over_start_ = 0.0;
  }
  LOG_INFO("ask_take_over: {}, last_ask_take_over: {}, ",
           planning_ask_take_over_enable_, last_planning_ask_take_over_enable);
  LOG_INFO("ask_take_over time: {:.4f}", planning_ask_take_over_start_);

  if (upt_reason == StopReason::HUMAN_TAKE_OVER ||
      upt_reason == StopReason::REACH_STATION ||
      upt_reason == StopReason::PARK_IOV ||
      upt_reason == StopReason::INVALID_HILL_START ||
      upt_reason == StopReason::HAVE_TO_TAKEOVER ||
      upt_reason == StopReason::JUNCTION_LEFT_TURN ||
      upt_reason == StopReason::NEAR_BLOCKED_LANE) {
    last_stop_reason = upt_reason;
    LOG_INFO("Final_global_state: {}", StopReason_Name(upt_reason));
    data_center_->mutable_global_state_proxy()
        ->mutable_global_state()
        ->set_stop_reason(upt_reason);
    return true;
  }
  StopReason update_reason = last_stop_reason;

  // smoother
  switch (upt_reason) {
    case StopReason::NOT_STOP:
      not_sop_.Push(1);
      stop_from_planning_.Push(0);
      vehicle_collision_.Push(0);
      invalid_pos_.Push(0);
      if (not_sop_.GetNumber() >
          0.8 * FLAGS_planning_human_interface_trigger_conts) {
        if (not_sop_.GetAverage() >
            FLAGS_planning_human_interface_trigger_probability) {
          update_reason = upt_reason;
        }
      }
      break;
    case StopReason::ESTOP_FROM_PLANNING:
      not_sop_.Push(0);
      stop_from_planning_.Push(1);
      vehicle_collision_.Push(0);
      invalid_pos_.Push(0);
      if (stop_from_planning_.GetNumber() >
          0.8 * FLAGS_planning_human_interface_trigger_conts) {
        if (stop_from_planning_.GetAverage() >
            FLAGS_planning_human_interface_trigger_probability) {
          update_reason = upt_reason;
        }
      }
      break;
    case StopReason::VEHILCE_COLLISION:
      not_sop_.Push(0);
      stop_from_planning_.Push(0);
      vehicle_collision_.Push(1);
      invalid_pos_.Push(0);
      if (vehicle_collision_.GetNumber() >
          0.8 * FLAGS_planning_human_interface_trigger_conts) {
        if (vehicle_collision_.GetAverage() >
            FLAGS_planning_human_interface_trigger_probability) {
          update_reason = upt_reason;
        }
      }
      break;
    case StopReason::INVALID_POSITION:
      not_sop_.Push(0);
      stop_from_planning_.Push(0);
      vehicle_collision_.Push(0);
      invalid_pos_.Push(1);
      if (invalid_pos_.GetNumber() >
          0.8 * FLAGS_planning_human_interface_trigger_conts) {
        if (invalid_pos_.GetAverage() >
            FLAGS_planning_human_interface_trigger_probability) {
          update_reason = upt_reason;
        }
      }
      break;
    default:
      break;
  }
  if (last_stop_reason == update_reason) {
    LOG_INFO("keep the stop reason");
  } else {
    last_stop_reason = update_reason;
    LOG_INFO("Final_global_state: {}", update_reason);
    data_center_->mutable_global_state_proxy()
        ->mutable_global_state()
        ->set_stop_reason(update_reason);
    return true;
  }

  return true;
}

void HumanInterfaceDecider::UpdateClosestObsS(const double s) {
  closest_front_obs_s_ = s;
}

void HumanInterfaceDecider::UpdateFrontObsInfo() {
  front_have_static_obs_ = closest_front_obs_s_ < kFrontStaticObstDistThreshold;
  LOG_INFO("front_have_static_obs_:{},static_s:{}", front_have_static_obs_,
           closest_front_obs_s_);
}

void HumanInterfaceDecider::AbnormalStopMonitor() {
  if (data_center_->is_in_port_odd()) {
    abnormal_stop_start_ = -1.0;
    abnormal_stop_move_count_ = 0.0;
    return;
  }
  UpdateFrontObsInfo();
  auto &interface_config =
      config::PlanningConfig::Instance()->plan_config().human_interface;
  auto &vehicle_state = data_center_->vehicle_state_utm();
  if (data_center_->is_auto_driving() && data_center_->have_task() &&
      vehicle_state.LinearVelocity() < interface_config.slow_speed_threshold &&
      !data_center_->is_waiting_traffic_light() && !front_have_static_obs_) {
    Vec3d curr_utm{vehicle_state.X(), vehicle_state.Y(), 0.0};
    double now_t = cyber::Time::Now().ToSecond();
    abnormal_stop_start_ =
        abnormal_stop_start_ <= 0.0 ? now_t : abnormal_stop_start_;
    abnormal_stop_move_count_ += common::Distance2D(pre_utm_, curr_utm);
    if (now_t - abnormal_stop_start_ >=
        interface_config.no_obstacle_stop_time_threshold)
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::ABNORMAL_STOP);
  } else {
    abnormal_stop_start_ = -1.0;
    abnormal_stop_move_count_ = 0.0;
  }
}

void HumanInterfaceDecider::ObstacleBlockMonitor() {
  if (data_center_->is_in_port_odd() ||
      data_center_->no_abnormal_stop_check()) {
    obstacle_block_start_ = -1.0;
    obstacle_block_move_count_ = 0.0;
    return;
  }
  auto &interface_config =
      config::PlanningConfig::Instance()->plan_config().human_interface;
  auto &vehicle_state = data_center_->vehicle_state_utm();
  LOG_INFO(
      "ObstacleBlockMonitor:{},{},{},{},{}", data_center_->is_auto_driving(),
      data_center_->have_task(),
      vehicle_state.LinearVelocity() < interface_config.slow_speed_threshold,
      !data_center_->is_waiting_traffic_light(), front_have_static_obs_);
  if (data_center_->is_auto_driving() && data_center_->have_task() &&
      vehicle_state.LinearVelocity() < interface_config.slow_speed_threshold &&
      !data_center_->is_waiting_traffic_light() && front_have_static_obs_) {
    Vec3d curr_utm{vehicle_state.X(), vehicle_state.Y(), 0.0};
    double now_t = cyber::Time::Now().ToSecond();
    obstacle_block_start_ =
        obstacle_block_start_ <= 0.0 ? now_t : obstacle_block_start_;
    obstacle_block_move_count_ += common::Distance2D(pre_utm_, curr_utm);
    if (now_t - obstacle_block_start_ >=
        interface_config.block_time_ask_for_takeover)
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::OBSTACLE_BLOCK_TIMEOUT);
  } else {
    obstacle_block_start_ = -1.0;
    obstacle_block_move_count_ = 0.0;
  }
  closest_front_obs_s_ = 999.0;
}

void HumanInterfaceDecider::CheckReachStation() {
  auto &swap_context = data_center_->GetNavigationSwapContext();
  auto &vehicle_state = data_center_->vehicle_state_utm();
  auto &interface_config =
      config::PlanningConfig::Instance()->plan_config().human_interface;
  if (swap_context.routing_destion.IsUpdated()) {
    LOG_INFO("routing destination updated");
    swap_context.routing_destion.Swap(routing_dest_);
    have_reached_station_ = false;
  }
  if (swap_context.dist_to_routing_destination.Get() > 20.) {
    have_reached_station_ = false;
    data_center_->mutable_global_state_proxy()->SetReachStation(false);
    LOG_INFO("Ego is too far from routing destination, dis2dest = {:.2f}",
             swap_context.dist_to_routing_destination.Get());
    return;
  }
  LOG_INFO("have reached station = {}, data_center_->have_task() = {}",
           have_reached_station_, data_center_->have_task());
  if (!have_reached_station_ && data_center_->have_task()) {
    double now_t = cyber::Time::Now().ToSecond();
    if (vehicle_state.LinearVelocity() >
        interface_config.station_stop_speed_threshold)
      stop_start_t_ = -1.0;
    else
      stop_start_t_ = stop_start_t_ <= 0.0 ? now_t : stop_start_t_;
    Vec3d ego_pos{vehicle_state.X(), vehicle_state.Y(), vehicle_state.Z()};
    double dist2end{data_center_->master_info().distance_to_end()};
    LOG_INFO("routing_dest x = {:.2f}, y = {:.2f}, z = {:.2f}",
             routing_dest_.x(), routing_dest_.y(), routing_dest_.z());
    LOG_INFO("ego_pos x = {:.2f}, y = {:.2f}, z = {:.2f}", ego_pos.x(),
             ego_pos.y(), ego_pos.z());
    LOG_INFO("dis2end = {:2f}, stop_start_t_ = {}, now_t = {}", dist2end,
             stop_start_t_, now_t);
    if (dist2end <= interface_config.reach_station_hard_dist_threashold &&
        now_t - stop_start_t_ >=
            interface_config.reach_station_hard_stop_time) {
      LOG_INFO("reach station hard");
      data_center_->mutable_global_state_proxy()->SetReachStation(true);
      have_reached_station_ = true;
    } else if (dist2end <=
                   interface_config.reach_station_soft_dist_threashold &&
               now_t - stop_start_t_ >=
                   interface_config.reach_station_soft_stop_time) {
      LOG_INFO("reach station soft");
      data_center_->mutable_global_state_proxy()->SetReachStation(true);
      have_reached_station_ = true;
    }
  } else {
    data_center_->mutable_global_state_proxy()->SetReachStation(false);
  }
}

}  // namespace planning
}  // namespace neodrive

#include "triggered_stop_decider.h"

#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

TriggeredStopDecider::TriggeredStopDecider() { name_ = "TriggeredStopDecider"; }

TriggeredStopDecider::~TriggeredStopDecider() {}

void TriggeredStopDecider::Reset() {
  current_stop_time_ = 0.0;
  ReferencePoint tmp_pt;
  last_stop_pos_ = tmp_pt;
  last_stop_original_pos_ = tmp_pt;
  is_in_pose_adjust_mode_ = false;
}

ErrorCode TriggeredStopDecider::Execute(TaskInfo& task_info) {
  // set default false.
  is_trigger_stop_ = false;
  behavior_stop_vehicle_ = false;
  if (data_center_->global_state_proxy().is_init()) {
    LOG_INFO("current global state [{}] donot support TriggeredStop",
             data_center_->global_state_proxy().StateName());
    return ErrorCode::PLANNING_OK;
  }

  MakeStopDecision(task_info);

  if (is_trigger_stop_ == false) {
    Reset();
  }

  if (vehicle_state_.AbsoluteLinearVelocity() <=
      FLAGS_planning_adc_stop_velocity_threshold) {
    if (data_center_->planning_interface_msg.is_available &&
        trigger_cmd_ == neodrive::global::status::FINISH &&
        !data_center_->global_state_proxy().is_finish()) {
      LOG_INFO("set finish according to openapi command");
      data_center_->mutable_global_state_proxy()->SetFinish(true);
      data_center_->mutable_routing_result()->is_available = false;
      data_center_->planning_interface_msg.is_available = false;
    } else if (trigger_cmd_ == neodrive::global::status::WAIT &&
               !data_center_->global_state_proxy().is_wait()) {
      LOG_INFO("set wait according to openapi command");
      current_stop_time_ = common::util::TimeLogger::GetCurrentTimeseocnd();
      last_stop_pos_.set_x(vehicle_state_.X());
      last_stop_pos_.set_y(vehicle_state_.Y());
      data_center_->mutable_global_state_proxy()->set_wait();
    }
  }

  return ErrorCode::PLANNING_OK;
}

void TriggeredStopDecider::SaveTaskResults(TaskInfo& task_info) {
  data_center_->mutable_master_info()->set_is_trigger_stop(is_trigger_stop_);
  if (behavior_stop_vehicle_) {
    LOG_WARN("set_behavior_stop_vehicle");
    data_center_->mutable_master_info()->set_behavior_stop_vehicle(true);
  }
}

bool TriggeredStopDecider::MakeStopDecision(TaskInfo& task_info) {
  auto& global_state = data_center_->global_state_proxy().global_state();
  MasterInfo* master_info = data_center_->mutable_master_info();
  trigger_cmd_ = global::status::INIT;
  if (data_center_->pad_normal_command_msg.is_available) {
    auto& pad_normal_command = *(data_center_->pad_normal_command_msg.ptr);
    if (pad_normal_command.action() ==
        neodrive::global::control::ControlAction::OFF) {
      // recv stop signal
      LOG_INFO("received stop wait signal from pad");
      if (global_state.state() == neodrive::global::status::INIT ||
          global_state.state() == neodrive::global::status::FINISH) {
        LOG_INFO("curr state is {}, cannot set to wait", global_state.state());
        data_center_->pad_normal_command_msg.is_available = false;
      } else {
        trigger_cmd_ = neodrive::global::status::WAIT;
      }
    } else if (pad_normal_command.action() ==
               neodrive::global::control::ControlAction::AUTO) {
      LOG_INFO("received start signal from pad");
      if (global_state.state() == neodrive::global::status::CRUISE) {
        // already recorver from WAIT
        data_center_->pad_normal_command_msg.is_available = false;
      } else if (global_state.state() == neodrive::global::status::WAIT) {
        // recv start signal
        trigger_cmd_ = neodrive::global::status::CRUISE;
      }
    } else if (pad_normal_command.action() ==
               neodrive::global::control::ControlAction::ON) {
      if (prev_trigger_cmd_ == neodrive::global::status::WAIT) {
        // recover from wait only if receive AUTO signal
        trigger_cmd_ = neodrive::global::status::WAIT;
      }
    }
  }

  if (data_center_->planning_interface_msg.is_available) {
    auto& open_api_cmd = *(data_center_->planning_interface_msg.ptr);
    if (open_api_cmd.state() == neodrive::global::status::WAIT) {
      // recv stop signal
      LOG_INFO("received stop wait signal from openapi");
      if (global_state.state() == neodrive::global::status::INIT ||
          global_state.state() == neodrive::global::status::FINISH) {
        LOG_INFO("curr state is {}, cannot set to wait", global_state.state());
        data_center_->planning_interface_msg.is_available = false;
      } else {
        trigger_cmd_ = neodrive::global::status::WAIT;
      }
    } else if (open_api_cmd.state() == neodrive::global::status::FINISH) {
      LOG_INFO("received stop finish signal from openapi");
      if (global_state.state() != neodrive::global::status::FINISH) {
        // set finish and clear all station
        trigger_cmd_ = neodrive::global::status::FINISH;
      } else {
        double curr_time = common::util::TimeLogger::GetCurrentTimeseocnd();
        if (curr_time - data_center_->receive_openapi_command_time() >
            FLAGS_planning_clear_routing_time_threshold) {
          LOG_INFO("clear openapi sginal");
          data_center_->planning_interface_msg.is_available = false;
        }
      }
    } else if (open_api_cmd.state() == neodrive::global::status::CRUISE) {
      LOG_INFO("received restart signal from openapi");
      if (global_state.state() == neodrive::global::status::CRUISE) {
        // already recorver from WAIT
        data_center_->planning_interface_msg.is_available = false;
      } else if (global_state.state() == neodrive::global::status::WAIT) {
        // recv start signal
        trigger_cmd_ = neodrive::global::status::CRUISE;
      }
    }
  }

  prev_trigger_cmd_ = trigger_cmd_;
  if (trigger_cmd_ == neodrive::global::status::FINISH) {
    is_trigger_stop_ = true;
  } else if (trigger_cmd_ == neodrive::global::status::WAIT) {
    is_trigger_stop_ = true;
  } else if (trigger_cmd_ == neodrive::global::status::CRUISE) {
    is_trigger_stop_ = false;
    is_in_pose_adjust_mode_ = false;
    if (global_state.state() == neodrive::global::status::WAIT) {
      data_center_->mutable_global_state_proxy()->set_cruise();
    }
    return true;
  } else {
    is_in_pose_adjust_mode_ = false;
    // not receive trigger stop signal
    return true;
  }

  // have stop pos
  if (is_in_pose_adjust_mode_) {
    return ContinueWithCurrentStopPoint(task_info);
  }

  ReferencePoint target_stop_point;
  SLPoint vehicle_sl_point;
  double speed = vehicle_state_.LinearVelocity();
  double min_parking_stop_distance =
      VehicleParam::Instance()->front_edge_to_center() +
      FLAGS_planning_virtual_obstacle_length;

  if (!task_info.reference_line()->GetPointInFrenetFrameWithHeading(
          {vehicle_state_.X(), vehicle_state_.Y()}, vehicle_state_.Heading(),
          &vehicle_sl_point)) {
    LOG_ERROR("failed to GetPointInFrenetFrame");
    target_stop_point.set_x(vehicle_state_.X());
    target_stop_point.set_y(vehicle_state_.Y());
  } else {
    double stop_fence_s =
        vehicle_sl_point.s() +
        (speed * speed) / (2 * FLAGS_planning_parking_stop_max_deceleration) +
        min_parking_stop_distance;
    if (!task_info.reference_line()->GetNearestRefPoint(stop_fence_s,
                                                        &target_stop_point)) {
      LOG_WARN("failed to GetNearestRefPoint, s: {:.4f}", stop_fence_s);
      target_stop_point.set_x(vehicle_state_.X());
      target_stop_point.set_y(vehicle_state_.Y());
    }
  }
  last_stop_original_pos_ = target_stop_point;
  // can stop on road?
  ReferencePoint update_pt;
  if (!UpdateTargetStopPoint(task_info, target_stop_point, update_pt)) {
    LOG_ERROR("UpdateTargetStopPoint failed");
    return false;
  }
  last_stop_pos_ = update_pt;
  is_in_pose_adjust_mode_ = true;
  LOG_INFO("inner triggered mode: {}", is_trigger_stop_);
  return ContinueWithCurrentStopPoint(task_info);
}

bool TriggeredStopDecider::ContinueWithCurrentStopPoint(TaskInfo& task_info) {
  ReferencePoint curr_refer_pt;
  if (!task_info.reference_line()->GetNearestRefPointWithHeading(
          {vehicle_state_.X(), vehicle_state_.Y()}, vehicle_state_.Heading(),
          &curr_refer_pt)) {
    LOG_ERROR("GetNearestRefPoint failed, curr pos");
    return false;
  }

  ReferencePoint target_refer_pt;
  if (!task_info.reference_line()->GetNearestRefPoint(
          Vec2d{last_stop_pos_.x(), last_stop_pos_.y()}, &target_refer_pt)) {
    LOG_ERROR("GetNearestRefPoint failed, target pos");
    return false;
  }

  double dis_to_station = target_refer_pt.s() - curr_refer_pt.s();
  double finish_distance_threshold =
      FLAGS_planning_arrive_to_destination_distance_threshold;
  if (dis_to_station < finish_distance_threshold) {
    behavior_stop_vehicle_ = true;
  }

  data_center_->mutable_master_info()->set_reference_line_destination_point(
      last_stop_pos_);

  return true;
}

bool TriggeredStopDecider::UpdateTargetStopPoint(
    TaskInfo& task_info, const ReferencePoint& target_stop_point,
    ReferencePoint& update_point) {
  update_point = target_stop_point;
  // step 1: lane related

  // sanity check
  // ...

  // setp 2: hdmap related
  ReferencePoint inner_tmp_pt = update_point;
  UpdateTargetStopPointByHdmap(task_info, inner_tmp_pt, update_point);

  // sanity check
  // ...

  return true;
}

bool TriggeredStopDecider::UpdateTargetStopPointByHdmap(
    TaskInfo& task_info, const ReferencePoint& target_stop_point,
    ReferencePoint& update_point) {
  if (!(target_stop_point.is_in_junction() ||
        target_stop_point.is_in_clear_area())) {
    update_point = target_stop_point;
    return true;
  }

  double remain_dis = task_info.reference_line()->ref_points().back().s() -
                      target_stop_point.s();
  if (remain_dis < 1.0) {
    LOG_WARN("no enough distance to adjust target pos, remain dis: {:.4f}",
             remain_dis);
    return false;
  }
  std::size_t start_index{0}, end_index{0};
  if (!task_info.reference_line()->GetStartEndIndexBySLength(
          target_stop_point.s(), remain_dis, &start_index, &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return false;
  }

  bool bfind = false;
  for (std::size_t index = start_index; index <= end_index; ++index) {
    if (task_info.reference_line()->ref_points()[index].is_in_junction() ||
        task_info.reference_line()->ref_points()[index].is_in_clear_area()) {
      continue;
    }
    bfind = true;
    update_point = task_info.reference_line()->ref_points()[index];
    break;
  }
  if (!bfind) {
    LOG_WARN("failed to find proper target stop point");
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive

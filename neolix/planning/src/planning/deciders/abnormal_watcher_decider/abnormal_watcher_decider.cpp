#include "abnormal_watcher_decider.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

AbnormalWatcherDecider::AbnormalWatcherDecider() {
  name_ = "AbnormalWatcherDecider";
}

AbnormalWatcherDecider::~AbnormalWatcherDecider() { Reset(); }

void AbnormalWatcherDecider::Reset() {}

ErrorCode AbnormalWatcherDecider::Execute(TaskInfo& task_info) {
  const auto& abnormal_watcher_config =
      config::PlanningConfig::Instance()->plan_config().abnormal_watcher;
  const double max_car_angle = normalize_angle(40.0 / 57.29578);
  const double time = 0.1;
  const double max_lateral_dis =
      VehicleParam::Instance()->length() * std::sin(max_car_angle) * time;
  const double kTriggerPoseProtectionLateralThreshold =
      max_lateral_dis * abnormal_watcher_config.lateral_attention_ratio;
  constexpr double kTriggerPoseProtectionMaxSpeed = 1.0;
  update_limited_speed_ = false;
  if (task_info.current_frame() == nullptr ||
      task_info.reference_line() == nullptr ||
      task_info.last_frame() == nullptr) {
    return ErrorCode::PLANNING_OK;
  }
  LOG_INFO("kTriggerPoseProtectionLateralThreshold: {:.3f}",
           kTriggerPoseProtectionLateralThreshold);
  LOG_INFO("adc_point: {:.3f}, {:.3f}", vehicle_state_.X(), vehicle_state_.Y());
  LOG_INFO("last_adc_point: {:.3f}, {:.3f}",
           task_info.last_frame()->inside_planner_data().vel_x,
           task_info.last_frame()->inside_planner_data().vel_y);
  SLPoint curr_sl;
  if (!task_info.reference_line()->GetPointInFrenetFrame(
          {vehicle_state_.X(), vehicle_state_.Y()}, &curr_sl)) {
    LOG_ERROR("GetPointInFrenetFrame failed");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  SLPoint last_sl = task_info.last_frame()->inside_planner_data().init_sl_point;
  // use last frame ref
  auto last_refline = task_info.last_frame()->planning_data().reference_line();
  Vec2d last_ego_xy;

  if (last_refline == nullptr) {
    LOG_INFO("Get Last Ref line failed , just return");
    return ErrorCode::PLANNING_OK;
  }

  last_refline->GetPointInCartesianFrame(last_sl, &last_ego_xy);

  SLPoint trans_last_sl;
  task_info.reference_line()->GetPointInFrenetFrame(last_ego_xy,
                                                    &trans_last_sl);

  LOG_INFO("curr_sl: {:.3f}, {:.3f}", curr_sl.s(), curr_sl.l());
  LOG_INFO("trans_last_sl: {:.3f}, {:.3f}", trans_last_sl.s(),
           trans_last_sl.l());
  trigger_pose_protection_ = (std::abs(curr_sl.l() - trans_last_sl.l()) >=
                              kTriggerPoseProtectionLateralThreshold)
                                 ? true
                                 : false;
  if (trigger_pose_protection_) {
    LOG_INFO("trigger_pose_protection function, limit max_speed");
    limited_speed_ = kTriggerPoseProtectionMaxSpeed;
    update_limited_speed_ = true;
  }

  return ErrorCode::PLANNING_OK;
}

void AbnormalWatcherDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::ABNORMAL_WATCH);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "abnormal_watch {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

}  // namespace planning
}  // namespace neodrive

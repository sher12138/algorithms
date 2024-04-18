#include "algorithm"
#include "common/visualizer_event/visualizer_event.h"
#include "motorway_speed_astern_protc_decider.h"
namespace neodrive {
namespace planning {

namespace {

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using MotorwayIntersectionStageState =
    neodrive::global::planning::MotorwayIntersectionStageState;

constexpr double kComfortableAcc = 2.0;
constexpr double kDelaytime = 0.2;
constexpr double kTurnAround = M_PI / 6;
constexpr double kLimitSpeedProtectedLateralRange = 2.;
constexpr double kLateralBuffer = 2.;
constexpr double kPositionChange = 3.0;
constexpr double kRushLateralAttDis = 2.0;

constexpr int kValidCount = 50;  //
constexpr double kLateralAttDis = 4.0;

constexpr double kStateHeading = M_PI / 6;
constexpr double kMinSpeed = 10.0 / 3.6;
constexpr double kMinAccelerate = -3.5;

void VisNeedHandleObs(const Obstacle* const obs,
                      const std::vector<double> rgb) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("FrontProtectOBs");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  event->mutable_color()->set_r(rgb[0]);
  event->mutable_color()->set_g(rgb[1]);
  event->mutable_color()->set_b(rgb[2]);
  event->mutable_color()->set_a(rgb[3]);
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  auto polygon = event->mutable_polygon()->Add();
  for (const auto& point : obs->polygon_corners()) {
    set_pt(polygon->add_point(), point);
  }
}

bool CalEgoStopTime(const double adc_speed, const double adc_accelerate,
                    const double deacc, const double duration,
                    double& stop_time) {
  stop_time = 0.0;
  if (adc_speed < 0) {
    LOG_WARN("speed is negative!");
    return false;
  }
  if (!IsDoubleEqual(adc_accelerate, 0.0, 3)) {
    LOG_WARN("adc_accelerate is bigger!");
    return false;
  }
  if (adc_speed < 0.1 && IsDoubleEqual(adc_accelerate, 0.0, 0.5)) {
    stop_time = 0.;
    return true;
  }
  if (deacc < 0.) {
    LOG_WARN("deacc is negative!,can't stop!");
    return false;
  }
  // a -> 0
  if (adc_accelerate > 0.) {
    double t_de = adc_accelerate / deacc;
    stop_time = (t_de * adc_accelerate - deacc + 2 * adc_speed) / 2 / deacc;
    stop_time += duration;
  } else {
    stop_time = adc_speed / deacc;
  }
  return true;
}

}  // namespace

MotorwaySpeedAsternProtcDecider::~MotorwaySpeedAsternProtcDecider() { Reset(); }

MotorwaySpeedAsternProtcDecider::MotorwaySpeedAsternProtcDecider() {
  name_ = "MotorwaySpeedAsternProtcDecider";
}

ErrorCode MotorwaySpeedAsternProtcDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!Init(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedAsternProtcDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_speed_limit_) {
    // const double deceleration =
    //     std::max(-3.0, std::min((speed_limit_ - ego_state_.speed) * 10.0,
    //     0.0));
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::ASTERN_PROTECTION);
    internal_speed_limit.add_upper_bounds(speed_limit_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(deaccelerate_);
    LOG_INFO(
        "FRONT_NEAR_LOW_SPEED_JUMP_PROTECTED {} limit speed: speed = {:.2f}, "
        "acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        speed_limit_, deaccelerate_);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

void MotorwaySpeedAsternProtcDecider::PrintAllObsInfo() {}

bool MotorwaySpeedAsternProtcDecider::UpdateSequenceData(TaskInfo& task_info) {
  // TODO
  const auto& all_obstacle = task_info.decision_data()->all_obstacle();
  if (all_obstacle.empty()) {
    LOG_INFO("No need handle obstacle!");
    return true;
  }
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data;
  for (const auto& obs : all_obstacle) {
    if (obs == nullptr) {
      continue;
    }
    if (obs->is_virtual()) {
      // LOG_INFO("obs {} is virtual!", obs->id());
      continue;
    }
    if (!(obs->type() == Obstacle::ObstacleType::VEHICLE)) {
      // LOG_INFO("obs {} is not vehicle!", obs->id());
      continue;
    }
    if (obs->center_sl().s() < task_info.curr_sl().s()) {
      // LOG_INFO("obs {} is behind adc!", obs->id());
      continue;
    }
    if (std::fmin(std::fabs(obs->max_l()), std::fabs(obs->min_l())) > 5.5) {
      // LOG_INFO("obs {} is too far away!", obs->id());
      continue;
    }

    LOG_INFO("Update time sequence data, update obs id {}.", obs->id());
    time_sequence_data_.UpdateObsData(*obs);
    has_covered_ele_.push_back(obs->id());
  }

  time_sequence_data_.UpdateGarbage(has_covered_ele_);
  time_sequence_data_.PrintAllData();

  return true;
}

bool MotorwaySpeedAsternProtcDecider::ObstacleValidCheck(const Obstacle& obs) {
  return false;
}

void MotorwaySpeedAsternProtcDecider::Reset() {}

bool MotorwaySpeedAsternProtcDecider::Init(TaskInfo& task_info) {
  time_sequence_data_.UpdateCurrentTimeStamp(cyber::Time::Now().ToSecond());
  const auto& all_obstacle = task_info.decision_data()->all_obstacle();
  if (all_obstacle.empty()) {
    LOG_INFO("No need handle obstacle!");
    return true;
  }
  InitEgoBaseInfo(task_info);

  speed_limit_ = std::numeric_limits<double>::max();
  deaccelerate_ = 0.0;
  update_speed_limit_ = false;
  obs_id_to_heading_diff_map_.clear();
  need_handle_obs_id_vec_.clear();
  obs_id_to_path_heading_map_.clear();

  time_sequence_data_.UpdateObsIdMapUpdateSign();
  UpdateSequenceData(task_info);
  // time_sequence_data_.PrintAllData();
  time_sequence_data_.RemoveUnavailableData();
  // LOG_INFO("collect info :");
  // time_sequence_data_.PrintAllData();

  ExtractNeedHandleObsBaseHistoryInfo(task_info);
  return true;
}

void MotorwaySpeedAsternProtcDecider::InitEgoBaseInfo(TaskInfo& task_info) {
  const auto& adc_sl = task_info.curr_sl();
  const auto& adc_speed =
      task_info.current_frame()->inside_planner_data().vel_v;
  const auto& adc_acc = task_info.current_frame()->inside_planner_data().vel_a;
  const auto& adc_width = VehicleParam::Instance()->width();
  const auto& adc_length = VehicleParam::Instance()->length();
  const auto& adc_heading =
      task_info.current_frame()->inside_planner_data().vel_heading;
  ego_state_.s = adc_sl.s();
  ego_state_.l = adc_sl.l();
  ego_state_.width = adc_width;
  ego_state_.length = adc_length;
  ego_state_.speed = adc_speed;
  ego_state_.heading = adc_heading;  // TODO: modifty different
  ego_state_.speed_heading = adc_heading;
  ego_state_.acc = adc_acc;

  LOG_INFO(
      "init ego state (speed = {:.3f},acc = {:.3f},heading = {:.3f}, "
      "speed_heading = {:.3f})",
      ego_state_.speed, ego_state_.acc, ego_state_.heading,
      ego_state_.speed_heading);
}

void MotorwaySpeedAsternProtcDecider::ExtractNeedHandleObsBaseHistoryInfo(
    TaskInfo& task_info) {
  const auto& all_obstacle = task_info.decision_data()->all_obstacle();
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data;

  const auto& take_follow_decision_map =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context
          .motorway_iter_deduction_take_follow_decision_map;
  const auto& vel_heading =
      task_info.current_frame()->inside_planner_data().vel_heading;
  double ego_path_l{};
  // find for astern
  for (const auto& obs : all_obstacle) {
    if (obs->is_virtual()) {
      // LOG_INFO("obs {} is virtual!", obs->id());
      continue;
    }
    if (obs->speed() > 2.) {
      // LOG_INFO("obs {} speed is too high!", obs->id());
      continue;
    }
    if (obs->min_s() <
        task_info.curr_sl().s() + VehicleParam::Instance()->length() / 2) {
      // LOG_INFO("obs {} is behind adc!", obs->id());
      continue;
    }

    if (!speed_planner_common::ObsEgoAlongPathLateralMinDistance(obs, *path,
                                                                 ego_path_l)) {
      LOG_ERROR(
          "obs {} can't find available min path distance,using ego current l",
          obs->id());
      ego_path_l = ego_state_.l;
    };

    // base heading filter
    double heading_diff{}, path_heading{};
    speed_planner_common::ObsEgoAlongPathHeadingDiff(
        obs, *path, vel_heading, heading_diff, path_heading);

    if ((std::fmin(std::fabs(obs->max_l() - ego_path_l),
                   std::fabs(obs->min_l() - ego_path_l))) -
            ego_state_.width / 2 >
        kLateralAttDis) {
      // LOG_INFO("obs {}, is too far away!", obs->id());
      continue;
    }

    if (time_sequence_data_.FindById(obs->id()) == -1) {
      // LOG_INFO("obs {} is not in time sequence data!", obs->id());
      continue;
    }

    if (take_follow_decision_map.find(obs->id()) !=
            take_follow_decision_map.end() &&
        !take_follow_decision_map.at(obs->id())) {
      // LOG_INFO(
      // "obs {} is in follow decision map and decision is False,don't handle "
      // "!",
      // obs->id());
      continue;
    }

    if (obs->speed() > 0.) {
      if ((obs->center_sl().l() - ego_path_l) *
                  obs_id_to_path_heading_map_[obs->id()] >
              0. &&
          std::fmin(std::fabs(obs->max_l() - ego_path_l),
                    std::fabs(obs->min_l() - ego_path_l)) -
                  ego_state_.width / 2 >
              kRushLateralAttDis) {
        // LOG_INFO("obs id: {}, is far from ego path, don't handle",
        // obs->id());
        continue;
      }
    }

    // 5
    if ((path_heading < 5. / 180 * M_PI && path_heading > -5. / 180 * M_PI) ||
        (path_heading > 175. / 180 * M_PI) ||
        path_heading < -175. / 180 * M_PI) {
      // complate astern, skip
      // LOG_INFO("obs {} complate astern, {:.3f}", obs->id(), path_heading);
      continue;
    }
    if (time_sequence_data_.FindById(obs->id()) != -1 &&
        !time_sequence_data_.IsObsStatic(obs->id()) &&
        time_sequence_data_.IsExtremeHeadingChange(obs->id(), kTurnAround) &&
        time_sequence_data_.IsObsExistSinglePosChange(obs->id(),
                                                      kPositionChange)) {
      LOG_INFO("obs id {} heading change over value", obs->id());
      need_handle_obs_id_vec_.insert(obs->id());
      obs_id_to_heading_diff_map_[obs->id()] = heading_diff;
      obs_id_to_path_heading_map_[obs->id()] = path_heading;
      VisNeedHandleObs(obs, {18, 2, 255, 0.8});
      continue;
    }
  }
  LOG_INFO("need handle obs id num: {}", need_handle_obs_id_vec_.size());
  for (const auto& obs_id : need_handle_obs_id_vec_) {
    LOG_INFO("need handle obs id: {}", obs_id);
  }
  // add
};

void MotorwaySpeedAsternProtcDecider::StaticProtc(TaskInfo& task_info) {
  const auto& all_obstacle = task_info.decision_data()->all_obstacle();
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data;
  const auto& vel_heading =
      task_info.current_frame()->inside_planner_data().vel_heading;
  double ego_path_l{};
  for (const auto& obs : all_obstacle) {
    if (obs->is_virtual()) {
      // LOG_INFO("obs {} is virtual!", obs->id());
      continue;
    }
    if (obs->center_sl().l() < task_info.curr_sl().l()) {
      continue;
    }
    if (obs->min_s() < task_info.curr_sl().s()) {
      // LOG_INFO("obs {} is behind adc!", obs->id());
      continue;
    }
    if (obs->speed() > 1.0) {
      continue;
    }
    if (obs->type() != Obstacle::ObstacleType::VEHICLE) {
      // LOG_INFO("obs {} is not vehicle!", obs->id());
      continue;
    }
    if (need_handle_obs_id_vec_.find(obs->id()) !=
        need_handle_obs_id_vec_.end()) {
      // LOG_INFO("obs id {} is need handle, don't need protected!", obs->id());
      continue;
    }
    if (!speed_planner_common::ObsEgoAlongPathLateralMinDistance(obs, *path,
                                                                 ego_path_l)) {
      LOG_ERROR(
          "obs {} can't find available min path distance,using ego current l",
          obs->id());
      ego_path_l = ego_state_.l;
    };
    double heading_diff{}, path_heading{};
    speed_planner_common::ObsEgoAlongPathHeadingDiff(
        obs, *path, vel_heading, heading_diff, path_heading);
    bool heading_diff_in_range =
        (kStateHeading - M_PI < path_heading &&
         path_heading < -kStateHeading) ||
        (M_PI - kStateHeading > path_heading && path_heading > kStateHeading);
    double diff_lateral = std::fmin(std::fabs(obs->min_l() - ego_path_l),
                                    std::fabs(obs->max_l() - ego_path_l)) -
                          ego_state_.width / 2;
    if (diff_lateral < 0.0) {
      return;
    }
    LOG_INFO(
        "obs id: {}, min_l{:.3f},max_l {:.3f}, ego l {:3f},lateral diff "
        "{:.3f},path_heading {:.3f}",
        obs->id(), obs->min_l(), obs->max_l(), ego_path_l, diff_lateral,
        path_heading / M_PI * 180);
    if (heading_diff_in_range &&
        diff_lateral < kLimitSpeedProtectedLateralRange) {
      if (need_handle_obs_id_vec_.find(obs->id()) !=
          need_handle_obs_id_vec_.end()) {
        continue;
      }
      LOG_INFO("obs id {} lateral distance is to close, need protected!",
               obs->id());
      VisNeedHandleObs(obs, {165, 0, 0, 0.5});
      UpdateProtectedLimitSpeed(
          obs->min_s() - ego_state_.s -
              VehicleParam::Instance()->front_edge_to_center() -
              kLateralBuffer * ego_state_.speed,
          diff_lateral);
    }
  }
}

bool MotorwaySpeedAsternProtcDecider::UpdateProtectedLimitSpeed(
    const double long_distance, const double diff_lateral) {
  update_speed_limit_ = true;
  LOG_INFO("generate protc speed. input :{:.3f}, {:.3f}", long_distance,
           diff_lateral);
  if (long_distance < 0.0) {
    LOG_INFO("distance to obs is close, limit to diff_lateral!", diff_lateral);
    speed_limit_ = kMinSpeed;
    deaccelerate_ = 0.0;
    return true;
  }
  double deacc =
      (std::pow(ego_state_.speed, 2) - std::pow(diff_lateral * kMinSpeed, 2)) /
      2 / long_distance;
  speed_limit_ = std::min(speed_limit_, ego_state_.speed - 0.1 * deacc);
  speed_limit_ = std::max(speed_limit_, kMinSpeed);
  LOG_INFO("accelrate :{:.3f}", deacc);
  if (deacc > -kMinAccelerate) {
    deaccelerate_ = std::min(deaccelerate_, kMinAccelerate);
  } else {
    deaccelerate_ = std::min(deaccelerate_, -deacc);
  }
  LOG_INFO("speed limit {:.3f}", speed_limit_);
  return true;
}

bool MotorwaySpeedAsternProtcDecider::CalSafeStopTime(TaskInfo& task_info) {
  if (!CalEgoStopTime(ego_state_.speed, ego_state_.acc, kComfortableAcc,
                      kDelaytime, safe_stop_time_)) {
    LOG_ERROR("CalEgoStopTime error! adopt default config!");
    safe_stop_time_ = 3.0;
    return false;
  }
  return true;
}

bool MotorwaySpeedAsternProtcDecider::Process(TaskInfo& task_info) {
  // if (speed_planner_common::EgoInIntersection(task_info)) {
  //   whether_turn_scenario_ = true;
  //   if (!speed_planner_common::EgoInTYunproCross(task_info)) {
  //     LOG_INFO(
  //         "ego in intersection ,and not in park cross, don't need handle!");
  //     return true;
  //   }
  // }

  if (speed_planner_common::EgoInIntersection(task_info)) {
    LOG_INFO("ego in intersection, don't need handle!");
    return true;
  }
  if (speed_planner_common::EgoInTNonParkCross(task_info)) {
    LOG_INFO("ego in T park cross, don't need handle!");
    return true;
  }

  StaticProtc(task_info);
  const auto& all_obstacle = task_info.decision_data()->all_obstacle();
  for (auto& obs : all_obstacle) {
    // heading_diff
    if (need_handle_obs_id_vec_.find(obs->id()) ==
        need_handle_obs_id_vec_.end()) {
      continue;
    }

    if (time_sequence_data_.IsTempStatic(obs->id())) {
      // LOG_INFO("obs id {} is static, don't need handle!", obs->id());
      continue;
    }

    LOG_INFO("start generate limit for obs {}", obs->id());
    ego_state_.heading_diff = obs_id_to_heading_diff_map_[obs->id()];
    ego_state_.path_heading = obs_id_to_path_heading_map_[obs->id()];
    // current state don't statify back out solution
    // gap
    ego_state_.miu =
        (obs->width() + obs->length()) / 2 + ego_state_.width / 2;  //
    LOG_INFO("obs_id: {}, miu: {}", obs->id(), ego_state_.miu);
    ActionMap action_map(std::move(RssDistanceUtils{*obs, ego_state_}));

    const auto& ans = action_map.GetSpeedLimit();
    if (ans.second != 0.0) {
      update_speed_limit_ = true;
      deaccelerate_ = std::min(deaccelerate_, ans.second);
      speed_limit_ = ego_state_.speed;
    } else {
      speed_limit_ = std::min(speed_limit_, ans.first);
      update_speed_limit_ = true;
    }

    LOG_INFO(
        "front protect Process:obs_id: {}, speed_limit: {:.3f}, "
        "deaccelerate:{:.3f}",
        obs->id(), speed_limit_, deaccelerate_);
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive

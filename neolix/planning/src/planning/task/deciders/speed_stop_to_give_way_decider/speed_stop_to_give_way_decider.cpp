#include "speed_stop_to_give_way_decider.h"
#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using neodrive::global::prediction::Trajectory_PredictorType_FreeMove;

namespace neodrive {
namespace planning {

namespace {
const auto& speed_stop_to_give_way_decider_config_{
    config::PlanningConfig::Instance()
        ->planning_research_config()
        .speed_stop_to_give_way_decider_config};

}  // namespace

SpeedStopToGiveWayDecider::SpeedStopToGiveWayDecider() {
  name_ = "SpeedStopToGiveWayDecider";
}

SpeedStopToGiveWayDecider::~SpeedStopToGiveWayDecider() { Reset(); }

ErrorCode SpeedStopToGiveWayDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void SpeedStopToGiveWayDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::STOP_TO_GO);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(limited_deceleration_);
    LOG_INFO(
        "stop to give way {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, limited_deceleration_);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    last_limited_speed_ = limited_speed_;
  }
}

// 1st level
bool SpeedStopToGiveWayDecider::Process(TaskInfo& task_info) {
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }

  if (!in_intersection_) {
    return true;
  }

  ObserveLeftNearBigObs(task_info);
  if (adc_will_turn_left_) {
    RecognizeBlockAdcTurnObs(task_info);
  } else {
    RecognizeOvertakeTurnObs(task_info);
  }

  ProcessStopToGORisk(task_info);

  return true;
}

#pragma region
// 2nd level
bool SpeedStopToGiveWayDecider::Init(TaskInfo& task_info) {
  auto& inside_data = task_info.current_frame()->inside_planner_data();

  adc_original_boundary_ = task_info.adc_boundary_origin();
  adc_current_l_ = task_info.curr_sl().l();
  adc_current_s_ = task_info.curr_sl().s();
  adc_back_edge_s_ =
      adc_current_s_ - VehicleParam::Instance()->back_edge_to_center();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_current_heading_ = inside_data.vel_heading;
  adc_current_v_ = inside_data.vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  limited_deceleration_ = 0.0;

  risk_obs_min_size_ =
      std::min(VehicleParam::Instance()->width() - 0.1,
               static_cast<double>(
                   speed_stop_to_give_way_decider_config_.risk_obs_min_size));

  in_intersection_ = IsInIntersection(task_info);
  JudgeIfAdcWillTurnLeft(task_info);
  InitSideBySideCheckArea(task_info.reference_line(), inside_data,
                          task_info.adc_boundary_origin());

  ClearObsHistoryInfo();

  return true;
}

// 2nd level
void SpeedStopToGiveWayDecider::ProcessStopToGORisk(TaskInfo& task_info) {
  if ((adc_current_v_ > 1.5) || risk_obs_info_.empty()) {
    LOG_INFO(
        "not satisfy stop to give way scenario, adc_current_v_ {:.3f} and "
        "risk_obs_info_ size {}.",
        adc_current_v_, risk_obs_info_.size());
    return;
  }

  for (auto& iter : risk_obs_info_) {
    if (!ObsInGiveWayArea(task_info, iter.second.obs_ptr)) {
      LOG_INFO("obs [{}] has risk, but not in give way area!", iter.first);
      continue;
    }

    ++iter.second.cnt;
    if (iter.second.is_static &&
        (iter.second.cnt >
         speed_stop_to_give_way_decider_config_.static_obs_wait_cycle)) {
      LOG_INFO(
          "not wait static obs {} after {:.2f}s.", iter.first,
          0.1 * speed_stop_to_give_way_decider_config_.static_obs_wait_cycle);
      continue;
    }

    update_limited_speed_ = true;
    limited_speed_ = 0.0;
    limited_deceleration_ = -1.0;
    LOG_INFO("stop to wait obs [{}]", iter.first);
  }
}
#pragma endregion

#pragma region
// 3rd level
void SpeedStopToGiveWayDecider::ClearObsHistoryInfo() {
  for (auto& iter : obs_history_info_) {
    iter.second.lost_cnt++;
  }
  auto obs_heading_info_tmp = obs_history_info_;
  for (auto& iter : obs_history_info_) {
    if (iter.second.lost_cnt > 2) {
      obs_heading_info_tmp.erase(iter.first);
    }
  }
  std::swap(obs_history_info_, obs_heading_info_tmp);
  LOG_INFO("obs_history_info_ num {}", obs_history_info_.size());
}

// 3rd
void SpeedStopToGiveWayDecider::ObserveLeftNearBigObs(TaskInfo& task_info) {
  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();
  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }

    if (std::min(dynamic_obs_vec[i]->length(), dynamic_obs_vec[i]->width()) <
        risk_obs_min_size_) {
      continue;
    }

    if (!check_left_overtake_turn_area_.has_overlap(
            dynamic_obs_vec[i]->PolygonBoundary())) {
      continue;
    }

    UpdataDynamicObsInfo(dynamic_obs_vec[i]);
    // UpdataObsLatDisInfo(task_info, dynamic_obs_vec[i]);

    LOG_INFO(
        "dynamic obs [{}] type {} length width: {:.3f}, {:.3f}; obs_speed, "
        "adc_current_v:{:.3f}, {:.3f}; obs heading {:.3f}",
        dynamic_obs_vec[i]->id(),
        static_cast<double>(dynamic_obs_vec[i]->type()),
        dynamic_obs_vec[i]->length(), dynamic_obs_vec[i]->width(),
        dynamic_obs_vec[i]->speed(), adc_current_v_,
        dynamic_obs_vec[i]->heading());
  }

  const auto& static_obstacle_vec = task_info.current_frame()
                                        ->planning_data()
                                        .decision_data()
                                        .static_obstacle();
  for (std::size_t i = 0; i < static_obstacle_vec.size(); ++i) {
    if (static_obstacle_vec[i] == nullptr) {
      continue;
    }

    if (std::min(static_obstacle_vec[i]->length(),
                 static_obstacle_vec[i]->width()) < risk_obs_min_size_) {
      continue;
    }

    if (!check_left_overtake_turn_area_.has_overlap(
            static_obstacle_vec[i]->PolygonBoundary())) {
      continue;
    }

    LOG_INFO(
        "static obs [{}] type {} length width: {:.3f}, {:.3f}; obs_speed, "
        "adc_current_v:{:.3f}, {:.3f}; obs heading {:.3f}",
        static_obstacle_vec[i]->id(),
        static_cast<double>(static_obstacle_vec[i]->type()),
        static_obstacle_vec[i]->length(), static_obstacle_vec[i]->width(),
        static_obstacle_vec[i]->speed(), adc_current_v_,
        static_obstacle_vec[i]->heading());

    UpdateDynamicToStaticObs(static_obstacle_vec[i]);
  }
}

// 3rd level
void SpeedStopToGiveWayDecider::RecognizeBlockAdcTurnObs(TaskInfo& task_info) {
  // TODO(lgf):Recognize obstacles who are at left side and will block ego when
  // ego will turn left.
}

// 3rd level
void SpeedStopToGiveWayDecider::RecognizeOvertakeTurnObs(TaskInfo& task_info) {
  auto risk_obs_info_tmp = risk_obs_info_;
  risk_obs_info_.clear();
  for (const auto& iter : obs_history_info_) {
    if (!HaveFrontAcrossRisk(iter.second)) {
      continue;
    }

    if (iter.second.is_static) {
      // 1.static obstacles: if obs is already risk and static, copy original
      // info; otherwise create new static risk obs.
      if (risk_obs_info_tmp.find(iter.first) != risk_obs_info_tmp.end() &&
          risk_obs_info_tmp[iter.first].is_static) {
        risk_obs_info_[iter.first] = risk_obs_info_tmp[iter.first];
        risk_obs_info_[iter.first].obs_ptr = iter.second.obs_ptr;
      } else {
        risk_obs_info_[iter.first] = RiskObsInfo{iter.second.obs_ptr, true, 0};
      }
    } else {
      // 2.dynamic obstacles: if obs is already risk and dynamic, copy original
      // info; otherwise create new dynamic risk obs.
      if (risk_obs_info_tmp.find(iter.first) != risk_obs_info_tmp.end() &&
          !risk_obs_info_tmp[iter.first].is_static) {
        risk_obs_info_[iter.first] = risk_obs_info_tmp[iter.first];
        risk_obs_info_[iter.first].obs_ptr = iter.second.obs_ptr;
      } else {
        risk_obs_info_[iter.first] = RiskObsInfo{iter.second.obs_ptr, false, 0};
      }
    }
  }
}

// 3rd level
bool SpeedStopToGiveWayDecider::ObsInGiveWayArea(TaskInfo& task_info,
                                                 const Obstacle* obs_ptr) {
  double obs_adc_lat_dis =
      speed_planner_common::GetObs2AdcLateralDis(*obs_ptr, adc_current_l_);
  double obs_path_lat_dis{0.0};
  double lat_dis = speed_planner_common::GetObsToPathLatDis(
                       task_info.current_frame()
                           ->outside_planner_data()
                           .speed_obstacle_context.adc_sl_boundaries,
                       *obs_ptr, obs_path_lat_dis)
                       ? obs_path_lat_dis
                       : obs_adc_lat_dis;
  const auto& obs_boundary = obs_ptr->PolygonBoundary();

  return (lat_dis < speed_stop_to_give_way_decider_config_.give_way_width) &&
         (obs_boundary.end_s() >
          adc_front_edge_s_ -
              speed_stop_to_give_way_decider_config_.give_way_back_buffer) &&
         (obs_boundary.start_s() <
          adc_front_edge_s_ +
              speed_stop_to_give_way_decider_config_.give_way_front_buffer);
}
#pragma endregion

#pragma region
// 4th level
bool SpeedStopToGiveWayDecider::HaveFrontAcrossRisk(
    const ObsHistoryInfo& obs_history_info) {
  // 1. filter obs by s:if end_s of obs less then adc_front_s, not deal.
  if (obs_history_info.obs_ptr->max_s() < adc_front_edge_s_) {
    return false;
  }
  if (obs_history_info.obs_ptr->speed() > 2.5) {
    // perception often error when speed is slow
    return false;
  }

  // 2.risk obs 1:heading_diff(45~135) && history lateral movement && lateral
  // distance less than threshold
  double obs_adc_heading_diff_abs = std::abs(
      normalize_angle(adc_current_heading_ - obs_history_info.last_heading));
  double history_lat_move =
      obs_history_info.lat_dis.front() - obs_history_info.lat_dis.back();
  double lat_dis_threshold = std::max(
      2.0, std::min(4.0, 0.5 * std::max(obs_history_info.obs_ptr->length(),
                                        obs_history_info.obs_ptr->width())));
  LOG_INFO(
      "obs [{}]: obs_adc_heading_diff_abs {:.3f}, accum_heading_diff {:.3f}, "
      "history_lat_move {:.3f}, last_lat_dis {:.3f}, lat_dis_threshold {:.3f}",
      obs_history_info.obs_ptr->id(), obs_adc_heading_diff_abs,
      obs_history_info.accum_heading_diff, history_lat_move,
      obs_history_info.last_lat_dis, lat_dis_threshold);
  if ((obs_adc_heading_diff_abs > M_PI_4) &&
      (obs_adc_heading_diff_abs < M_PI_4 + M_PI_2)) {
    if ((history_lat_move > 0.5) &&
        (obs_history_info.last_lat_dis < lat_dis_threshold)) {
      LOG_INFO("has accross intention !");
      return true;
    }
  } else if (obs_adc_heading_diff_abs < M_PI_4) {
    // risk obs 2: heading_diff < 45 && obs accumulate heading_diff && history
    // lateral movement && lateral distance
    if (((obs_adc_heading_diff_abs > 0.26) ||
         (std::abs(obs_history_info.accum_heading_diff) >
          speed_stop_to_give_way_decider_config_.obs_turning_threshold /
              2.0)) &&
        (history_lat_move > 0.4) &&
        (obs_history_info.last_lat_dis < lat_dis_threshold)) {
      LOG_INFO("has overtake turn intention !");
      return true;
    }
  } else if (obs_adc_heading_diff_abs > M_PI_4 + M_PI_2) {
    if (((obs_adc_heading_diff_abs < M_PI - 0.26) ||
         (std::abs(obs_history_info.accum_heading_diff) >
          speed_stop_to_give_way_decider_config_.obs_turning_threshold /
              2.0)) &&
        (history_lat_move > 0.4) &&
        (obs_history_info.last_lat_dis < lat_dis_threshold)) {
      LOG_INFO("has inverse across intention !");
      return true;
    }
  }

  return false;
}

// 4th level
void SpeedStopToGiveWayDecider::UpdataDynamicObsInfo(Obstacle* obs) {
  if (obs_history_info_.find(obs->id()) == obs_history_info_.end()) {
    auto& obs_history_info = obs_history_info_[obs->id()];
    obs_history_info.obs_ptr = obs;
    obs_history_info.is_static = false;
    obs_history_info.lost_cnt = 0;
    // 1.heading
    obs_history_info.last_heading = normalize_angle(obs->heading());
  } else {
    auto& obs_history_info = obs_history_info_[obs->id()];
    obs_history_info.obs_ptr = obs;
    obs_history_info.is_static = false;
    obs_history_info.lost_cnt = 0;
    // 1.heading
    obs_history_info.heading_diff.push_back(
        normalize_angle(obs->heading() - obs_history_info.last_heading));
    obs_history_info.accum_heading_diff += obs_history_info.heading_diff.back();
    obs_history_info.last_heading = normalize_angle(obs->heading());
    if (obs_history_info.heading_diff.size() >
        speed_stop_to_give_way_decider_config_.history_info_size) {
      obs_history_info.accum_heading_diff -=
          obs_history_info.heading_diff.front();
      obs_history_info.heading_diff.pop_front();
    }
  }

  // 2.lateral distance
  auto& obs_history_info = obs_history_info_[obs->id()];
  double obs_adc_lat_dis = obs->min_l() - adc_original_boundary_.end_l();
  obs_history_info.lat_dis.push_back(obs_adc_lat_dis);
  obs_history_info.last_lat_dis = obs_adc_lat_dis;
  if (obs_history_info.lat_dis.size() >
      speed_stop_to_give_way_decider_config_.history_info_size) {
    obs_history_info.lat_dis.pop_front();
  }
}

// 4th level
void SpeedStopToGiveWayDecider::UpdateDynamicToStaticObs(Obstacle* obs) {
  if (obs_history_info_.find(obs->id()) == obs_history_info_.end()) {
    LOG_INFO("obs [{}] is static all the time, not deal.", obs->id());
    return;
  }

  auto& obs_history_info = obs_history_info_[obs->id()];
  obs_history_info.obs_ptr = obs;
  obs_history_info.is_static = true;
  obs_history_info.lost_cnt = 0;
  // 1.heading
  double obs_heading_diff =
      normalize_angle(obs->heading() - obs_history_info.last_heading);
  if (std::abs(obs_heading_diff) > 0.05) {
    obs_history_info.heading_diff.push_back(obs_heading_diff);
    obs_history_info.accum_heading_diff += obs_history_info.heading_diff.back();
    if (obs_history_info.heading_diff.size() >
        speed_stop_to_give_way_decider_config_.history_info_size) {
      obs_history_info.accum_heading_diff -=
          obs_history_info.heading_diff.front();
      obs_history_info.heading_diff.pop_front();
    }
    obs_history_info.last_heading = normalize_angle(obs->heading());
  }
  // 2.lateral distance
  double obs_adc_lat_dis = obs->min_l() - adc_original_boundary_.end_l();
  if (std::abs(obs_adc_lat_dis) > 0.02) {
    obs_history_info.lat_dis.push_back(obs_adc_lat_dis);
    obs_history_info.last_lat_dis = obs_adc_lat_dis;
    if (obs_history_info.lat_dis.size() >
        speed_stop_to_give_way_decider_config_.history_info_size) {
      obs_history_info.lat_dis.pop_front();
    }
  }
  LOG_INFO("obs [{}] change from dynamic to static, heading {:.4f}", obs->id(),
           obs->heading());
}

void SpeedStopToGiveWayDecider::SearchRiskArea(TaskInfo& task_info) {
  risk_area_info_.Reset();

  auto ref_line = task_info.reference_line();
  const auto& junction_list = ref_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }

    if (adc_current_s_ > risk_area_info_.risk_area_end_s) {
      double risk_area_predeal_s =
          overlap.start_s - VehicleParam::Instance()->length();
      double risk_area_start_s = overlap.start_s;
      if (risk_area_predeal_s <= adc_current_s_ &&
          overlap.end_s >= adc_current_s_) {
        risk_area_info_.risk_area_predeal_s = risk_area_predeal_s;
        risk_area_info_.risk_area_start_s = risk_area_start_s;
        risk_area_info_.risk_area_end_s = overlap.end_s;
        risk_area_info_.junction_end_s = overlap.end_s;
        LOG_INFO("adc in junction: {:.3f},  {:.3f}, {:.3f}",
                 risk_area_predeal_s, overlap.start_s, overlap.end_s);
        if (junction_ptr->Type() !=
            static_cast<uint32_t>(JunctionType::IN_ROAD)) {
          LOG_INFO("adc is in right turn risk area: {:.3f},  {:.3f}, {:.3f}",
                   risk_area_info_.risk_area_predeal_s,
                   risk_area_info_.risk_area_start_s,
                   risk_area_info_.risk_area_end_s);
          return;
        }
      }
    } else {
      if ((overlap.start_s > risk_area_info_.junction_end_s + 70) ||
          (junction_ptr->Type() ==
           static_cast<uint32_t>(JunctionType::IN_ROAD))) {
        risk_area_info_.Reset();
        LOG_INFO(
            "Last junction_ens_s, next overlap start_s, type:{:.3f}, {:.3f}, "
            "{};  adjacent two junctions are all IN_ROAD, not risk area.",
            risk_area_info_.junction_end_s, overlap.start_s,
            junction_ptr->Type());
        return;
      }

      LOG_INFO(
          "next junction is not IN_ROAD,adc is in right turn risk area: "
          "{:.3f},  {:.3f}, {:.3f}",
          risk_area_info_.risk_area_predeal_s,
          risk_area_info_.risk_area_start_s, risk_area_info_.risk_area_end_s);
      return;
    }
  }

  LOG_INFO("Not find risk area.");
  risk_area_info_.Reset();
}

bool SpeedStopToGiveWayDecider::IsInIntersection(TaskInfo& task_info) {
  SearchRiskArea(task_info);
  return (risk_area_info_.risk_area_predeal_s <= adc_current_s_ &&
          risk_area_info_.risk_area_end_s >= adc_current_s_);
}

void SpeedStopToGiveWayDecider::JudgeIfAdcWillTurnLeft(TaskInfo& task_info) {
  InitTurnLeftArea(task_info);
  adc_will_turn_left_ = (adc_current_s_ > left_turn_info_.left_turn_start_s) &&
                        (adc_current_s_ < left_turn_info_.left_turn_end_s);
  if (adc_will_turn_left_) {
    LOG_INFO("checked adc will turn left.");
  }
}

void SpeedStopToGiveWayDecider::InitTurnLeftArea(TaskInfo& task_info) {
  if (adc_current_s_ < left_turn_info_.next_check_s) {
    return;
  }

  const auto& ref_points = task_info.reference_line()->ref_points();
  int left_turn_cnt = 0;
  for (int i = 0; i < ref_points.size() - 2; i += 2) {
    if (ref_points[i].s() < adc_current_s_) {
      continue;
    }
    if (ref_points[i].s() > adc_current_s_ + 60) {
      break;
    }

    // kappa is positive when turn left
    if (ref_points[i].kappa() > config::PlanningConfig::Instance()
                                    ->plan_config()
                                    .speed_plan.turn_left_min_kappa) {
      if (0 == left_turn_cnt) {
        left_turn_info_.left_turn_start_s = ref_points[i].s() - 5.0;
      }
      left_turn_cnt++;
    } else {
      if (left_turn_cnt > 5) {
        left_turn_info_.left_turn_end_s = ref_points[i].s() + 2.0;
        left_turn_info_.next_check_s = left_turn_info_.left_turn_end_s;
        LOG_INFO(
            "Find new left turn area: left_turn_start_s {:.3f}, "
            "left_turn_end_s {:.3f}, next_check_s {:.3f}",
            left_turn_info_.left_turn_start_s, left_turn_info_.left_turn_end_s,
            left_turn_info_.next_check_s);
        return;
      }
      left_turn_cnt = 0;
    }
  }

  left_turn_info_.Reset();
  left_turn_info_.next_check_s = adc_current_s_ + 30;
  return;
}

bool SpeedStopToGiveWayDecider::InitSideBySideCheckArea(
    const ReferenceLinePtr& ref_ptr, const InsidePlannerData& inside_data,
    const Boundary& adc_boundary) {
  check_left_overtake_turn_area_ = std::move(Boundary{
      adc_boundary.start_s() -
          speed_stop_to_give_way_decider_config_.check_area_back_buffer,
      adc_boundary.end_s() +
          speed_stop_to_give_way_decider_config_.check_area_front_buffer,
      adc_boundary.end_l(),
      adc_boundary.end_l() +
          speed_stop_to_give_way_decider_config_.check_area_width});

  LOG_INFO(
      "check_left_overtake_turn_area_, s_s, e_s, s_l, e_l: {:.2f}, "
      "{:.2f}, {:.2f}, {:.2f} ",
      check_left_overtake_turn_area_.start_s(),
      check_left_overtake_turn_area_.end_s(),
      check_left_overtake_turn_area_.start_l(),
      check_left_overtake_turn_area_.end_l());

  return true;
}
#pragma endregion

}  // namespace planning
}  // namespace neodrive

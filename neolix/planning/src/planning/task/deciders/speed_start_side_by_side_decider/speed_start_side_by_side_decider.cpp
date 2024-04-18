#include "speed_start_side_by_side_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using neodrive::global::prediction::Trajectory_PredictorType_FreeMove;

namespace neodrive {
namespace planning {

namespace {
const auto& speed_start_side_by_side_decider_config_{
    config::PlanningConfig::Instance()
        ->planning_research_config()
        .speed_start_side_by_side_decider_config};

}  // namespace

SpeedStartSideBySideDecider::SpeedStartSideBySideDecider() {
  name_ = "SpeedStartSideBySideDecider";
}

SpeedStartSideBySideDecider::~SpeedStartSideBySideDecider() { Reset(); }

ErrorCode SpeedStartSideBySideDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  if (!DataCheck(task_info)) {
    LOG_ERROR("DataCheck failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

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

void SpeedStartSideBySideDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::STOP_TO_GO);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(limited_deceleration_);
    LOG_INFO(
        "STOP_TO_GO {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, limited_deceleration_);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    last_limited_speed_ = limited_speed_;
  }
}

bool SpeedStartSideBySideDecider::Process(TaskInfo& task_info) {
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }

  if (!in_intersection_) {
    return true;
  }

  PrepareData(task_info);

  if (is_red_to_green_ && (adc_current_v_ < 1.0) &&
      (stop_to_go_info_.size() > 0)) {
    LOG_INFO(
        "red light turn green, adc_current_v_ {:.3f} and stop_to_go_info_ {} "
        "satisfy stop to go scenario.",
        adc_current_v_, stop_to_go_info_.size());
    ClearHistoryCnt();
    stop_to_go_flag_ = true;
  }

  if (stop_to_go_flag_) {
    ProcessStopToGORisk();
  }

  return true;
}

bool SpeedStartSideBySideDecider::DataCheck(TaskInfo& task_info) {
  if (task_info.current_frame() == nullptr) {
    LOG_ERROR("current_frame is nullptr.");
    return false;
  }

  if (task_info.last_frame() == nullptr) {
    LOG_ERROR("last_frame is nullptr.");
    return false;
  }
  if (task_info.current_frame()->mutable_outside_planner_data() == nullptr) {
    LOG_ERROR("mutable_outside_planner_data() is nullptr.");
    return false;
  }

  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dynamic_obstacles_decision;

  for (const auto& obs_decision : dynamic_obstacles_decision) {
    if (obs_decision.lower_points.empty() ||
        obs_decision.upper_points.empty() ||
        obs_decision.lower_points_heading_diff.empty() ||
        obs_decision.upper_points_heading_diff.empty()) {
      LOG_ERROR(
          "obs_decision.lower_points or obs_decision.upper_points is "
          "empty.");
      return false;
    }
  }
  return true;
}

// Clear history count caused by SD
void SpeedStartSideBySideDecider::ClearHistoryCnt() {
  for (auto& iter : stop_to_go_info_) {
    if (iter.second.is_static) {
      iter.second.cnt = 0;
    }
  }
}

void SpeedStartSideBySideDecider::ProcessStopToGORisk() {
  for (auto& iter : stop_to_go_info_) {
    if (iter.second.is_static) {
      if (++iter.second.cnt >
          speed_start_side_by_side_decider_config_.static_obs_wait_cycle) {
        LOG_INFO("not wait static obs {} after 3s.", iter.first);
        continue;
      }

    } else if (iter.second.obs_ptr->speed() > adc_current_v_) {
      double overtake_ttc = (adc_front_edge_s_ -
                             iter.second.obs_ptr->PolygonBoundary().start_s()) /
                            (iter.second.obs_ptr->speed() - adc_current_v_);
      // TODO(lgf):add other logic after qingpeng fix computing err of frenet
      // coordinate.
      if (overtake_ttc < 0.5) {
        LOG_INFO(
            "debug dynamic obs {}, overtake_ttc {:.3f}; adc_front_edge_s_ "
            "{:.3f}, obs start_s {:.3f}, obs speed {:.3f}, adc_current_v_ "
            "{:.3f}",
            iter.first, overtake_ttc, adc_front_edge_s_,
            iter.second.obs_ptr->PolygonBoundary().start_s(),
            iter.second.obs_ptr->speed(), adc_current_v_);
        //   LOG_INFO("not wait dynamic obs {}, because ttc {:.3f} is safe.",
        //            iter.first, overtake_ttc);
        //   continue;
      }
    }

    update_limited_speed_ = true;
    limited_speed_ = 0.0;
    limited_deceleration_ = -1.0;
    LOG_INFO("stop to wait obs [{}]", iter.first);
  }

  if (!update_limited_speed_) {
    LOG_INFO("clear stop_to_go_flag_.");
    stop_to_go_flag_ = false;
  }
}

void SpeedStartSideBySideDecider::PrepareData(TaskInfo& task_info) {
  const auto& outside_data = task_info.current_frame()->outside_planner_data();
  PrepareTrafficLightData(
      outside_data.speed_obstacle_context.virtual_obstacle_decision);
  PrepareRiskObsData(task_info);
}

void SpeedStartSideBySideDecider::PrepareRiskObsData(TaskInfo& task_info) {
  auto stop_to_go_info_tmp = stop_to_go_info_;
  stop_to_go_info_.clear();

  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();
  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }

    if (!check_side_by_side_area_.has_overlap(
            dynamic_obs_vec[i]->PolygonBoundary())) {
      if (stop_to_go_info_tmp.find(dynamic_obs_vec[i]->id()) !=
          stop_to_go_info_tmp.end()) {
        const auto& obs_boundary = dynamic_obs_vec[i]->PolygonBoundary();
        LOG_INFO(
            "debug dynamic obs [{}] localization change, start_s, end_s, "
            "start_l, "
            "end_l: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
            obs_boundary.start_s(), obs_boundary.end_s(),
            obs_boundary.start_l(), obs_boundary.end_l());
      }
      continue;
    }

    double heading_diff = std::abs(normalize_angle(
        dynamic_obs_vec[i]->velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading));
    if (std::min(dynamic_obs_vec[i]->length(), dynamic_obs_vec[i]->width()) <
        risk_obs_judge_size_) {
      continue;
    }

    if (heading_diff > M_PI_2) {
      continue;
    }

    LOG_INFO(
        "dynamic obs [{}] type {} length width: {:.3f}, {:.3f}; obs_speed, "
        "adc_current_v:{:.3f}, {:.3f}; heading_diff {:.3f}",
        dynamic_obs_vec[i]->id(),
        static_cast<double>(dynamic_obs_vec[i]->type()),
        dynamic_obs_vec[i]->length(), dynamic_obs_vec[i]->width(),
        dynamic_obs_vec[i]->speed(), adc_current_v_, heading_diff);
    if ((stop_to_go_info_tmp.find(dynamic_obs_vec[i]->id()) !=
         stop_to_go_info_tmp.end()) &&
        !stop_to_go_info_tmp[dynamic_obs_vec[i]->id()].is_static) {
      stop_to_go_info_[dynamic_obs_vec[i]->id()] =
          stop_to_go_info_tmp[dynamic_obs_vec[i]->id()];
    } else {
      stop_to_go_info_[dynamic_obs_vec[i]->id()] =
          RiskObsInfo{dynamic_obs_vec[i], false, 0};
    }
  }

  const auto& static_obstacle_vec = task_info.current_frame()
                                        ->planning_data()
                                        .decision_data()
                                        .static_obstacle();
  for (std::size_t i = 0; i < static_obstacle_vec.size(); ++i) {
    if (static_obstacle_vec[i] == nullptr) {
      continue;
    }

    if (!check_side_by_side_area_.has_overlap(
            static_obstacle_vec[i]->PolygonBoundary())) {
      // coordinate
      if (stop_to_go_info_tmp.find(static_obstacle_vec[i]->id()) !=
          stop_to_go_info_tmp.end()) {
        const auto& obs_boundary = static_obstacle_vec[i]->PolygonBoundary();
        LOG_INFO(
            "debug static obs [{}] localization change, start_s, end_s, "
            "start_l, end_l: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
            obs_boundary.start_s(), obs_boundary.end_s(),
            obs_boundary.start_l(), obs_boundary.end_l());
      }
      continue;
    }

    if (std::min(static_obstacle_vec[i]->length(),
                 static_obstacle_vec[i]->width()) < risk_obs_judge_size_) {
      continue;
    }

    if (static_obstacle_vec[i]->PolygonBoundary().end_s() < adc_current_s_) {
      continue;
    }

    LOG_INFO(
        "static obs [{}] type {} length width: {:.3f}, {:.3f}; obs_speed, "
        "adc_current_v:{:.3f}, {:.3f}",
        static_obstacle_vec[i]->id(),
        static_cast<double>(static_obstacle_vec[i]->type()),
        static_obstacle_vec[i]->length(), static_obstacle_vec[i]->width(),
        static_obstacle_vec[i]->speed(), adc_current_v_);
    if (stop_to_go_info_tmp.find(static_obstacle_vec[i]->id()) !=
            stop_to_go_info_tmp.end() &&
        stop_to_go_info_tmp[static_obstacle_vec[i]->id()].is_static) {
      stop_to_go_info_[static_obstacle_vec[i]->id()] =
          stop_to_go_info_tmp[static_obstacle_vec[i]->id()];
    } else {
      stop_to_go_info_[static_obstacle_vec[i]->id()] =
          RiskObsInfo{static_obstacle_vec[i], true, 0};
    }
  }
  LOG_INFO("risk obs num is {}", stop_to_go_info_.size());
}

void SpeedStartSideBySideDecider::PrepareTrafficLightData(
    const std::vector<SpeedObstacleDecision>& virtual_obstacle_decision) {
  for (const auto& iter : virtual_obstacle_decision) {
    if (iter.obstacle.virtual_type() == VirtualObstacle::TRAFFIC_LIGHT) {
      if (iter.lower_points.empty()) {
        continue;
      }
      current_red_light_[iter.obstacle.id()] =
          TraficLightInfo{iter.lower_points.front().first.s(),
                          iter.lower_points.front().first.t()};
      LOG_INFO("obs [{}] is red light at (s,t) ({:.3f}, {:.3f})",
               iter.obstacle.id(),
               current_red_light_[iter.obstacle.id()].lower_s,
               current_red_light_[iter.obstacle.id()].lower_t);
    }
  }

  LOG_INFO("last_red_light_ size {} , current_red_light_ size {}",
           last_red_light_.size(), current_red_light_.size());
  for (const auto& iter : last_red_light_) {
    if (current_red_light_.find(iter.first) == current_red_light_.end()) {
      is_red_to_green_ = true;
      break;
    }
  }

  last_red_light_ = current_red_light_;
  current_red_light_.clear();
}

bool SpeedStartSideBySideDecider::Init(TaskInfo& task_info) {
  auto& inside_data = task_info.current_frame()->inside_planner_data();

  adc_current_s_ = task_info.curr_sl().s();
  adc_back_edge_s_ =
      adc_current_s_ - VehicleParam::Instance()->back_edge_to_center();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  limited_deceleration_ = 0.0;

  is_red_to_green_ = false;
  risk_obs_judge_size_ = std::min(
      VehicleParam::Instance()->width() - 0.1,
      static_cast<double>(
          speed_start_side_by_side_decider_config_.risk_obs_judge_size));

  in_intersection_ = IsInIntersection(task_info);
  InitSideBySideCheckArea(task_info.reference_line(), inside_data,
                          task_info.adc_boundary_origin());
  return true;
}

bool SpeedStartSideBySideDecider::IsInIntersection(TaskInfo& task_info) {
  auto ref_line = task_info.reference_line();
  const auto& junction_list = ref_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }

    double deal_area_start_s =
        overlap.start_s - VehicleParam::Instance()->length();
    double deal_area_end_s = overlap.end_s;
    if (deal_area_start_s <= adc_current_s_ &&
        deal_area_end_s >= adc_current_s_ &&
        (junction_ptr->Type() !=
         static_cast<uint32_t>(JunctionType::IN_ROAD))) {
      return true;
    } else if (deal_area_start_s > adc_current_s_) {
      break;
    }
  }
  return false;
}

bool SpeedStartSideBySideDecider::InitSideBySideCheckArea(
    const ReferenceLinePtr& ref_ptr, const InsidePlannerData& inside_data,
    const Boundary& adc_boundary) {
  check_side_by_side_area_ = std::move(Boundary{
      adc_boundary.start_s() -
          speed_start_side_by_side_decider_config_.check_area_back_buffer,
      adc_boundary.end_s() +
          speed_start_side_by_side_decider_config_.check_area_front_buffer,
      adc_boundary.end_l(),
      adc_boundary.end_l() +
          speed_start_side_by_side_decider_config_.check_area_width});

  LOG_INFO(
      "check_side_by_side_area_, s_s, e_s, s_l, e_l: {:.2f}, "
      "{:.2f}, {:.2f}, {:.2f} ",
      check_side_by_side_area_.start_s(), check_side_by_side_area_.end_s(),
      check_side_by_side_area_.start_l(), check_side_by_side_area_.end_l());

  return true;
}

}  // namespace planning
}  // namespace neodrive

#include "speed_vehicle_meeting_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double k_attention_dis = 15.0;
constexpr double k_time_buffer = 1;
}  // namespace

SpeedVehicleMeetingDecider::SpeedVehicleMeetingDecider() {
  name_ = "SpeedVehicleMeetingDecider";
}

SpeedVehicleMeetingDecider::~SpeedVehicleMeetingDecider() { Reset(); }

ErrorCode SpeedVehicleMeetingDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (!Init(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

void SpeedVehicleMeetingDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_speed_limit_) {
    accelerate_ =
        std::max(-3.0, std::min(0.0, (speed_limit_ - adc_current_v_) * 10.0));
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::VEHICLE_MEETING);
    internal_speed_limit.add_upper_bounds(speed_limit_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(accelerate_);
    LOG_INFO(
        "VEHICLE_MEETING {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        speed_limit_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

bool SpeedVehicleMeetingDecider::Init(TaskInfo& task_info) {
  if (!update_speed_limit_) {
    last_limited_speed_ =
        task_info.current_frame()->inside_planner_data().vel_v;
  }
  update_speed_limit_ = false;
  speed_limit_ = std::numeric_limits<double>::infinity();
  accelerate_ = 0.0;

  adc_current_s_ = task_info.curr_sl().s();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;

  auto reference_line = task_info.reference_line();
  // get front lane meeting infos
  double vehicle_meeting_find_dis = std::max(40.0, adc_current_v_ * 5.0);
  lane_id_s_.clear();
  for (const auto& point : reference_line->ref_points()) {
    if (point.s() < adc_current_s_) continue;
    if (point.s() > adc_current_s_ + vehicle_meeting_find_dis) break;
    if (point.lane_meeting_ranges()) {
      // update need conitnue
      // store each lane id overlap  min_s ,max_s
      auto meetingss = point.lane_meeting_ranges()->meetings;
      for (const auto& meetings : meetingss) {
        for (const auto& lane_segments : meetings.prevs) {
          for (const auto& lane_segment : lane_segments) {
            auto iter = lane_id_s_.find(lane_segment.id);
            if (iter != lane_id_s_.end()) {
              iter->second.push_back(point);
            } else {
              lane_id_s_.insert(std::make_pair(lane_segment.id,
                                               std::vector<ReferencePoint>()));
            }
          }
        }
      }
    }
  }
  LOG_INFO("lane_id_map_s size: {}", lane_id_s_.size());
  return true;
}

bool SpeedVehicleMeetingDecider::Process(TaskInfo& task_info) {
  if (lane_id_s_.empty()) {
    LOG_INFO("lane_meeting_ranges empty, skip.");
    return true;
  }
  // get obstacle id dealt in collision check
  std::unordered_set<std::size_t> obs_dealt_set{};
  for (const auto& obs_decision :
       task_info.current_frame()
           ->outside_planner_data()
           .speed_obstacle_context.dynamic_obstacles_decision) {
    obs_dealt_set.insert(obs_decision.obstacle.id());
  }

  auto adc_boundary = task_info.adc_boundary();
  auto dynamic_obstacles = task_info.decision_data()->dynamic_obstacle();
  attention_ids_.clear();
  attention_project_v_.clear();
  attention_start_s_.clear();
  thw_info_.clear();
  attention_obs_overlap_in_time_.clear();
  attention_obs_overlap_out_time_.clear();
  ego_overlap_start_s_in_time_.clear();
  ego_overlap_start_s_out_time_.clear();

  for (const auto& obstacle : dynamic_obstacles) {
    if (obstacle->is_virtual() || obstacle->is_static()) {
      continue;
    }
    if (obs_dealt_set.find(obstacle->id()) != obs_dealt_set.end()) {
      continue;
    }
    if (obstacle->speed() < 0.5) continue;
    auto iter = lane_id_s_.find(obstacle->matched_lane_id());
    if (iter == lane_id_s_.end()) {
      continue;
    }
    // judge weather have collision info
    if (!JudgeHasCollisionWithEGO(task_info, obstacle)) {
      continue;
    };
    // obs is forward driving and behind adc_s + dis_to_junction
    double end_s = obstacle->PolygonBoundary().end_s();
    double start_s = obstacle->PolygonBoundary().start_s();
    double heading_diff = normalize_angle(
        obstacle->velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading);
    if (std::fabs(heading_diff) > M_PI_2) {
      continue;
    }
    if (start_s > adc_boundary.end_s() + k_attention_dis) continue;

    ReferencePoint start_pt{}, end_pt{};

    for (auto& pt : iter->second) {
      if (start_pt.s() == 0.0) start_pt = pt;
      if (end_pt.s() != 0.0 && pt.s() - end_pt.s() > 3) {
        break;
      }
      end_pt = pt;
    }
    if (end_pt.s() - start_pt.s() < 2) continue;
    if (start_pt.s() < start_s) continue;
    double obs_near_time =
        (obstacle->center().distance_to(start_pt) - obstacle->length() / 2) /
        obstacle->speed();
    double obs_far_time = (obstacle->center().distance_to(end_pt) * 1.1 +
                           obstacle->length() / 2) /
                          obstacle->speed();
    double ego_far_time =
        (end_pt.s() - adc_boundary.start_s()) /
        task_info.current_frame()->inside_planner_data().vel_v;
    double ego_near_time =
        (start_pt.s() - adc_boundary.end_s()) /
        task_info.current_frame()->inside_planner_data().vel_v;

    if (obs_near_time > 20) continue;
    LOG_INFO("overlap start s {}, overlap end s{},", start_pt.s(), end_pt.s());
    LOG_INFO(
        "obs id: {}, ego_near_start_s_time: {:.3f},obs_near_start_s_time: "
        "{:.3f},ego_far_end_s_time: {:.3f},obs_far_end_s_time: {:.3f}",
        obstacle->id(), ego_near_time, obs_near_time, ego_far_time,
        obs_far_time);
    if (ego_near_time > obs_far_time + k_time_buffer) continue;
    if (ego_far_time < obs_near_time - k_time_buffer) continue;

    attention_obs_overlap_in_time_.emplace_back(obs_near_time);
    attention_obs_overlap_out_time_.emplace_back(obs_far_time);
    ego_overlap_start_s_in_time_.emplace_back(ego_near_time);
    ego_overlap_start_s_out_time_.emplace_back(ego_far_time);

    double project_vel = obstacle->speed() * std::cos(heading_diff);
    attention_ids_.emplace_back(obstacle->id());
    thw_info_.emplace_back(
        (start_s >= adc_boundary.start_s())
            ? ((start_s - adc_boundary.start_s()) /
               std::fmax(
                   0.1, task_info.current_frame()->inside_planner_data().vel_v))
            : ((adc_boundary.start_s() - start_s) /
               std::fmax(0.1, project_vel)));
    attention_project_v_.emplace_back(project_vel);
    attention_start_s_.emplace_back(start_s);
  }
  if (thw_info_.empty()) {
    LOG_INFO("matched obstacle empty, skip.");
    return true;
  }
  std::size_t min_thw_index{0};
  std::size_t min_int_index{0};
  double min_thw{thw_info_.front()};
  double min_int{attention_obs_overlap_in_time_.front()};
  for (std::size_t index = 1; index < thw_info_.size(); ++index) {
    if (min_thw > thw_info_[index]) {
      min_thw = thw_info_[index];
      min_thw_index = index;
    }

    if (min_int > attention_obs_overlap_in_time_[index]) {
      min_int = attention_obs_overlap_in_time_[index];
      min_int_index = index;
    }
  }

  LOG_INFO("min_thw: {:.3f}", min_thw);
  if (min_thw > 3.0) {
    LOG_INFO("min_thw > 3.0s, skip.");
    return true;
  }

  if (min_int > 5) {
    LOG_INFO("min_int > 5s, skip.");
    return true;
  }

  double alpha = 1.0;
  if (attention_project_v_[min_int_index] > adc_current_v_) {
    LOG_INFO("Closest obs project speed > ego speed, skip!");
    return true;
  }
  if (attention_project_v_[min_int_index] < 0.0) {
    attention_project_v_[min_int_index] = 0.0;
    alpha = 2.0;
  }

  accelerate_ = -(adc_current_v_ - attention_project_v_[min_int_index]) /
                ego_overlap_start_s_in_time_[min_int_index] * alpha;
  accelerate_ = accelerate_ < 0.0 ? accelerate_ : 0.0;
  double new_speed_limit = std::fmax(0.0, adc_current_v_ + accelerate_ * 0.1);
  LOG_INFO("update! curr_v :{:.3f}, new_speed_limit: {:.3f},accelerate:{:.3f}",
           adc_current_v_, new_speed_limit, accelerate_);

  // speed limit
  update_speed_limit_ = true;
  speed_limit_ = std::fmin(speed_limit_, new_speed_limit);
  LOG_INFO("speed_limit is {:.3f}", speed_limit_);
  return true;
}

bool SpeedVehicleMeetingDecider::JudgeHasCollisionWithEGO(
    TaskInfo& task_info, const Obstacle* const obs) {
  const auto adc_xy = task_info.adc_point();
  const double obs_tan = std::tan(obs->velocity_heading());
  const auto& path_point = task_info.current_frame()
                               ->outside_planner_data()
                               .path_data->path()
                               .path_points();
  if (path_point.empty()) return false;
  const auto& path_tail_point = path_point.back();
  if (path_tail_point.x() == adc_xy.x() && path_tail_point.y() == adc_xy.y()) {
    LOG_INFO("adc path only one point or circle! no ignore.");
    return false;
  }
  auto cal_cross_point = [](const double x, const double y, const double k,
                            const double x1, double y1, const double k1,
                            bool& flag) -> std::pair<double, double> {
    if (k == k1) {
      flag = false;
      return std::make_pair(0, 0);
    }
    double x0 = (y1 - y - k1 * x1 + k * x) / (k - k1);
    double y0 = k * (x0 - x) + y;
    return std::make_pair(x0, y0);
  };
  bool flag = true;
  double ego_tan = path_tail_point.x() - adc_xy.x() == 0
                       ? std::numeric_limits<double>::infinity()
                       : (path_tail_point.y() - adc_xy.y()) /
                             (path_tail_point.x() - adc_xy.x());

  auto cross_point =
      cal_cross_point(obs->center().x(), obs->center().y(), obs_tan, adc_xy.x(),
                      adc_xy.y(), ego_tan, flag);
  if (!flag) return false;  // error!
  // cross judge
  if (std::cos(obs->velocity_heading()) *
          (obs->center().x() - cross_point.first) >
      0) {
    LOG_INFO("{} no cross point, ignore", obs->id());
    return false;
  }
  if ((cross_point.first - path_tail_point.x()) *
          (adc_xy.x() - cross_point.first) <
      0) {
    LOG_INFO("obs {} hasn't overlap with ego, ignore!", obs->id());
    return false;
  }

  if (std::pow(cross_point.first - adc_xy.x(), 2) +
          std::pow(cross_point.second - adc_xy.y(), 2) >
      225) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive

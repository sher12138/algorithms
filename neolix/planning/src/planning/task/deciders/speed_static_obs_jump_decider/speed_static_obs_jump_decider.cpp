#include "speed_static_obs_jump_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_limit_trans.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

SpeedStaticObsJumpDecider::SpeedStaticObsJumpDecider() {
  name_ = "SpeedStaticObsJumpDecider";
}

SpeedStaticObsJumpDecider::~SpeedStaticObsJumpDecider() { Reset(); }

ErrorCode SpeedStaticObsJumpDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  if (!DataCheck(task_info)) {
    LOG_ERROR("DataCheck failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_SKIP;
  }
  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

void SpeedStaticObsJumpDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::STATIC_OBS_JUMP);
    internal_speed_limit.add_upper_bounds(speed_limit_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "STATIC_OBS_JUMP {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        speed_limit_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

void SpeedStaticObsJumpDecider::Reset(){};

bool SpeedStaticObsJumpDecider::Init(TaskInfo& task_info) {
  speed_static_obs_jump_config_ =
      &config::PlanningConfig::Instance()->plan_config().speed_static_obs_jump;
  if (!speed_static_obs_jump_config_) {
    return false;
  }

  update_limited_speed_ = false;
  max_speed_ = DataCenter::Instance()->drive_strategy_max_speed();
  speed_limit_ = max_speed_ + 1.0;
  collide_infos_.clear();

  const auto& init_point =
      task_info.current_frame()->inside_planner_data().init_point;
  adc_current_v_ = init_point.velocity();
  init_state_ = {0., adc_current_v_, 0.};
  check_dis_ = std::max(5.0, adc_current_v_ * 8.0);
  adc_boundary_ = task_info.adc_boundary_origin();

  ClearObsHistoryInfo();
  UpdataObsHistoryInfo(task_info);

  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  double en_large_buffer{FLAGS_planning_speed_plan_enlarge_self_buffer};
  double front_en_large_buffer{en_large_buffer};
  double back_en_large_buffer{en_large_buffer};
  if (!speed_planner_common::GetAdcEnlargeBuffer(
          inside_data, inside_data.curr_multi_level, &en_large_buffer,
          &front_en_large_buffer, &back_en_large_buffer)) {
    LOG_ERROR("get adc self en_large buffer failed.");
  }
  multi_level_enlarge_buffer_ =
      std::max(front_en_large_buffer, back_en_large_buffer);
  LOG_INFO(
      "current_level, front_buffer, back_buffer, multi_level_buffer: {}, "
      "{:.3f}, {:.3f}, {:.3f}",
      inside_data.curr_multi_level, front_en_large_buffer, back_en_large_buffer,
      multi_level_enlarge_buffer_);

  return true;
}

bool SpeedStaticObsJumpDecider::Process(TaskInfo& task_info) {
  ComputeCollideInfo(task_info);

  auto& dp_st_map_ignore_static_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dp_st_map_ignore_static_obs_id;
  for (const auto& [id, collide_info] : collide_infos_) {
    if (collide_info.path_lat_dis >
        speed_static_obs_jump_config_->adc_enlarge_buffer) {
      // Speed Limit
      speed_limit_ =
          std::min(speed_limit_, ComputeSpeedLimit(id, collide_info));
      LOG_INFO("ignore static obs [{}] speed_limit_ {:.3f}", id, speed_limit_);
      // Ignore
      dp_st_map_ignore_static_obs_id.insert(id);
    }
  }
  if (speed_limit_ < max_speed_) {
    update_limited_speed_ = true;
  }

  return true;
}

bool SpeedStaticObsJumpDecider::DataCheck(TaskInfo& task_info) {
  if (task_info.last_frame() == nullptr) {
    LOG_ERROR("last_frame is nullptr.");
    return false;
  }

  if (task_info.current_frame() == nullptr) {
    LOG_ERROR("current_frame is nullptr.");
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

void SpeedStaticObsJumpDecider::ComputeCollideInfo(TaskInfo& task_info) {
  const auto& static_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.static_obstacles_decision;
  if (static_obstacles_decision.empty()) return;
  if ((task_info.current_frame()->outside_planner_data().path_data ==
       nullptr) ||
      task_info.current_frame()
          ->outside_planner_data()
          .path_data->path()
          .path_points()
          .empty()) {
    LOG_ERROR("path is nullptr or points is empty.");
    return;
  }
  const auto& path_pts = task_info.current_frame()
                             ->outside_planner_data()
                             .path_data->path()
                             .path_points();

  /// Binary search left bound (a[l] >= v)
  auto LbsIdx = [](const std::vector<PathPoint>& a, const double v) {
    int m = static_cast<int>(a.size());
    if (m == 1) return 0;
    if (v < a[0].s()) return 0;
    if (v > a[m - 2].s()) return m - 2;
    int l = 0, r = m - 2;
    while (l < r) {
      int mid = l + (r - l) / 2;
      if (a[mid].s() < v) {
        l = mid + 1;
      } else {
        r = mid;
      }
    }
    return l;
  };

  for (const auto& obs_info : static_obstacles_decision) {
    if (obs_info.lower_points.empty()) {
      continue;
    }
    if ((obs_info.obstacle.type() == Obstacle::ObstacleType::BICYCLE) ||
        (obs_info.obstacle.type() == Obstacle::ObstacleType::VEHICLE) ||
        (obs_info.obstacle.type() == Obstacle::ObstacleType::PEDESTRIAN)) {
      continue;
    }
    double collid_s = obs_info.lower_points.front().first.s();
    if (collid_s > check_dis_) {
      continue;
    }
    int start_index = std::min(LbsIdx(path_pts, collid_s - 0.5),
                               static_cast<int>(path_pts.size() - 1));
    int end_index = std::min(
        LbsIdx(path_pts, collid_s + std::max(1.0, obs_info.obstacle.length())),
        static_cast<int>(path_pts.size() - 1));
    double min_dis = 1e5, s = 0.0;
    for (int index = start_index; index <= end_index; ++index) {
      const auto& pt = path_pts[index];
      auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
          {pt.x(), pt.y()}, pt.theta(), 0., 0., 0.);
      double dis = obs_info.obstacle.polygon().distance_to(adc_box);
      if (dis < min_dis) {
        min_dis = dis;
        s = pt.s();
      }
    }
    StaticObsDisInfo static_obs_dis_info{
        .path_lat_dis = min_dis,
        .adc_cartisian_dis =
            obs_info.obstacle.PolygonBoundary().distance_to(adc_boundary_),
        .s = s};
    collide_infos_.insert({obs_info.obstacle.id(), static_obs_dis_info});
    LOG_INFO(
        "insert obs [{}] collide_infos_: adc_cartisian_dis {:.3f}, "
        "path_lat_dis {:.3f}, s {:.3f}",
        obs_info.obstacle.id(), static_obs_dis_info.adc_cartisian_dis,
        static_obs_dis_info.path_lat_dis, static_obs_dis_info.s);
  }
}

double SpeedStaticObsJumpDecider::ComputeSpeedLimit(
    int obs_id, const StaticObsDisInfo& collision_info) {
  std::array<double, 3> end_state{
      std::max(collision_info.s, init_state_[0]) + 0.1,
      ComputeEndSpeedLimit(obs_id, collision_info), 0.};

  double forward_time = 3.0;
  return SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "static_obs_jump_limit", init_state_, end_state, max_speed_,
      forward_time);
}

double SpeedStaticObsJumpDecider::ComputeEndSpeedLimit(
    int obs_id, const StaticObsDisInfo& collision_info) {
  double ans{max_speed_};

  // condition1:
  ans = std::max(0.0, collision_info.path_lat_dis *
                          speed_static_obs_jump_config_->lat_dis_speed_ratio);

  // condition2:
  ans = (collision_info.adc_cartisian_dis > 2.0) ? std::max(ans, 1.0) : ans;

  // condition3:
  if (history_dynamic_obs_.find(obs_id) != history_dynamic_obs_.end()) {
    ans = 0.0;
    LOG_INFO("obs [{}] change from dynamic to static.", obs_id);
  }

  return ans;
}

void SpeedStaticObsJumpDecider::ClearObsHistoryInfo() {
  for (auto& iter : history_dynamic_obs_) {
    iter.second++;
  }

  auto history_dynamic_obs_tmp = history_dynamic_obs_;
  for (auto& iter : history_dynamic_obs_) {
    if (iter.second > 15) {
      history_dynamic_obs_tmp.erase(iter.first);
    }
  }
  std::swap(history_dynamic_obs_, history_dynamic_obs_tmp);
  LOG_INFO("history_dynamic_obs_ num {}", history_dynamic_obs_.size());
}

void SpeedStaticObsJumpDecider::UpdataObsHistoryInfo(TaskInfo& task_info) {
  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();

  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }

    history_dynamic_obs_[dynamic_obs_vec[i]->id()] = 0;
  }
}

}  // namespace planning
}  // namespace neodrive

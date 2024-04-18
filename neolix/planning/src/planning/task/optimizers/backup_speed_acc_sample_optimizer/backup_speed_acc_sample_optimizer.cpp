#include "backup_speed_acc_sample_optimizer.h"

namespace neodrive {
namespace planning {

namespace {
const double kQueryTime = 2.0;
}

BackupSpeedAccSampleOptimizer::BackupSpeedAccSampleOptimizer() {
  name_ = "BackupSpeedAccSampleOptimizer";
}

BackupSpeedAccSampleOptimizer::~BackupSpeedAccSampleOptimizer() { Reset(); }

ErrorCode BackupSpeedAccSampleOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    Clear();
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (frame->outside_planner_data().speed_succeed_tasks > 0) {
    Clear();
    return ErrorCode::PLANNING_OK;
  }
  if (frame->outside_planner_data().speed_slow_down) {
    Clear();
    LOG_INFO("skip task, speed_slow_down");
    return ErrorCode::PLANNING_OK;
  }

  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool BackupSpeedAccSampleOptimizer::Init(TaskInfo& task_info) {
  double max_a = config::PlanningConfig::Instance()
                     ->planning_research_config()
                     .backup_speed_optimizer_config.sample_max_a;
  is_in_door_ = task_info.curr_referline_pt().lane_type_is_indoor_lane();
  auto& speed_limits = data_center_->behavior_speed_limits().speed_limits();
  for (const auto& speed_limit : speed_limits) {
    if (speed_limit.source_type() == SpeedLimitType::COLLISION_RISK &&
        speed_limit.upper_bounds_size() == 1) {
      max_a = std::min(max_a, speed_limit.acceleration());
      break;
    }
  }
  LOG_INFO("modify by collision risk max_a = {:.3f}", max_a);
  CalMaxAccByMainPlanner(
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.backup_cipv,
      task_info.current_frame()->outside_planner_data().speed_obstacle_context,
      max_a);
  LOG_INFO("modify by main planner max_a = {:.3f}", max_a);
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;

  double min_a = config::PlanningConfig::Instance()
                     ->planning_research_config()
                     .backup_speed_optimizer_config.sample_min_a;

  if (is_in_door_) {
    min_a = config::PlanningConfig::Instance()
                ->planning_research_config()
                .backup_speed_optimizer_config.indoor_min_a;
    LOG_INFO("is_in_door {},ego in door, min_a = {:.3f}", is_in_door_, min_a);
  }
  need_min_acc_ = min_a;
  double step = config::PlanningConfig::Instance()
                    ->planning_research_config()
                    .backup_speed_optimizer_config.sample_step;
  int sample_count =
      max_a <= min_a ? 0 : static_cast<int>((max_a - min_a) / step);

  acc_sample_.clear();
  for (int i = 0; i < sample_count; ++i) {
    acc_sample_.push_back(max_a - step * i);
  }
  acc_sample_.push_back(min_a);
  LOG_INFO("Sample result: max_a:{:.3f},min_a:{:.3f},sample_count:{}", max_a,
           min_a, sample_count);

  return true;
}

bool BackupSpeedAccSampleOptimizer::Process(TaskInfo& task_info) {
  std::vector<Obstacle> valid_obs;
  std::unordered_set<int> reverse_obs_id;
  auto& outside_data = task_info.current_frame()->outside_planner_data();
  auto& static_ignore_obs =
      outside_data.speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;

  speed_planner_common::ExtractReverseObstacle(
      outside_data.speed_obstacle_context.dynamic_obstacles_decision,
      *(outside_data.path_data),
      task_info.current_frame()->inside_planner_data().init_point.theta(),
      reverse_obs_id);
  speed_planner_common::ExtractValidObstacle(
      outside_data.speed_obstacle_context, valid_obs);

  bool exist_dynamic{false};
  for (const auto& obs : valid_obs) {
    if (!obs.is_static()) {
      exist_dynamic = true;
      break;
    }
  }
  // keep move up
  if (!exist_dynamic) {
    need_min_acc_ = std::max(
        need_min_acc_, config::PlanningConfig::Instance()
                               ->planning_research_config()
                               .backup_speed_optimizer_config.static_min_a *
                           1.0);
    LOG_INFO("No dynamic need check, limit min acc = {:.3f}", need_min_acc_);
  }
  // exist obstacle,
  for (auto& acc : acc_sample_) {
    if (acc < need_min_acc_) break;
    ResampleAcc(task_info, acc);
    const double station_error =
        data_center_->control_command_msg.ptr->contrl_context()
            .long_ctrl_ctx()
            .station_error();
    const auto trajectory = speed_planner_common::CombineSpeedAndPath(
        task_info.current_frame()->inside_planner_data().init_point,
        *(task_info.current_frame()->outside_planner_data().path_data),
        speed_point_, station_error, false);
    double collision_time = 2.5;
    if (speed_planner_common::FinalObstacleCollisionSanityCheck(
            valid_obs, reverse_obs_id, trajectory, collision_time,
            kQueryTime)) {
      ResampleAcc(task_info, std::max(acc - 1.0, acc_sample_.back()));
      StoreSpeedResult(task_info);
      LOG_INFO("backup result, final acc = {:.3f}", acc - 1.0);
      return true;
    }
  }
  // get slowdown
  LOG_INFO("backup can't genenerate, emergancy brake!");
  CallSlowDown(task_info);
  return true;
}

bool BackupSpeedAccSampleOptimizer::ResampleAcc(TaskInfo& task_info,
                                                double acc) {
  // ego state
  double ego_a = task_info.current_frame()->inside_planner_data().vel_a;
  double ego_v = task_info.current_frame()->inside_planner_data().vel_v;
  double ego_s = 0.0;
  double ego_t = 0.0;

  speed_point_.clear();
  int sample_count = static_cast<int>(time_length_ / resolution_);
  for (int i = 0; i < sample_count; ++i) {
    SpeedPoint sp{};
    double last_v = speed_point_.empty() ? ego_v : speed_point_.back().v();
    double last_s = speed_point_.empty() ? ego_s : speed_point_.back().s();
    double last_a = speed_point_.empty() ? ego_a : speed_point_.back().a();
    // double new_a = (i == 0 ? 0.0 : last_a - 0.1 * (last_a - acc));

    double new_v = std::max(0.0, last_v + acc * resolution_);
    // cal new_v max_v need a

    sp.set_v(new_v);
    double new_s = last_s + (last_v + sp.v()) / 2.0 * resolution_;
    sp.set_s(new_s);
    sp.set_a(speed_point_.empty() ? ego_a : acc);  // re --> a
    sp.set_t(i * resolution_);
    speed_point_.emplace_back(std::move(sp));
  }

  return true;
}

void BackupSpeedAccSampleOptimizer::StoreSpeedResult(TaskInfo& task_info) {
  auto& frame = task_info.current_frame();
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  outside_planner_data_ptr->speed_data->set_speed_vector(speed_point_);
  outside_planner_data_ptr->speed_context.dp_st_data.smoothed_speed =
      speed_point_;
  outside_planner_data_ptr->speed_succeed_tasks += 1;
  outside_planner_data_ptr->speed_context.is_emergency = false;
}

void BackupSpeedAccSampleOptimizer::CallSlowDown(TaskInfo& task_info) {
  ResampleAcc(task_info, need_min_acc_);
  StoreSpeedResult(task_info);
}

void BackupSpeedAccSampleOptimizer::Clear() {
  pass_check_ = false;
  sentinal_a_ = 0.0;
}

bool BackupSpeedAccSampleOptimizer::CalMaxAccByMainPlanner(
    const BackupCipv& backup_cipv,
    const SpeedObstacleContext& speed_obstacle_context, double& max_a) {
  if (!backup_cipv.has_cipv) {
    LOG_INFO("no cipv, no need to check");
    return false;
  }
  for (const auto& obs_dec : speed_obstacle_context.static_obstacles_decision) {
    if (obs_dec.obstacle.id() == backup_cipv.cipv_id) {
      UpdateMaxA(obs_dec, max_a);
      return true;
    }
  }

  for (const auto& obs_dec :
       speed_obstacle_context.dynamic_obstacles_decision) {
    if (obs_dec.obstacle.id() == backup_cipv.cipv_id) {
      UpdateMaxA(obs_dec, max_a);
      return true;
    }
  }

  return false;
}

void BackupSpeedAccSampleOptimizer::UpdateMaxA(
    const SpeedObstacleDecision& obs_dec, double& max_a) {
  if (obs_dec.lower_points.empty() ||
      obs_dec.lower_points_heading_diff.empty()) {
    LOG_INFO("no lower points, no need to check");
    return;
  }
  double max_accessibility_s = obs_dec.lower_points.front().first.s();
  double obs_speed =
      obs_dec.obstacle.speed() * std::cos(obs_dec.lower_points_heading_diff[0]);
  double a1 = (std::pow(obs_speed, 2) - std::pow(adc_current_v_, 2)) / 2 /
              max_accessibility_s;
  double a2 = (obs_speed - adc_current_v_) /
              (obs_dec.lower_points.front().first.t() + 1e-8);
  max_a = std::min(max_a, std::min(a1, a2));
  LOG_INFO("update max_a by cipv, max_a = {:.3f}", max_a);
}

}  // namespace planning
}  // namespace neodrive

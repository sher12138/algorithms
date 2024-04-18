#include "motorway_speed_backup_acc_sample_optimizer.h"

namespace neodrive {
namespace planning {

namespace {
const double kQueryTime = 2.0;
const double kFirstProtectTime = 0.5;
const double kSecondProtectTime = 1.0;
}  // namespace

MotorwaySpeedBackupAccSampleOptimizer::MotorwaySpeedBackupAccSampleOptimizer() {
  name_ = "MotorwaySpeedBackupAccSampleOptimizer";
}

MotorwaySpeedBackupAccSampleOptimizer::
    ~MotorwaySpeedBackupAccSampleOptimizer() {
  Reset();
}

ErrorCode MotorwaySpeedBackupAccSampleOptimizer::Execute(TaskInfo& task_info) {
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
    Clear();
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    Clear();
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedBackupAccSampleOptimizer::Init(TaskInfo& task_info) {
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  adc_current_pos_ = task_info.adc_point();
  adc_heading_ = task_info.current_frame()->inside_planner_data().vel_heading;
  adc_current_sl_ = task_info.curr_sl();
  double max_a =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .backup_speed_optimizer_config
          .sample_max_a;  // if no limit data, must have a init value;
  is_in_door_ = task_info.curr_referline_pt().lane_type_is_indoor_lane();
  // config json can valid _res cearch_config()

  auto& speed_limits =
      DataCenter::Instance()->behavior_speed_limits().speed_limits();
  for (const auto& speed_limit : speed_limits) {
    if (speed_limit.source_type() == SpeedLimitType::COLLISION_RISK &&
        speed_limit.upper_bounds_size() == 1) {
      max_a = std::min(max_a, speed_limit.acceleration());
      break;
    }
  }
  LOG_INFO("modify by collision_risk limit _speed, max_a = {:.3f}", max_a);
  // base main planner to acquire max a
  CalMaxAccByMainPlanner(task_info.current_frame()
                             ->outside_planner_data()
                             .motorway_speed_obstacle_context.backup_cipv,
                         task_info.current_frame()
                             ->outside_planner_data()
                             .motorway_speed_obstacle_context,
                         max_a);
  LOG_INFO("modify by main planner, max_a = {:.3f}", max_a);

  // back cipv
  bool trigger_back_cipv = task_info.current_frame()
                               ->mutable_outside_planner_data()
                               ->motorway_speed_context.trigger_backcipv;
  if (trigger_back_cipv) {
    max_a = std::min(max_a, task_info.current_frame()
                                ->mutable_outside_planner_data()
                                ->motorway_speed_context.conflict_brake);
    LOG_INFO("The maximum value after backcipv correction is {:.3f}", max_a);
  }

  double min_a = config::PlanningConfig::Instance()
                     ->planning_research_config()
                     .backup_speed_optimizer_config.sample_min_a;
  if (is_in_door_) {
    min_a = config::PlanningConfig::Instance()
                ->planning_research_config()
                .backup_speed_optimizer_config.indoor_min_a;
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
  LOG_INFO("Sample range, from:({:.3f} to {:.3f}), sample is {}",
           acc_sample_.front(), acc_sample_.back(), sample_count + 1);

  sentinal_a_.clear();
  return true;
}

bool MotorwaySpeedBackupAccSampleOptimizer::Process(TaskInfo& task_info) {
  std::vector<Obstacle> valid_obs;
  std::unordered_set<int> reverse_obs_id;
  auto& speed_obs_con = task_info.current_frame()
                            ->outside_planner_data()
                            .motorway_speed_obstacle_context;
  speed_planner_common::ExtractReverseObstacle(
      speed_obs_con.multi_cipv_dynamic_obstacles_decision,
      *(task_info.current_frame()->outside_planner_data().path_data),
      task_info.current_frame()->inside_planner_data().init_point.theta(),
      reverse_obs_id);
  speed_planner_common::ExtractValidObstacle(speed_obs_con, valid_obs);

  bool exist_dynamic{false};
  for (const auto& obs : valid_obs) {
    if (!obs.is_static()) {
      exist_dynamic = true;
      break;
    }
  }

  if (!exist_dynamic) {
    need_min_acc_ = std::max(
        need_min_acc_, config::PlanningConfig::Instance()
                               ->planning_research_config()
                               .backup_speed_optimizer_config.static_min_a *
                           1.0);
    LOG_INFO("No dynamic need check, limit min acc = {:.3f}", need_min_acc_);
  }
  for (auto& acc : acc_sample_) {
    double collision_time = 5.0;
    if (acc < need_min_acc_) break;
    ResampleAcc(task_info, acc);

    const double station_error =
        data_center_->control_command_msg.ptr->contrl_context()
            .long_ctrl_ctx()
            .station_error();
    // tra create start from init_point, so must to add station_error
    const auto trajectory = speed_planner_common::CombineSpeedAndPath(
        task_info.current_frame()->inside_planner_data().init_point,
        *(task_info.current_frame()->outside_planner_data().path_data),
        speed_point_, station_error, false);

    if (speed_planner_common::FinalObstacleCollisionSanityCheck(
            valid_obs, reverse_obs_id, trajectory, collision_time,
            kQueryTime)) {
      ResampleAcc(task_info, std::max(acc - 1.0, acc_sample_.back()));
      sentinal_a_.push_back(std::make_pair(acc, collision_time));
      break;
    }
    sentinal_a_.push_back(std::make_pair(acc, collision_time));
  }

  // multi frame check
  if (adc_current_v_ > 7.) {
    GenerateFinalTrajectory(task_info);  //
  }

  StoreSpeedResult(task_info);
  return true;
}

void MotorwaySpeedBackupAccSampleOptimizer::GenerateFinalTrajectory(
    TaskInfo& task_info) {
  double push_down_acc = sentinal_a_.back().first;
  if (history_ans_.empty()) {
    // current first frame
    // push down collision time > 0.5  acc
    for (const auto& [acc, collision_time] : sentinal_a_) {
      if (collision_time > kFirstProtectTime) {
        push_down_acc = acc;
        LOG_INFO("first touch, push_down_acc = {:.3f}", push_down_acc);
        break;
      }
    }
  } else if (history_ans_.size() == 1) {
    // find collision time > 1.0
    for (const auto& [acc, collision_time] : sentinal_a_) {
      if (collision_time > kSecondProtectTime) {
        push_down_acc = acc;
        LOG_INFO("second touch, push_down_acc = {:.3f}", push_down_acc);
        break;
      }
    }
  } else {
    push_down_acc = sentinal_a_.back().first;
    LOG_INFO("normal touch, push_down_acc = {:.3f}", push_down_acc);
  }
  ResampleAcc(task_info, std::max(push_down_acc, acc_sample_.back()));

  history_ans_.push_back(push_down_acc);
};
bool MotorwaySpeedBackupAccSampleOptimizer::ResampleAcc(TaskInfo& task_info,
                                                        double acc) {
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

    double new_v = std::max(0.0, last_v + acc * resolution_);
    sp.set_v(new_v);
    double new_s = last_s + (last_v + sp.v()) / 2.0 * resolution_;
    sp.set_s(std::max(new_s, 0.0));
    sp.set_a(speed_point_.empty() ? ego_a : acc);
    sp.set_t(i * resolution_);
    speed_point_.emplace_back(std::move(sp));
  }
  return true;
}

void MotorwaySpeedBackupAccSampleOptimizer::StoreSpeedResult(
    TaskInfo& task_info) {
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

void MotorwaySpeedBackupAccSampleOptimizer::CallSlowDown(TaskInfo& task_info) {
  ResampleAcc(task_info, need_min_acc_);
  StoreSpeedResult(task_info);
}

void MotorwaySpeedBackupAccSampleOptimizer::Clear() { history_ans_.clear(); }

bool MotorwaySpeedBackupAccSampleOptimizer::CalMaxAccByMainPlanner(
    const BackupCipv& backup_cipv,
    const MotorwaySpeedObstacleContext& speed_obstacle_context, double& max_a) {
  if (!backup_cipv.has_cipv) {
    LOG_INFO("no cipv");
    return false;
  }
  for (const auto& obs_dec :
       speed_obstacle_context.multi_cipv_static_obstacles_decision) {
    if (obs_dec.obstacle.id() == backup_cipv.cipv_id) {
      UpdateMaxA(obs_dec, max_a);
      return true;
    }
  }

  for (const auto& obs_dec :
       speed_obstacle_context.multi_cipv_dynamic_obstacles_decision) {
    if (obs_dec.obstacle.id() == backup_cipv.cipv_id) {
      UpdateMaxA(obs_dec, max_a);
      return true;
    }
  }

  return false;
}

void MotorwaySpeedBackupAccSampleOptimizer::UpdateMaxA(
    const MotorwayMultiCipvSpeedObstacleDecision& obs_dec, double& max_a) {
  double max_accessibility_s = obs_dec.lower_points.front().first.s();
  /*
    question:
    (v1^2 - v0^2) / 2(s^` - s0) = a
    t0 = (v1 - v0) / a
    t0 > t1
    s^` < s1'

    -->
    a >= (v1^2 - v0^2) / 2(s^'-s0)
    a <= (v1 - v0) / t
  */
  if (obs_dec.lower_points.empty() ||
      obs_dec.lower_points_heading_diff.empty()) {
    LOG_INFO("no lower points, no need to check");
    return;
  }
  double obs_speed =
      obs_dec.obstacle.speed() * std::cos(obs_dec.lower_points_heading_diff[0]);
  double a1 = (std::pow(obs_speed, 2) - std::pow(adc_current_v_, 2)) / 2 /
              max_accessibility_s;
  double a2 =
      (obs_speed - adc_current_v_) / obs_dec.lower_points.front().first.t();
  max_a = std::min(max_a, std::min(a1, a2));
  LOG_INFO("update max_a by cipv, max_a = {:.3f}", max_a);
}

}  // namespace planning
}  // namespace neodrive

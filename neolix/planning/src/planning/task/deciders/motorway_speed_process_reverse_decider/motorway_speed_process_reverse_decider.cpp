#include "motorway_speed_process_reverse_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

MotorwaySpeedProcessReverseDecider::MotorwaySpeedProcessReverseDecider() {
  name_ = "MotorwaySpeedProcessReverseDecider";
}

ErrorCode MotorwaySpeedProcessReverseDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    LOG_INFO("path successed tasks is 0, skip rest tasks.");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  LOG_INFO(">>>> Check process reverse obs decider work normal");
  if (!Init(task_info)) {
    LOG_ERROR("Check process reverse obs decider Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("process reverse obs decider Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedProcessReverseDecider::Init(TaskInfo& task_info) {
  adc_current_s_ =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();

  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();

  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  reset();
  return true;
}
void MotorwaySpeedProcessReverseDecider::reset() {
  // reset
  reverse_obs_list_.clear();
}

bool MotorwaySpeedProcessReverseDecider::Process(TaskInfo& task_info) {
  LOG_INFO("start process reverse obs ");
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  // 多种：逆行忽略的同时加减速度限速
  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;

  // if (JudgeEgoInJunction(task_info)) {
  //   LOG_INFO("ego is in junction , donot ignore obs ");
  //   return true;
  // }

  const auto& ignore_dynamic_obs =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context.ignore_dynamic_obs_id;
  for (const auto& obs_decision : dynamic_obstacles_decision) {
    if (JudgeObsInJunction(task_info,
                           obs_decision.obstacle.PolygonBoundary().start_s())) {
      LOG_INFO("obs is in junction , ignore", obs_decision.obstacle.id());
      continue;
    }
    if (ignore_dynamic_obs.find(obs_decision.obstacle.id()) !=
        ignore_dynamic_obs.end()) {
      LOG_INFO("obs id : {} is ignore before", obs_decision.obstacle.id());
      continue;
    }
    if (obs_decision.obstacle.type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO("obs id : {} is people ,ignore", obs_decision.obstacle.id());
      continue;
    }
    PathPoint closest_pt{};
    auto& obstacle = obs_decision.obstacle;
    double path_heading_near_obs =
        path.query_closest_point(obstacle.center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff =
        normalize_angle(obstacle.velocity_heading() - path_heading_near_obs);
    bool if_reverse{false};
    if (std::abs(heading_diff) < 2.355) {
      LOG_INFO("obs heading diff : {} is smaller than 3/4 pi ,ignore",
               heading_diff / 3.14 * 180);
      continue;
    };
    // 剩下的才是应该处理的
    ReverseObs single_reverse_obs{
        obs_decision.obstacle.id(), obs_decision,
        obs_decision.obstacle.PolygonBoundary().start_s() - adc_current_s_,
        false, heading_diff};
    reverse_obs_list_.emplace_back(std::move(single_reverse_obs));
  }
  CalProcessAction(task_info);
  return true;
}

MotorwaySpeedProcessReverseDecider::~MotorwaySpeedProcessReverseDecider() {}

void MotorwaySpeedProcessReverseDecider::CalProcessAction(TaskInfo& task_info) {
  const auto& reverse_conf = config::PlanningConfig::Instance()
                                 ->planning_research_config()
                                 .speed_reversed_repulsive_field_caution;
  auto& ignore_dynamic_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.ignore_dynamic_obs_id;
  if (reverse_obs_list_.empty()) return;

  for (auto& reverse_obs : reverse_obs_list_) {
    double obs_speed_long =
        std::abs(reverse_obs.reverse_obstacle_decision.obstacle.speed() *
                 std::abs(reverse_obs.heading_diff));

    double risk_dis_1 = std::abs(std::pow(adc_current_v_, 2) / (2 * 4) +
                                 (adc_current_v_ / 4) * obs_speed_long);
    double risk_dis_2 =
        (adc_current_v_ * 2 + reverse_conf.act_time * reverse_conf.a_ego_acc) /
            2 * reverse_conf.act_time +
        std::pow(
            (adc_current_v_ + reverse_conf.act_time * reverse_conf.a_ego_acc),
            2) /
            (2 * reverse_conf.a_ego_brake) +
        (obs_speed_long * 2 + reverse_conf.act_time * reverse_conf.a_obs_acc) /
            2 * reverse_conf.act_time +
        std::pow(
            (obs_speed_long + reverse_conf.act_time * reverse_conf.a_obs_acc),
            2) /
            (2 * reverse_conf.a_obs_brake);
    double risk_dis = std::max(risk_dis_1, risk_dis_2);
    if (reverse_obs.obs_longitudinal_dis < risk_dis) {
      LOG_INFO("obs dis {} is smaller risk dis : {} ,donot process it here",
               reverse_obs.obs_longitudinal_dis, risk_dis);
    } else {
      reverse_obs.if_ignore = true;
      LOG_INFO("obs id  {} is larger than  risk dis : {} , process it here",
               reverse_obs.obs_id, risk_dis);
    }
  }

  for (auto& reverse_obs : reverse_obs_list_) {
    if (reverse_obs.if_ignore) {
      ignore_dynamic_obs_id.insert(reverse_obs.obs_id);

      double limit_speed = std::min(adc_current_v_ - 0.5 * 1, 0.0);
      neodrive::global::planning::SpeedLimit internal_speed_limit{};
      internal_speed_limit.set_source_type(SpeedLimitType::REVERSED_OBS);
      internal_speed_limit.add_upper_bounds(limit_speed);
      internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
      internal_speed_limit.set_acceleration(-0.5);
      LOG_INFO("Reverse decider  limit acc = {:.2f} for obs id : {}", -0.5,
               reverse_obs.obs_id);
      data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
          internal_speed_limit);
    }
  }
}

bool MotorwaySpeedProcessReverseDecider::JudgeObsInJunction(
    TaskInfo& task_info, const double& obs_s) {
  auto ref_line = task_info.reference_line();
  const auto& junction_list = ref_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }

    double deal_area_start_s =
        overlap.start_s - VehicleParam::Instance()->length() - 10.0;
    double deal_area_end_s = overlap.end_s;
    if (deal_area_start_s <= obs_s && deal_area_end_s >= obs_s &&
        (junction_ptr->Type() !=
         static_cast<uint32_t>(JunctionType::IN_ROAD))) {
      return true;
    } else if (deal_area_start_s > obs_s) {
      break;
    }
  }
  return false;
}

void MotorwaySpeedProcessReverseDecider::Reset(){};

void MotorwaySpeedProcessReverseDecider::SaveTaskResults(TaskInfo& task_info) {}

}  // namespace planning
}  // namespace neodrive
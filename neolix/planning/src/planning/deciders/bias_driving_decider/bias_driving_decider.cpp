#include "bias_driving_decider.h"

#include "src/planning/config/planning_config.h"
#include "src/planning/deciders/station_stop_decider/station_stop_decider.h"
#include "src/planning/deciders/triggered_stop_decider/triggered_stop_decider.h"
#include "src/planning/scenario_manager/scenario_manager.h"

namespace neodrive {
namespace planning {

BiasDrivingDecider::BiasDrivingDecider() { name_ = "BiasDrivingDecider"; }

BiasDrivingDecider::~BiasDrivingDecider() {}

void BiasDrivingDecider::Reset() {}

ErrorCode BiasDrivingDecider::Execute(TaskInfo& task_info) {
  is_need_bias_driving_ = false;
  is_front_lane_turning_ = false;
  is_junction_close_ = false;

  if (task_info.current_frame() == nullptr ||
      task_info.reference_line() == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  auto is_station_stop_need_pull_over =
      data_center_->master_info().is_station_stop_need_pull_over();
  if (!data_center_->global_state_proxy().is_cruise()) {
    return ErrorCode::PLANNING_OK;
  }

  do {
    if (!is_station_stop_need_pull_over) {
      // check [front lane turning],[junction close]
      if (!IsFrontLaneTurning(task_info, is_front_lane_turning_)) {
        LOG_ERROR("IsFrontLaneTurning failed.");
        return ErrorCode::PLANNING_ERROR_FAILED;
      }
      // front lane is turning, exit bias driving.
      if (true == is_front_lane_turning_) {
        LOG_ERROR("front lane is turning.");
        break;
      }
      if (!IsJunctionClose(task_info, is_junction_close_)) {
        LOG_ERROR("IsJunctionClose failed.");
        return ErrorCode::PLANNING_ERROR_FAILED;
      }
      // junction close, exit bias driving.
      if (true == is_junction_close_) {
        LOG_ERROR("front lane is closing junction.");
        break;
      }
    }

    // calc front bound
    const auto& ref_points = task_info.reference_line()->ref_points();
    double preview_distance = std::max(
        vehicle_state_.LinearVelocity() * FLAGS_planning_trajectory_time_length,
        20.0);
    double min_left_bound = 1000.0, min_right_bound = 1000.0;
    if (!scenario_common::FrontMinLaneBound(ref_points, preview_distance,
                                            task_info.curr_sl(), min_left_bound,
                                            min_right_bound)) {
      LOG_ERROR("CalcFrontLaneBound failed");
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
    LOG_INFO(
        "min_rb:{:.2f}, adc_rl:{:.2f}, dis2rlb:{:.2f}, min_lb:{:.2f}, "
        "adc_ll:{:.2f}, dis2llb:{:.2f}",
        min_right_bound, task_info.adc_boundary().start_l(),
        task_info.adc_boundary().start_l() - min_right_bound, min_left_bound,
        task_info.adc_boundary().end_l(),
        min_left_bound - task_info.adc_boundary().end_l());
    // station
    bool skip_flag = false;
    if (!ProcessStationFunction(task_info, min_left_bound, min_right_bound,
                                skip_flag)) {
      LOG_ERROR("ProcessStationFunction failed");
    }
    if (skip_flag) {
      break;
    }

    // triggered stop
    if (!ProcessTriggeredStopFunction(task_info, min_left_bound,
                                      min_right_bound, skip_flag)) {
      LOG_ERROR("ProcessTriggeredStopFunction failed");
    }
    if (skip_flag) {
      break;
    }

    // broad road
    if (!ProcessBroadRoadFunction(task_info, min_left_bound, min_right_bound,
                                  skip_flag)) {
      LOG_ERROR("ProcessBroadRoadFunction failed");
    }
    if (skip_flag) {
      break;
    }

    if (!ProcessRoadHasBound(task_info, min_left_bound, min_right_bound,
                             skip_flag)) {
      LOG_ERROR("ProcessRoadHasBound failed");
    }
    if (skip_flag) {
      break;
    }
  } while (false);  // break here.

  return ErrorCode::PLANNING_OK;
}

void BiasDrivingDecider::SaveTaskResults(TaskInfo& task_info) {
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  data_center_->mutable_master_info()->set_need_bias_driving(
      is_need_bias_driving_);
  data_center_->mutable_master_info()->set_keep_bias_distance(
      keep_bias_distance_);
  auto& observe_ref_l = task_info.current_frame()
                            ->outside_planner_data()
                            .path_observe_ref_l_info.observe_ref_l;

  // save monitor message.
  static char str_buffer[256];
  sprintf(str_buffer,
          "[BIAS][junction: %d][lane_turn: %d][is_bias: %d][bias_dis: "
          "%.1lf][keep_bias_dis: %.1lf][k:%.1lf]",
          is_junction_close_, is_front_lane_turning_, is_need_bias_driving_,
          observe_ref_l, keep_bias_distance_, filter_k_);
  data_center_->SetMonitorString(str_buffer, MonitorItemSource::BIAS_DRIVING);
}

bool BiasDrivingDecider::ProcessStationFunction(TaskInfo& task_info,
                                                const double& min_left_bound,
                                                const double& min_right_bound,
                                                bool& skip_flag) {
  skip_flag = false;

  // station stop
  auto is_station_stop = data_center_->master_info().is_station_stop();
  auto is_station_stop_need_pull_over =
      data_center_->master_info().is_station_stop_need_pull_over();
  auto is_destination_in_unlaoding_zone =
      data_center_->master_info().is_destination_in_unlaoding_zone();
  if (!is_station_stop || !is_station_stop_need_pull_over) {
    return true;
  }
  auto& observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  // prefer left/right
  bool is_bias_to_right = FLAGS_planning_default_left_right_side &&
                          !is_destination_in_unlaoding_zone;

  ComputePullOverRefL(task_info, min_left_bound, min_right_bound,
                      FLAGS_planning_shift_pull_over_residual_distance,
                      is_bias_to_right);

  YieldDynamicObsSideBack(task_info, min_left_bound, min_right_bound);
  skip_flag = true;
  is_need_bias_driving_ = true;
  LOG_INFO("enable bias driving, val: {:.4f}", observe_ref_l);
  return true;
}

bool BiasDrivingDecider::ProcessTriggeredStopFunction(
    TaskInfo& task_info, const double& min_left_bound,
    const double& min_right_bound, bool& skip_flag) {
  skip_flag = false;
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  // station stop
  bool is_in_stop_mode = data_center_->master_info().is_trigger_stop();
  bool need_pull_over = plan_config.bias_driving.trigger_stop_need_pull_over;
  if (!(is_in_stop_mode && need_pull_over)) return true;
  auto& observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  // prefer left/right
  ComputePullOverRefL(task_info, min_left_bound, min_right_bound,
                      FLAGS_planning_shift_pull_over_residual_distance,
                      FLAGS_planning_default_left_right_side);
  LOG_INFO("start to pull over to right side in triggered stop");

  YieldDynamicObsSideBack(task_info, min_left_bound, min_right_bound);
  skip_flag = true;
  is_need_bias_driving_ = true;
  LOG_INFO("Triggered station Stop, val: {:.4f}", observe_ref_l);
  return true;
}

bool BiasDrivingDecider::ProcessBroadRoadFunction(TaskInfo& task_info,
                                                  const double& min_left_bound,
                                                  const double& min_right_bound,
                                                  bool& skip_flag) {
  auto& plan_config = config::PlanningConfig::Instance()->plan_config();

  skip_flag = false;
  if (false == plan_config.bias_driving.enable_board_road_bias) return true;

  // filter road width.
  static bool is_last_frame_board_road = false;
  if (is_last_frame_board_road) {
    if (min_left_bound + min_right_bound <
        plan_config.bias_driving.board_road_width_exit_threshold) {
      // exit board road.
      is_last_frame_board_road = false;
      return true;
    } else {
      // stay in board road.
    }
  } else {
    if (min_left_bound + min_right_bound >
        plan_config.bias_driving.board_road_width_into_threshold) {
      // into board road.
      is_last_frame_board_road = true;
    } else {
      // stay in normal road.
      return true;
    }
  }
  auto& observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  // prefer left/right
  double residual_dis = plan_config.bias_driving.board_road_bias_residual;
  ComputePullOverRefL(task_info, min_left_bound, min_right_bound,
                      FLAGS_planning_shift_pull_over_residual_distance,
                      FLAGS_planning_default_left_right_side);
  LOG_INFO("start to pull over to right side in board road");

  YieldDynamicObsSideBack(task_info, min_left_bound, min_right_bound);
  skip_flag = true;
  is_need_bias_driving_ = true;

  LOG_INFO("board road bias driving, val: {:.4f}", observe_ref_l);
  return true;
}

void BiasDrivingDecider::YieldDynamicObsSideBack(TaskInfo& task_info,
                                                 const double& left_bound,
                                                 const double& right_bound) {
  if (task_info.last_frame() == nullptr) {
    LOG_INFO("last_frame null");
    return;
  }
  auto decision_data =
      task_info.last_frame()->planning_data().mutable_decision_data();
  if (decision_data == nullptr) {
    LOG_INFO("decision_data null");
    return;
  }
  auto& ref_ptr = task_info.reference_line();
  if (ref_ptr == nullptr || ref_ptr->ref_points().empty()) {
    return;
  }
  const auto& inside_data = task_info.last_frame()->inside_planner_data();
  const auto& outside_data = task_info.last_frame()->outside_planner_data();
  auto const& bias_driving_config =
      config::PlanningConfig::Instance()->plan_config().bias_driving;
  const auto& adc_boundary = outside_data.path_obstacle_context.adc_boundary;
  std::vector<ObstacleBoundary> dynamic_obs_boundary{};
  const double& adc_velocity =
      DataCenter::Instance()->vehicle_state_proxy().LinearVelocity();
  if (task_info.decision_data()->dynamic_obstacle().empty()) {
    data_center_->mutable_master_info()->set_need_yield(false);
    LOG_ERROR("dynamic_obstacle is empty");
    return;
  }
  for (auto& obstacle : decision_data->dynamic_obstacle()) {
    LOG_DEBUG(
        "obstacle[{}] min_s: {:.4f}, max_s: {:.4f}, min_l: {:.4f}, max_l: "
        "{:.4f}, v: {:.4f}",
        obstacle->id(), obstacle->min_s(), obstacle->max_s(), obstacle->min_l(),
        obstacle->max_l(), obstacle->speed());
    if (obstacle->max_l() <
            right_bound + bias_driving_config.min_shift_distance ||
        obstacle->min_l() > left_bound - bias_driving_config.min_shift_distance)
      continue;
    if (obstacle->min_s() > adc_boundary.end_s() ||
        obstacle->max_s() + bias_driving_config.preview_back_far_distance <
            adc_boundary.start_s())
      continue;
    if (FLAGS_planning_default_left_right_side) {
      if (obstacle->min_l() > adc_boundary.end_l() ||
          obstacle->max_l() + bias_driving_config.side_far_obs_distance <
              adc_boundary.start_l())
        continue;
    } else {
      if (obstacle->max_l() < adc_boundary.start_l() ||
          obstacle->min_l() - bias_driving_config.side_far_obs_distance >
              adc_boundary.end_l())
        continue;
    }
    ReferencePoint reference_point;
    if (!ref_ptr->GetNearestRefPoint(obstacle->center_sl().s(),
                                     &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double heading_diff = std::abs(normalize_angle(
        obstacle->velocity_heading() - reference_point.heading()));
    if (heading_diff > bias_driving_config.same_direction_heading_threshold) {
      LOG_INFO("skip oblique obstacle[{}] heading_diff: {:.4f}", obstacle->id(),
               heading_diff);
      continue;
    }

    if (obstacle->speed() < bias_driving_config.slow_obs_velocity_threshold &&
        obstacle->speed() < adc_velocity - 2.0)
      continue;
    double delt_s = adc_boundary.start_s() - obstacle->max_s();
    if (obstacle->speed() > adc_velocity + kMathEpsilon) {
      double delt_v = obstacle->speed() - adc_velocity;
      double delt_t = delt_s / delt_v;
      if (delt_t > 6.0) continue;
    }

    if (obstacle->speed() + kMathEpsilon < adc_velocity && delt_s > 5.0) {
      LOG_INFO("rear side dynamic obstacles is too slow.");
      continue;
    }

    dynamic_obs_boundary.emplace_back(
        ObstacleBoundary{obstacle->PolygonBoundary(), obstacle});
    LOG_INFO("find side back dynamic id:{},", obstacle->id());
  }
  if (dynamic_obs_boundary.empty()) {
    data_center_->mutable_master_info()->set_need_yield(false);
    LOG_INFO("obs_valid_boundary is empty.");
    return;
  }
  std::sort(dynamic_obs_boundary.begin(), dynamic_obs_boundary.end(),
            [](const auto& a, const auto& b) {
              return a.first.end_l() > b.first.end_l();
            });
  double rear_dynamic_obs_max_l = dynamic_obs_boundary.front().first.end_l();
  auto& observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  observe_ref_l = std::min(outside_data.path_observe_ref_l_info.observe_ref_l,
                           rear_dynamic_obs_max_l + 0.6);

  if (!CreateYieldVirtualObs(task_info)) {
    LOG_ERROR("RoutingDestinationVirtualObs failed.");
    return;
  }
  LOG_INFO("rear_dynamic_obs_max_l:{:3f},observe_ref_l:{:3f}.",
           rear_dynamic_obs_max_l, observe_ref_l);
  return;
}

bool BiasDrivingDecider::CreateYieldVirtualObs(TaskInfo& task_info) {
  const auto curr_s = task_info.curr_sl().s();
  const auto distance_to_end = data_center_->master_info().distance_to_end();
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  int id{0};
  ReferencePoint destination_ref_point;
  data_center_->mutable_master_info()->set_need_yield(false);

  if (distance_to_end > 7.0) {
    double routing_destination_s = curr_s + distance_to_end +
                                   common::config::CommonConfig::Instance()
                                       ->ego_car_config()
                                       .front_edge_to_center -
                                   5.0;
    data_center_->mutable_master_info()->set_yield_to_end_s(
        routing_destination_s);
    data_center_->mutable_master_info()->set_need_yield(true);
    LOG_INFO("yield destination s:{:2f}", routing_destination_s);
  } else if (distance_to_end > 2.0) {
    double routing_destination_s = curr_s +
                                   common::config::CommonConfig::Instance()
                                       ->ego_car_config()
                                       .front_edge_to_center +
                                   2.0;
    LOG_INFO("yield destination s:{:2f}", routing_destination_s);
    data_center_->mutable_master_info()->set_yield_to_end_s(
        routing_destination_s);
    data_center_->mutable_master_info()->set_need_yield(true);
  }
  return true;
}

bool BiasDrivingDecider::IsInRoadHasBound(const TaskInfo& task_info,
                                          bool& flag) {
  auto const& plan_config = config::PlanningConfig::Instance()->plan_config();
  // filter road width.
  constexpr double width_filter = 0.2;
  static bool is_last_frame_road_has_bound = false;
  // check road bound.
  for (auto& ref_pt : task_info.reference_line()->ref_points()) {
    if (ref_pt.s() < task_info.curr_sl().s()) {
      continue;
    }
    if (ref_pt.s() >
        task_info.curr_sl().s() + plan_config.bias_driving.road_bound_preview) {
      break;
    }
    if (std::fabs(ref_pt.left_road_bound() - ref_pt.left_bound()) < 0.3 &&
        std::fabs(ref_pt.right_road_bound() - ref_pt.right_bound()) < 0.3 &&
        ((ref_pt.right_bound() + ref_pt.right_bound()) <
                 is_last_frame_road_has_bound
             ? plan_config.bias_driving.road_has_bound_max_width + width_filter
             : plan_config.bias_driving.road_has_bound_max_width)) {
      LOG_INFO("front has road boundary.");
      flag = true;
      break;
    }
  }
  is_last_frame_road_has_bound = flag;
  return true;
}

bool BiasDrivingDecider::ProcessRoadHasBound(TaskInfo& task_info,
                                             const double min_left_bound,
                                             const double min_right_bound,
                                             bool& finish_flag) {
  auto const& plan_config = config::PlanningConfig::Instance()->plan_config();
  bool enable_road_has_bound_bias =
      plan_config.bias_driving.enable_road_has_bound_bias;
  if (false == enable_road_has_bound_bias) {
    LOG_INFO("disable road has bound bias driving.");
    return true;
  }

  bool is_in_road_has_bound = false;
  if (!IsInRoadHasBound(task_info, is_in_road_has_bound)) {
    LOG_ERROR("can not judge is_in_road_has_bound.");
    return false;
  }
  if (false == is_in_road_has_bound) {
    return true;
  }
  double road_has_bound_residual =
      plan_config.bias_driving.road_has_bound_residual;
  auto& observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;

  ComputePullOverRefL(task_info, min_left_bound, min_right_bound,
                      FLAGS_planning_shift_pull_over_residual_distance,
                      FLAGS_planning_default_left_right_side);

  YieldDynamicObsSideBack(task_info, min_left_bound, min_right_bound);
  is_need_bias_driving_ = true;

  LOG_INFO("road has bound, bias_driving,ref_l:{}.", observe_ref_l);
  finish_flag = true;
  return true;
}

bool BiasDrivingDecider::ProcessRoadHasBoundAdjacentJunction(
    const TaskInfo& task_info, double& keep_bias_distance) {
  auto const& plan_config = config::PlanningConfig::Instance()->plan_config();
  // front has junction, keep bias until junction area.
  for (const auto& junction_overlap :
       task_info.reference_line()->junction_overlaps()) {
    if (task_info.curr_sl().s() +
                plan_config.bias_driving.junction_concern_distance >
            junction_overlap.start_s &&
        task_info.curr_sl().s() +
                plan_config.bias_driving.front_junction_close_distance <
            junction_overlap.start_s) {
      keep_bias_distance =
          junction_overlap.start_s - task_info.curr_sl().s() -
          plan_config.bias_driving.front_junction_close_distance;
      break;
    }
  }
  return true;
}

bool BiasDrivingDecider::IsFrontLaneTurning(const TaskInfo& task_info,
                                            bool& is_front_lane_turning) {
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  is_front_lane_turning = false;
  for (const auto& pt : task_info.reference_line()->ref_points()) {
    if (pt.s() > task_info.curr_sl().s() +
                     plan_config.bias_driving.lane_turn_preview_distance)
      break;
    if (pt.s() <= task_info.curr_sl().s()) continue;
    if (pt.s() >
        task_info.curr_sl().s() + data_center_->master_info().distance_to_end())
      continue;
    if (fabs(pt.kappa()) >=
        plan_config.bias_driving.lane_turn_kappa_threshold) {
      is_front_lane_turning = true;
      break;
    }
  }
  return true;
}

bool BiasDrivingDecider::IsJunctionClose(const TaskInfo& task_info,
                                         bool& is_junction_close) {
  auto const& plan_config = config::PlanningConfig::Instance()->plan_config();
  constexpr double kInjunctionThreshold = 1.0;
  // need exit bias driving in junction or junction close front.
  for (const auto& junction_overlap :
       task_info.reference_line()->junction_overlaps()) {
    if (junction_overlap.start_s >
        task_info.curr_sl().s() + data_center_->master_info().distance_to_end())
      break;
    if (task_info.curr_sl().s() +
                plan_config.bias_driving.front_junction_close_distance >
            junction_overlap.start_s &&
        task_info.curr_sl().s() <
            junction_overlap.end_s + kInjunctionThreshold) {
      is_junction_close = true;
      break;
    }
  }
  return true;
}

void BiasDrivingDecider::ComputePullOverRefL(TaskInfo& task_info,
                                             const double min_left_bound,
                                             const double min_right_bound,
                                             double bias_distance_to_bound,
                                             bool bias_to_right) {
  auto& observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;

  if (bias_to_right) {
    observe_ref_l =
        std::min((min_right_bound + VehicleParam::Instance()->width() * 0.5 +
                  bias_distance_to_bound),
                 0.0);
    LOG_INFO(
        "start to pull over to right side,"
        "observe_ref_l:{:.2f}",
        observe_ref_l);
  } else {
    observe_ref_l =
        std::max((min_left_bound - VehicleParam::Instance()->width() * 0.5 -
                  bias_distance_to_bound),
                 0.0);
    LOG_INFO(
        "start to pull over to left side, "
        "observe_ref_l:{:.2f}",
        observe_ref_l);
  }
  if (task_info.last_frame() == nullptr) return;
  double near_end_distance = 7.0;
  if (data_center_->master_info().distance_to_end() > near_end_distance) {
    filter_k_ = std::clamp(5.0 / data_center_->master_info().distance_to_end(),
                           0.1, 0.3);
    auto& last_observe_ref_l = task_info.last_frame()
                                   ->outside_planner_data()
                                   .path_observe_ref_l_info.observe_ref_l;
    observe_ref_l =
        observe_ref_l * filter_k_ + (1 - filter_k_) * last_observe_ref_l;
    LOG_INFO("curr l: {:.3f}, last l: {:.3f}, k: {:.2f} ", observe_ref_l,
             last_observe_ref_l, filter_k_);
  }
}

}  // namespace planning
}  // namespace neodrive

#include "motorway_speed_virtual_obs_decider.h"

#include "src/planning/math/frame_conversion/sl_analytic_transformation.h"
#include "src/planning/traffic_law/traffic_law_manager.h"

namespace neodrive {
namespace planning {

MotorwaySpeedVirtualObsDecider::MotorwaySpeedVirtualObsDecider() {
  name_ = "MotorwaySpeedVirtualObsDecider";
}

MotorwaySpeedVirtualObsDecider::~MotorwaySpeedVirtualObsDecider() { Reset(); }

ErrorCode MotorwaySpeedVirtualObsDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
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

bool MotorwaySpeedVirtualObsDecider::Init(TaskInfo& task_info) {
  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  if (task_info.reference_line() == nullptr ||
      task_info.reference_line()->ref_points().size() < 2) {
    LOG_ERROR("reference_line is invalid.");
    return false;
  }
  if (task_info.current_frame()->mutable_planning_data() == nullptr) {
    LOG_ERROR("mutable_planning_data == nullptr.");
    return false;
  }
  if (task_info.current_frame()
          ->mutable_planning_data()
          ->mutable_decision_data() == nullptr) {
    LOG_ERROR("mutable_decision_data == nullptr.");
    return false;
  }

  is_cruise_ = inside_planner_data.is_parking_out_slot ||
               inside_planner_data.is_parking_in_slot ||
               inside_planner_data.is_reverse_driving ||
               inside_planner_data.is_inlane_uturn ||
               inside_planner_data.is_pose_adjust ||
               inside_planner_data.is_outlane_uturn;
  return true;
}

bool MotorwaySpeedVirtualObsDecider::Process(TaskInfo& task_info) {
  if (!ReferenceLineEndVirtualObs(task_info)) {
    LOG_ERROR("RoutingDestinationVirtualObs failed.");
    return false;
  }
  if (!RoutingDestinationVirtualObs(task_info)) {
    LOG_ERROR("RoutingDestinationVirtualObs failed.");
    return false;
  }
  if (!StationModeVirtualObs(task_info)) {
    LOG_ERROR("StationVirtualObs failed.");
    return false;
  }
  if (!TrafficLawVirtualObs(task_info)) {
    LOG_ERROR("TrafficLawVirtualObs failed.");
    return false;
  }
  return true;
}

bool MotorwaySpeedVirtualObsDecider::ReferenceLineEndVirtualObs(
    TaskInfo& task_info) const {
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  const double init_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  auto destination_ref_point = task_info.current_frame()
                                   ->inside_planner_data()
                                   .reference_line_destination_point;
  if (task_info.current_frame() == nullptr ||
      task_info.reference_line() == nullptr || path.path_points().empty()) {
    return false;
  }
  if (task_info.reference_line()->ref_points().back().s() -
          path.path_points().back().s() - init_s >
      1.5) {
    LOG_INFO("reference line is long enough");
    return true;
  }
  double destination_s = path.path_points().back().s() - 0.1 + init_s;
  ReferencePoint tmp_refer_pt;
  if (!task_info.reference_line()->GetNearestRefPoint(destination_s,
                                                      &tmp_refer_pt)) {
    LOG_ERROR("GetNearestRefPoint failed");
  } else {
    destination_ref_point = tmp_refer_pt;
    LOG_INFO(
        "destination is near the end of reference line,end of "
        "refline:{:.2f},new "
        "destination:{:.2f}",
        task_info.reference_line()->ref_points().back().s(), destination_s);
  }
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  int id{0};
  if (decision_data->create_virtual_obstacle(destination_ref_point,
                                             VirtualObstacle::DESTINATION,
                                             &id) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create destination virtual_obstacle.");
    return false;
  }
  return true;
}

bool MotorwaySpeedVirtualObsDecider::RoutingDestinationVirtualObs(
    TaskInfo& task_info) const {
  const auto curr_s = task_info.curr_sl().s();
  const auto distance_to_end =
      task_info.current_frame()->inside_planner_data().distance_to_end;
  double max_speed = DataCenter::Instance()->drive_strategy_max_speed();
  if (distance_to_end >
      config::PlanningConfig::Instance()->plan_config().common.total_time *
          max_speed) {
    // far away from routing destination and don't create virtual obs
    return true;
  }
  double routing_destination_s =
      curr_s + std::fmax(0.0, distance_to_end) +
      VehicleParam::Instance()->front_edge_to_center() +
      config::PlanningConfig::Instance()
          ->plan_config()
          .common.destination_virtual_obs_buffer;
  LOG_INFO("routing_destination_s = {:.2f}", routing_destination_s);

  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  const double init_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  LOG_INFO("refline end:{:.2f}",
           task_info.reference_line()->ref_points().back().s());

  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data->create_virtual_obstacle(
          path, init_s, routing_destination_s,
          VirtualObstacle::ROUTING_DESTINATION) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create destination virtual_obstacle.");
    return false;
  }
  return true;
}

bool MotorwaySpeedVirtualObsDecider::StationModeVirtualObs(
    TaskInfo& task_info) const {
  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  bool create_flag = data_center_->master_info().is_trigger_stop() ||
                     data_center_->master_info().is_in_inner_station_mode();
  if (!create_flag) return true;
  const auto& reference_line = task_info.reference_line();
  if (task_info.reference_line() == nullptr) return false;
  auto destination_ref_point =
      inside_planner_data.reference_line_destination_point;
  LOG_INFO("original destination x:{}, y:{}", destination_ref_point.x(),
           destination_ref_point.y());

  Vec2d xy_pt;
  xy_pt.set_x(destination_ref_point.x());
  xy_pt.set_y(destination_ref_point.y());
  ReferencePoint tmp_refer_pt;
  if (!reference_line->GetNearestRefPoint(xy_pt, &tmp_refer_pt)) {
    LOG_ERROR("GetNearestRefPoint failed, use original destination pt");
  } else {
    double prefer_s = tmp_refer_pt.s();
    if (!inside_planner_data.is_reverse_driving) {
      prefer_s += VehicleParam::Instance()->front_edge_to_center();
    } else {
      prefer_s += VehicleParam::Instance()->back_edge_to_center();
    }
    if (!reference_line->GetNearestRefPoint(prefer_s, &tmp_refer_pt)) {
      LOG_ERROR("GetNearestRefPoint failed, use original destination pt");
    } else {
      destination_ref_point = tmp_refer_pt;
    }
  }

  LOG_INFO("create destination x:{}, y:{}", destination_ref_point.x(),
           destination_ref_point.y());

  int id{0};
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data->create_virtual_obstacle(destination_ref_point,
                                             VirtualObstacle::DESTINATION,
                                             &id) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create destination virtual_obstacle.");
  }

  return true;
}

bool MotorwaySpeedVirtualObsDecider::TrafficLawVirtualObs(
    TaskInfo& task_info) const {
  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  bool apply_traffic_law = true;
  if (inside_planner_data.is_parking_out_slot ||
      inside_planner_data.is_parking_in_slot ||
      inside_planner_data.is_reverse_driving ||
      inside_planner_data.is_inlane_uturn ||
      inside_planner_data.is_pose_adjust ||
      inside_planner_data.is_outlane_uturn) {
    apply_traffic_law = false;
    LOG_INFO("donot apply traffic law");
  }
  if (apply_traffic_law) {
    TrafficLawManager traffic_law_manager;
    const auto& outside_planner_data =
        task_info.current_frame()->outside_planner_data();
    if (traffic_law_manager.ApplyLaw(&task_info, inside_planner_data,
                                     outside_planner_data) != 0) {
      LOG_ERROR("Failed to apply traffic law.");
      return false;
    }
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive

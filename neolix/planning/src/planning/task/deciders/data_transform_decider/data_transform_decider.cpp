#include "data_transform_decider.h"

#include "common/math/util.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/em_planning_data.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/common/euler_angles_zxy.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

using neodrive::global::planning::ScenarioState;

DataTransformDecider::DataTransformDecider() { name_ = "DataTransformDecider"; }

DataTransformDecider::~DataTransformDecider() { Reset(); }

ErrorCode DataTransformDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!Init(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool DataTransformDecider::Init(TaskInfo& task_info) {
  if (task_info.current_frame().get() == nullptr) {
    LOG_ERROR("current_frame.get() == nullptr.");
    return false;
  }
  if (task_info.current_frame()->planning_data().type() != "EMPlanningData") {
    LOG_ERROR("planning_data_type != EMPlanningData.");
    return false;
  }
  return true;
}

bool DataTransformDecider::Process(TaskInfo& task_info) const {
  // 1. behavior decision flags: master_info -> inside_planner_data
  BehaviorInfoTransform(task_info);
  // 2. vehicle state
  VehicleStateTransform(task_info);
  // 3. plannng data
  if (!PlanningDataTransform(task_info)) {
    LOG_ERROR("PlanningDataTransform failed.");
    return false;
  }
  // 4. Protection data
  ProtectionDataTransform(task_info);

  // 6. lms safty check
  if (FLAGS_planning_enable_lms &&
      task_info.current_frame()->inside_planner_data().vel_v <=
          FLAGS_planning_enable_lms_speed_threshold) {
    LmsSafetyCheck(task_info);
  }

  return true;
}

void DataTransformDecider::BehaviorInfoTransform(TaskInfo& task_info) const {
  // 1. bias driving
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  auto inside_planner_data_ptr =
      task_info.current_frame()->mutable_inside_planner_data();
  inside_planner_data_ptr->is_bias_driving =
      data_center_->master_info().need_bias_driving();
  // inside_planner_data_ptr->bias_value =
  //     data_center_->master_info().bias_driving_val();
  // TEST
  LOG_INFO("bias driving: {}",
           task_info.current_frame()->inside_planner_data().is_bias_driving);

  // 2. distance to end
  inside_planner_data_ptr->distance_to_end =
      data_center_->master_info().distance_to_end();
  // TEST
  LOG_INFO("distance to end: {:.3f}",
           task_info.current_frame()->inside_planner_data().distance_to_end);

  inside_planner_data_ptr->is_parking_in_slot =
      data_center_->global_state_proxy().is_parking_in();
  inside_planner_data_ptr->is_parking_out_slot =
      data_center_->global_state_proxy().is_parking_out();

  // 9. lane borrow
  inside_planner_data_ptr->is_lane_borrowing =
      data_center_->master_info().curr_scenario() == ScenarioState::DETOUR ||
              data_center_->master_info().curr_scenario() ==
                  ScenarioState::MOTORWAY_DETOUR
          ? true
          : false;

  // reverse lane detour
  outside_planner_data_ptr->consider_reverse_lane_detour_obs = false;
  auto& adc_position =
      data_center_->master_info().reverse_lane_detour_context().adc_position;
  bool is_considered_adc_position =
      adc_position ==
          ReverseLaneDetourContext::AdcPosition::LEFTMOST_LANE_COMPLETE ||
      adc_position == ReverseLaneDetourContext::AdcPosition::DIVIDER_CROSSING ||
      adc_position ==
          ReverseLaneDetourContext::AdcPosition::REVERSE_LANE_COMPLETE;
  outside_planner_data_ptr->consider_reverse_lane_detour_obs =
      inside_planner_data_ptr->is_lane_borrowing && is_considered_adc_position;
  LOG_INFO(
      "consider_reverse_lane_detour_obs:{}, is_lane_borrowing:{}, "
      "adc_position:{}, is_considered_adc_position:{}",
      outside_planner_data_ptr->consider_reverse_lane_detour_obs,
      inside_planner_data_ptr->is_lane_borrowing,
      static_cast<int>(adc_position), is_considered_adc_position);

  if (true == task_info.current_frame()
                  ->mutable_inside_planner_data()
                  ->is_lane_borrowing) {
    task_info.current_frame()
        ->mutable_inside_planner_data()
        ->is_prepare_borrowing =
        data_center_->master_info().lane_borrow_context().stage ==
                    DetourStageState::PREPARE ||
                data_center_->master_info()
                        .motorway_lane_borrow_context()
                        .stage == MotorwayDetourStageState::PREPARE
            ? true
            : false;
  } else {
    // default fail.
    task_info.current_frame()
        ->mutable_inside_planner_data()
        ->is_prepare_borrowing = false;
  }
  if (task_info.current_frame()->inside_planner_data().is_lane_borrowing) {
    inside_planner_data_ptr->lane_borrow_side =
        data_center_->master_info().lane_borrow_context().borrow_side;
  }

  // TEST
  LOG_INFO(
      "lane borrow[in, prepare]: {}, {}",
      task_info.current_frame()->inside_planner_data().is_lane_borrowing,
      task_info.current_frame()->inside_planner_data().is_prepare_borrowing);

  // // 10. behavior stop
  // task_info.current_frame()->mutable_outside_planner_data()->speed_slow_down
  // =
  //     data_center_->master_info().behavior_stop_vehicle();

  // 11. multi level mode
  inside_planner_data_ptr->curr_multi_level =
      task_info.current_multi_level_mode();
  // TEST
  LOG_INFO("multi level mode: {}",
           task_info.current_frame()->inside_planner_data().curr_multi_level);

  // 12. pass by scenario
  inside_planner_data_ptr->curr_pass_by_mode = task_info.current_pass_by_mode();
  // TEST
  LOG_INFO("pass by scenario: {}",
           task_info.current_frame()->inside_planner_data().curr_pass_by_mode);

  // 13. destination point
  task_info.current_frame()
      ->mutable_inside_planner_data()
      ->reference_line_destination_point =
      data_center_->master_info().reference_line_destination_point();
  LOG_INFO("desitination_point: {:.3f}, {:.3f}",
           task_info.current_frame()
               ->inside_planner_data()
               .reference_line_destination_point.x(),
           task_info.current_frame()
               ->inside_planner_data()
               .reference_line_destination_point.y());
  // 15. coordinate type
  inside_planner_data_ptr->trajectory_mode = CoordinateType::ODOMETRY;

  // 16. indoor
  ReferencePoint pt{};
  task_info.reference_line()->GetNearestRefPoint(task_info.curr_sl().s(), &pt);
  inside_planner_data_ptr->is_indoor = pt.lane_type_is_indoor_lane();
  // 17. in the park
  inside_planner_data_ptr->is_in_the_park = data_center_->is_in_port_odd();
  if (inside_planner_data_ptr->is_in_the_park) LOG_INFO("in the park");
}

void DataTransformDecider::VehicleStateTransform(TaskInfo& task_info) const {
  const auto& veh_state = DataCenter::Instance()->vehicle_state_proxy();
  auto inside_planner_data_ptr =
      task_info.current_frame()->mutable_inside_planner_data();

  inside_planner_data_ptr->vel_x = veh_state.X();
  inside_planner_data_ptr->vel_y = veh_state.Y();
  inside_planner_data_ptr->vel_heading = normalize_angle(veh_state.Heading());
  inside_planner_data_ptr->vel_v = veh_state.LinearVelocity();
  inside_planner_data_ptr->vel_a = veh_state.LinearAcceleration();
  inside_planner_data_ptr->vel_steer_angle = veh_state.SteerPercent();
  inside_planner_data_ptr->vel_pitch_angle = veh_state.Pitch();
  // TEST
  LOG_INFO(
      "Vehicle State[x, y, heading, v, a, steer_angle, pitch]: {:.3f}, {:.3f}, "
      "{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
      task_info.current_frame()->inside_planner_data().vel_x,
      task_info.current_frame()->inside_planner_data().vel_y,
      task_info.current_frame()->inside_planner_data().vel_heading,
      task_info.current_frame()->inside_planner_data().vel_v,
      task_info.current_frame()->inside_planner_data().vel_a,
      task_info.current_frame()->inside_planner_data().vel_steer_angle,
      task_info.current_frame()->inside_planner_data().vel_pitch_angle);
}

bool DataTransformDecider::PlanningDataTransform(TaskInfo& task_info) const {
  auto inside_planner_data_ptr =
      task_info.current_frame()->mutable_inside_planner_data();
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  const auto& reference_line = task_info.reference_line();

  // 1. init point
  auto& init_point = inside_planner_data_ptr->init_point;
  init_point = task_info.current_frame()->planning_data().init_planning_point();
  LOG_INFO(
      "init_point[x, y, theta, kappa, v, a, j]: {:.3f}, {:.3f}, {:.3f}, "
      "{:.3f}, "
      "{:.3f}, {:.3f}, {:.3f}",
      init_point.x(), init_point.y(), init_point.theta(), init_point.kappa(),
      init_point.velocity(), init_point.acceleration(), init_point.jerk());

  // 2. init_sl_point
  auto& init_sl_point = inside_planner_data_ptr->init_sl_point;
  if (!reference_line->GetPointInFrenetFrameWithHeading(
          init_point.coordinate(), init_point.theta(), &init_sl_point)) {
    LOG_ERROR(
        "get current point in frenet coor failed curr x: {:.4f}, curr y: "
        "{:.4f}, "
        "ref_points size: {}",
        init_point.coordinate().x(), init_point.coordinate().y(),
        reference_line->ref_points().size());
    return false;
  }
  LOG_INFO("init_sl_point[s, l]: {:.3f}, {:.3f}", init_sl_point.s(),
           init_sl_point.l());

  // 3. frenet_init_point
  auto& frenet_init_point = outside_planner_data_ptr->frenet_init_point;
  if (!reference_line->GetPointInFrenetFrameWithHeading(
          {init_point.x(), init_point.y()}, init_point.theta(),
          &frenet_init_point)) {
    LOG_ERROR("Fail to map init point to sl coordinate.");
    return false;
  }
  LOG_INFO("frenet_init_point: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
           frenet_init_point.s(), frenet_init_point.l(), frenet_init_point.dl(),
           frenet_init_point.ddl());

  // 4. init_point_ref_point
  auto& init_point_ref_point = outside_planner_data_ptr->init_point_ref_point;
  if (!reference_line->GetNearestRefPoint(frenet_init_point.s(),
                                          &init_point_ref_point)) {
    LOG_ERROR("GetNearestRefPoint fail.");
    return false;
  }

  // 5. frenet_veh_point and frenet_veh_point_ref_point
  auto& frenet_veh_point = outside_planner_data_ptr->frenet_veh_real_point;
  if (!reference_line->GetPointInFrenetFrameWithHeading(
          {inside_planner_data_ptr->vel_x, inside_planner_data_ptr->vel_y},
          inside_planner_data_ptr->vel_heading, &frenet_veh_point)) {
    LOG_ERROR("Fail to map init point to sl coordinate.");
    return false;
  }
  auto& veh_reference_point =
      outside_planner_data_ptr->veh_real_reference_point;
  if (!reference_line->GetNearestRefPoint(frenet_veh_point.s(),
                                          &veh_reference_point)) {
    LOG_ERROR("GetNearestRefPoint fail ");
    return false;
  }

  // 6. history data
  const EMPlanningData* last_planning_data{nullptr};
  if (task_info.last_frame() != nullptr) {
    task_info.current_frame()
        ->mutable_inside_planner_data()
        ->last_computed_trajectory =
        task_info.last_frame()->planning_data().computed_trajectory();
    task_info.current_frame()->mutable_outside_planner_data()->path_context =
        task_info.last_frame()->outside_planner_data().path_context;
    last_planning_data = dynamic_cast<const EMPlanningData*>(
        &(task_info.last_frame()->planning_data()));
  }
  if (task_info.last_frame() != nullptr && last_planning_data != nullptr) {
    if (last_planning_data->path_data().path().path_points().size() > 2) {
      inside_planner_data_ptr->last_path_data = last_planning_data->path_data();
    }
    if (last_planning_data->speed_data().speed_vector().size() > 2) {
      task_info.current_frame()
          ->mutable_inside_planner_data()
          ->last_speed_data = last_planning_data->speed_data();
    }
  }

  // 7. path data, speed data
  auto planning_data = dynamic_cast<EMPlanningData*>(
      task_info.current_frame()->mutable_planning_data());
  task_info.current_frame()->mutable_outside_planner_data()->path_data =
      planning_data->mutable_path_data();
  task_info.current_frame()->mutable_outside_planner_data()->speed_data =
      planning_data->mutable_speed_data();
  return true;
}

void DataTransformDecider::ProtectionDataTransform(TaskInfo& task_info) const {
  const double control_steer_cmd =
      DataCenter::Instance()->control_command_msg.ptr->steering_target();
  const double canbus_steer_cmd =
      DataCenter::Instance()->vehicle_state_proxy().SteerPercent();
  auto inside_planner_data_ptr =
      task_info.current_frame()->mutable_inside_planner_data();
  if (std::fabs(control_steer_cmd - canbus_steer_cmd) > 30.0) {
    LOG_INFO(
        "control steer cmd [{:.3f}] and canbus resp [{:.3f}] differs a lot, "
        "set "
        "speed_limit to 1",
        control_steer_cmd, canbus_steer_cmd);

    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::CANBUS_ERROR);
    internal_speed_limit.add_upper_bounds(1.0);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "CANBUS_ERROR {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        1.0, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }

  double control_lateral_err{0.};
  double control_heading_err{0.};
  const auto& control_feed_back =
      *(DataCenter::Instance()->control_command_msg.ptr);
  if (control_feed_back.has_contrl_context()) {
    const auto& control_context = control_feed_back.contrl_context();
    if (control_context.has_lat_ctrl_ctx() &&
        control_context.lat_ctrl_ctx().has_lateral_error() &&
        control_context.lat_ctrl_ctx().has_heading_error()) {
      control_lateral_err = control_context.lat_ctrl_ctx().lateral_error();
      control_heading_err = control_context.lat_ctrl_ctx().heading_error();
    }
  }
  inside_planner_data_ptr->control_lat_error = control_lateral_err;
  inside_planner_data_ptr->control_head_error = control_heading_err;
  if (std::abs(control_lateral_err) >= 0.4 ||
      std::abs(control_heading_err) >= 0.35) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::CONTROL_ERROR);
    internal_speed_limit.add_upper_bounds(std::max(
        DataCenter::Instance()->vehicle_state_proxy().LinearVelocity() - 1.5,
        2.0));
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "CONTROL_ERROR {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        1.0, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

  } else if (std::abs(control_lateral_err) >= 0.25 ||
             std::abs(control_heading_err) >= 0.17) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::CONTROL_ERROR);
    internal_speed_limit.add_upper_bounds(std::max(
        DataCenter::Instance()->vehicle_state_proxy().LinearVelocity() - 1.0,
        3.0));
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "CANBUS_ERROR {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        2.0, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

bool DataTransformDecider::LmsSafetyCheck(TaskInfo& task_info) const {
  const auto& lms_state =
      *(DataCenter::Instance()->lslidar_perception_obstacles_msg.ptr);
  double curr_time = common::util::TimeLogger::GetCurrentTimeseocnd();
  double delta_time = curr_time - lms_state.header().timestamp_sec();
  if (delta_time > 3.0) {
    LOG_INFO("lms time differ larger than 3s, skip it.");
    return true;
  }
  auto inside_planner_data_ptr =
      task_info.current_frame()->mutable_inside_planner_data();
  inside_planner_data_ptr->lms_pts.clear();
  // coordinate transform
  Vec2d tmp_pt;
  double shift_x = VehicleParam::Instance()->front_edge_to_center() - 0.252;
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  double curr_x = task_info.current_frame()->inside_planner_data().vel_x;
  double curr_y = task_info.current_frame()->inside_planner_data().vel_y;
  double curr_heading =
      task_info.current_frame()->inside_planner_data().vel_heading;

  size_t valid_size = 0;
  for (int i = 0; i < lms_state.perception_obstacle().size(); ++i) {
    auto& pt = lms_state.perception_obstacle()[i];
    tmp_pt.set_x(pt.position().y());
    tmp_pt.set_y(pt.position().x());
    // filter
    if (tmp_pt.x() > FLAGS_planning_lms_horizontal_range ||
        fabs(tmp_pt.y()) > FLAGS_planning_lms_lateral_range)
      continue;
    // transform
    tmp_pt.set_x(tmp_pt.x() - shift_x);

    vehicle2earth(curr_x, curr_y, curr_heading, tmp_pt.x(), tmp_pt.y(), 0.0,
                  tmp_x, tmp_y, tmp_theta);

    tmp_pt.set_x(tmp_x);
    tmp_pt.set_y(tmp_y);
    inside_planner_data_ptr->lms_pts.push_back(tmp_pt);
    valid_size++;
  }
  LOG_INFO("valid 1d lms size: {}", valid_size);

  return true;
}

}  // namespace planning
}  // namespace neodrive

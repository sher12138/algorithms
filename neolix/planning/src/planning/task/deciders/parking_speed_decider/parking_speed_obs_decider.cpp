#include "parking_speed_obs_decider.h"

namespace neodrive {
namespace planning {

ParkingSpeedObsDecider::ParkingSpeedObsDecider() {
  name_ = "ParkingSpeedObsDecider";
}

void ParkingSpeedObsDecider::Reset() {}

ParkingSpeedObsDecider::~ParkingSpeedObsDecider() { Reset(); }

void ParkingSpeedObsDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode ParkingSpeedObsDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  auto outside_ptr = frame->mutable_outside_planner_data();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (frame->outside_planner_data().path_data == nullptr) {
    LOG_ERROR("path_data == nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  bool collide{false};

  if (data_center_->parking_ptr() &&
      data_center_->parking_ptr()->is_park_in()) {
    SetIgnoreObsList(task_info);
  }

  if (data_center_->parking_ptr() &&
      !data_center_->parking_ptr()->is_park_in()) {
    SetIgnoreObsListForParkingOut(task_info);
  }

  if (parking_config.steering_detect) {
    SteeringDetect(task_info, collide);
  }

  if (!collide && parking_config.path_detect) {
    PathDetect(task_info, collide);
  }

  if (!collide && parking_config.heading_detect) {
    HeadingDetect(task_info, collide);
  }

  if (collide) {
    outside_ptr->parking_outside_data.parking_speed_cmd = 0.;
    outside_ptr->speed_slow_down = true;
  }

  return ErrorCode::PLANNING_OK;
}

void ParkingSpeedObsDecider::SteeringDetect(TaskInfo& task_info,
                                            bool& collide) const {
  auto& frame = task_info.current_frame();
  const auto& inside_data = frame->inside_planner_data();
  std::vector<Polygon2d> ego_polygon_vec;
  const double ego_pre_dis{
      config::PlanningConfig::Instance()->plan_config().parking.prediction_dis};
  double steering = inside_data.vel_steer_angle / 100. *
                    VehicleParam::Instance()->max_steer_angle() /
                    VehicleParam::Instance()->steer_ratio();
  LOG_INFO("steering is {:.2f}", steering);
  for (double s = 0.; s < ego_pre_dis; s += 0.1) {
    Vec2d ego_pos{inside_data.vel_x, inside_data.vel_y};
    ego_pos += Vec2d{s * std::cos(steering), s * std::sin(steering)};
    Polygon2d ego_polygon = VehicleParam::Instance()->get_adc_polygon(
        ego_pos, steering, 0.0, 0.0, 0.0);
    ego_polygon_vec.emplace_back(ego_polygon);
  }

  const auto& obs_vec = frame->planning_data().decision_data().all_obstacle();
  for (auto obs_ptr : obs_vec) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual()) {
      continue;
    }
    for (const auto& ego_polygon : ego_polygon_vec) {
      if (ego_polygon.has_overlap(obs_ptr->polygon())) {
        collide = true;
        LOG_INFO("obs {} detected collide with ego", obs_ptr->id());
        break;
      }
    }
    if (collide) break;
  }
}

void ParkingSpeedObsDecider::PathDetect(TaskInfo& task_info,
                                        bool& collide) const {
  auto& frame = task_info.current_frame();
  const auto& inside_data = frame->inside_planner_data();
  if (frame->outside_planner_data().path_data == nullptr ||
      frame->outside_planner_data().path_data->path().path_points().empty()) {
    return;
  }
  const auto& path_points =
      frame->outside_planner_data().path_data->path().path_points();
  std::vector<Polygon2d> ego_polygon_vec;
  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  double ego_pre_dis{parking_config.prediction_dis};
  double lateral_buffer{parking_config.lateral_buffer};
  if (data_center_->parking_ptr() &&
      data_center_->parking_ptr()->OriginPark() &&
      data_center_->parking_ptr()->OriginPark()->Type() ==
          global::hdmap::ParkingSpace_ParkingType_VERTICAL &&
      data_center_->master_info().drive_direction() ==
          MasterInfo::DriveDirection::DRIVE_BACKWARD) {
    ego_pre_dis = parking_config.vertical_parking_space.prediction_dis;
    lateral_buffer = parking_config.vertical_parking_space.lateral_buffer;
  }
  if (data_center_->master_info().curr_stage() == "FINISH") {
    lateral_buffer = 0.;
  }
  for (const auto& point : path_points) {
    if (point.s() > ego_pre_dis) break;
    Polygon2d ego_polygon = VehicleParam::Instance()->get_adc_polygon(
        point, point.theta(), lateral_buffer, 0.0, 0.0);
    ego_polygon_vec.emplace_back(ego_polygon);
  }
  const auto& ignore_obs_set =
      frame->outside_planner_data()
          .speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  const auto& obs_vec = frame->planning_data().decision_data().all_obstacle();
  for (auto obs_ptr : obs_vec) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual()) {
      continue;
    }
    if (ignore_obs_set.count(obs_ptr->id())) {
      continue;
    }
    if (obs_ptr->sub_type() == PerceptionObstacle::ST_TRAFFICCONE) {
      continue;
    }
    for (const auto& ego_polygon : ego_polygon_vec) {
      if (ego_polygon.has_overlap(obs_ptr->polygon())) {
        collide = true;
        LOG_INFO("obs {} detected collide with ego", obs_ptr->id());
        break;
      }
    }
    if (collide) break;
  }
}

void ParkingSpeedObsDecider::SetIgnoreObsList(TaskInfo& task_info) {
  auto& frame = task_info.current_frame();
  const auto& inside_data = frame->inside_planner_data();
  auto outside_data = frame->mutable_outside_planner_data();
  auto& ignore_obs_set =
      outside_data->speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  if (!data_center_->parking_ptr() ||
      !data_center_->parking_ptr()->OriginPark() ||
      !data_center_->parking_ptr()->is_park_in()) {
    return;
  }
  if (data_center_->parking_ptr()->OriginPark()->Type() ==
      global::hdmap::ParkingSpace_ParkingType_HORIZONTAL) {
    const auto& obs_vec = frame->planning_data().decision_data().all_obstacle();
    const auto& points = data_center_->parking_ptr()->OriginPark()->Points();
    Vec3d origin_utm{points[0].x, points[0].y,
                     data_center_->parking_ptr()->Heading()};
    const double vertical_buffer{
        parking_config.horizontal_parking_space.parking_space_vertical_buffer};
    for (auto obs_ptr : obs_vec) {
      bool ignore{true};
      for (const auto& corner : obs_ptr->polygon_corners()) {
        Vec3d corner_odom{corner.x(), corner.y(), 0.};
        Vec3d corner_veh, corner_utm, corner_rel;
        common::ConvertToRelativeCoordinate(
            corner_odom,
            {vehicle_state_.X(), vehicle_state_.Y(), vehicle_state_.Heading()},
            corner_veh);
        common::ConvertToWorldCoordinate(
            corner_veh,
            {data_center_->vehicle_state_utm().X(),
             data_center_->vehicle_state_utm().Y(),
             data_center_->vehicle_state_utm().Heading()},
            corner_utm);
        common::ConvertToRelativeCoordinate(corner_utm, origin_utm, corner_rel);
        if (corner_rel.x() < -data_center_->parking_ptr()->Width()) {
          LOG_INFO("Ignore obs {} by parking filter", obs_ptr->id());
          ignore_obs_set.insert(obs_ptr->id());
          break;
        }
        if (corner_rel.x() <= 0. && corner_rel.y() <= 0. + vertical_buffer &&
            corner_rel.y() >=
                -data_center_->parking_ptr()->Length() - vertical_buffer) {
          LOG_INFO("not ignore obs x = {:.2f}, y = {:.2f}", corner_rel.x(),
                   corner_rel.y());
          ignore = false;
          break;
        }
        if (corner_rel.x() >= 0.) {
          ignore = false;
          break;
        }
      }
      if (ignore) {
        LOG_INFO("Ignore obs {} by parking filter", obs_ptr->id());
        ignore_obs_set.insert(obs_ptr->id());
      }
    }
  } else if (data_center_->parking_ptr()->OriginPark()->Type() ==
             global::hdmap::ParkingSpace_ParkingType_VERTICAL) {
    const auto& obs_vec = frame->planning_data().decision_data().all_obstacle();
    const auto& points = data_center_->parking_ptr()->OriginPark()->Points();
    const auto& vertical_config = config::PlanningConfig::Instance()
                                      ->plan_config()
                                      .parking.vertical_parking_space;
    Vec3d origin_utm{points[1].x, points[1].y,
                     data_center_->parking_ptr()->Heading()};
    for (auto obs_ptr : obs_vec) {
      bool other_parking_space{true}, behind_parking_space{true};
      for (const auto& corner : obs_ptr->polygon_corners()) {
        Vec3d corner_odom{corner.x(), corner.y(), 0.};
        Vec3d corner_veh, corner_utm, corner_rel;
        common::ConvertToRelativeCoordinate(
            corner_odom,
            {vehicle_state_.X(), vehicle_state_.Y(), vehicle_state_.Heading()},
            corner_veh);
        common::ConvertToWorldCoordinate(
            corner_veh,
            {data_center_->vehicle_state_utm().X(),
             data_center_->vehicle_state_utm().Y(),
             data_center_->vehicle_state_utm().Heading()},
            corner_utm);
        common::ConvertToRelativeCoordinate(corner_utm, origin_utm, corner_rel);
        const double lateral_buffer{
            vertical_config.parking_space_lateral_buffer};
        const double vertical_buffer{
            vertical_config.parking_space_vertical_buffer};
        if (corner_rel.y() >= 0. + lateral_buffer &&
            corner_rel.y() <=
                data_center_->parking_ptr()->Width() - lateral_buffer &&
            corner_rel.x() <= 0. + vertical_buffer) {
          LOG_INFO("not ignore obs x = {:.2f}, y = {:.2f}", corner_rel.x(),
                   corner_rel.y());
          other_parking_space = false;
        }
        if (corner_rel.x() >= 0. + vertical_buffer) {
          other_parking_space = false;
        }
        if (corner_rel.x() >= -data_center_->parking_ptr()->Length()) {
          behind_parking_space = false;
        }
        if (!other_parking_space && !behind_parking_space) break;
      }
      if (other_parking_space || behind_parking_space) {
        LOG_INFO("Ignore obs {} by parking filter", obs_ptr->id());
        ignore_obs_set.insert(obs_ptr->id());
      }
    }
  }
  // camera freespace filter
  if (parking_config.enable_camera_freespace &&
      data_center_->master_info().drive_direction() ==
          MasterInfo::DriveDirection::DRIVE_BACKWARD) {
    const auto& obs_vec = frame->planning_data().decision_data().all_obstacle();
    const auto ego_polygon = VehicleParam::Instance()->get_adc_polygon(
        {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.0,
        0.0, 0.0);
    for (auto* obs_ptr : obs_vec) {
      if (obs_ptr != nullptr && obs_ptr->frame_cnt() == 1 &&
          obs_ptr->polygon().distance_to(ego_polygon) <
              parking_config.camera_obs_filter_dis) {
        camera_ignore_obs_.insert(obs_ptr->id());
      }
    }
    for (int id : camera_ignore_obs_) {
      auto it = std::find_if(obs_vec.begin(), obs_vec.end(),
                             [id](const Obstacle* obs_ptr) {
                               return obs_ptr != nullptr && obs_ptr->id() == id;
                             });
      if (it == obs_vec.end()) {
        camera_ignore_obs_.erase(id);
      } else {
        LOG_INFO("Ignore obs {} by camera freespace filter", id);
        ignore_obs_set.insert(id);
      }
    }
  }
}

void ParkingSpeedObsDecider::SetIgnoreObsListForParkingOut(
    TaskInfo& task_info) const {
  if (!data_center_->parking_ptr() ||
      !data_center_->parking_ptr()->OriginPark() ||
      data_center_->parking_ptr()->OriginPark()->Type() !=
          global::hdmap::ParkingSpace_ParkingType_HORIZONTAL) {
    return;
  }
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  auto& ignore_obs_set =
      outside_data->speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  auto lane_ptr = data_center_->parking_ptr()->OverlapLane();
  common::math::Vec2d ego_utm{data_center_->vehicle_state_utm().X(),
                              data_center_->vehicle_state_utm().Y()};
  if (!lane_ptr->IsOnLane(ego_utm)) {
    LOG_INFO("Ego is not in parking overlap lane");
    return;
  }
  double ego_lane_s{0.}, ego_lane_l{0.};
  lane_ptr->GetProjection(ego_utm, &ego_lane_s, &ego_lane_l);
  const auto& obs_vec =
      task_info.current_frame()->planning_data().decision_data().all_obstacle();
  const auto& points = data_center_->parking_ptr()->OriginPark()->Points();
  Vec3d origin_utm{points[0].x, points[0].y,
                   data_center_->parking_ptr()->Heading()};
  const double ignore_dis{
      config::PlanningConfig::Instance()
          ->plan_config()
          .parking.horizontal_parking_space.lane_ignore_dis_threshold};
  for (auto obs_ptr : obs_vec) {
    bool ignore{true};
    for (const auto& corner : obs_ptr->polygon_corners()) {
      Vec3d corner_odom{corner.x(), corner.y(), 0.};
      Vec3d corner_veh, corner_utm, corner_rel;
      common::ConvertToRelativeCoordinate(
          corner_odom,
          {vehicle_state_.X(), vehicle_state_.Y(), vehicle_state_.Heading()},
          corner_veh);
      common::ConvertToWorldCoordinate(
          corner_veh,
          {data_center_->vehicle_state_utm().X(),
           data_center_->vehicle_state_utm().Y(),
           data_center_->vehicle_state_utm().Heading()},
          corner_utm);
      common::ConvertToRelativeCoordinate(corner_utm, origin_utm, corner_rel);
      if (corner_rel.x() < lane_ptr->GetWidth(ego_lane_s) - ignore_dis) {
        LOG_INFO("obs {} corner_rel.x() = {:.2f}, lane width = {:.2f}",
                 obs_ptr->id(), corner_rel.x(), lane_ptr->GetWidth(ego_lane_s));
        ignore = false;
        break;
      }
    }
    if (ignore) {
      LOG_INFO("Ignore obs {} by parking filter", obs_ptr->id());
      ignore_obs_set.insert(obs_ptr->id());
    }
  }
}

void ParkingSpeedObsDecider::HeadingDetect(TaskInfo& task_info,
                                           bool& collide) const {
  const auto& obs_vec =
      task_info.current_frame()->planning_data().decision_data().all_obstacle();
  const auto& ignore_obs_set =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  double ego_pre_dis{parking_config.heading_detect_pre_dis};
  std::vector<Polygon2d> ego_polygon_vec;
  for (double d = 0.; d < ego_pre_dis; d += 0.05) {
    Vec2d point{vehicle_state_.X() + d * std::cos(vehicle_state_.Heading()),
                vehicle_state_.Y() + d * std::sin(vehicle_state_.Heading())};
    Polygon2d ego_polygon = VehicleParam::Instance()->get_adc_polygon(
        point, vehicle_state_.Heading(), 0., 0., 0.);
    ego_polygon_vec.emplace_back(ego_polygon);
  }

  for (auto obs_ptr : obs_vec) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual()) {
      continue;
    }
    // if (ignore_obs_set.count(obs_ptr->id())) {
    //   continue;
    // }
    if (obs_ptr->sub_type() == PerceptionObstacle::ST_TRAFFICCONE) {
      continue;
    }
    for (const auto& ego_polygon : ego_polygon_vec) {
      if (ego_polygon.has_overlap(obs_ptr->polygon())) {
        collide = true;
        LOG_INFO("obs {} detected collide with ego", obs_ptr->id());
        break;
      }
    }
    if (collide) break;
  }
}

bool ParkingSpeedObsDecider::GetCameraPoints(
    const global::perception::SingleCameraSegmentation& camera_segments,
    std::vector<Vec2d>& camera_points,
    std::vector<std::vector<Vec2d>>& holes) const {
  if (camera_segments.segments().empty()) return false;
  auto& pose =
      data_center_->environment().perception_proxy().Perception().odom_pose();
  auto& loc = pose.position();
  auto& q = pose.orientation();
  const double theta =
      std::atan2(2. * (q.qw() * q.qz() + q.qx() * q.qy()),
                 1. - 2. * (q.qy() * q.qy() + q.qz() * q.qz()));
  // for test
  LOG_INFO(
      "enviroment odom x = {:.2f}, y = {:.2f}, theta = {:.2f}, datacenter odom "
      "x = {:.2f}, y = {:.2f}, theta = {:.2f}",
      loc.x(), loc.y(), theta, vehicle_state_.X(), vehicle_state_.Y(),
      vehicle_state_.Heading());
  const double st{std::sin(theta)}, ct{std::cos(theta)};
  for (auto& segment : camera_segments.segments()) {
    for (auto& point : segment.bev_points()) {
      double x_odom{point.x() * ct - point.y() * st + loc.x()};
      double y_odom{point.x() * st + point.y() * ct + loc.y()};
      camera_points.emplace_back(x_odom, y_odom);
    }
    for (auto& hole : segment.holes()) {
      std::vector<Vec2d> hole_points;
      for (auto& point : hole.hole_points()) {
        double x_odom{point.x() * ct - point.y() * st + loc.x()};
        double y_odom{point.x() * st + point.y() * ct + loc.y()};
        hole_points.emplace_back(x_odom, y_odom);
      }
      holes.emplace_back(hole_points);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive

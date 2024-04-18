#include "parking_speed_dynamic_obs_decider.h"

#include "reference_line/reference_line_util.h"
#include "src/planning/common/math/math_utils.h"

namespace neodrive {
namespace planning {

ParkingSpeedDynamicObsDecider::ParkingSpeedDynamicObsDecider() {
  name_ = "ParkingSpeedDynamicObsDecider";
}

void ParkingSpeedDynamicObsDecider::Reset() {}

ParkingSpeedDynamicObsDecider::~ParkingSpeedDynamicObsDecider() { Reset(); }

void ParkingSpeedDynamicObsDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode ParkingSpeedDynamicObsDecider::Execute(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  auto& path_accumulated_s =
      outside_planner_data_ptr->speed_obstacle_context.path_accumulated_s;
  if (!ComputePathAccumulatedS(
          *task_info.current_frame()->outside_planner_data().path_data,
          &path_accumulated_s)) {
    LOG_ERROR("ComputePathAccumulatedS failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  auto& path_adc_boxes =
      outside_planner_data_ptr->speed_obstacle_context.adc_boundaries;
  auto& path_adc_sl_boundaries =
      outside_planner_data_ptr->speed_obstacle_context.adc_sl_boundaries;

  if (!BuildPathAdcBoundingBoxesAndAABoxes(
          task_info, task_info.current_frame()->inside_planner_data(),
          task_info.reference_line(), *(outside_planner_data_ptr->path_data),
          &path_adc_boxes, &path_adc_sl_boundaries)) {
    LOG_ERROR("BuilPathAdcBoundingBoxes failed!");
    return ErrorCode::PLANNING_ERROR_FAILED;
  };
  // 2. path_adc_pre_boxes
  auto& path_adc_pre_boxes =
      outside_planner_data_ptr->speed_obstacle_context.adc_pre_boundaries;
  auto& pre_path_points =
      outside_planner_data_ptr->speed_obstacle_context.pre_path_points;
  if (!BuildPathAdcPreBoundingBoxes(
          task_info.current_frame()->inside_planner_data(),
          *(outside_planner_data_ptr->path_data), &path_adc_pre_boxes,
          &pre_path_points)) {
    LOG_WARN("BuildPathAdcPreBoundingBoxes failed.");
  }
  if (!DynamicObstaclePreDecision(task_info)) {
    LOG_ERROR("DynamicObstaclePreDecision failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool ParkingSpeedDynamicObsDecider::DynamicObstaclePreDecision(
    TaskInfo& task_info) const {
  const auto reference_line = task_info.reference_line();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const double curr_s{task_info.curr_sl().s()};
  if (outside_data == nullptr) {
    return false;
  }

  LOG_INFO("___dynamic_obstacle_pre_decision infos___:");
  for (auto obs_ptr : dynamic_obs_vec) {
    if (obs_ptr == nullptr) continue;
    bool ignore{false};
    IsDynamicObsNeedIgnore(inside_data, *obs_ptr, ignore);
    LOG_INFO("Dynamic obs id: {}, start collision check", obs_ptr->id());
    if (!CollisionCheckObstacleWithTrajectory(task_info, *obs_ptr)) {
      LOG_ERROR("Dynamic obs [{}] collision check failed.", obs_ptr->id());
      return false;
    }
  }
  return true;
}

bool ParkingSpeedDynamicObsDecider::IsDynamicObsNeedIgnore(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    bool& is_ignore) const {
  auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  is_ignore = false;
  SLPoint adc_sl_point = inside_data.init_sl_point;

  double adc_s_on_reference_line{inside_data.init_sl_point.s()};
  adc_s_on_reference_line -=
      inside_data.is_reverse_driving
          ? VehicleParam::Instance()->front_edge_to_center()
          : VehicleParam::Instance()->back_edge_to_center();
  double front_buffer = std::max(
      inside_data.vel_v * plan_config.speed_plan.dynamic_obs_area_min_time,
      static_cast<double>(plan_config.speed_plan.dynamic_obs_area_min_length));
  double end_s = obstacle.PolygonBoundary().end_s();
  double start_s = obstacle.PolygonBoundary().start_s();
  double start_l = obstacle.PolygonBoundary().start_l();
  double end_l = obstacle.PolygonBoundary().end_l();
  double heading_diff =
      normalize_angle(obstacle.velocity_heading() - inside_data.vel_heading);
  double project_vel_parallel_car_vel =
      obstacle.speed() * std::cos(heading_diff);
  LOG_INFO(
      "dynamic obs id[{}], dynamic obs type[{}], s_s: {:.4f}, e_s: {:.4f}, "
      "s_l: {:.4f}, e_l: {:.4f}, adc_s: {:.4f}, head_diff: {:.4f}, proj_v: "
      "{:.4f}",
      obstacle.id(), static_cast<int>(obstacle.type()), start_s, end_s, start_l,
      end_l, adc_s_on_reference_line, heading_diff,
      project_vel_parallel_car_vel);

  if (std::abs(heading_diff) > M_PI_2) {
    // TEST
    LOG_DEBUG("reverse obs");
    if (end_s <= adc_s_on_reference_line + VehicleParam::Instance()->length()) {
      // back of vehicle, ignore
      LOG_INFO("reverse obs:back of vehicle, ignore");
      is_ignore = true;
    }
    if (project_vel_parallel_car_vel < -0.5) {
      double expect_obs_forward_s =
          std::abs(project_vel_parallel_car_vel) * 4.0;
      if (start_s - expect_obs_forward_s >=
          adc_s_on_reference_line + VehicleParam::Instance()->length() +
              front_buffer) {
        // front of vehicle, ignore
        LOG_INFO("reverse obs:front of vehicle, ignore");
        is_ignore = true;
      }
    }
  } else {
    // TEST
    LOG_DEBUG("forward obs");
    if (end_s <=
        adc_s_on_reference_line + VehicleParam::Instance()->length() * 0.5) {
      // back of vehicle, ignore
      LOG_INFO("forward obs:back of vehicle, ignore");
      is_ignore = true;
    } else if (start_s >= adc_s_on_reference_line +
                              VehicleParam::Instance()->length() +
                              front_buffer) {
      // front of vehicle, ignore
      LOG_INFO("forward obs:front of vehicle, ignore");
      is_ignore = true;
    } else {
      // complex situation, if the obs is totaly besides vehicle, ignore
      // convert to local
      std::vector<Vec2d> box_corner = obstacle.polygon_corners();
      Boundary obs_box_local;
      double x_target_local = 0.0;
      double y_target_local = 0.0;

      for (const auto& tmp_corner : box_corner) {
        x_target_local =
            (tmp_corner.x() - inside_data.vel_x) *
                cos(inside_data.vel_heading) +
            (tmp_corner.y() - inside_data.vel_y) * sin(inside_data.vel_heading);
        y_target_local =
            (inside_data.vel_x - tmp_corner.x()) *
                sin(inside_data.vel_heading) +
            (tmp_corner.y() - inside_data.vel_y) * cos(inside_data.vel_heading);
        obs_box_local.set_start_s(
            std::min(obs_box_local.start_s(), x_target_local));
        obs_box_local.set_end_s(
            std::max(obs_box_local.end_s(), x_target_local));
        obs_box_local.set_start_l(
            std::min(obs_box_local.start_l(), y_target_local));
        obs_box_local.set_end_l(
            std::max(obs_box_local.end_l(), y_target_local));
      }
      // besides vehicle ?
      if (obs_box_local.start_s() >=
              -VehicleParam::Instance()->back_edge_to_center() &&
          obs_box_local.end_s() <=
              VehicleParam::Instance()->front_edge_to_center() * 0.6 &&
          ((obs_box_local.start_l() >=
                VehicleParam::Instance()->width() * 0.5 &&
            obs_box_local.end_l() <=
                VehicleParam::Instance()->width() * 0.5 + 1.0) ||
           (obs_box_local.start_l() >=
                -VehicleParam::Instance()->width() * 0.5 - 1.0 &&
            obs_box_local.end_l() <=
                -VehicleParam::Instance()->width() * 0.5))) {
        if (project_vel_parallel_car_vel <= inside_data.vel_v * 0.8) {
          is_ignore = true;
          // TEST
          LOG_INFO("forward obs:obs is besides vehicle, ignore");
          LOG_INFO(
              "s_s: {:.4f}, e_s: {:.4f}, s_l: {:.4f}, e_l: {:.4f}, "
              "threshold[{:.4f},{:.4f}, "
              "|{:.4f}|, |{:.4f}|]",
              obs_box_local.start_s(), obs_box_local.end_s(),
              obs_box_local.start_l(), obs_box_local.end_l(),
              -VehicleParam::Instance()->back_edge_to_center(),
              VehicleParam::Instance()->front_edge_to_center() * 0.6,
              VehicleParam::Instance()->width() * 0.5,
              VehicleParam::Instance()->width() * 0.5 + 1.0);
        }
      }
    }
  }
  LOG_INFO("igore this obs?: {}", is_ignore);

  return true;
}

bool ParkingSpeedDynamicObsDecider::CollisionCheckObstacleWithTrajectory(
    TaskInfo& task_info, const Obstacle& obstacle) const {
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  if (outside_data == nullptr) return false;
  if (obstacle.length() < 1e-4 || obstacle.width() < 1e-4) {
    LOG_ERROR("Obstacle [{}] length({:.4f}) < 1e-4 || width({:.4f}) < 1e-4",
              obstacle.id(), obstacle.length(), obstacle.width());
    return false;
  }

  const auto& adc_bounding_boxes =
      outside_data->speed_obstacle_context.adc_boundaries;
  if (adc_bounding_boxes.size() < 3) {
    LOG_ERROR("adc bounding boxes size < 3.");
    return false;
  }
  if (outside_data->path_data->path().path_points().size() !=
      adc_bounding_boxes.size()) {
    LOG_ERROR("path_points size != adc_bounding_boxes size.");
    return false;
  }

  auto path_points = outside_data->path_data->path().path_points();
  bool adc_on_reference_line_flag =
      IsAdcOnReferenceLine(task_info.reference_line(), inside_data);
  double adc_front_theta = path_points.front().theta();

  // prediction trajectory collision check
  LOG_DEBUG("start prediction trajectory collision check:");
  CollisionCheckWithPredictionTrajectory(task_info, adc_on_reference_line_flag,
                                         obstacle);

  return true;
}

void ParkingSpeedDynamicObsDecider::CollisionCheckWithPredictionTrajectory(
    TaskInfo& task_info, const bool adc_on_reference_line_flag,
    const Obstacle& obstacle) const {
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto& path_points = outside_data->path_data->path().path_points();
  const auto& adc_bounding_boxes =
      outside_data->speed_obstacle_context.adc_boundaries;
  LOG_INFO("adc_front_theta is {:.4f}", path_points.front().theta());
  const auto& adc_corner_pt_coordinate =
      outside_data->speed_obstacle_context.adc_corner_pt_coordinate;
  const auto& prediction_config =
      config::PlanningConfig::Instance()->plan_config().prediction;

  const auto& pred_traj = obstacle.uniform_trajectory();
  if (pred_traj.num_of_points() < 2) {
    LOG_ERROR("trajectory points less 2.");
    return;
  }

  std::size_t low_index{0};
  std::size_t high_index{path_points.size() - 1};
  int lower_adc_first_index = std::numeric_limits<int>::max();
  std::vector<std::pair<STPoint, double>> lower_points{};
  std::vector<std::pair<STPoint, double>> upper_points{};
  std::vector<double> lower_points_heading_diff{};
  std::vector<double> upper_points_heading_diff{};
  std::vector<double> lower_project_speed{};
  std::vector<double> upper_project_speed{};

  std::vector<double> pre_traj_headings{};
  std::vector<double> sample_t{};
  std::vector<Polygon2d> obstacle_polygons{};
  std::vector<Boundary> obstacle_boundaries{};
  AdcCollideCornerPoint adc_first_collide_corner_point{
      AdcCollideCornerPoint::NONE};
  double heading_diff = normalize_angle(obstacle.velocity_heading() -
                                        path_points.front().theta());
  double project_vel = obstacle.speed() * std::cos(heading_diff);
  bool reverse_obs = project_vel < 0.01 ? true : false;
  LOG_INFO("heading_diff, project_vel, reverse : {:.4f}, {:.4f}, {}",
           heading_diff, project_vel, reverse_obs);

  for (std::size_t j = 0; j < pred_traj.trajectory_points().size(); ++j) {
    TrajectoryPoint point{};
    if (!pred_traj.trajectory_point_at(j, point)) {
      LOG_ERROR("trajectory point at {} failed", j);
      continue;
    }
    pre_traj_headings.push_back(point.theta());

    sample_t.emplace_back(point.relative_time());
    low_index = 0;
    high_index = path_points.size() - 1;
    bool find_high{false};
    bool find_low{false};

    Polygon2d obs_polygon = Utility::get_trajectory_point_polygon(
        obstacle.center(), point, obstacle.velocity_heading(), point.theta(),
        obstacle.polygon());
    GetAdcFirstCollideCornerPoint(obs_polygon, adc_corner_pt_coordinate,
                                  adc_first_collide_corner_point);
    FindHighAndLowWithPolygon(adc_bounding_boxes, obs_polygon, &find_high,
                              &find_low, &high_index, &low_index);
    bool is_intersect = find_high && find_low;
    if (is_intersect) {
      bool ignore = adc_on_reference_line_flag && low_index == 0 &&
                    point.relative_time() > 0.0 &&
                    path_points[high_index].s() <
                        VehicleParam::Instance()->front_edge_to_center();
      if (ignore) {
        LOG_INFO(
            "ignore obs[{}] traj[{}] lagged behind adc. s_lower[{:.4f}], "
            "s_upper[{:.4f}]",
            obstacle.id(), j, path_points[low_index].s(),
            path_points[high_index].s());
        continue;
      }
      if (low_index >= high_index) {
        continue;
      }

      Boundary obstacle_boundary{};
      if (!ref_line_util::ComputePolygonBoundary(
              task_info.reference_line(),
              Polygon2d(obs_polygon.min_area_bounding_box()),
              &obstacle_boundary)) {
        LOG_WARN("compute polygon's boundary on reference_line failed.");
      }

      if (lower_adc_first_index == std::numeric_limits<int>::max()) {
        lower_adc_first_index = low_index;
      }
      obstacle_polygons.emplace_back(obs_polygon);
      obstacle_boundaries.emplace_back(obstacle_boundary);
      double set_time = point.relative_time();
      double lower_point_heading_diff =
          point.theta() - normalize_angle(path_points[low_index].theta());
      lower_point_heading_diff = normalize_angle(lower_point_heading_diff);
      lower_points_heading_diff.emplace_back(lower_point_heading_diff);
      double lower_point_obs_v = std::max(0.0, obstacle.speed());
      lower_point_obs_v *= std::cos(lower_point_heading_diff);

      lower_points.emplace_back(STPoint(path_points[low_index].s(), set_time),
                                lower_point_obs_v);
      lower_project_speed.emplace_back(lower_point_obs_v);

      double upper_point_heading_diff =
          point.theta() - normalize_angle(path_points[high_index].theta());
      upper_point_heading_diff = normalize_angle(upper_point_heading_diff);
      upper_points_heading_diff.emplace_back(upper_point_heading_diff);
      double upper_point_obs_v = std::max(0.0, obstacle.speed());
      upper_point_obs_v *= std::cos(upper_point_heading_diff);

      upper_points.emplace_back(STPoint(path_points[high_index].s(), set_time),
                                upper_point_obs_v);
      upper_project_speed.emplace_back(upper_point_obs_v);
    }
    if (!reverse_obs &&
        (point.relative_time() > FLAGS_planning_prediction_trust_time_length +
                                     prediction_config.trust_time_delta)) {
      break;
    }
    if (reverse_obs && (point.relative_time() >
                        FLAGS_planning_prediction_trust_time_length +
                            prediction_config.reverse_trust_time_delta)) {
      break;
    }
  }
  if (lower_points.size() < 2 || upper_points.size() < 2) {
    LOG_ERROR(
        "ignore obs[{}], upper_points size is [{}], lower_points size is "
        "[{}]",
        obstacle.id(), upper_points.size(), lower_points.size());
    return;
  }
  double collide_s = lower_adc_first_index >= path_points.size()
                         ? path_points.back().s()
                         : path_points[lower_adc_first_index].s();

  outside_data->speed_obstacle_context.dynamic_obstacles_decision.emplace_back(
      SpeedObstacleDecision{
          .lower_points = lower_points,
          .upper_points = upper_points,
          .lower_points_heading_diff = lower_points_heading_diff,
          .upper_points_heading_diff = upper_points_heading_diff,
          .obstalce_polygons = obstacle_polygons,
          .obstacle_boundaries = obstacle_boundaries,
          .sample_t = sample_t,
          .obstacle_pre_traj_headings = pre_traj_headings,
          .obstacle = obstacle,
          .collide = true,
          .reverse = reverse_obs,
          .lower_adc_first_index = lower_adc_first_index,
          .adc_first_collide_corner_point = adc_first_collide_corner_point});
  LOG_INFO("obs[{}], decision collide {}, adc_first_collide_corner_point {}.",
           obstacle.id(), true,
           static_cast<int>(adc_first_collide_corner_point));
}

bool ParkingSpeedDynamicObsDecider::IsAdcOnReferenceLine(
    const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data) const {
  const auto& init_planning_point = inside_data.init_point;
  Vec2d init_planning_point_coordinate = init_planning_point.coordinate();
  SLPoint sl_pt{};
  reference_line->GetPointInFrenetFrame(init_planning_point, &sl_pt);
  bool init_planning_point_on_line =
      ref_line_util::IsOnRoad(reference_line, sl_pt);
  if (init_planning_point_on_line) {
    return true;
  }

  std::vector<SLPoint> corners;
  if (!ref_line_util::GetAdcBoundingBoxSL(
          reference_line, {inside_data.vel_x, inside_data.vel_y},
          inside_data.vel_heading, &corners)) {
    LOG_ERROR("GetAdcBoundingBoxSL failed");
    return false;
  }
  std::size_t positive_l_num = 0;
  std::size_t negetive_l_num = 0;
  for (const auto& sl_point : corners) {
    if (std::fabs(sl_point.l()) < 0.01) {
      return true;
    } else if (sl_point.l() > 0) {
      ++positive_l_num;
    } else if (sl_point.l() < 0) {
      ++negetive_l_num;
    }
  }
  return positive_l_num > 0 && negetive_l_num > 0;
}

void ParkingSpeedDynamicObsDecider::GetAdcFirstCollideCornerPoint(
    const Polygon2d& obs_polygon, const std::vector<Vec2d>& adc_corners,
    AdcCollideCornerPoint& adc_first_collide_corner_point) const {
  if (adc_first_collide_corner_point != AdcCollideCornerPoint::NONE ||
      adc_corners.size() < static_cast<int>(AdcCollideCornerPoint::NONE)) {
    return;
  }
  // VisObsPolygon2d(obs_polygon);
  if (obs_polygon.is_point_in(
          adc_corners[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)])) {
    adc_first_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
  } else if (obs_polygon.is_point_in(adc_corners[static_cast<int>(
                 AdcCollideCornerPoint::RIGHT_REAR)])) {
    adc_first_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
  } else if (obs_polygon.is_point_in(adc_corners[static_cast<int>(
                 AdcCollideCornerPoint::RIGHT_FRONT)])) {
    adc_first_collide_corner_point = AdcCollideCornerPoint::RIGHT_FRONT;
  } else if (obs_polygon.is_point_in(adc_corners[static_cast<int>(
                 AdcCollideCornerPoint::LEFT_FRONT)])) {
    adc_first_collide_corner_point = AdcCollideCornerPoint::LEFT_FRONT;
  }
}

void ParkingSpeedDynamicObsDecider::FindHighAndLowWithPolygon(
    const std::vector<Box2d>& adc_bounding_boxes, const Polygon2d& obstacle_box,
    bool* find_high, bool* find_low, std::size_t* high_index,
    std::size_t* low_index) const {
  if (find_high == nullptr || find_low == nullptr || high_index == nullptr ||
      low_index == nullptr) {
    LOG_ERROR("input invalid");
    return;
  }
  while (*high_index >= *low_index) {
    if ((*find_high) && (*find_low)) {
      break;
    }
    if (!(*find_low)) {
      if (!obstacle_box.has_overlap(
              Polygon2d(adc_bounding_boxes[*low_index]))) {
        (*low_index) += 2;
      } else {
        *find_low = true;
        if (*low_index > 0) {
          if (obstacle_box.has_overlap(
                  Polygon2d(adc_bounding_boxes[*low_index - 1]))) {
            *low_index = *low_index - 1;
          }
        }
      }
    }
    if (!(*find_high)) {
      if (!obstacle_box.has_overlap(
              Polygon2d(adc_bounding_boxes[*high_index]))) {
        if (*high_index > 3) {
          (*high_index) -= 2;
        } else {
          --(*high_index);
        }
      } else {
        *find_high = true;
        if (*high_index + 1 < adc_bounding_boxes.size()) {
          if (obstacle_box.has_overlap(
                  Polygon2d(adc_bounding_boxes[*high_index + 1]))) {
            *high_index = *high_index + 1;
          }
        }
      }
    }
  }
}

bool ParkingSpeedDynamicObsDecider::ComputePathAccumulatedS(
    const PathData& path_data, std::vector<double>* accumulated_s) const {
  if (accumulated_s == nullptr) return false;
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }

  accumulated_s->clear();
  accumulated_s->push_back(0.);
  for (std::size_t i = 1; i < path_data.path().path_points().size(); ++i) {
    auto& curr_pt = path_data.path().path_points()[i];
    auto& last_pt = path_data.path().path_points()[i - 1];
    accumulated_s->push_back(curr_pt.distance_to(last_pt) +
                             accumulated_s->back());
  }
  return true;
}

bool ParkingSpeedDynamicObsDecider::BuildPathAdcBoundingBoxesAndAABoxes(
    TaskInfo& task_info, const InsidePlannerData& inside_data,
    const ReferenceLinePtr ref_ptr, const PathData& path_data,
    std::vector<Box2d>* path_adc_boxes,
    std::vector<Boundary>* path_adc_sl_boundaries) const {
  double en_large_buffer{FLAGS_planning_speed_plan_enlarge_self_buffer};
  double front_en_large_buffer{en_large_buffer};
  double back_en_large_buffer{en_large_buffer};
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  if (!speed_planner_common::GetAdcEnlargeBuffer(
          inside_data, inside_data.curr_multi_level, &en_large_buffer,
          &front_en_large_buffer, &back_en_large_buffer)) {
    LOG_ERROR("get adc self en_large buffer failed.");
  }
  if (inside_data.curr_scenario_state == ScenarioState::BACK_OUT) {
    en_large_buffer = front_en_large_buffer = back_en_large_buffer = 0;
  }
  LOG_INFO("current_level, front_buffer, back_buffer: {}, {:.3f}, {:.3f}",
           inside_data.curr_multi_level, front_en_large_buffer,
           back_en_large_buffer);
  if (!BuildPathAdcBoundingBoxes(
          task_info.current_frame()->inside_planner_data(),
          *(outside_planner_data_ptr->path_data), en_large_buffer,
          front_en_large_buffer, back_en_large_buffer, path_adc_boxes)) {
    LOG_ERROR("BuildPathAdcBoundingBoxes failed.");
    return false;
  }
  if (!BuildPathAdcAABoxes(task_info.reference_line(),
                           task_info.current_frame()->inside_planner_data(),
                           *(outside_planner_data_ptr->path_data),
                           en_large_buffer, front_en_large_buffer,
                           back_en_large_buffer, path_adc_sl_boundaries)) {
    LOG_WARN("BuildPathAdcAABoxes failed.");
  }
  return true;
}

bool ParkingSpeedDynamicObsDecider::BuildPathAdcBoundingBoxes(
    const InsidePlannerData& inside_data, const PathData& path_data,
    const double& en_large_buffer, const double& front_en_large_buffer,
    const double& back_en_large_buffer,
    std::vector<Box2d>* path_adc_boxes) const {
  if (path_adc_boxes == nullptr) return false;
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }

  const auto& veh_path = path_data.path().path_points();
  path_adc_boxes->clear();
  for (const auto& path_point : veh_path) {
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {path_point.x(), path_point.y()}, path_point.theta(), en_large_buffer,
        front_en_large_buffer, back_en_large_buffer);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build adc bounding box from path point at s[{}] failed",
                path_point.s());
      return false;
    }
    path_adc_boxes->emplace_back(adc_box);
  }
  if (path_adc_boxes->size() != veh_path.size()) {
    LOG_ERROR("path_adc_boxes_.size({}) != veh_path.size({})",
              path_adc_boxes->size(), veh_path.size());
    return false;
  }
  return true;
}

bool ParkingSpeedDynamicObsDecider::BuildPathAdcAABoxes(
    const ReferenceLinePtr ref_ptr, const InsidePlannerData& inside_data,
    const PathData& path_data, const double& en_large_buffer,
    const double& front_en_large_buffer, const double& back_en_large_buffer,
    std::vector<Boundary>* adc_sl_boundaries) const {
  if (adc_sl_boundaries == nullptr) return false;
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }
  const auto& veh_path = path_data.path().path_points();
  adc_sl_boundaries->clear();
  for (const auto& path_point : veh_path) {
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {path_point.x(), path_point.y()}, path_point.theta(), en_large_buffer,
        front_en_large_buffer, back_en_large_buffer);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build adc bounding box from path point at s[{}] failed",
                path_point.s());
      continue;
    }
    Boundary boundary{};
    if (!ref_line_util::ComputePolygonBoundary(ref_ptr, Polygon2d(adc_box),
                                               &boundary)) {
      LOG_WARN("compute boundary failed.");
      continue;
    }
    adc_sl_boundaries->emplace_back(boundary);
  }
  if (adc_sl_boundaries->size() != veh_path.size()) {
    LOG_WARN("adc_sl_boundaries->size({}) != veh_path.size({})",
             adc_sl_boundaries->size(), veh_path.size());
  }
  return true;
}

bool ParkingSpeedDynamicObsDecider::BuildPathAdcPreBoundingBoxes(
    const InsidePlannerData& inside_data, const PathData& path_data,
    std::vector<Box2d>* path_adc_pre_boxes,
    std::vector<PathPoint>* pre_path_points) const {
  auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  if (path_adc_pre_boxes == nullptr) return false;
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }
  path_adc_pre_boxes->clear();
  pre_path_points->clear();

  // TODO:(zhp) 如果采用横向path-stitch分离的方案，该预测模型需要进行修正
  double veh_x = inside_data.vel_x;
  double veh_y = inside_data.vel_y;
  double veh_v = inside_data.vel_v;
  double veh_heading = inside_data.vel_heading;
  double delta_s = 0.0;
  const std::vector<PathPoint>& veh_path = path_data.path().path_points();
  double predict_dis =
      std::fmin(data_center_->vehicle_state_proxy().AbsoluteLinearVelocity() *
                    plan_config.speed_plan.predict_trajectory_time,
                plan_config.speed_plan.max_predict_trajectory_dis);

  double en_large_buffer{FLAGS_planning_speed_plan_enlarge_self_buffer};
  double front_en_large_buffer{en_large_buffer};
  double back_en_large_buffer{en_large_buffer};
  if (!speed_planner_common::GetAdcEnlargeBuffer(
          inside_data, inside_data.curr_multi_level, &en_large_buffer,
          &front_en_large_buffer, &back_en_large_buffer)) {
    LOG_ERROR("get adc self en_large buffer failed.");
  }
  double sum_s = 0.;
  for (std::size_t i = 0; i < veh_path.size(); ++i) {
    if (i >= 1) {
      delta_s = std::sqrt(std::pow(veh_path[i].x() - veh_path[i - 1].x(), 2) +
                          std::pow(veh_path[i].y() - veh_path[i - 1].y(), 2));
      sum_s += delta_s;
      if (sum_s > predict_dis) {
        break;
      }
      veh_x += delta_s * std::cos(veh_heading);
      veh_y += delta_s * std::sin(veh_heading);
      double kappa = veh_path[i].kappa();
      if (kappa > 0.25) kappa = 0.25;
      if (kappa < -0.25) kappa = -0.25;
      veh_heading += delta_s * kappa;
      veh_heading = normalize_angle(veh_heading);
    }
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {veh_x, veh_y}, veh_heading, en_large_buffer, front_en_large_buffer,
        back_en_large_buffer);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build pre adc bounding box from path point at s[{}] failed",
                veh_path[i].s());
      return false;
    }
    path_adc_pre_boxes->emplace_back(adc_box);
    pre_path_points->emplace_back(
        PathPoint({veh_x, veh_y}, veh_heading, 0.0, 0.0, 0.0, veh_path[i].s()));
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive

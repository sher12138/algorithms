#include "speed_overtake_turn_right_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using neodrive::global::prediction::Trajectory_PredictorType_FreeMove;

namespace neodrive {
namespace planning {

namespace {
const auto& obs_overtake_turn_right_config_{config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .obs_overtake_turn_right};

bool IsSmallObs(const Obstacle* const obs_ptr) {
  if (nullptr == obs_ptr) {
    LOG_ERROR("obs is nullptr.");
    return false;
  }
  bool is_little_size = std::pow(std::max(obs_ptr->length(), obs_ptr->width()),
                                 2.0) < config::PlanningConfig::Instance()
                                            ->plan_config()
                                            .common.little_obs_area_threshold;
  return ((obs_ptr->type() == Obstacle::ObstacleType::BICYCLE) ||
          (obs_ptr->type() == Obstacle::ObstacleType::PEDESTRIAN) ||
          is_little_size);
}

void GetAdcFirstCollideCornerPoint(
    const Polygon2d& obs_polygon, const std::vector<Vec2d>& adc_corners,
    AdcCollideCornerPoint& adc_first_collide_corner_point) {
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

}  // namespace

SpeedOvertakeTurnRightDecider::SpeedOvertakeTurnRightDecider() {
  name_ = "SpeedOvertakeTurnRightDecider";
}

SpeedOvertakeTurnRightDecider::~SpeedOvertakeTurnRightDecider() { Reset(); }

ErrorCode SpeedOvertakeTurnRightDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!DataCheck(task_info)) {
    LOG_ERROR("DataCheck failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void SpeedOvertakeTurnRightDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    UpdatedLimitedSpeed();
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::OBS_TURN_RIGHT_RISK);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(limited_deceleration_);
    LOG_INFO(
        "OBS_TURN_RIGHT_RISK {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, limited_deceleration_);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    last_limited_speed_ = limited_speed_;
  }
}

bool SpeedOvertakeTurnRightDecider::Process(TaskInfo& task_info) {
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }

  if (!in_right_turn_risk_area_) {
    LOG_INFO("adc not in_right_turn_risk_area_");
    return true;
  }

  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();
  UpdateDynamicToStaticObs(task_info);

  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }

    if (IsSmallObs(dynamic_obs_vec[i])) {
      continue;
    }

    double heading_diff = normalize_angle(
        dynamic_obs_vec[i]->velocity_heading() - inside_data.vel_heading);
    if (std::abs(heading_diff) > M_PI_2) {
      continue;
    }

    const auto& obs_boundary = dynamic_obs_vec[i]->PolygonBoundary();
    if (!check_obs_turn_right_area_.has_overlap(obs_boundary)) {
      continue;
    }

    LOG_INFO("Update obs {} info:", dynamic_obs_vec[i]->id());
    UpdataObsHeadingInfo(task_info, dynamic_obs_vec[i]);
    UpdateObsPredictionInfo(task_info, dynamic_obs_vec[i]);

    double obs2adc_lat_dis = speed_planner_common::GetObs2AdcLateralDis(
        *dynamic_obs_vec[i], task_info.curr_sl().l());
    if ((obs2adc_lat_dis < 0.0) && (obs_boundary.end_s() < adc_back_edge_s_)) {
      LOG_INFO("Big obs [{}] queue behind adc, not limit speed.",
               dynamic_obs_vec[i]->id());
      continue;
    }
    if ((dynamic_obs_vec[i]->speed() < adc_current_v_) &&
        (obs_boundary.end_s() < adc_back_edge_s_)) {
      LOG_INFO("Big obs [{}] behind and slower than adc, not limit speed.",
               dynamic_obs_vec[i]->id());
      continue;
    }

    DealTurnRightCollisionRisk(task_info, dynamic_obs_vec[i]);
  }

  return true;
}

bool SpeedOvertakeTurnRightDecider::DataCheck(TaskInfo& task_info) {
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

void SpeedOvertakeTurnRightDecider::UpdateObsPredictionInfo(
    TaskInfo& task_info, const Obstacle* const obs) {
  if (obs_prediction_info_.find(obs->id()) != obs_prediction_info_.end()) {
    obs_prediction_info_[obs->id()].Reset();
  }
  auto& obs_prediction_info = obs_prediction_info_[obs->id()];
  obs_prediction_info.updated = true;

  auto& predic_traj = obs->prediction_trajectories();
  if (predic_traj.empty() ||
      (predic_traj.front().predictor_type() ==
       Trajectory_PredictorType_FreeMove) ||
      predic_traj.front().trajectory_points().size() < 2) {
    obs_prediction_info.obs_will_turn = false;
    obs_prediction_info.accum_heading_diff = 0.0;
    LOG_WARN(
        "obs [{}] prediction error: empty {}, predictor_type {}, trajectory "
        "size {}",
        obs->id(), predic_traj.empty(),
        static_cast<int>(predic_traj.front().predictor_type()),
        predic_traj.front().trajectory_points().size());
    return;
  }

  const auto& pts = predic_traj.front().trajectory_points();
  double last_heading = normalize_angle(pts.front().theta());
  for (int i = 1; i < pts.size(); i++) {
    double cur_heading = normalize_angle(pts[i].theta());
    obs_prediction_info.accum_heading_diff +=
        normalize_angle(cur_heading - last_heading);
    last_heading = cur_heading;
    if (std::abs(obs_prediction_info.accum_heading_diff) >
        obs_overtake_turn_right_config_.obs_turning_threshold) {
      obs_prediction_info.obs_will_turn = true;
      LOG_WARN(
          "obs [{}] prediction at point [{}] accum_heading_diff {:.3f}, "
          "trajectory size {}",
          obs->id(), i, obs_prediction_info.accum_heading_diff,
          predic_traj.front().trajectory_points().size());
      break;
    }
  }
}

void SpeedOvertakeTurnRightDecider::DealTurnRightCollisionRisk(
    TaskInfo& task_info, const Obstacle* obs) {
  LimitSpeedAtRiskArea(task_info, obs);
  if ((car_go_direction_ != GoDirectionType::TURN_LEFT) &&
      (obs->PolygonBoundary().end_s() < adc_back_edge_s_)) {
    LOG_INFO(
        "obs [{}] totally behind adc and adc not turn left and only generally "
        "limit speed.",
        obs->id());
    return;
  }

  LimitSpeedByObs(task_info, obs);
  GetNewObsDecision(task_info, obs);
  ModifyOriginObsDecision(task_info);
}

void SpeedOvertakeTurnRightDecider::LimitSpeedAtRiskArea(TaskInfo& task_info,
                                                         const Obstacle* obs) {
  update_limited_speed_ = true;
  target_speed_ = std::min(target_speed_, strategy_max_limit_speed_ / 2.0);
  limited_deceleration_ =
      std::min(limited_deceleration_,
               static_cast<double>(
                   obs_overtake_turn_right_config_.default_deceleration));

  LOG_INFO(
      "adc in risk area and observe obs [{}], set target_speed_ {:.3f}, "
      "limited_deceleration_ {:.3f}",
      obs->id(), target_speed_, limited_deceleration_);
}

void SpeedOvertakeTurnRightDecider::LimitSpeedByObs(TaskInfo& task_info,
                                                    const Obstacle* obs) {
  const auto& obs_boundary = obs->PolygonBoundary();
  if (obs_boundary.end_s() > adc_back_edge_s_) {
    limited_deceleration_ =
        std::min(limited_deceleration_,
                 1.6 * obs_overtake_turn_right_config_.default_deceleration);
    LOG_INFO("big obs [{}] overtake adc, set limited_deceleration_ {:.3f}.",
             obs->id(), limited_deceleration_);
  }

  double heading_diff = normalize_angle(
      obs->velocity_heading() -
      task_info.current_frame()->inside_planner_data().vel_heading);
  LOG_INFO("heading_diff, accum_heading_diff: {:.2f}, {:.2f}",
           std::abs(heading_diff),
           std::abs(obs_heading_info_[obs->id()].accum_heading_diff));

  // check: limit speed when obs is on path(collision check use polygon,)
  if ((obs_decision_info_.find(obs->id()) != obs_decision_info_.end()) &&
      (obs_decision_info_[obs->id()].second.lower_points.front().first.t() <
       1e-2) &&
      (obs_boundary.start_s() > adc_front_edge_s_ + 0.3)) {
    LOG_INFO(
        "obs [{}] on path and have safe distance, not limit speed by lat_dis",
        obs->id());
  } else if ((obs_boundary.end_s() > adc_back_edge_s_) &&
             (obs_boundary.start_s() < risk_area_info_.risk_area_end_s) &&
             ((std::abs(heading_diff) >
               obs_overtake_turn_right_config_.obs_turning_threshold) ||
              (std::abs(obs_heading_info_[obs->id()].accum_heading_diff) >
               obs_overtake_turn_right_config_.obs_turning_threshold))) {
    // check: obs's localization && obs's heading_diff
    double l_dis = std::max(
        obs_boundary.start_l() - check_obs_turn_right_area_.start_l(), 0.0);
    double l_dis_limite_speed =
        l_dis * obs_overtake_turn_right_config_.l_dis_limit_speed_ratio;
    update_limited_speed_ = true;
    target_speed_ = std::max(std::min(target_speed_, l_dis_limite_speed), 0.0);
    limited_deceleration_ =
        std::min(limited_deceleration_,
                 3.0 * obs_overtake_turn_right_config_.default_deceleration);
    LOG_INFO(
        "big obs [{}] near adc in risk area, l_dis {:.3f}, target_speed_ "
        "{:.3f}, limited_deceleration_ {:.3f}",
        obs->id(), l_dis, target_speed_, limited_deceleration_);
  }
}

void SpeedOvertakeTurnRightDecider::UpdatedLimitedSpeed() {
  if (adc_current_v_ < target_speed_) {
    limited_speed_ = target_speed_;
    limited_deceleration_ = 0.0;
    return;
  }

  limited_speed_ =
      std::min(limited_speed_, last_limited_speed_ +
                                   kPlanningCycleTime * limited_deceleration_);
  limited_speed_ = std::max(limited_speed_, target_speed_);

  LOG_INFO(
      "target_speed_ {:.2f}, set limited_speed_ {:.2f}, limited_deceleration_ "
      "{:.2f}",
      target_speed_, limited_speed_, limited_deceleration_);
}

void SpeedOvertakeTurnRightDecider::GetNewObsDecision(TaskInfo& task_info,
                                                      const Obstacle* obs) {
  auto& obs_prediction_info = obs_prediction_info_[obs->id()];
  bool has_prediction_decision{false};
  SpeedObstacleDecision new_decision;
  if (obs_prediction_info.obs_will_turn) {
    has_prediction_decision =
        CollisionCheckWithPredictionTrajectory(task_info, *obs, new_decision);
  }

  if (obs_prediction_info.obs_will_turn && has_prediction_decision) {
    if (obs_decision_info_.find(obs->id()) == obs_decision_info_.end()) {
      obs_decision_info_[obs->id()].first = true;
      obs_decision_info_[obs->id()].second = std::move(new_decision);
      LOG_INFO("obs [{}] generate prediction collision decision.", obs->id());
      return;
    }

    const auto& origin_decision = obs_decision_info_[obs->id()].second;
    if ((new_decision.lower_points.front().first.s() <
         origin_decision.lower_points.front().first.s()) ||
        (new_decision.lower_points.front().first.t() <
         origin_decision.lower_points.front().first.t())) {
      new_decision.is_in_left_turn_area = origin_decision.is_in_left_turn_area;
      new_decision.is_in_right_turn_area =
          origin_decision.is_in_right_turn_area;
      obs_decision_info_[obs->id()].first = true;
      obs_decision_info_[obs->id()].second = std::move(new_decision);
      LOG_INFO("update obs [{}] decision from uniform to prediction.",
               obs->id());
      return;
    } else {
      LOG_INFO(
          "obs [{}] new_decision lower pt({:.3f}, {:.3f}) bigger than origin, "
          "not update.",
          obs->id(), new_decision.lower_points.front().first.s(),
          new_decision.lower_points.front().first.t());
    }
  } else if (std::abs(obs_heading_info_[obs->id()].accum_heading_diff) >
             obs_overtake_turn_right_config_.obs_turning_threshold / 2.0) {
    if (obs_decision_info_.find(obs->id()) == obs_decision_info_.end()) {
      return;
    }
    auto origin_decision = obs_decision_info_[obs->id()].second;
    auto& obs_heading_info = obs_heading_info_[obs->id()];
    double buffer =
        obs->length() *
        std::min(std::abs(obs_heading_info.accum_heading_diff) /
                     obs_overtake_turn_right_config_.obs_turning_threshold,
                 static_cast<double>(
                     obs_overtake_turn_right_config_.obs_max_buffer_ratio));
    LOG_INFO(
        "update obs [{}] decision to buffer {:.3f}, because length {:.3f}, "
        "accum_heading_diff {:.3f}",
        obs->id(), buffer, obs->length(), obs_heading_info.accum_heading_diff);
    for (auto& low_pt : origin_decision.lower_points) {
      low_pt.first.set_s(std::max(low_pt.first.s() - buffer, 0.0));
    }
    origin_decision.risk_obs = true;

    obs_decision_info_[obs->id()].first = true;
    obs_decision_info_[obs->id()].second = std::move(origin_decision);
  }
}

void SpeedOvertakeTurnRightDecider::ModifyOriginObsDecision(
    TaskInfo& task_info) {
  auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dynamic_obstacles_decision;
  // update old
  for (auto& decision : dynamic_obstacles_decision) {
    if (obs_decision_info_[decision.obstacle.id()].first) {
      decision = obs_decision_info_[decision.obstacle.id()].second;
      obs_decision_info_[decision.obstacle.id()].first = false;
      LOG_INFO("modify obs [{}] decision in outside_data.",
               decision.obstacle.id());
    }
  }

  // add new
  for (auto& iter : obs_decision_info_) {
    if (iter.second.first) {
      dynamic_obstacles_decision.emplace_back(iter.second.second);
      LOG_INFO("add new obs [{}] decision to outside_data.", iter.first);
    }
  }
}

bool SpeedOvertakeTurnRightDecider::CollisionCheckWithPredictionTrajectory(
    TaskInfo& task_info, const Obstacle& obstacle,
    SpeedObstacleDecision& decision) {
  const auto& outside_data = task_info.current_frame()->outside_planner_data();
  if (obstacle.prediction_trajectories().empty() ||
      obstacle.prediction_trajectories().front().num_of_points() < 2 ||
      outside_data.path_data == nullptr ||
      outside_data.path_data->path().path_points().empty()) {
    LOG_ERROR("trajectory points less 2.");
    return false;
  }

  const auto& pred_traj = obstacle.prediction_trajectories().front();
  const auto& path_points = outside_data.path_data->path().path_points();
  const auto& adc_bounding_boxes =
      outside_data.speed_obstacle_context.adc_boundaries;
  const auto& adc_corner_pt_coordinate =
      outside_data.speed_obstacle_context.adc_corner_pt_coordinate;
  std::size_t low_index{0};
  std::size_t high_index{path_points.size() - 1};
  if (adc_bounding_boxes.size() < 3) {
    LOG_ERROR("adc bounding boxes size < 3.");
    return false;
  }
  if (path_points.size() != adc_bounding_boxes.size()) {
    LOG_ERROR("path_points size != adc_bounding_boxes size.");
    return false;
  }

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
        obstacle.center(), {point.x(), point.y()}, obstacle.velocity_heading(),
        point.theta(), obstacle.polygon());
    GetAdcFirstCollideCornerPoint(obs_polygon, adc_corner_pt_coordinate,
                                  adc_first_collide_corner_point);
    FindHighAndLowWithPolygon(adc_bounding_boxes, obs_polygon, &find_high,
                              &find_low, &high_index, &low_index);
    bool is_intersect = find_high && find_low;
    if (is_intersect) {
      bool ignore = low_index == 0 && point.relative_time() > 0.0 &&
                    path_points[high_index].s() <
                        VehicleParam::Instance()->front_edge_to_center();
      if (ignore) {
        LOG_INFO(
            "obs[{}] traj[{}] lagged behind adc. s_lower[{:.4f}], "
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
    if (point.relative_time() > FLAGS_planning_prediction_trust_time_length +
                                    config::PlanningConfig::Instance()
                                        ->plan_config()
                                        .prediction.trust_time_delta) {
      break;
    }
  }
  if (lower_points.size() < 2 || upper_points.size() < 2) {
    LOG_ERROR(
        "obs[{}], upper_points size is [{}], lower_points size is "
        "[{}]",
        obstacle.id(), upper_points.size(), lower_points.size());
    return false;
  }

  decision.lower_points = lower_points;
  decision.upper_points = upper_points;
  decision.lower_points_heading_diff = lower_points_heading_diff;
  decision.upper_points_heading_diff = upper_points_heading_diff;
  decision.sample_t = sample_t;
  decision.obstalce_polygons = obstacle_polygons;
  decision.obstacle_boundaries = obstacle_boundaries;
  decision.obstacle_pre_traj_headings = pre_traj_headings;
  decision.obstacle = obstacle;
  decision.collide = true;
  decision.reverse = false;
  decision.is_in_left_turn_area = false;
  decision.is_in_right_turn_area = false;
  decision.risk_obs = true;
  decision.lower_adc_first_index = lower_adc_first_index;
  decision.adc_first_collide_corner_point = adc_first_collide_corner_point;
  LOG_INFO("obs[{}], decision collide {}, adc_first_collide_corner_point {}.",
           obstacle.id(), decision.collide,
           static_cast<int>(adc_first_collide_corner_point));
  return true;
}

bool SpeedOvertakeTurnRightDecider::Init(TaskInfo& task_info) {
  const auto& adc_corner_pt_coordinate =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.adc_corner_pt_coordinate;
  if (adc_corner_pt_coordinate.size() <
      static_cast<int>(AdcCollideCornerPoint::NONE)) {
    LOG_ERROR("adc_corner_pt_coordinate size < 4");
    return false;
  }

  adc_current_s_ = task_info.curr_sl().s();
  adc_back_edge_s_ =
      adc_current_s_ - VehicleParam::Instance()->back_edge_to_center();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  limited_deceleration_ = 0.0;
  target_speed_ = 1e5;
  strategy_max_limit_speed_ =
      DataCenter::Instance()->drive_strategy_max_speed();
  auto& inside_data = task_info.current_frame()->inside_planner_data();
  adc_polygon_ = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.0, 0.0,
      0.0);
  in_right_turn_risk_area_ = IsInRightTurnRiskArea(task_info);
  ClearObsHistoryInfo();
  UpdateObsDecisionInfo(task_info);
  if (!InitCollisionCheckArea(task_info.reference_line(), inside_data,
                              task_info.adc_boundary_origin())) {
    LOG_ERROR("Fail to init collision check area.");
    return false;
  }

  // Judge if ego car go straight or will turn.
  if (!speed_planner_common::JudgeDirectionCarWillGo(
          task_info.reference_line(),
          task_info.current_frame()->inside_planner_data(),
          car_go_direction_)) {
    LOG_ERROR("Fail to judge the direction car will go.");
    return false;
  }

  return true;
}

void SpeedOvertakeTurnRightDecider::UpdateObsDecisionInfo(TaskInfo& task_info) {
  obs_decision_info_.clear();
  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dynamic_obstacles_decision;
  for (auto& obs_decision : dynamic_obstacles_decision) {
    obs_decision_info_[obs_decision.obstacle.id()].first = false;
    obs_decision_info_[obs_decision.obstacle.id()].second = obs_decision;
  }
}

void SpeedOvertakeTurnRightDecider::ClearObsHistoryInfo() {
  for (auto& iter : obs_heading_info_) {
    iter.second.lost_cnt++;
  }
  auto obs_heading_info_tmp = obs_heading_info_;
  for (auto& iter : obs_heading_info_) {
    if (iter.second.lost_cnt > 10) {
      obs_heading_info_tmp.erase(iter.first);
    }
  }
  std::swap(obs_heading_info_, obs_heading_info_tmp);
  LOG_INFO("obs_heading_info_ num {}", obs_heading_info_.size());

  auto obs_prediction_info_tmp = obs_prediction_info_;
  for (auto& iter : obs_prediction_info_) {
    if (!iter.second.updated) {
      obs_prediction_info_tmp.erase(iter.first);
    } else {
      obs_prediction_info_tmp[iter.first].updated = false;
    }
  }
  std::swap(obs_prediction_info_, obs_prediction_info_tmp);
  LOG_INFO("obs_prediction_info_ num {}", obs_prediction_info_.size());
}

bool SpeedOvertakeTurnRightDecider::InitCollisionCheckArea(
    const ReferenceLinePtr& ref_ptr, const InsidePlannerData& inside_data,
    const Boundary& adc_boundary) {
  double check_area_front_length =
      std::max(static_cast<double>(
                   obs_overtake_turn_right_config_.check_area_front_length),
               adc_current_v_ * 2.0);
  double check_area_back_length =
      std::max(static_cast<double>(
                   obs_overtake_turn_right_config_.check_area_back_length),
               adc_current_v_ * 2.0);
  check_obs_turn_right_area_ = std::move(Boundary{
      adc_boundary.end_s() - check_area_back_length,
      adc_boundary.end_s() + check_area_front_length, adc_boundary.end_l(),
      adc_boundary.end_l() + obs_overtake_turn_right_config_.check_area_width});

  LOG_INFO(
      "adc_boundary.end_s: {:.3f}; check_obs_turn_right_area_, s_s, e_s, s_l, "
      "e_l: {:.2f}, {:.2f}, {:.2f}, {:.2f} ",
      adc_boundary.end_s(), check_obs_turn_right_area_.start_s(),
      check_obs_turn_right_area_.end_s(), check_obs_turn_right_area_.start_l(),
      check_obs_turn_right_area_.end_l());

  return true;
}

void SpeedOvertakeTurnRightDecider::UpdateDynamicToStaticObs(
    TaskInfo& task_info) {
  const auto& static_obs_vec = task_info.current_frame()
                                   ->planning_data()
                                   .decision_data()
                                   .static_obstacle();
  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (static_obs_vec[i] == nullptr) {
      continue;
    }

    if (obs_heading_info_.find(static_obs_vec[i]->id()) !=
        obs_heading_info_.end()) {
      UpdataObsHeadingInfo(task_info, static_obs_vec[i]);
      LOG_INFO("obs [{}] change from dynamic to static, heading {:.4f}",
               static_obs_vec[i]->id(), static_obs_vec[i]->heading());
    }
  }
}

void SpeedOvertakeTurnRightDecider::UpdataObsHeadingInfo(
    TaskInfo& task_info, const Obstacle* const obs) {
  if (obs_heading_info_.find(obs->id()) == obs_heading_info_.end()) {
    auto& obs_heading_info = obs_heading_info_[obs->id()];
    obs_heading_info.last_heading = normalize_angle(obs->heading());
    obs_heading_info.lost_cnt = 0;
  } else {
    auto& obs_heading_info = obs_heading_info_[obs->id()];
    obs_heading_info.heading_diff.push_back(
        normalize_angle(obs->heading() - obs_heading_info.last_heading));
    obs_heading_info.accum_heading_diff += obs_heading_info.heading_diff.back();
    if (obs_heading_info.heading_diff.size() >
        obs_overtake_turn_right_config_.info_size_heading) {
      obs_heading_info.accum_heading_diff -=
          obs_heading_info.heading_diff.front();
      obs_heading_info.heading_diff.pop_front();
    }
    obs_heading_info.last_heading = normalize_angle(obs->heading());
    obs_heading_info.lost_cnt = 0;
  }
}

void SpeedOvertakeTurnRightDecider::SearchRiskArea(TaskInfo& task_info) {
  risk_area_info_.Reset();

  auto ref_line = task_info.reference_line();
  const auto& junction_list = ref_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }

    if (adc_current_s_ > risk_area_info_.risk_area_end_s) {
      double risk_area_predeal_s =
          overlap.start_s - std::max(10.0, adc_current_v_ * 4.0);
      double risk_area_start_s = overlap.start_s;
      if (risk_area_predeal_s <= adc_current_s_ &&
          overlap.end_s >= adc_current_s_) {
        risk_area_info_.risk_area_predeal_s = risk_area_predeal_s;
        risk_area_info_.risk_area_start_s = risk_area_start_s;
        risk_area_info_.junction_end_s = overlap.end_s;
        LOG_INFO("adc in junction: {:.3f},  {:.3f}, {:.3f}",
                 risk_area_predeal_s, overlap.start_s, overlap.end_s);
        if (junction_ptr->Type() !=
            static_cast<uint32_t>(JunctionType::IN_ROAD)) {
          risk_area_info_.risk_area_end_s =
              overlap.start_s + (overlap.end_s - overlap.start_s) / 2.0;
          LOG_INFO("adc is in right turn risk area: {:.3f},  {:.3f}, {:.3f}",
                   risk_area_info_.risk_area_predeal_s,
                   risk_area_info_.risk_area_start_s,
                   risk_area_info_.risk_area_end_s);
          return;
        }
        risk_area_info_.risk_area_end_s = overlap.end_s;
      }
    } else {
      if ((overlap.start_s > risk_area_info_.junction_end_s + 70) ||
          (junction_ptr->Type() ==
           static_cast<uint32_t>(JunctionType::IN_ROAD))) {
        risk_area_info_.Reset();
        LOG_INFO(
            "Last junction_ens_s, next overlap start_s, type:{:.3f}, {:.3f}, "
            "{};  adjacent two junctions are all IN_ROAD, not risk area.",
            risk_area_info_.junction_end_s, overlap.start_s,
            junction_ptr->Type());
        return;
      }

      LOG_INFO(
          "next junction is not IN_ROAD,adc is in right turn risk area: "
          "{:.3f},  {:.3f}, {:.3f}",
          risk_area_info_.risk_area_predeal_s,
          risk_area_info_.risk_area_start_s, risk_area_info_.risk_area_end_s);
      return;
    }
  }

  LOG_INFO("Not find risk area.");
  risk_area_info_.Reset();
}

bool SpeedOvertakeTurnRightDecider::IsInRightTurnRiskArea(TaskInfo& task_info) {
  SearchRiskArea(task_info);
  return (risk_area_info_.risk_area_predeal_s <= adc_current_s_ &&
          risk_area_info_.risk_area_end_s >= adc_current_s_);
}

void SpeedOvertakeTurnRightDecider::FindHighAndLowWithPolygon(
    const std::vector<Box2d>& adc_bounding_boxes, const Polygon2d& obstacle_box,
    bool* find_high, bool* find_low, std::size_t* high_index,
    std::size_t* low_index) {
  if (find_high == nullptr || find_low == nullptr || high_index == nullptr ||
      low_index == nullptr) {
    LOG_ERROR("input invalid");
    return;
  }
  while (*high_index >= *low_index) {
    if ((*find_high) && (*find_low)) {
      LOG_DEBUG("failed to find high and low index");
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

}  // namespace planning
}  // namespace neodrive

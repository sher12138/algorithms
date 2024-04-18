#include "speed_planner_common.h"

#include "common/util/time_logger.h"
#include "map_application/map_application.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/speed/speed_data.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/math/public_math/utility.h"

using namespace ::autobot::cyberverse;

namespace neodrive {
namespace planning {
namespace speed_planner_common {

bool Reverse(const Obstacle *obstacle, const PathData &path_data,
             double heading) {
  PathPoint closest_pt{};
  double path_heading_near_obs =
      path_data.path().query_closest_point(obstacle->center(), closest_pt)
          ? closest_pt.theta()
          : heading;
  // need consider pos is left or right
  double heading_diff =
      normalize_angle(obstacle->velocity_heading() - path_heading_near_obs);
  // only consider heading diff > 3/4 pi
  if (std::abs(heading_diff) < 2.3562) {
    return false;
  }
  return true;
}

void MaxDecelCalculator(const InsidePlannerData &inside_data,
                        const OutsidePlannerData &outside_data) {
  LOG_INFO("__max_decel_calculator__:");
  double max_deceleration_dynamic_by_obs = 1.0;
  double curr_v{inside_data.vel_v};
  double obs_v{0.0};
  double static_obs_s{outside_data.path_data->path().path_points().back().s() +
                      100.0};
  double dynamic_obs_s{outside_data.path_data->path().path_points().back().s() +
                       100.0};
  bool collided{false};
  const auto static_obstacles_decision =
      outside_data.speed_obstacle_context.static_obstacles_decision;
  const auto dynamic_obstacles_decision =
      outside_data.speed_obstacle_context.dynamic_obstacles_decision;
  LOG_INFO("static_obstacles collided cal :");
  for (const auto &obstacle_decision : static_obstacles_decision) {
    LOG_INFO("id, virtual, lower_points_size = [{}], {}, {}",
             obstacle_decision.obstacle.id(),
             obstacle_decision.obstacle.is_virtual(),
             obstacle_decision.lower_points.size());
    // if (obstacle_decision.obstacle.is_virtual()) continue;
    for (const auto &low_point : obstacle_decision.lower_points) {
      LOG_INFO("low_point_s {:.4f}:", low_point.first.s());
      static_obs_s = std::fmin(static_obs_s, low_point.first.s());
      collided = true;
    }
  }

  LOG_INFO("dynamic_obstacles collided cal :");
  for (const auto &obstacle_decision : dynamic_obstacles_decision) {
    LOG_INFO("id, virtual, lower_points_size = [{}], {}, {}",
             obstacle_decision.obstacle.id(),
             obstacle_decision.obstacle.is_virtual(),
             obstacle_decision.lower_points.size());
    for (std::size_t index = 0; index < obstacle_decision.lower_points.size();
         ++index) {
      LOG_DEBUG("low_point_s {:.4f}:",
                obstacle_decision.lower_points[index].first.s());
      if (dynamic_obs_s > obstacle_decision.lower_points[index].first.s()) {
        dynamic_obs_s = obstacle_decision.lower_points[index].first.s();
        obs_v = obstacle_decision.lower_points[index].second;
        if (obs_v < -0.01) obs_v = 0.0;
      }
      collided = true;
    }
  }

  LOG_INFO("static_obs_s = {:.4f}, dynamic_obs_s = {:.4f}", static_obs_s,
           dynamic_obs_s);
  double static_accel{max_deceleration_dynamic_by_obs};
  double dynamic_accel{max_deceleration_dynamic_by_obs};
  const auto &common_config =
      config::PlanningConfig::Instance()->plan_config().common;
  auto max_deceleration = common_config.max_av_minus * 1.1;

  if (static_obs_s < outside_data.path_data->path().path_points().back().s()) {
    if (static_obs_s < 2.2) {
      static_accel = max_deceleration;
    } else {
      static_accel = curr_v * curr_v / (2 * (static_obs_s - 2.0));
      static_accel = std::min(static_accel, max_deceleration);
    }
  }
  if (dynamic_obs_s < outside_data.path_data->path().path_points().back().s()) {
    if (dynamic_obs_s < 2.2) {
      dynamic_accel = max_deceleration;
    } else {
      dynamic_accel =
          (curr_v * curr_v - obs_v * obs_v) / (2 * (dynamic_obs_s - 2.0));
      dynamic_accel = std::min(dynamic_accel, max_deceleration);
    }
  }

  std::vector<double> candidate{static_accel, dynamic_accel,
                                max_deceleration_dynamic_by_obs};
  max_deceleration_dynamic_by_obs =
      *std::max_element(candidate.begin(), candidate.end());

  LOG_INFO("slow down, max_deceleration is {:.4f}",
           max_deceleration_dynamic_by_obs);
}

bool RoadBoundSafeCheck(const ReferenceLinePtr &reference_line,
                        const InsidePlannerData &inside_data,
                        OutsidePlannerData &outside_planner_data,
                        PathData *const path_data, bool &fail_safe_stop) {
  if (path_data == nullptr) {
    LOG_ERROR("input data invalid");
    return false;
  }
  if (path_data->path().path_points().size() < 2) {
    LOG_ERROR("input path points is empty");
    return false;
  }
  fail_safe_stop = false;

  // reverse lane detour check
  bool ignore_left_road_limit =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_lane_borrow_context()
          ->stage == DetourStageState::REVERSE_LANE_BORROWING;

  // distance, time calculator
  const auto &common_config =
      config::PlanningConfig::Instance()->plan_config().common;
  auto max_deceleration = common_config.max_av_minus * 1.1;

  const double vel = inside_data.vel_v;
  double forward_check_dis =
      VehicleParam::Instance()->front_edge_to_center() +
      vel * vel / max_deceleration / 2.0 +
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .speed_slow_down_decider_config.curb_check_distance;
  LOG_INFO("forward_check_dis = {:.4f}", forward_check_dis);
  const auto &path_pts = path_data->path().path_points();
  SLPoint sl_pt;
  double en_large_buffer = 0.0;
  std::vector<Vec2d> tmp_corners;
  do {
    for (std::size_t i = 0; i < path_pts.size(); ++i) {
      auto &pt = path_pts.at(i);
      if (pt.s() - path_pts.front().s() > forward_check_dis) {
        break;
      }
      // Generate box using points on path
      Box2d box = VehicleParam::Instance()->get_adc_bounding_box(
          {pt.x(), pt.y()}, pt.theta(), en_large_buffer, en_large_buffer,
          en_large_buffer);
      box.get_all_corners(&tmp_corners);

      for (auto &corner_pt : tmp_corners) {
        if (!reference_line->GetPointInFrenetFrameWithHeading(
                {corner_pt.x(), corner_pt.y()}, pt.theta(), &sl_pt)) {
          LOG_ERROR("get point to frenet coordinate failed");
          continue;
        }
        ReferencePoint tmp_refer_pt;
        if (!reference_line->GetNearestRefPoint(sl_pt.s(), &tmp_refer_pt)) {
          LOG_ERROR("GetNearestRefPoint failed, s: {:.4f}, index: {}",
                    sl_pt.s(), i);
          continue;
        }
        // double left_road_bound = tmp_refer_pt.left_road_bound();
        // double right_road_bound = tmp_refer_pt.right_road_bound();
        double left_road_bound =
            tmp_refer_pt.left_boundary_edge_type() == BoundaryEdgeType::VIRTUAL
                ? 100.0
                : (ignore_left_road_limit
                       ? tmp_refer_pt.left_reverse_road_bound()
                       : tmp_refer_pt.left_road_bound());
        double right_road_bound =
            tmp_refer_pt.right_boundary_edge_type() == BoundaryEdgeType::VIRTUAL
                ? 100.0
                : tmp_refer_pt.right_road_bound();

        if (sl_pt.l() >
                left_road_bound - FLAGS_planning_speed_border_shrink_dis ||
            sl_pt.l() <
                -right_road_bound + FLAGS_planning_speed_border_shrink_dis) {
          LOG_INFO(
              "at s: [{:.4f}], path s: {:.4f}, vehilce is out of bound, left "
              "road "
              "bound: {:.4f}, "
              "right road bound: {:.4f}, point l: {:.4f}",
              sl_pt.s(), pt.s(), left_road_bound, right_road_bound, sl_pt.l());
          // more debug log info
          LOG_DEBUG("point x: {:.4f}, y: {:.4f}, theta: {:.4f}", pt.x(), pt.y(),
                    pt.theta());
          LOG_DEBUG(
              "closest refer point x: {:.4f}, y: {:.4f}, theta: {:.4f}, s: "
              "{:.4f}",
              tmp_refer_pt.x(), tmp_refer_pt.y(), tmp_refer_pt.heading(),
              tmp_refer_pt.s());
          fail_safe_stop = true;
          break;
        }
      }
      if (fail_safe_stop) {
        break;
      }
    }
  } while (false);

  outside_planner_data.ask_take_over_immediately = false;
  double curr_time = common::util::TimeLogger::GetCurrentTimeseocnd();
  static double road_bound_collision_start_time = 0.0;
  if (road_bound_collision_start_time < 1.0) {
    road_bound_collision_start_time = curr_time;
  }
  if (!fail_safe_stop) {
    road_bound_collision_start_time = curr_time;
  } else {
    LOG_INFO("trigger RoadBound_safe_check");
    double delta_time = curr_time - road_bound_collision_start_time;
    if ((delta_time) > 60.0) {
      LOG_WARN(
          "vehicle road bound stop more than 60s: {:.4f}, ask for takeover",
          delta_time);
      outside_planner_data.ask_take_over_immediately = true;
    }
  }
  return true;
}

bool PerceptionCurbSafeCheck(const InsidePlannerData &inside_data,
                             PathData *const path_data) {
  if (path_data == nullptr) {
    LOG_ERROR("input data invalid");
    return false;
  }
  if (path_data->path().path_points().size() < 2) {
    LOG_ERROR("input path points is empty");
    return false;
  }
  if (inside_data.is_indoor) {
    LOG_DEBUG("is indoor");
    return true;
  }
  if (inside_data.is_in_the_park) {
    LOG_INFO("is in the park");
    return true;
  }
  double en_large_buffer = 0.0;
  const auto &curb_lines = DataCenter::Instance()
                               ->environment()
                               .perception_lanes_proxy()
                               .camera_curb_lines();
  const auto &path_pts = path_data->path().path_points();
  double vel = inside_data.vel_v;
  const auto &common_config =
      config::PlanningConfig::Instance()->plan_config().common;
  auto max_deceleration = common_config.max_av_minus * 1.1;
  double forward_check_dis =
      VehicleParam::Instance()->front_edge_to_center() +
      vel * vel / max_deceleration / 2.0 +
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .speed_slow_down_decider_config.curb_check_distance;
  LOG_INFO("forward_check_dis = {:.4f}", forward_check_dis);
  for (const auto &pt : path_pts) {
    if (pt.s() - path_pts.front().s() > forward_check_dis) {
      break;
    }
    // Generate box using points on path
    Box2d box = VehicleParam::Instance()->get_adc_bounding_box(
        {pt.x(), pt.y()}, pt.theta(), en_large_buffer, en_large_buffer,
        en_large_buffer);
    for (const auto &curb_line : curb_lines) {
      for (const auto &curb_segment : curb_line) {
        if (box.has_overlap(curb_segment)) {
          LOG_INFO("curb check: box center ({:.4f}, {:.4f})", box.center_x(),
                   box.center_y());
          std::vector<Vec2d> tmp_corners;
          box.get_all_corners(&tmp_corners);
          for (const auto &corner : tmp_corners) {
            LOG_INFO("corner ({:.4f}, {:.4f})", corner.x(), corner.y());
          }
          LOG_INFO("curb segment ({:.4f}, {:.4f}), ({:.4f}, {:.4f})",
                   curb_segment.start().x(), curb_segment.start().y(),
                   curb_segment.end().x(), curb_segment.end().y());
          LOG_INFO("Path box has overlap with curb.");
          return false;
        }
      }
    }
  }
  return true;
}

bool LmsSensorCheck(const InsidePlannerData &inside_data,
                    PathData *const path_data, bool &fail_safe_stop) {
  fail_safe_stop = false;
  if (inside_data.lms_pts.empty() || path_data == nullptr) {
    LOG_INFO("lms_pts.empty() || path_box.empty()");
    return false;
  }

  std::size_t point_in_box = 0;
  for (const auto &pt : path_data->path().path_points()) {
    if (pt.s() > 3.0) break;
    Box2d box = VehicleParam::Instance()->get_adc_bounding_box(
        {pt.x(), pt.y()}, pt.theta(), 0.0, 0.0, 0.0);
    std::vector<Vec2d> corners;
    box.get_all_corners(&corners);
    point_in_box = 0;
    for (std::size_t j = 0; j < inside_data.lms_pts.size(); ++j) {
      auto &currpt = inside_data.lms_pts[j];
      if (PointInPolygon(currpt.x(), currpt.y(), corners)) {
        ++point_in_box;
      }
    }
    if (point_in_box >= 3) {
      LOG_INFO("lms collision found, num {}, s: {:.4f}", point_in_box, pt.s());
      fail_safe_stop = true;
      break;
    }
  }

  return true;
}

bool SpeedDataSanityCheck(const std::vector<SpeedPoint> *speed_pts) {
  if (speed_pts == nullptr) return false;
  auto static_max_speed = DataCenter::Instance()->drive_strategy_max_speed();
  for (std::size_t i = 0; i < speed_pts->size(); ++i) {
    auto &pt = speed_pts->at(i);
    if (pt.v() > 3 * static_max_speed || pt.v() < -3 * static_max_speed ||
        pt.s() < -100.0 || pt.s() > 1000.0) {
      return false;
    }
  }
  return true;
}

bool FinalObstacleCollisionSanityCheck(
    TrajectoryPoint init_point, const PathData &path_data,
    const SpeedObstacleContext &obs_decision,
    const std::unordered_set<int> &need_ignore_obs_dy,
    const std::unordered_set<int> &need_ignore_obs_st,
    const double station_error, std::vector<SpeedPoint> &speed_point) {
  // get trajectory -->
  auto trajectory =
      CombineSpeedAndPath(init_point, path_data, speed_point, station_error);
  std::vector<Obstacle> valid_obs;
  std::unordered_set<int> reverse_obs_id;

  ExtractReverseObstacle(obs_decision.dynamic_obstacles_decision, path_data,
                         init_point.theta(), reverse_obs_id);

  ExtractValidObstacle(obs_decision, valid_obs);
  double collision_time = 5.0;
  return FinalObstacleCollisionSanityCheck(valid_obs, reverse_obs_id,
                                           trajectory, collision_time);
}  // namespace planning

bool FinalObstacleCollisionSanityCheck(
    TrajectoryPoint init_point, const PathData &path_data,
    const MotorwaySpeedObstacleContext &obs_decision,
    const std::unordered_set<int> &need_ignore_obs_dy,
    const std::unordered_set<int> &need_ignore_obs_st,
    const double station_error, std::vector<SpeedPoint> &speed_point) {
  // get trajectory -->
  auto trajectory =
      CombineSpeedAndPath(init_point, path_data, speed_point, station_error);
  std::vector<Obstacle> valid_obs;
  std::unordered_set<int> reverse_obs_id;

  ExtractReverseObstacle(obs_decision.multi_cipv_dynamic_obstacles_decision,
                         path_data, init_point.theta(), reverse_obs_id);
  ExtractValidObstacle(obs_decision, valid_obs);

  double collision_time = 5.0;
  return FinalObstacleCollisionSanityCheck(valid_obs, reverse_obs_id,
                                           trajectory, collision_time);
}

bool FinalObstacleCollisionSanityCheck(
    const std::vector<Obstacle> &need_check_obs_list,
    const std::unordered_set<int> &reverse_obs_id,
    const PublishableTrajectory &combined_trajectory, double &collision_time,
    double query_time) {
  const auto &traj_points = combined_trajectory.trajectory_points();
  if (traj_points.empty() || need_check_obs_list.empty()) {
    return true;
  }
  std::vector<double> sign(need_check_obs_list.size(), -1.0);

  auto obs_stragtegy_generate = [&traj_points](const Obstacle &obs) -> double {
    // base heading ensure skip
    if (std::abs((obs.velocity_heading() - traj_points[0].theta())) < 1.05)
      return 0.3;
  };

  for (size_t i = 0; i < need_check_obs_list.size(); ++i) {
    sign[i] = obs_stragtegy_generate(need_check_obs_list[i]);
  }

  for (const auto &tra_point : traj_points) {
    const auto curr_relative_time = tra_point.relative_time();
    if (curr_relative_time < 1e-3) continue;
    if (curr_relative_time > query_time) break;

    // need check obs, generate adc_box
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {tra_point.x(), tra_point.y()}, tra_point.theta(), 0.1, 0.1, 0.1);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build adc bounding box from path point at s[{:.3f}] failed",
                tra_point.s());
      // skip this point, check next,
      continue;
    }
    // find_need_check_obs(tra_point);
    const auto ego_polygon = Polygon2d(adc_box);
    // find must need check obs

    for (size_t ind = 0; ind < need_check_obs_list.size(); ++ind) {
      if (std::abs(sign[ind] + 1.0) > 1e-3) {
        if (curr_relative_time < sign[ind]) continue;  // no check!
      }
      if (reverse_obs_id.find(need_check_obs_list[ind].id()) !=
              reverse_obs_id.end() &&
          curr_relative_time > 0.5) {
        // LOG_INFO("reverse obs {}", need_check_obs_list[ind].id());
        continue;
      }
      const auto &obs = need_check_obs_list[ind];
      const double x = obs.center().x() + std::cos(obs.velocity_heading()) *
                                              obs.speed() * curr_relative_time;
      const double y = obs.center().y() + std::sin(obs.velocity_heading()) *
                                              obs.speed() * curr_relative_time;

      Polygon2d obs_polygon = Utility::get_trajectory_point_polygon(
          obs.center(), {x, y}, obs.velocity_heading(), obs.velocity_heading(),
          obs.polygon());
      if (ego_polygon.has_overlap(obs_polygon)) {
        collision_time = std::min(collision_time, curr_relative_time);
        LOG_INFO(
            "can't pass sanity check, ego trajecotry has collision with obs "
            "{} "
            "at time {:.3f},{:.3f},{:.3f},{:.3f}",
            obs.id(), curr_relative_time, x, y, tra_point.s());
        return false;
      }
    }
  }
  LOG_INFO("pass sanity check!");
  return true;
}

// generate used
PublishableTrajectory CombineSpeedAndPath(
    TrajectoryPoint init_point, const PathData &path_data,
    const std::vector<SpeedPoint> &speed_pts, const double station_error,
    const bool relative_time) {
  bool is_exist_station_error = false;
  if (station_error < -0.4) {  // only consider brake nagtive
    is_exist_station_error = true;
    double init_s = init_point.s();
    init_point.set_s(init_point.s() - station_error);
    LOG_INFO(
        "station_error = {:.3f}, traj init point from s = {:.3f}, to s= {:.3f}",
        station_error, init_s, init_point.s());
  }
  PublishableTrajectory combined_trajectory;
  SpeedData speed_data{};
  for (size_t i = 0; i < speed_pts.size(); i++) {
    speed_data.mutable_speed_vector()->push_back(speed_pts[i]);
  }
  double time_resolution = 0.02;
  double time_length = 5.0;
  for (double cur_rel_time = time_resolution;
       cur_rel_time < speed_data.total_time() && cur_rel_time <= time_length;
       cur_rel_time += time_resolution) {
    SpeedPoint speed_point;
    if (!speed_data.get_speed_point_with_time(cur_rel_time, &speed_point)) {
      LOG_ERROR("Fail to get speed point with relative time {:.3f}",
                cur_rel_time);
      break;
    }
    if (speed_point.s() > path_data.path().param_length()) {
      break;
    }
    PathPoint path_point;
    if (!path_data.get_path_point_with_path_s(speed_point.s(), &path_point)) {
      LOG_ERROR(
          "Fail to get path data with s {:.3f} at t {:.3f}, path total length "
          "{:.3f} ",
          speed_point.s(), speed_point.t(), path_data.path().param_length());
      break;
    }
    path_point.set_s(init_point.s() + path_point.s());

    TrajectoryPoint trajectory_point(path_point, speed_point.v(),
                                     speed_point.a(), speed_point.j(),
                                     speed_point.t());
    combined_trajectory.append_trajectory_point(trajectory_point);
  }

  return combined_trajectory;
}

void ExtractValidObstacle(const MotorwaySpeedObstacleContext &speed_obs_context,
                          std::vector<Obstacle> &valid_obs) {
  const auto &need_ignore_obs_dy = speed_obs_context.ignore_dynamic_obs_id;
  const auto &need_ignore_obs_st = speed_obs_context.ignore_static_obs_id;
  const auto &need_ignore_obs_only_in_backup =
      speed_obs_context.speed_backup_ignore_dynamic_obs_id;

  for (const auto &obs :
       speed_obs_context.multi_cipv_dynamic_obstacles_decision) {
    if (need_ignore_obs_dy.find(obs.obstacle.id()) ==
            need_ignore_obs_dy.end() &&
        need_ignore_obs_only_in_backup.find(obs.obstacle.id()) ==
            need_ignore_obs_only_in_backup.end()) {
      valid_obs.push_back(obs.obstacle);
    }
  }

  for (const auto &obs :
       speed_obs_context.multi_cipv_static_obstacles_decision) {
    if (need_ignore_obs_st.find(obs.obstacle.id()) ==
            need_ignore_obs_st.end() &&
        need_ignore_obs_only_in_backup.find(obs.obstacle.id()) ==
            need_ignore_obs_only_in_backup.end()) {
      valid_obs.push_back(obs.obstacle);
    }
  }

  for (const auto &obs :
       speed_obs_context.multi_cipv_virtual_obstacle_decision) {
    if (need_ignore_obs_only_in_backup.find(obs.obstacle.id()) ==
        need_ignore_obs_only_in_backup.end()) {
      valid_obs.push_back(obs.obstacle);
    }
  }
  LOG_INFO("Valid obs number is {}", valid_obs.size());
  for (const auto &obs : valid_obs) {
    LOG_INFO("need check obs:{}", obs.id());
  }
}

void ExtractValidObstacle(const SpeedObstacleContext &speed_obs_context,
                          std::vector<Obstacle> &valid_obs) {
  const auto &need_ignore_obs_dy =
      speed_obs_context.dp_st_map_ignore_dynamic_obs_id;
  const auto &need_ignore_obs_st =
      speed_obs_context.dp_st_map_ignore_static_obs_id;
  const auto &need_ignore_obs_only_in_backup =
      speed_obs_context.speed_backup_ignore_dynamic_obs_id;

  for (const auto &obs : speed_obs_context.dynamic_obstacles_decision) {
    if (need_ignore_obs_dy.find(obs.obstacle.id()) ==
            need_ignore_obs_dy.end() &&
        need_ignore_obs_only_in_backup.find(obs.obstacle.id()) ==
            need_ignore_obs_only_in_backup.end()) {
      valid_obs.push_back(obs.obstacle);
    }
  }

  for (const auto &obs : speed_obs_context.static_obstacles_decision) {
    if (need_ignore_obs_st.find(obs.obstacle.id()) ==
            need_ignore_obs_st.end() &&
        need_ignore_obs_only_in_backup.find(obs.obstacle.id()) ==
            need_ignore_obs_only_in_backup.end()) {
      valid_obs.push_back(obs.obstacle);
    }
  }

  for (const auto &obs : speed_obs_context.virtual_obstacle_decision) {
    if (need_ignore_obs_only_in_backup.find(obs.obstacle.id()) ==
        need_ignore_obs_only_in_backup.end()) {
      valid_obs.push_back(obs.obstacle);
    }
  }
  LOG_INFO("Valid obs number is {}", valid_obs.size());
  for (const auto &obs : valid_obs) {
    LOG_INFO("exist obs id {}", obs.id());
  }
}

bool IsStaticObsNeedIgnore(const SLPoint &vel_sl, const Boundary &obs_boundary,
                           const double &left_road_bound,
                           const double &right_road_bound, bool &is_valid,
                           const bool is_forward) {
  is_valid = false;
  if (obs_boundary.start_l() > left_road_bound ||
      obs_boundary.end_l() < right_road_bound) {
    LOG_INFO("static obs is outside of road, ignore");
    is_valid = true;
    return true;
  }

  double valid_s = vel_sl.s();
  if (is_forward) {
    valid_s -= VehicleParam::Instance()->back_edge_to_center();
  } else {
    valid_s -= VehicleParam::Instance()->front_edge_to_center();
  }

  if (obs_boundary.end_s() < valid_s) {
    LOG_INFO("static obs is behind adc, ignore");
    is_valid = true;
  }
  return true;
}

bool IsDynamicObsNeedIgnore(const InsidePlannerData &inside_data,
                            const Obstacle &obstacle, bool &is_valid) {
  is_valid = false;
  SLPoint adc_sl_point = inside_data.init_sl_point;

  double adc_s_on_reference_line = adc_sl_point.s();
  if (inside_data.is_reverse_driving) {
    adc_s_on_reference_line -= VehicleParam::Instance()->front_edge_to_center();
  } else {
    adc_s_on_reference_line -= VehicleParam::Instance()->back_edge_to_center();
  }

  double front_buffer = std::max(inside_data.vel_v * 8.0, 15.0);

  double end_s = obstacle.PolygonBoundary().end_s();
  double start_s = obstacle.PolygonBoundary().start_s();

  double heading_diff =
      normalize_angle(obstacle.velocity_heading() - inside_data.vel_heading);
  double project_vel = obstacle.speed() * std::cos(heading_diff);

  LOG_INFO(
      "dynamic obs id[{}], s_s: {:.4f}, e_s: {:.4f}, adc_s: {:.4f}, "
      "head_diff: "
      "{:.4f}, proj_v: {:.4f}",
      obstacle.id(), start_s, end_s, adc_s_on_reference_line, heading_diff,
      project_vel);

  if (std::abs(heading_diff) > M_PI_2) {
    LOG_INFO("reverse obs");
    if (end_s <= adc_s_on_reference_line + VehicleParam::Instance()->length()) {
      // back of vehicle, ignore
      is_valid = true;
    }
    if (project_vel < -0.5) {
      double expect_obs_forward_s = std::abs(project_vel) * 4.0;
      if (start_s - expect_obs_forward_s >=
          adc_s_on_reference_line + VehicleParam::Instance()->length() +
              front_buffer) {
        // front of vehicle, ignore
        is_valid = true;
      }
    }
  } else {
    LOG_INFO("forward obs");
    if (end_s <=
        adc_s_on_reference_line + VehicleParam::Instance()->length() * 0.5) {
      // back of vehicle, ignore
      is_valid = true;
    } else if (start_s >= adc_s_on_reference_line +
                              VehicleParam::Instance()->length() +
                              front_buffer) {
      // front of vehicle, ignore
      is_valid = true;
    } else {
      // complex situation, if the obs is totaly besides vehicle, ignore
      // convert to local
      std::vector<Vec2d> box_corner = obstacle.polygon_corners();
      Boundary obs_box_local;
      double x_target_local = 0.0;
      double y_target_local = 0.0;

      for (const auto &tmp_corner : box_corner) {
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
        if (project_vel <= inside_data.vel_v * 0.8) {
          is_valid = true;
          LOG_INFO("obs is besides vehicle, ignore");
          LOG_INFO(
              "s_s: {:.4f}, e_s: {:.4f}, s_l: {:.4f}, e_l: {:.4f}, "
              "threshold[{:.4f}, "
              "{:.4f}, |{:.4f}|, |{:.4f}|]",
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
  LOG_INFO("igore this obs?: {}", is_valid);

  return true;
}

void ExtractReverseObstacle(
    const std::vector<MotorwayMultiCipvSpeedObstacleDecision>
        &has_collsion_obstacle,
    const PathData &path_data, double heading,
    std::unordered_set<int> &reverse_obs_id) {
  for (const auto &dy_dec : has_collsion_obstacle) {  // decision ignore
    if (Reverse(&dy_dec.obstacle, path_data, heading)) {
      LOG_INFO("reverse obs {},ingore!", dy_dec.obstacle.id());
      reverse_obs_id.insert(dy_dec.obstacle.id());
    }  // reverse : maybe need re calculate
  }
}

void ExtractReverseObstacle(
    const std::vector<SpeedObstacleDecision> &has_collsion_obstacle,
    const PathData &path_data, double heading,
    std::unordered_set<int> &reverse_obs_id) {
  for (const auto &dy_dec : has_collsion_obstacle) {  // decision ignore
    if (Reverse(&dy_dec.obstacle, path_data, heading)) {
      reverse_obs_id.insert(dy_dec.obstacle.id());
    }  // reverse : maybe need re calculate
  }
}

bool GetAdcEnlargeBuffer(const InsidePlannerData &inside_data,
                         const std::size_t curr_mode, double *en_large_buffer,
                         double *front_en_large_buffer,
                         double *back_en_large_buffer) {
  if (en_large_buffer == nullptr || front_en_large_buffer == nullptr ||
      back_en_large_buffer == nullptr)
    return false;
  *en_large_buffer = FLAGS_planning_speed_plan_enlarge_self_buffer;
  *front_en_large_buffer = *en_large_buffer;
  *back_en_large_buffer = *en_large_buffer;

  if (curr_mode == MultiLevelMode::MLM_HIGHSPEED) {
    if (inside_data.is_reverse_driving) {
      *back_en_large_buffer += 0.1;
    } else {
      *front_en_large_buffer += 0.1;
    }
    *en_large_buffer += 0.1;
  } else if (curr_mode == MultiLevelMode::MLM_NORMAL) {
  } else if (curr_mode == MultiLevelMode::MLM_PASSINGBY) {
    *en_large_buffer = FLAGS_planning_speed_plan_aggressive_enlarge_self_buffer;
    if (inside_data.is_reverse_driving) {
      *front_en_large_buffer = std::min(0.1, *en_large_buffer);
      *back_en_large_buffer = *en_large_buffer;
    } else {
      *front_en_large_buffer = *en_large_buffer;
      *back_en_large_buffer = std::min(0.1, *en_large_buffer);
    }
  } else if (curr_mode == MultiLevelMode::MLM_PASSINGTHROUGH) {
    *en_large_buffer = FLAGS_planning_speed_plan_risk_enlarge_self_buffer;
    if (inside_data.is_reverse_driving) {
      *front_en_large_buffer = std::min(0.1, *en_large_buffer);
      *back_en_large_buffer = *en_large_buffer;
    } else {
      *front_en_large_buffer = *en_large_buffer;
      *back_en_large_buffer = std::min(0.1, *en_large_buffer);
    }
  } else if (curr_mode == MultiLevelMode::MLM_STRONGPASSING) {
    *en_large_buffer = FLAGS_planning_speed_plan_risk_enlarge_self_buffer;
    if (inside_data.is_reverse_driving) {
      *front_en_large_buffer = std::min(0.1, *en_large_buffer);
      *back_en_large_buffer = *en_large_buffer - 0.05;
    } else {
      *front_en_large_buffer = *en_large_buffer - 0.05;
      *back_en_large_buffer = std::min(0.1, *en_large_buffer);
    }
  } else {
    LOG_ERROR("inside_data.curr_multi_level is out of 0 and 4.");
    return false;
  }
  return true;
}

std::string GetCurrentModeName(const std::size_t mode) {
  if (mode == MultiLevelMode::MLM_HIGHSPEED) {
    return "MLM_HIGHSPEED";
  } else if (mode == MultiLevelMode::MLM_NORMAL) {
    return "MLM_NORMAL";
  } else if (mode == MultiLevelMode::MLM_PASSINGBY) {
    return "MLM_PASSINGBY";
  } else if (mode == MultiLevelMode::MLM_PASSINGTHROUGH) {
    return "MLM_PASSINGTHROUGH";
  } else if (mode == MultiLevelMode::MLM_STRONGPASSING) {
    return "MLM_STRONGPASSING";
  }
  LOG_ERROR("get mode failed.");
  return "";
}

bool CreepProcess(const InsidePlannerData &inside_planner_data,
                  EMPlanningData *planning_data) {
  if (!FLAGS_planning_enable_creep_speed_process) {
    LOG_INFO("creep_process is not enabled");
    return true;
  }
  double curr_v = inside_planner_data.vel_v;

  if (std::abs(curr_v) >= 0.5) {
    LOG_DEBUG("fabs(curr_v) >= 0.5, skip creep");
    return true;
  }
  // in creep process?
  auto final_traj = planning_data->mutable_computed_trajectory();
  double final_s = 0.0;
  double final_speed = 0.0;
  double average_speed = 0.0;
  std::size_t index = 0;
  for (index = 0; index < final_traj->trajectory_points().size(); index++) {
    auto &pt = final_traj->trajectory_points().at(index);
    if (pt.relative_time() > 1000.0) break;

    average_speed += pt.velocity();
    final_s = pt.s();
    final_speed = pt.velocity();
  }
  if (index != 0) {
    average_speed /= index;
  }

  if (final_s < 0.5 || final_s > 2.0) {
    LOG_DEBUG("final_s[{:.4f}], skip creep", final_s);
    return true;
  }
  if (final_speed < 1.e-3) {
    LOG_DEBUG("final_speed[{:.4f}], skip creep", final_speed);
    return true;
  }
  if (average_speed >= 0.45) {
    LOG_DEBUG("average_speed[{:.4f}], skip creep", average_speed);
    return true;
  }

  // enable creep
  LOG_INFO("before creep process");
  for (std::size_t i = 0; i < final_traj->trajectory_points().size(); i += 10) {
    auto &pt = final_traj->trajectory_points().at(i);
    if (pt.relative_time() > 1000.0) break;
    LOG_DEBUG("t: {:.4f}, s: {:.4f}, v: {:.4f}", pt.relative_time(), pt.s(),
              pt.velocity());
  }

  for (index = 0; index < final_traj->trajectory_points().size(); index++) {
    auto &pt = final_traj->mutable_trajectory_points()->at(index);
    if (pt.relative_time() > 1000.0) {
      pt.set_velocity(0.0);
    }
    if (pt.s() < final_s) {
      pt.set_velocity(0.5);
    } else {
      pt.set_velocity(0.0);
    }
  }

  LOG_INFO("after creep process");
  for (std::size_t i = 0; i < final_traj->trajectory_points().size(); i += 10) {
    auto &pt = final_traj->trajectory_points().at(i);
    if (pt.relative_time() > 1000.0) break;
    LOG_DEBUG("t: {:.4f}, s: {:.4f}, v: {:.4f}", pt.relative_time(), pt.s(),
              pt.velocity());
  }

  return true;
}

double CalSafeSpeedDeltaParallelAdc(const double heading_diff) {
  static constexpr double kMaxHeadingDiff{1.52};  // rad
  static constexpr double kMinSpeedDelta{0.3};    // m/s
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  double heading_diff_clamp =
      std::min(std::abs(heading_diff) *
                   plan_config.speed_plan.speed_delta_and_headingdiff_ratio,
               kMaxHeadingDiff);
  return std::max(std::tan(heading_diff_clamp), kMinSpeedDelta);
}

bool IsApproachingAdc(const Obstacle *const obs,
                      const InsidePlannerData &inside_data) {
  static constexpr double kApproximateEqualZero{1e-3};
  if (obs->speed() < kApproximateEqualZero) {
    return false;
  }

  auto cross_prod = [](double heading_1, double heading_2) {
    return std::cos(heading_1) * std::sin(heading_2) -
           std::cos(heading_2) * std::sin(heading_1);
  };

  if (obs->center_sl().l() < inside_data.init_sl_point.l()) {
    return cross_prod(inside_data.vel_heading, obs->velocity_heading()) >
           kApproximateEqualZero;
  }
  return cross_prod(obs->velocity_heading(), inside_data.vel_heading) >
         kApproximateEqualZero;
}

double GetObs2AdcLateralDis(const Obstacle &obs, double adc_current_l) {
  double obs_l_span = obs.max_l() - obs.min_l();
  return std::max(obs.max_l(),
                  adc_current_l + VehicleParam::Instance()->width() / 2) -
         std::min(obs.min_l(),
                  adc_current_l - VehicleParam::Instance()->width() / 2) -
         VehicleParam::Instance()->width() - obs_l_span;
}

bool InMergingArea(TaskInfo &task_info) {
  const auto &traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if (traffic_conflict_zone_context.type ==
          TrafficConflictZoneContext::connectionType::Merging ||
      traffic_conflict_zone_context.type ==
          TrafficConflictZoneContext::connectionType::NMerging) {
    return true;
  }
  if (traffic_conflict_zone_context.near_type ==
          TrafficConflictZoneContext::connectionType::Merging ||
      traffic_conflict_zone_context.near_type ==
          TrafficConflictZoneContext::connectionType::NMerging) {
    return true;
  }
  if (traffic_conflict_zone_context.rear_type ==
          TrafficConflictZoneContext::connectionType::Merging ||
      traffic_conflict_zone_context.rear_type ==
          TrafficConflictZoneContext::connectionType::NMerging) {
    return true;
  }
  return false;
}

bool InDivergingArea(TaskInfo &task_info) {
  const auto &traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if (traffic_conflict_zone_context.type ==
          TrafficConflictZoneContext::connectionType::Diverging ||
      traffic_conflict_zone_context.type ==
          TrafficConflictZoneContext::connectionType::NDiverging) {
    return true;
  }
  if (traffic_conflict_zone_context.near_type ==
          TrafficConflictZoneContext::connectionType::Diverging ||
      traffic_conflict_zone_context.near_type ==
          TrafficConflictZoneContext::connectionType::NDiverging) {
    return true;
  }
  if (traffic_conflict_zone_context.rear_type ==
          TrafficConflictZoneContext::connectionType::Diverging ||
      traffic_conflict_zone_context.rear_type ==
          TrafficConflictZoneContext::connectionType::NDiverging) {
    return true;
  }
  return false;
}

bool JudgeDirectionCarWillGo(const ReferenceLinePtr &ref_ptr,
                             const InsidePlannerData &inside_data,
                             GoDirectionType &car_go_direction) {
  if (nullptr == ref_ptr) {
    LOG_DEBUG("reference_line is nullptr");
    return false;
  }

  const auto &speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  double front_check_buffer =
      std::max(inside_data.vel_v * speed_plan_config.judge_turn_min_time,
               static_cast<double>(speed_plan_config.judge_turn_min_length));
  std::size_t start_index{0}, end_index{0};
  if (!ref_ptr->GetStartEndIndexBySLength(inside_data.init_sl_point.s(),
                                          front_check_buffer, &start_index,
                                          &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return false;
  }

  int left_continuous_num{0};
  int right_continuous_num{0};
  for (std::size_t i = start_index; i <= end_index; ++i) {
    const auto &p = ref_ptr->ref_points()[i];
    left_continuous_num = p.kappa() > speed_plan_config.judge_turn_min_kappa
                              ? (left_continuous_num + 1)
                              : 0;

    right_continuous_num =
        p.kappa() < -1.0 * speed_plan_config.judge_turn_min_kappa
            ? (right_continuous_num + 1)
            : 0;

    if (left_continuous_num >=
        speed_plan_config.judge_turn_continuous_point_num) {
      LOG_INFO("adc turn left.");
      car_go_direction = GoDirectionType::TURN_LEFT;
      return true;
    }
    if (right_continuous_num >=
        speed_plan_config.judge_turn_continuous_point_num) {
      LOG_INFO("adc turn right.");
      car_go_direction = GoDirectionType::TURN_RIGHT;
      return true;
    }
  }
  car_go_direction = GoDirectionType::GO_STRAIGHT;
  return true;
}

bool ObsEgoAlongPathHeadingDiff(const Obstacle *obstacle,
                                const PathData &path_data, double ego_heading,
                                double &heading_diff, double &path_heaeding) {
  PathPoint closest_pt{};
  double path_heading_near_obs =
      path_data.path().query_closest_point(obstacle->center(), closest_pt)
          ? closest_pt.theta()
          : ego_heading;
  path_heaeding = normalize_angle(path_heading_near_obs - obstacle->heading() );
  heading_diff = normalize_angle(path_heading_near_obs - obstacle->heading() );
  return true;
}

bool ObsEgoAlongPathHeadingDiffBaseSpeedHeading(const Obstacle *obstacle,
                                                const PathData &path_data,
                                                double ego_heading,
                                                double &heading_diff) {
  PathPoint closest_pt{};
  double path_heading_near_obs =
      path_data.path().query_closest_point(obstacle->center(), closest_pt)
          ? closest_pt.theta()
          : ego_heading;
  heading_diff =
      normalize_angle(path_heading_near_obs - obstacle->velocity_heading());
  return true;
}

bool ObsEgoAlongPathLateralMinDistance(const Obstacle *obstacle,
                                       const PathData &path_data,
                                       double &lateral_min_diff) {
  FrenetFramePoint pt;
  if (path_data.frenet_path().interpolate(obstacle->min_s(), pt)) {
    lateral_min_diff = pt.l();
    return true;
  };
  return false;
}

bool GetObsToPathLatDis(const std::vector<Boundary> &adc_sl_boundaries,
                        const Obstacle &obs, double &obs_path_lat_dis) {
  if (adc_sl_boundaries.empty()) {
    return false;
  }

  obs_path_lat_dis = std::numeric_limits<double>::infinity();

  auto bs_idx = [&adc_sl_boundaries](auto s) -> size_t {
    if (s < adc_sl_boundaries.front().start_s()) return 0;
    std::size_t m = adc_sl_boundaries.size();
    if (s > adc_sl_boundaries.back().end_s()) return m - 1;

    std::size_t l = 0, r = m - 1;
    while (l < r) {
      int mid = l + (r - l) / 2;
      if (adc_sl_boundaries[mid].start_s() < s) {
        l = mid + 1;
      } else {
        r = mid;
      }
    }
    return l;
  };

  std::size_t adc_start_index = bs_idx(obs.PolygonBoundary().start_s());
  std::size_t adc_end_index = bs_idx(obs.PolygonBoundary().end_s());
  for (std::size_t index = adc_start_index; index < adc_end_index; ++index) {
    double dis = adc_sl_boundaries[index].distance_to(obs.PolygonBoundary());
    obs_path_lat_dis = std::min(obs_path_lat_dis, dis);
  }

  return true;
}

std::vector<Vec2d> GetOdoLidarPoints(
    const neodrive::global::perception::Freespace &freespace) {
  std::vector<Vec2d> lidar_points, ans;
  for (const auto &p : freespace.freespace()) {
    lidar_points.emplace_back(p.x(), p.y());
  }
  auto &pose = DataCenter::Instance()
                   ->environment()
                   .perception_proxy()
                   .Perception()
                   .odom_pose();
  auto &loc = pose.position();
  auto &q = pose.orientation();
  const double theta =
      std::atan2(2. * (q.qw() * q.qz() + q.qx() * q.qy()),
                 1. - 2. * (q.qy() * q.qy() + q.qz() * q.qz()));
  const double st{std::sin(theta)}, ct{std::cos(theta)};
  for (const auto &point : lidar_points) {
    double x_odom{point.x() * ct - point.y() * st + loc.x()};
    double y_odom{point.x() * st + point.y() * ct + loc.y()};
    ans.emplace_back(x_odom, y_odom);
  }
  return ans;
}

bool UnprotectStright(TaskInfo &task_info, double detect_distance) {
  // straight and no traffic light
  return !(scenario_common::IsFrontHasLaneTurn(task_info, detect_distance) ||
           scenario_common::IsFrontHasTrafficLight(task_info, detect_distance));
}

bool EgoInTParkCross(TaskInfo &task_info) {
  // straight, no traffic light and have divide or merge
  auto plan_config_ptr = &config::PlanningConfig::Instance()->plan_config();
  double detect_distance =
      plan_config_ptr->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }

  // junction
  for (const auto &[junction_ptr, overlap] :
       task_info.reference_line()->junctions()) {
    if (overlap.object_id != 0 &&
        (task_info.curr_sl().s() + detect_distance > overlap.start_s &&
         task_info.curr_sl().s() < overlap.end_s)) {
      if (junction_ptr->Type() ==
              static_cast<uint32_t>(
                  autobot::cyberverse::Junction::JunctionType::T_CROSS_ROAD) &&
          UnprotectStright(task_info, detect_distance)) {
        // divide
        const auto &successor_ids =
            task_info.frame()
                ->outside_planner_data()
                .traffic_conflict_zone_context.successor_ids;
        for (const auto &successor : successor_ids) {
          bool in_this_junction = false;
          const auto &junctions = successor.lane_ptr->Junctions();
          for (uint32_t i = 0; i < junctions.size(); ++i) {
            if (junctions[i].object_id == overlap.object_id) {
              LOG_DEBUG("in_this_junction");
              in_this_junction = true;
              break;
            }
          }
          if (!in_this_junction) {
            LOG_DEBUG("is not in_this_junction");
            continue;
          }
          if (static_cast<Lane::TurningType>(successor.lane_ptr->TurnType()) !=
              Lane::TurningType::NO_TURN) {
            //   NO_TURN = 1, LEFT_TURN = 2, RIGHT_TURN = 3, U_TURN = 4,
            LOG_INFO("successor turn type is {}",
                     successor.lane_ptr->TurnType());
            return true;
          }
        }

        // merge
        const auto &merging_ids =
            task_info.frame()
                ->outside_planner_data()
                .traffic_conflict_zone_context.merging_ids;
        for (const auto &merging : merging_ids) {
          bool in_this_junction = false;
          const auto &junctions = merging.lane_ptr->Junctions();
          for (uint32_t i = 0; i < junctions.size(); ++i) {
            if (junctions[i].object_id == overlap.object_id) {
              LOG_DEBUG("in_this_junction");
              in_this_junction = true;
              break;
            }
          }
          if (!in_this_junction) {
            LOG_DEBUG("is not in_this_junction");
            continue;
          }
          if (static_cast<Lane::TurningType>(merging.lane_ptr->TurnType()) !=
              Lane::TurningType::NO_TURN) {
            LOG_INFO("merging turn type is {}", merging.lane_ptr->TurnType());
            return true;
          }
        }
        return false;
      }
      break;
    }
  }
  return false;
}

bool EgoInTNonParkCross(TaskInfo &task_info) {
  // T_cross_road and have traffic light or turn
  auto plan_config_ptr = &config::PlanningConfig::Instance()->plan_config();
  double detect_distance =
      plan_config_ptr->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }

  // junction
  for (const auto &[junction_ptr, overlap] :
       task_info.reference_line()->junctions()) {
    if (overlap.object_id != 0 &&
        (task_info.curr_sl().s() + detect_distance > overlap.start_s &&
         task_info.curr_sl().s() < overlap.end_s)) {
      if (junction_ptr->Type() ==
              static_cast<uint32_t>(
                  autobot::cyberverse::Junction::JunctionType::T_CROSS_ROAD) &&
          !UnprotectStright(task_info, detect_distance)) {
        return true;
      }
      break;
    }
  }
  return false;
}

bool EgoInIntersection(TaskInfo &task_info) {
  // in cross_road
  auto plan_config_ptr = &config::PlanningConfig::Instance()->plan_config();
  double detect_distance =
      plan_config_ptr->intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance = std::max(
        detect_distance,
        plan_config_ptr->intersection_scenario.approach_time_threshold *
            task_info.last_frame()->inside_planner_data().vel_v);
  }

  // junction
  for (const auto &[junction_ptr, overlap] :
       task_info.reference_line()->junctions()) {
    if (overlap.object_id != 0 &&
        (task_info.curr_sl().s() + detect_distance > overlap.start_s &&
         task_info.curr_sl().s() < overlap.end_s)) {
      if (junction_ptr->Type() ==
          static_cast<uint32_t>(
              autobot::cyberverse::Junction::JunctionType::CROSS_ROAD)) {
        return true;
      }
      break;
    }
  }
  return false;
}

}  // namespace speed_planner_common
}  // namespace planning
}  // namespace neodrive

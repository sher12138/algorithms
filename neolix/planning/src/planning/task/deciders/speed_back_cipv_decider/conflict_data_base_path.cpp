#include "conflict_data_base_path.h"

#include "src/planning/common/math/util.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {

namespace {

void VisObsPredictionSegment(const Vec2d& start_pt, const Vec2d& end_pt) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("PrediceitonSegment");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto polyline = event->mutable_polyline()->Add();
  set_pt(polyline->add_point(), start_pt);
  set_pt(polyline->add_point(), end_pt);
}

void VisAdcCornerPoints(const std::vector<Vec2d>& adc_pts) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("MergingAdcCornerPts");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& pt : adc_pts) {
    set_pts(event, pt);
  }
}

}  // namespace

BackDataBasePath::BackDataBasePath()
    : BackDataInterface("BackDataBaseOnPath") {}

std::vector<ConnectionConflictInfo>
BackDataBasePath::ComputeConflictMergeInData(TaskInfo& task_info) {
  std::vector<ConnectionConflictInfo> ans{};
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if ((!HasMergingArea(task_info))) {
    LOG_INFO("No Merging Area!");
    return ans;
  }
  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return ans;
  }

  const auto& merging_ids = traffic_conflict_zone_context.merging_ids;
  std::unordered_set<uint64_t> merging_lane_ids;
  for (auto iter = merging_ids.begin(); iter != merging_ids.end(); iter++) {
    merging_lane_ids.insert(iter->id);
  }
  for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
    if (!interactive_agent_ids_.count(obs->id())) {
      continue;
    }
    if (merging_lane_ids.find(obs->matched_lane_id()) ==
        merging_lane_ids.end()) {
      continue;
    }
    ConnectionConflictInfo c;
    c.lane_id = obs->matched_lane_id();
    CalAgentInfo(obs, &c.agent);
    SetConflictParam(&c);
    c.agent.id = obs->id();
    if (!CollisionCheckWithObstaclePolyline(
            task_info.current_frame()->inside_planner_data(), *obs,
            task_info.current_frame()->mutable_outside_planner_data(),
            &c.conflict_area_bound)) {
      std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
      c.conflict_area_bound = cab;
    }

    ans.emplace_back(c);
  }

  return ans;
}

std::vector<ConnectionConflictInfo>
BackDataBasePath::ComputeConflictMeetingData(TaskInfo& task_info) {
  std::vector<ConnectionConflictInfo> ans{};
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return ans;
  }
  for (int i = 0; i < 2; ++i) {
    if (traffic_conflict_zone_context.meeting_geoinfo[i]) {
      auto mg = traffic_conflict_zone_context.meeting_geoinfo[i];
      for (auto it = mg->begin(); it != mg->end(); it++) {
        auto& mlc = it->first;
        // dynamic obs
        for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
          if (!interactive_agent_ids_.count(obs->id())) {
            continue;
          }
          if (obs->matched_lane_id() != mlc.lane_id) {
            continue;
          }
          if (std::abs(obs->matched_lane_heading_deviation()) >=
              M_PI_2 * 0.67) {
            LOG_INFO("ignore obs id is {}, heading diff is {}", obs->id(),
                     std::abs(obs->matched_lane_heading_deviation()));
            continue;
          }
          ConnectionConflictInfo c;
          c.lane_id = obs->matched_lane_id();
          CalAgentInfo(obs, &c.agent);
          SetConflictParam(&c);
          c.agent.id = obs->id();
          if (!CollisionCheckWithObstaclePolyline(
                  task_info.current_frame()->inside_planner_data(), *obs,
                  task_info.current_frame()->mutable_outside_planner_data(),
                  &c.conflict_area_bound)) {
            std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
            c.conflict_area_bound = cab;
          }
          ans.emplace_back(c);
        }
      }
    }
  }
  return ans;
}

std::vector<ConnectionConflictInfo> BackDataBasePath::ComputeConflictCustomData(
    TaskInfo& task_info) {
  std::vector<ConnectionConflictInfo> ans{};
  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return ans;
  }
  for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
    if (!interactive_agent_ids_.count(obs->id())) {
      continue;
    }
    ConnectionConflictInfo c;
    c.lane_id = obs->matched_lane_id();
    CalAgentInfo(obs, &c.agent);
    SetConflictParam(&c);
    c.agent.id = obs->id();
    if (!CollisionCheckWithObstaclePolyline(
            task_info.current_frame()->inside_planner_data(), *obs,
            task_info.current_frame()->mutable_outside_planner_data(),
            &c.conflict_area_bound)) {
      std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
      c.conflict_area_bound = cab;
    }

    ans.emplace_back(c);
  }

  return ans;
}

bool BackDataBasePath::CollisionCheckWithObstaclePolyline(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    OutsidePlannerData* const outside_data,
    std::vector<double>* conflict_area_bound) {
  const auto& pred_traj = obstacle.uniform_trajectory();
  if (pred_traj.num_of_points() < 2) {
    LOG_ERROR("trajectory points less 2.");
    return false;
  }
  // 1.obs_near_adc_prediction_segment
  OBS_SIDE obs_side{OBS_SIDE::UNKNOWN};
  Segment2d obs_prediction_segment{};
  Vec2d obs_base_pt{};
  GetObsPredictionSegment(inside_data, obstacle, obs_side,
                          obs_prediction_segment, obs_base_pt);

  // 2.adc_bounding_boxes，
  AdcCollideCornerPoint adc_first_collide_corner_point{
      AdcCollideCornerPoint::NONE};
  AdcCollideCornerPoint ego_collide_corner_point{AdcCollideCornerPoint::NONE};
  AdcCollideCornerPoint agent_collide_corner_point{AdcCollideCornerPoint::NONE};
  const auto& path_points = outside_data->path_data->path().path_points();
  int path_first_collide_i = path_points.size() + 2;
  int path_ego_end_collide_i = path_points.size() + 2;
  int path_agent_end_collide_i = path_points.size() + 2;
  Vec2d path_first_collide_pt{}, path_ego_end_collide_pt{},
      path_agent_end_collide_pt{};
  GetFirstCollideInfo(outside_data, obs_prediction_segment, obs_side,
                      adc_first_collide_corner_point, ego_collide_corner_point,
                      agent_collide_corner_point, path_first_collide_i,
                      path_first_collide_pt, path_ego_end_collide_i,
                      path_ego_end_collide_pt, path_agent_end_collide_i,
                      path_agent_end_collide_pt);

  // 3.lower:s,t,adc_first_collide_corner_point.
  if (!GetLaneMergingInfo(path_points, path_first_collide_i, obstacle.id(),
                          path_first_collide_pt, path_ego_end_collide_i,
                          path_agent_end_collide_i, path_agent_end_collide_pt,
                          obstacle.length(), obs_base_pt,
                          conflict_area_bound)) {
    LOG_WARN("Calculate Lane Merging Info Failed!");
    return false;
  }
  return true;
}

void BackDataBasePath::GetObsPredictionSegment(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    OBS_SIDE& obs_side, Segment2d& obs_prediction_segment, Vec2d& obs_base_pt) {
  double obs_nearest_local_y{std::numeric_limits<double>::infinity()};
  double obs_nearest_local_x{std::numeric_limits<double>::infinity()};
  std::vector<Vec2d> obs_bounding_box_corners;
  obstacle.bounding_box().get_all_corners(&obs_bounding_box_corners);
  for (const auto& pt : obs_bounding_box_corners) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                  pt.x(), pt.y(), obstacle.velocity_heading(), obs_local_x,
                  obs_local_y, obs_local_heading);
    if (std::abs(normalize_angle(obstacle.velocity_heading() -
                                 inside_data.vel_heading)) < M_PI_2) {
      if (std::abs(obs_local_y) < obs_nearest_local_y) {
        obs_nearest_local_y = std::abs(obs_local_y);
        obs_base_pt = pt;
        obs_side = obs_local_y > 0.0 ? OBS_SIDE::LEFT : OBS_SIDE::RIGHT;
      }
    } else {
      if (obs_local_x < obs_nearest_local_x) {
        obs_nearest_local_x = obs_local_x;
        obs_base_pt = pt;
        obs_side = obs_local_y > 0.0 ? OBS_SIDE::LEFT : OBS_SIDE::RIGHT;
      }
    }
  }

  Vec2d segment_end_pt{
      obs_base_pt.x() + conflict_data_base_path_config_.obs_prediction_dis *
                            std::cos(obstacle.velocity_heading()),
      obs_base_pt.y() + conflict_data_base_path_config_.obs_prediction_dis *
                            std::sin(obstacle.velocity_heading())};
  obs_prediction_segment = std::move(Segment2d(obs_base_pt, segment_end_pt));
  VisObsPredictionSegment(obs_base_pt, segment_end_pt);
}

void BackDataBasePath::GetEmittedFromObsCenter(
    const Obstacle& obstacle, Segment2d& obs_prediction_segment,
    Vec2d& obs_base_pt) {
  obs_base_pt = obstacle.center();
  Vec2d segment_end_pt{
      obs_base_pt.x() + conflict_data_base_path_config_.obs_prediction_dis *
                            std::cos(obstacle.velocity_heading()),
      obs_base_pt.y() + conflict_data_base_path_config_.obs_prediction_dis *
                            std::sin(obstacle.velocity_heading())};
  obs_prediction_segment = std::move(Segment2d(obs_base_pt, segment_end_pt));
  VisObsPredictionSegment(obs_base_pt, segment_end_pt);
}

bool BackDataBasePath::CheckFirstCollide(
    const OutsidePlannerData* const outside_data,
    const Segment2d& obs_prediction_segment) {
  const auto& adc_corner_pt_coordinate =
      outside_data->motorway_speed_obstacle_context.adc_corner_pt_coordinate;
  Segment2d current_adc_back_edge{adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_REAR)],
                                  adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::RIGHT_REAR)]};
  Segment2d current_adc_left_edge{adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_REAR)],
                                  adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_FRONT)]};
  Segment2d current_adc_right_edge{adc_corner_pt_coordinate[static_cast<int>(
                                       AdcCollideCornerPoint::RIGHT_REAR)],
                                   adc_corner_pt_coordinate[static_cast<int>(
                                       AdcCollideCornerPoint::RIGHT_FRONT)]};
  Vec2d path_first_collide_pt{0.0, 0.0};
  if (obs_prediction_segment.get_intersect(current_adc_back_edge,
                                           &path_first_collide_pt)) {
    LOG_INFO("obs collide adc at back.");
    return true;
  }
  if (obs_prediction_segment.get_intersect(current_adc_left_edge,
                                           &path_first_collide_pt)) {
    LOG_INFO("obs collide adc at left.");
    return true;
  }
  if (obs_prediction_segment.get_intersect(current_adc_right_edge,
                                           &path_first_collide_pt)) {
    LOG_INFO("obs collide adc at right.");
    return true;
  }
  return false;
}

void BackDataBasePath::GetFirstCollideInfo(
    const OutsidePlannerData* const outside_data,
    const Segment2d& obs_prediction_segment, const OBS_SIDE& obs_side,
    AdcCollideCornerPoint& adc_first_collide_corner_point,
    AdcCollideCornerPoint& ego_collide_corner_point,
    AdcCollideCornerPoint& agent_collide_corner_point,
    int& path_first_collide_i, Vec2d& path_first_collide_pt,
    int& path_ego_end_collide_i, Vec2d& path_ego_end_collide_pt,
    int& path_agent_end_collide_i, Vec2d& path_agent_end_collide_pt) {
  const auto& adc_corner_pt_coordinate =
      outside_data->speed_obstacle_context.adc_corner_pt_coordinate;
  Segment2d current_adc_back_edge{adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_REAR)],
                                  adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::RIGHT_REAR)]};
  Segment2d current_adc_left_edge{adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_REAR)],
                                  adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_FRONT)]};
  Segment2d current_adc_right_edge{adc_corner_pt_coordinate[static_cast<int>(
                                       AdcCollideCornerPoint::RIGHT_REAR)],
                                   adc_corner_pt_coordinate[static_cast<int>(
                                       AdcCollideCornerPoint::RIGHT_FRONT)]};
  if (obs_prediction_segment.get_intersect(current_adc_back_edge,
                                           &path_first_collide_pt)) {
    path_first_collide_i = 0, path_ego_end_collide_i = 0,
    path_agent_end_collide_i = 0;
    if (OBS_SIDE::RIGHT == obs_side) {
      adc_first_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
      ego_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
      agent_collide_corner_point = AdcCollideCornerPoint::RIGHT_FRONT;
    } else {
      adc_first_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
      ego_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
      agent_collide_corner_point = AdcCollideCornerPoint::LEFT_FRONT;
    }
    LOG_INFO("obs collide adc at back.");
    return;
  }
  if (OBS_SIDE::LEFT == obs_side &&
      obs_prediction_segment.get_intersect(current_adc_left_edge,
                                           &path_first_collide_pt)) {
    path_first_collide_i = 0, path_ego_end_collide_i = 0,
    path_agent_end_collide_i = 0;
    adc_first_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
    ego_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
    agent_collide_corner_point = AdcCollideCornerPoint::LEFT_FRONT;
    LOG_INFO("obs collide adc at left.");
    return;
  }
  if (OBS_SIDE::RIGHT == obs_side &&
      obs_prediction_segment.get_intersect(current_adc_right_edge,
                                           &path_first_collide_pt)) {
    path_first_collide_i = 0, path_ego_end_collide_i = 0,
    path_agent_end_collide_i = 0;
    adc_first_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
    ego_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
    agent_collide_corner_point = AdcCollideCornerPoint::RIGHT_FRONT;
    LOG_INFO("obs collide adc at right.");
    return;
  }

  bool find_first_collide_pt{false}, find_ego_end_collide_pt{false},
      find_agent_end_collide_pt{false};
  const auto& path_points = outside_data->path_data->path().path_points();
  for (size_t i = 0; i < path_points.size(); ++i) {
    std::vector<Vec2d> adc_corner_pts;
    GetAdcCornerPointCoordinate(path_points[i].coordinate(),
                                path_points[i].theta(), adc_corner_pts);
    if (adc_corner_pts.size() < static_cast<int>(AdcCollideCornerPoint::NONE)) {
      LOG_WARN("number of adc corner points less than 4.");
      continue;
    }
    VisAdcCornerPoints(adc_corner_pts);

    Segment2d path_adc_back_edge{
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)],
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_REAR)]};
    Segment2d path_adc_front_edge{
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_FRONT)],
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_FRONT)]};
    if (!find_first_collide_pt) {
      if (obs_prediction_segment.get_intersect(path_adc_back_edge,
                                               &path_first_collide_pt)) {
        find_first_collide_pt = true;
      } else if (OBS_SIDE::RIGHT == obs_side) {
        Segment2d adc_side_edge = Segment2d{
            adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_REAR)],
            adc_corner_pts[static_cast<int>(
                AdcCollideCornerPoint::RIGHT_FRONT)]};
        if (obs_prediction_segment.get_intersect(adc_side_edge,
                                                 &path_first_collide_pt)) {
          find_first_collide_pt = true;
        }
      } else {
        Segment2d adc_side_edge = Segment2d{
            adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)],
            adc_corner_pts[static_cast<int>(
                AdcCollideCornerPoint::LEFT_FRONT)]};
        if (obs_prediction_segment.get_intersect(adc_side_edge,
                                                 &path_first_collide_pt)) {
          find_first_collide_pt = true;
        }
      }

      if (find_first_collide_pt) {
        path_first_collide_i = i;
      }
    }
    if (!find_ego_end_collide_pt) {
      if (obs_prediction_segment.get_intersect(path_adc_back_edge,
                                               &path_ego_end_collide_pt)) {
        find_ego_end_collide_pt = true;
        path_ego_end_collide_i = i;
      }
    }
    if (!find_agent_end_collide_pt) {
      if (obs_prediction_segment.get_intersect(path_adc_back_edge,
                                               &path_agent_end_collide_pt)) {
        find_agent_end_collide_pt = true;
        path_agent_end_collide_i = i;
      }
    }
    if (find_first_collide_pt && find_ego_end_collide_pt &&
        find_agent_end_collide_pt) {
      break;
    }
  }
}

bool BackDataBasePath::GetLaneMergingInfo(
    const std::vector<PathPoint>& path_points, const int path_first_collide_i,
    const int obs_id, const Vec2d& path_first_collide_pt,
    const int ego_end_collide_i, const int agent_end_collide_i,
    const Vec2d& agent_end_collide_pt, const double agent_length,
    const Vec2d& obs_base_pt, std::vector<double>* conflict_area_bound) {
  bool miss_ego_end_pt{false}, miss_agent_end_pt{false};
  if (path_first_collide_i >= path_points.size() || path_points.empty()) {
    LOG_WARN("Don't Find First Collide Pt!");
    return false;
  }
  if (path_first_collide_i == 0) {
    LOG_WARN("Ego in Conflict Area!");
    return false;
  }
  if (ego_end_collide_i >= path_points.size()) {
    miss_ego_end_pt = true;
    LOG_WARN("Don't Found Ego Right-of-Way Pt!");
  }
  if (agent_end_collide_i >= path_points.size()) {
    miss_agent_end_pt = true;
    LOG_WARN("Don't Found Agent Right-of-Way Pt!");
  }

  double ego_s{0.0}, agent_s{0.0}, ego_rd{0.0}, agent_rd{0.0};
  ego_rd = path_points[path_first_collide_i].s();
  agent_rd =
      std::sqrt(std::pow(path_first_collide_pt.x() - obs_base_pt.x(), 2) +
                std::pow(path_first_collide_pt.y() - obs_base_pt.y(), 2));
  ego_s = miss_ego_end_pt
              ? (ego_rd + 3.0)
              : std::max(ego_rd, path_points[ego_end_collide_i].s());
  agent_s =
      miss_agent_end_pt
          ? (agent_rd + 3.0)
          : (std::sqrt(
                 std::pow(agent_end_collide_pt.x() - obs_base_pt.x(), 2) +
                 std::pow(agent_end_collide_pt.y() - obs_base_pt.y(), 2)) +
             agent_length);
  std::vector<double> area_bound{agent_rd, agent_s, ego_s, ego_rd};
  LOG_INFO("Obs:{}, Agent RD:{:3f}, Agent S:{:3f}, Ego RD:{:3f}, Ego S:{:3f}",
           obs_id, agent_rd, agent_s, ego_rd, ego_s);
  *conflict_area_bound = area_bound;
  return true;
}

void BackDataBasePath::GetAdcCornerPointCoordinate(
    const Vec2d& adc_coordinate, const double adc_heading,
    std::vector<Vec2d>& adc_corner_pt_coordinate) {
  adc_corner_pt_coordinate.clear();
  adc_corner_pt_coordinate.resize(
      static_cast<int>(AdcCollideCornerPoint::NONE));
  double x_g = 0.0;
  double y_g = 0.0;
  double theta_g = 0.0;
  vehicle2earth(adc_coordinate.x(), adc_coordinate.y(), adc_heading,
                -VehicleParam::Instance()->back_edge_to_center(),
                VehicleParam::Instance()->left_edge_to_center() +
                    conflict_data_base_path_config_.adc_buffer,
                0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)] =
      std::move(Vec2d(x_g, y_g));
  vehicle2earth(adc_coordinate.x(), adc_coordinate.y(), adc_heading,
                -VehicleParam::Instance()->back_edge_to_center(),
                -VehicleParam::Instance()->right_edge_to_center() -
                    conflict_data_base_path_config_.adc_buffer,
                0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_REAR)] = std::move(Vec2d(x_g, y_g));
  vehicle2earth(adc_coordinate.x(), adc_coordinate.y(), adc_heading,
                VehicleParam::Instance()->front_edge_to_center(),
                -VehicleParam::Instance()->right_edge_to_center() -
                    conflict_data_base_path_config_.adc_buffer,
                0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_FRONT)] = std::move(Vec2d(x_g, y_g));
  vehicle2earth(adc_coordinate.x(), adc_coordinate.y(), adc_heading,
                VehicleParam::Instance()->front_edge_to_center(),
                VehicleParam::Instance()->left_edge_to_center() +
                    conflict_data_base_path_config_.adc_buffer,
                0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::LEFT_FRONT)] = std::move(Vec2d(x_g, y_g));
}

bool BackDataBasePath::CheckCollisionForDiverging(TaskInfo& task_info,
                                                  const Obstacle& obstacle) {
  const auto& pred_traj = obstacle.uniform_trajectory();
  if (pred_traj.num_of_points() < 2) {
    LOG_ERROR("trajectory points less 2.");
    return false;
  }
  // 1.obs_near_adc_prediction_segment
  Segment2d obs_prediction_segment{};
  Vec2d obs_base_pt{};
  GetEmittedFromObsCenter(obstacle, obs_prediction_segment, obs_base_pt);

  // 2.adc_bounding_boxes，
  return CheckFirstCollide(
      task_info.current_frame()->mutable_outside_planner_data(),
      obs_prediction_segment);
}

}  // namespace planning
}  // namespace neodrive

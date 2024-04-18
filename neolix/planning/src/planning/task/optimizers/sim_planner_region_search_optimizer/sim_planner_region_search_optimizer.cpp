#include "sim_planner_region_search_optimizer.h"

#include "common/visualizer_event/visualizer_event.h"
#include "math/curve1d/spline.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/sim_planner/sim_map.h"
namespace neodrive {
namespace planning {

SimPlannerRegionSearchOptimizer::SimPlannerRegionSearchOptimizer() {
  name_ = "SimPlannerRegionSearchOptimizer";
}

SimPlannerRegionSearchOptimizer::~SimPlannerRegionSearchOptimizer() { Reset(); }

void VisGraph(const std::vector<sim_planner::Vehicle>& sim_traj) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("simplanner");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& pt : sim_traj) {
    Vec2d xy_point = pt.state().vec_position;
    // LOG_INFO("xy points:{:.4f}, {:.4f}", xy_point.x(), xy_point.y());
    set_pts(event, xy_point);
  }
}

void VisTrackPointsGraph(
    const std::vector<sim_planner::SimMapPoint>& track_pts) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("track_points");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& pt : track_pts) {
    Vec2d xy_point(pt.x, pt.y);
    // LOG_INFO("xy points:{:.4f}, {:.4f}", xy_point.x(), xy_point.y());
    set_pts(event, xy_point);
  }
}

void VisTrackGraph(const std::vector<sim_planner::Vehicle>& sim_rp_traj) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("simplanner_track");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& pt : sim_rp_traj) {
    Vec2d xy_point = pt.state().vec_position;
    // LOG_INFO("xy points:{:.4f}, {:.4f}", xy_point.x(), xy_point.y());
    set_pts(event, xy_point);
  }
}

void VisAllTrajGraph(
    const std::vector<std::vector<sim_planner::Vehicle>>& sim_trajs,
    const std::vector<bool>& sim_res) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("all_traj");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.1);
  };
  for (int i = 0; i < sim_trajs.size(); ++i) {
    if (!sim_res[i]) continue;
    std::vector<sim_planner::Vehicle> sim_traj = sim_trajs[i];
    for (const auto& pt : sim_traj) {
      Vec2d xy_point = pt.state().vec_position;
      set_pts(event, xy_point);
    }
  }
}

void VisAllTrackTrajGraph(
    const std::vector<std::vector<sim_planner::Vehicle>>& sim_rp_trajs,
    const std::vector<bool>& sim_res) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("all_track_traj");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.1);
  };
  for (int i = 0; i < sim_rp_trajs.size(); ++i) {
    if (!sim_res[i]) continue;
    std::vector<sim_planner::Vehicle> sim_rp_traj = sim_rp_trajs[i];
    for (const auto& pt : sim_rp_traj) {
      Vec2d xy_point = pt.state().vec_position;
      set_pts(event, xy_point);
    }
  }
}

void VisSimMapGraph(const std::vector<sim_planner::SimMapRoad>& sim_map) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event_odd = vis::EventSender::Instance()->GetEvent("odd_road_lane");
  event_odd->set_type(visualizer::Event::k3D);
  event_odd->add_attribute(visualizer::Event::kOdom);
  auto event_even = vis::EventSender::Instance()->GetEvent("even_road_lane");
  event_even->set_type(visualizer::Event::k3D);
  event_even->add_attribute(visualizer::Event::kOdom);
  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.05);
  };
  for (const auto& sim_road : sim_map) {
    for (const auto& sim_lane : sim_road.lanes) {
      int pts_size = sim_lane.pts.size();
      if (pts_size == 0) continue;
      Vec2d first_pt(sim_lane.pts.front().x, sim_lane.pts.front().y),
          mid_pt(sim_lane.pts[pts_size / 2].x, sim_lane.pts[pts_size / 2].y),
          last_pt(sim_lane.pts.back().x, sim_lane.pts.back().y);
      if (sim_road.road_id % 2 == 0) {
        // LOG_INFO(
        //     "road id:{}, even points 1:{}{}, even points 2:{}{}, even points
        //     " "3:{}{}", sim_road.road_id, first_pt.x(), first_pt.y(),
        //     mid_pt.x(), mid_pt.y(), last_pt.x(), last_pt.y());
        set_pts(event_even, first_pt);
        set_pts(event_even, mid_pt);
        set_pts(event_even, last_pt);
      } else {
        // LOG_INFO(
        //     "road id:{}, odd points 1:{}{}, odd points 2:{}{}, odd points "
        //     "3:{}{}",
        //     sim_road.road_id, first_pt.x(), first_pt.y(), mid_pt.x(),
        //     mid_pt.y(), last_pt.x(), last_pt.y());
        set_pts(event_odd, first_pt);
        set_pts(event_odd, mid_pt);
        set_pts(event_odd, last_pt);
      }
    }
  }
}

ErrorCode SimPlannerRegionSearchOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& obs_decision =
      outside_data->path_obstacle_context.obstacle_decision;
  const auto& init_sl_point = inside_data.init_sl_point;
  const auto& init_point = inside_data.init_point;
  const auto& lat_boundaries = outside_data->road_obs_path_boundries;
  auto reference_line = task_info.reference_line();
  double road_extend_dis = config::PlanningConfig::Instance()
                               ->planning_research_config()
                               .path_road_graph_config.road_extend_dis;

  double left_road_start_s{-1.0}, right_road_start_s{-1.0};
  for (const auto& info : lat_boundaries) {
    if (std::abs(info.left_lane_bound - info.left_road_bound) < 0.3) {
      left_road_start_s = info.s;
      break;
    }
  }
  for (const auto& info : lat_boundaries) {
    if (std::abs(info.right_lane_bound - info.right_road_bound) < 0.3) {
      right_road_start_s = info.s;
      break;
    }
  }
  LOG_INFO("left/right_road_start_s: {:.3f}, {:.3f}", left_road_start_s,
           right_road_start_s);

  /// compute lane boundary and obtacles
  std::vector<SLPoint> right_boundary{};
  std::vector<SLPoint> left_boundary{};
  std::vector<SLPoint> right_road_boundary{};
  std::vector<SLPoint> left_road_boundary{};
  double min_tunnel_width{1.4};
  double car_length{3.0};
  for (const auto& boundary : lat_boundaries) {
    right_boundary.emplace_back(SLPoint(
        boundary.s, ((boundary.s > right_road_start_s - road_extend_dis &&
                      boundary.s < right_road_start_s &&
                      right_road_start_s > lat_boundaries.front().s)
                         ? boundary.right_lane_bound
                         : boundary.right_bound)));
    left_boundary.emplace_back(SLPoint(
        boundary.s, ((boundary.s > left_road_start_s - road_extend_dis &&
                      boundary.s < left_road_start_s &&
                      left_road_start_s > lat_boundaries.front().s)
                         ? boundary.left_lane_bound
                         : boundary.left_bound)));
    right_road_boundary.emplace_back(
        SLPoint(boundary.s, boundary.right_road_bound));
    left_road_boundary.emplace_back(
        SLPoint(boundary.s, boundary.left_road_bound));

    LOG_DEBUG(
        "s, l_r_b, l_l_b, l_b, r_r_b, r_l_b, r_b: {:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}, {:.3f}, {:.3f}, {:.3f}",
        boundary.s, boundary.left_road_bound, boundary.left_lane_bound,
        left_boundary.back().l(), boundary.right_road_bound,
        boundary.right_lane_bound, right_boundary.back().l());
  }

  std::vector<AABox2d> obstacles_aaboxes{};
  for (std::size_t i = 0; i < obs_decision.size(); ++i) {
    if (obs_decision[i].obstacle_boundary.boundary.end_s() <
        init_sl_point.s()) {
      continue;
    }
    if (obs_decision[i].decision_type != Decision::DecisionType::GO_LEFT &&
        obs_decision[i].decision_type != Decision::DecisionType::GO_RIGHT &&
        obs_decision[i].decision_type != Decision::DecisionType::YIELD_DOWN) {
      continue;
    }
    if (obs_decision[i].obstacle_boundary.obstacle.is_virtual()) {
      continue;
    }
    if (data_center_->master_info().distance_to_end() <
            FLAGS_planning_station_look_forward_dis &&
        obs_decision[i].obstacle_boundary.boundary.start_s() >
            data_center_->master_info().distance_to_end() +
                inside_data.init_sl_point.s()) {
      continue;
    }
    double front_enlarge_s =
        obs_decision[i].obstacle_boundary.obstacle.type() ==
                Obstacle::ObstacleType::VEHICLE
            ? 1.0
            : 0.0;
    double back_enlarge_s = obs_decision[i].obstacle_boundary.obstacle.type() ==
                                    Obstacle::ObstacleType::VEHICLE
                                ? 3.0
                                : 0.0;
    front_enlarge_s = std::max(front_enlarge_s, 0.1);
    back_enlarge_s = std::max(back_enlarge_s, 0.1);
    const auto boundary =
        obs_decision[i].obstacle_boundary.obstacle_ptr->PolygonBoundary();
    std::vector<Vec2d> aabox2d_points{
        {boundary.start_s() - front_enlarge_s, boundary.start_l()},
        {boundary.start_s() - front_enlarge_s, boundary.end_l()},
        {boundary.end_s() + back_enlarge_s, boundary.end_l()},
        {boundary.end_s() + back_enlarge_s, boundary.start_l()}};

    // If there are obstacles near the car, the part of the obstacle behind the
    // car will be ignored
    double ref_s = outside_data->path_obstacle_context.adc_boundary.start_s();
    if (!inside_data.is_reverse_driving &&
        (boundary.start_s() < ref_s && boundary.end_s() > ref_s)) {
      LOG_INFO("ref_s: {:.4f}, start_s: {:.4f}, end_s: {:.4f}", ref_s,
               boundary.start_s(), boundary.end_s());
      std::vector<SLPoint> obstacle_box_sl =
          obs_decision[i].obstacle_boundary.obstacle.polygon_sl();
      // before print
      for (std::size_t j = 0; j < obstacle_box_sl.size(); ++j) {
        LOG_INFO("polygon points: {}, s: {:.4f}, l: {:.4f}", j,
                 obstacle_box_sl[j].s(), obstacle_box_sl[j].l());
      }
      std::vector<SLPoint> obstacle_box_sl_new;
      obstacle_box_sl.emplace_back(obstacle_box_sl[0]);
      for (std::size_t k = 0; k < obstacle_box_sl.size() - 1; ++k) {
        SLPoint first = obstacle_box_sl[k], second = obstacle_box_sl[k + 1];
        if (ref_s > std::min(first.s(), second.s()) &&
            ref_s < std::max(first.s(), second.s())) {
          double ref_l = ((ref_s - first.s()) * (second.l() - first.l()) /
                          (second.s() - first.s())) +
                         first.l();
          obstacle_box_sl_new.emplace_back(SLPoint(ref_s, ref_l));
          LOG_INFO(
              "obs_id: {}, (s1,l1): ({:.4f},{:.4f}), (s2,l2): ({:.4f},{:.4f}), "
              "(sr,lr): ({:.4f},{:.4f})",
              obs_decision[i].obstacle_boundary.obstacle.id(), first.s(),
              first.l(), second.s(), second.l(), ref_s, ref_l);
        }
      }
      for (std::size_t k = 0; k < obstacle_box_sl.size() - 1; ++k) {
        if (obstacle_box_sl[k].x() >= ref_s) {
          obstacle_box_sl_new.emplace_back(obstacle_box_sl[k]);
        }
      }
      std::sort(obstacle_box_sl_new.begin(), obstacle_box_sl_new.end(),
                [](auto& a, auto& b) { return a.l() < b.l(); });
      double l_min = obstacle_box_sl_new.front().l(),
             l_max = obstacle_box_sl_new.back().l();
      // after print
      for (std::size_t j = 0; j < obstacle_box_sl_new.size(); ++j) {
        LOG_INFO("fix polygon points: {}, s: {:.4f}, l: {:.4f}", j,
                 obstacle_box_sl_new[j].x(), obstacle_box_sl_new[j].y());
      }
      LOG_INFO(
          "old start_l: {:.4f}, end_l: {:.4f}, new start_l: {:.4f}, "
          "end_l: {:.4f}",
          boundary.start_l(), boundary.end_l(), l_min, l_max);
      aabox2d_points[0] = Vec2d(ref_s, l_min);
      aabox2d_points[1] = Vec2d(ref_s, l_max);
      aabox2d_points[2] = Vec2d(boundary.end_s() + back_enlarge_s, l_max);
      aabox2d_points[3] = Vec2d(boundary.end_s() + back_enlarge_s, l_min);
    }

    AABox2d box(aabox2d_points);
    box.set_id(obs_decision[i].obstacle_boundary.obstacle.id());
    box.set_obs_type(
        static_cast<int>(obs_decision[i].obstacle_boundary.obstacle.type()));
    LOG_INFO("id, box: {}, {}", box.id(), box.points_debug_string());
    obstacles_aaboxes.emplace_back(box);
  }
  outside_data->path_context.lateral_static_obstacles =
      std::move(obstacles_aaboxes);

  const auto& init_pt = task_info.frame()->inside_planner_data().init_point;

  /// Create sim map
  LOG_INFO("Simmap creat start.....");
  if (!sim_planner::SimMap::Instance()->createSimMap(task_info)) {
    LOG_ERROR("CreateSimMap failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("Simmap creat finish.....");
  VisSimMapGraph(sim_planner::SimMap::Instance()->road_map());

  /// Sim planner for region search
  sim_planner::Task sim_task;
  sim_task.is_under_ctrl = true;
  sim_task.user_desired_vel = 10.0;
  sim_planner::State state{
      // .time_stamp = DataCenter::Instance()->init_frame_time(),
      .time_stamp = 0,
      .vec_position = Vec2d(init_pt.x(), init_pt.y()),
      .angle = init_pt.theta(),
      .curvature = init_pt.kappa(),
      .velocity = std::max(init_pt.velocity(), 2.0),
      .acceleration = init_pt.acceleration(),
      .steer = data_center_->vehicle_state_odometry().SteerPercent()};

  std::vector<sim_planner::Vehicle> sim_traj{};
  std::vector<std::vector<sim_planner::Vehicle>> sim_trajs{};
  sim_traj.clear();
  sim_trajs.clear();
  LOG_INFO("Simplanner start run......");
  sim_planner::SimLateralManager sim_lateral_manager(
      context, last_snapshot, last_task, lc_context, last_lc_proposal,
      preliminary_active_requests);

  // if (!sim_lateral_manager.Run(DataCenter::Instance()->init_frame_time(),
  //                              sim_task, state, &context, &last_snapshot,
  //                              &last_task, &lc_context, &last_lc_proposal,
  //                              &preliminary_active_requests)) {
  //   LOG_ERROR("sim_lateral_manager failed.");
  //   return ErrorCode::PLANNING_ERROR_FAILED;
  // }

  bool is_sim_lateral_success = true;
  if (!sim_lateral_manager.run(0, sim_task, state, &context, &last_snapshot,
                               &last_task, &lc_context, &last_lc_proposal,
                               &preliminary_active_requests)) {
    LOG_ERROR("sim_lateral_manager failed.");
    // back up
    is_sim_lateral_success = false;
    // return ErrorCode::PLANNING_ERROR_FAILED;
  }
  std::unordered_map<std::string, int> all_lanes_id =
      sim_lateral_manager.getAllLaneID();

  if (!is_sim_lateral_success) {
    sim_planner::Vehicle v;
    v.setState(state);
    sim_traj = {v};
  } else {
    sim_trajs = last_snapshot.forward_trajs;
    std::vector<bool> sim_res = last_snapshot.sim_res;
    sim_traj = sim_trajs[last_snapshot.processed_winner_id];
    VisAllTrajGraph(sim_trajs, sim_res);
  }

  VisGraph(sim_traj);

  LOG_INFO("Simplanner run finish.....");

  /// region search
  auto bounds_info = RegionDecisionGraphSearch{}.GenerateBoundInfo(
      outside_data->path_context.lateral_static_obstacles, left_boundary,
      right_boundary, left_road_boundary, right_road_boundary,
      init_sl_point.l(), init_sl_point.s(), reference_line, sim_traj);
  if (bounds_info.empty()) {
    LOG_WARN("ReginDecisionGraphSearch failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  outside_data->path_context.valid_region_end_s =
      bounds_info.back().upper_point.s();
  outside_data->path_context.valid_backup_end_s =
      bounds_info.back().upper_point.s();
  outside_data->path_context.original_path_boundary.path_boundary = bounds_info;

  double extend_length = 1.5 * VehicleParam::Instance()->length();
  std::vector<Vec2d> left_pt_xy{}, right_pt_xy{};
  Vec2d pt1, pt2{};
  for (const auto& info : bounds_info) {
    // front extend
    if (std::abs(info.lower_point.s() - bounds_info.front().lower_point.s()) <
        kMathEpsilon) {
      for (double s = info.lower_point.s() - extend_length;
           s < info.lower_point.s(); s += 0.1) {
        if (!reference_line->GetPointInCartesianFrame({s, info.lower_point.l()},
                                                      &pt1)) {
          LOG_ERROR("failed get closest point.");
          continue;
        }
        if (!reference_line->GetPointInCartesianFrame({s, info.upper_point.l()},
                                                      &pt2)) {
          LOG_ERROR("failed get closest point.");
          continue;
        }
        right_pt_xy.emplace_back(pt1), left_pt_xy.emplace_back(pt2);
      }
      continue;
    }
    // valid add
    if (!reference_line->GetPointInCartesianFrame(
            {info.lower_point.s(), info.lower_point.l()}, &pt1)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    if (!reference_line->GetPointInCartesianFrame(
            {info.upper_point.s(), info.upper_point.l()}, &pt2)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    right_pt_xy.emplace_back(pt1), left_pt_xy.emplace_back(pt2);
    // back extend
    if (std::abs(info.lower_point.s() - bounds_info.back().lower_point.s()) <
        kMathEpsilon) {
      for (double s = info.lower_point.s() + 0.1;
           s < info.lower_point.s() + extend_length; s += 0.1) {
        if (!reference_line->GetPointInCartesianFrame({s, info.lower_point.l()},
                                                      &pt1)) {
          LOG_ERROR("failed get closest point.");
          continue;
        }
        if (!reference_line->GetPointInCartesianFrame({s, info.upper_point.l()},
                                                      &pt2)) {
          LOG_ERROR("failed get closest point.");
          continue;
        }
        right_pt_xy.emplace_back(pt1), left_pt_xy.emplace_back(pt2);
      }
      continue;
    }
  }

  outside_data->path_context.original_path_boundary.left_xy_boundary =
      std::move(left_pt_xy);
  outside_data->path_context.original_path_boundary.right_xy_boundary =
      std::move(right_pt_xy);

  /// lane borrow scenario identify
  outside_data->lane_borrow_scenario =
      StateIdentify(inside_data, bounds_info, init_sl_point,
                    inside_data.is_lane_borrowing, outside_data);

  /// compute tunnel width
  if (!ComputeTunnelWidth(bounds_info, outside_data)) {
    LOG_WARN("ComputeTunnelWidth failed");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool SimPlannerRegionSearchOptimizer::ComputeTunnelWidth(
    const std::vector<PathRegion::Bound>& bounds_info,
    OutsidePlannerData* const outside_data) {
  if (outside_data == nullptr) {
    LOG_ERROR("input nullptr err");
    return false;
  }
  if (bounds_info.empty()) {
    LOG_ERROR("bounds_info is empty, err");
    return false;
  }

  const double prefer_len =
      outside_data->path_context.prefer_valid_length + 9.0;
  double min_width = std::numeric_limits<double>::max();
  double start_s = outside_data->frenet_init_point.s();
  for (std::size_t i = 0; i < bounds_info.size(); ++i) {
    if (bounds_info[i].lower_point.s() - start_s > prefer_len) break;
    double left_bound = bounds_info[i].upper_id >= 0
                            ? bounds_info[i].upper_point.l()
                            : bounds_info[i].upper_point.l();
    double right_bound = bounds_info[i].lower_id >= 0
                             ? bounds_info[i].lower_point.l()
                             : bounds_info[i].lower_point.l();
    double width = left_bound - right_bound;
    if (min_width > width) {
      min_width = width;
    }
  }
  LOG_INFO("tunnel width for multi level model: {:.3f}", min_width);
  outside_data->multi_mode_data.valid_width = min_width;

  return true;
}

LaneBorrowScenario SimPlannerRegionSearchOptimizer::StateIdentify(
    const InsidePlannerData& inside_data,
    const std::vector<PathRegion::Bound>& bounds_info,
    const SLPoint& init_sl_point, const bool is_lane_borrow,
    OutsidePlannerData* const outside_data) {
  // if (!is_lane_borrow) return LaneBorrowScenario::NONE;
  if (!is_lane_borrow) {
    return LaneBorrowScenario::NONE;
  }

  // single lane road link
  bool left_is_road{false}, right_is_road{false};
  for (const auto& bound_info : bounds_info) {
    if (!left_is_road &&
        bound_info.upper_type == PathRegion::Bound::BoundType::ROAD) {
      left_is_road = true;
    }
    if (!right_is_road &&
        bound_info.lower_type == PathRegion::Bound::BoundType::ROAD) {
      right_is_road = true;
    }
    if (left_is_road && right_is_road) break;
  }
  bool single_lane_road_link = left_is_road && right_is_road;
  LOG_INFO("single_lane_road_link: {}", (int)single_lane_road_link);

  // left bound min value; right bound max value
  double min_left_bound{1000.}, max_right_bound{-1000.};
  for (const auto& bound_info : bounds_info) {
    min_left_bound = (min_left_bound > bound_info.upper_point.l())
                         ? bound_info.upper_point.l()
                         : min_left_bound;
    max_right_bound = (max_right_bound < bound_info.lower_point.l())
                          ? bound_info.lower_point.l()
                          : max_right_bound;
  }
  double attention_bound =
      (inside_data.lane_borrow_side == LaneBorrowContext::BorrowSide::Left)
          ? max_right_bound
          : min_left_bound;
  LOG_INFO("attention_bound: {:.3f}", attention_bound);

  // sapce: LEFT_BORROW  -> left_lane_bound  - right_bound
  //        RIGHT_BORROW -> right_lane_bound - left_bound
  double left_min_space{1000.}, right_min_space{1000.};
  for (const auto& bound_info : bounds_info) {
    left_min_space = (left_min_space > (bound_info.upper_lane_bound_point.l() -
                                        bound_info.lower_point.l()))
                         ? (bound_info.upper_lane_bound_point.l() -
                            bound_info.lower_point.l())
                         : left_min_space;
    right_min_space =
        (right_min_space >
         (bound_info.lower_lane_bound_point.l() - bound_info.upper_point.l()))
            ? (bound_info.lower_lane_bound_point.l() -
               bound_info.upper_point.l())
            : right_min_space;
  }
  double min_free_space =
      (inside_data.lane_borrow_side == LaneBorrowContext::BorrowSide::Left)
          ? left_min_space
          : right_min_space;
  LOG_INFO("min_free_space: {:.3f}", min_free_space);

  // adc_boundary sl and heading
  double adc_upper_l =
      init_sl_point.l() + 0.5 * VehicleParam::Instance()->width();
  double adc_lower_l =
      init_sl_point.l() - 0.5 * VehicleParam::Instance()->width();
  double adc_attention_l =
      (inside_data.lane_borrow_side == LaneBorrowContext::BorrowSide::Left)
          ? adc_lower_l
          : adc_upper_l;
  LOG_INFO("adc_attention_l: {:.3f}", adc_attention_l);

  double adc_heading = inside_data.init_point.theta();
  double closet_ref_point_heading =
      outside_data->veh_real_reference_point.heading();
  bool adc_has_enter_status =
      (std::fabs(adc_heading - closet_ref_point_heading) * 180 / M_PI < 20.0) &&
      ((inside_data.lane_borrow_side == LaneBorrowContext::BorrowSide::Left)
           ? (adc_attention_l > attention_bound)
           : (adc_attention_l < attention_bound));
  LOG_INFO("adc_has_enter: {}", (int)adc_has_enter_status);

  // scenario
  if (single_lane_road_link && min_free_space >= 1.6) {
    LOG_INFO("current LaneBorrowScenario: {}", (int)(LaneBorrowScenario::BACK));
    return LaneBorrowScenario::BACK;
  }
  if (adc_has_enter_status) {
    LOG_INFO("current LaneBorrowScenario: {}",
             (int)(LaneBorrowScenario::CRUISE));
    return LaneBorrowScenario::CRUISE;
  }

  LOG_INFO("current LaneBorrowScenario: {}", (int)(LaneBorrowScenario::ENTER));
  return LaneBorrowScenario::ENTER;
}

}  // namespace planning
}  // namespace neodrive

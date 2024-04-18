#include "path_region_search_optimizer.h"

#include "common/visualizer_event/visualizer_event.h"
#include "math/curve1d/spline.h"
#include "region_decision_graph_search.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

void VisObsPolygon(const std::vector<std::vector<Vec2d>> &polygon_pts_vec,
                   const std::string &name) {
  if (!FLAGS_planning_enable_vis_event || polygon_pts_vec.empty()) return;
  LOG_INFO("name:{}, polygon_pts_vec.size:{}", name, polygon_pts_vec.size());
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  int i = 0;
  for (auto &polygon_pts : polygon_pts_vec) {
    auto polygon = event->mutable_polygon()->Add();
    for (auto &pt : polygon_pts) {
      set_pt(polygon->add_point(), pt);
    }
    auto text = event->mutable_text()->Add();
    set_pt(text->mutable_position(), polygon_pts[0]);
    text->set_text("id: " + std::to_string(i));
    ++i;
  }
}

void VisObsPolygon(ReferenceLinePtr ref_line, const std::vector<AABox2d> &boxes,
                   const std::string &name) {
  if (!FLAGS_planning_enable_vis_event || boxes.empty()) return;
  LOG_INFO("name:{}, boxes.size:{}", name, boxes.size());
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  for (auto &box : boxes) {
    std::vector<Vec2d> corners;
    box.get_all_corners(&corners);
    Vec2d pt{};
    auto polygon = event->mutable_polygon()->Add();
    for (auto &sl_pt : corners) {
      ref_line->GetPointInCartesianFrame(SLPoint(sl_pt.x(), sl_pt.y()), &pt);
      set_pt(polygon->add_point(), pt);
    }
  }
}

PathRegionSearchOptimizer::PathRegionSearchOptimizer() {
  name_ = "PathRegionSearchOptimizer";
}

PathRegionSearchOptimizer::~PathRegionSearchOptimizer() { Reset(); }

ErrorCode PathRegionSearchOptimizer::Execute(TaskInfo &task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &obs_decision =
      outside_data->path_obstacle_context.obstacle_decision;
  const auto &init_sl_point = inside_data.init_sl_point;
  const auto &init_point = inside_data.init_point;
  const auto &lat_boundaries = outside_data->road_obs_path_boundries;
  auto reference_line = task_info.reference_line();
  double road_extend_dis = config::PlanningConfig::Instance()
                               ->planning_research_config()
                               .path_road_graph_config.road_extend_dis;
  bool is_barrier_gate = data_center_->master_info().curr_scenario() ==
                         ScenarioState::BARRIER_GATE;

  double left_road_start_s{-1.0}, right_road_start_s{-1.0};
  for (const auto &info : lat_boundaries) {
    if (std::abs(info.left_lane_bound - info.left_road_bound) < 0.3) {
      left_road_start_s = info.s;
      break;
    }
  }
  for (const auto &info : lat_boundaries) {
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
  for (const auto &boundary : lat_boundaries) {
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

  std::vector<std::vector<Vec2d>> near_ego_polygon_vec{},
      near_ego_polygon_transfer_vec{};
  std::vector<std::vector<Vec2d>> low_speed_polygon_vec{},
      low_speed_polygon_transfer_vec{};
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
    if (!is_barrier_gate &&
        obs_decision[i].obstacle_boundary.obstacle.is_virtual()) {
      continue;
    }
    if (!data_center_->master_info().enable_static_detour() &&
        (obs_decision[i].obstacle_boundary.obstacle.type() !=
         Obstacle::ObstacleType::UNKNOWN_UNMOVABLE)) {
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
                                ? 1.0
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

    // Polygon Preprocess 1：
    if (!obs_decision[i].obstacle_boundary.obstacle.is_static()) {
      if (!LowSpeedPolygonPreProcess(
              obs_decision[i], reference_line, front_enlarge_s, back_enlarge_s,
              0.5, low_speed_polygon_vec, low_speed_polygon_transfer_vec,
              aabox2d_points))
        continue;
    }
    // Polygon Preprocess 2：
    double ref_s = outside_data->path_obstacle_context.adc_boundary.start_s();
    if (!inside_data.is_reverse_driving &&
        (boundary.start_s() < ref_s && boundary.end_s() > ref_s)) {
      if (!ObsNearPolygonPreProcess(
              obs_decision[i], reference_line, boundary, front_enlarge_s,
              back_enlarge_s, ref_s, near_ego_polygon_vec,
              near_ego_polygon_transfer_vec, aabox2d_points))
        continue;
    }
    // Polygon Preprocess 3：
    if (is_barrier_gate) {
      double adc_end_s =
          outside_data->path_obstacle_context.adc_boundary.end_s();
      BarrierGatePolygonPreProcess(adc_end_s, reference_line, aabox2d_points);
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

  // vis
  VisObsPolygon(low_speed_polygon_vec, "low_speed_polygon");
  VisObsPolygon(low_speed_polygon_transfer_vec, "low_speed_polygon_transfer");
  VisObsPolygon(near_ego_polygon_vec, "near_ego_polygon");
  VisObsPolygon(near_ego_polygon_transfer_vec, "near_ego_polygon_transfer");

  double pre_t =
      inside_data.lane_borrow_side == LaneBorrowContext::BorrowSide::Left ? 3.0
                                                                          : 1.0;
  double steering = inside_data.vel_steer_angle / 100. *
                    VehicleParam::Instance()->max_steer_angle() /
                    VehicleParam::Instance()->steer_ratio();
  double next_heading = normalize_angle(
      inside_data.vel_heading + init_point.velocity() * pre_t /
                                    VehicleParam::Instance()->wheel_base() *
                                    std::tan(steering));
  Vec2d pre_point{
      init_point.x() + init_point.velocity() * std::cos(next_heading) * pre_t,
      init_point.y() + init_point.velocity() * std::sin(next_heading) * pre_t};
  SLPoint sl_pt{};
  reference_line->GetPointInFrenetFrame(pre_point, &sl_pt);
  LOG_INFO(
      "vel_heading: {:.4f}, steering: {:.4f}, "
      "next_heading: {:.4f}, target l: {:.4f}",
      inside_data.vel_heading, steering, next_heading, sl_pt.l());

  LOG_INFO(
      "init_point x: {:.4f}, init_point y: {:.4f}, vel_heading: {:.4f}, "
      "velocity: {:.4f}, steering: {:.4f}",
      init_point.y(), init_point.y(), inside_data.vel_heading,
      init_point.velocity(), inside_data.vel_steer_angle);
  auto virtual_obs = CreatVirtualObs(task_info, steering, init_point.velocity(),
                                     inside_data.vel_heading);
  VisObsPolygon(reference_line, virtual_obs, "virtual obs");
  /// region search
  PathData *last_path_data{nullptr};
  if (task_info.last_frame() != nullptr)
    last_path_data = task_info.last_frame()->outside_planner_data().path_data;
  auto bounds_info = RegionDecisionGraphSearch{}.GenerateBoundInfo(
      outside_data->path_context.lateral_static_obstacles, virtual_obs,
      left_boundary, right_boundary, left_road_boundary, right_road_boundary,
      last_path_data, init_sl_point.l(), init_sl_point.s(),
      inside_data.lane_borrow_side, reference_line, sl_pt.l());
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
  for (const auto &info : bounds_info) {
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

bool PathRegionSearchOptimizer::ComputeTunnelWidth(
    const std::vector<PathRegion::Bound> &bounds_info,
    OutsidePlannerData *const outside_data) {
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

LaneBorrowScenario PathRegionSearchOptimizer::StateIdentify(
    const InsidePlannerData &inside_data,
    const std::vector<PathRegion::Bound> &bounds_info,
    const SLPoint &init_sl_point, const bool is_lane_borrow,
    OutsidePlannerData *const outside_data) {
  // if (!is_lane_borrow) return LaneBorrowScenario::NONE;
  if (!is_lane_borrow) {
    return LaneBorrowScenario::NONE;
  }

  // single lane road link
  bool left_is_road{false}, right_is_road{false};
  for (const auto &bound_info : bounds_info) {
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
  for (const auto &bound_info : bounds_info) {
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
  for (const auto &bound_info : bounds_info) {
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

bool PathRegionSearchOptimizer::LowSpeedPolygonPreProcess(
    const PathObstacleDecision &obs_decision_element,
    const ReferenceLinePtr &reference_line, double front_enlarge_s,
    double back_enlarge_s, double dt,
    std::vector<std::vector<Vec2d>> &low_speed_polygon_vec,
    std::vector<std::vector<Vec2d>> &low_speed_polygon_transfer_vec,
    std::vector<Vec2d> &aabox2d_points) {
  // Low speed obs'polygon should be transfered

  double speed = obs_decision_element.obstacle_boundary.obstacle.speed();
  double velocity_heading =
      obs_decision_element.obstacle_boundary.obstacle.velocity_heading();
  std::vector<SLPoint> polygon_sl =
      obs_decision_element.obstacle_boundary.obstacle.polygon_sl();
  std::vector<Vec2d> polygon_xy;
  for (auto &sl : polygon_sl) {
    Vec2d xy;
    if (!reference_line->GetPointInCartesianFrame(sl, &xy)) {
      LOG_WARN("failed get xy point.");
      return false;
    }
    polygon_xy.emplace_back(xy);
  }
  low_speed_polygon_vec.emplace_back(polygon_xy);  // before transfer
  std::vector<Vec2d> polygon_xy_all{polygon_xy};
  for (auto &pt : polygon_xy) {
    Vec2d pt_move{pt.x() + dt * speed * std::cos(velocity_heading),
                  pt.y() + dt * speed * std::sin(velocity_heading)};
    polygon_xy_all.emplace_back(pt_move);
  }
  Polygon2d hull;
  Polygon2d{}.compute_convex_hull(polygon_xy_all, &hull);  // get hull
  std::vector<Vec2d> polygon_xy_transfer = hull.points();
  low_speed_polygon_transfer_vec.emplace_back(
      polygon_xy_transfer);  // after transfer
  std::vector<SLPoint> polygon_sl_transfer;
  for (auto &xy : polygon_xy_transfer) {
    SLPoint sl;
    if (!reference_line->GetPointInFrenetFrame(xy, &sl)) {
      LOG_WARN("failed get sl point.");
      return false;
    }
    polygon_sl_transfer.emplace_back(sl);
  }
  auto compare_l = [](auto &a, auto &b) { return a.l() < b.l(); };
  auto compare_s = [](auto &a, auto &b) { return a.s() < b.s(); };
  double s_max = std::max_element(polygon_sl_transfer.begin(),
                                  polygon_sl_transfer.end(), compare_s)
                     ->s(),
         s_min = std::min_element(polygon_sl_transfer.begin(),
                                  polygon_sl_transfer.end(), compare_s)
                     ->s(),
         l_max = std::max_element(polygon_sl_transfer.begin(),
                                  polygon_sl_transfer.end(), compare_l)
                     ->l(),
         l_min = std::min_element(polygon_sl_transfer.begin(),
                                  polygon_sl_transfer.end(), compare_l)
                     ->l();
  aabox2d_points[0] = Vec2d(s_min - front_enlarge_s, l_min);
  aabox2d_points[1] = Vec2d(s_min - front_enlarge_s, l_max);
  aabox2d_points[2] = Vec2d(s_max + back_enlarge_s, l_max);
  aabox2d_points[3] = Vec2d(s_max + back_enlarge_s, l_min);
  return true;
}

bool PathRegionSearchOptimizer::ObsNearPolygonPreProcess(
    const PathObstacleDecision &obs_decision_element,
    const ReferenceLinePtr &reference_line, const Boundary &boundary,
    double front_enlarge_s, double back_enlarge_s, double ref_s,
    std::vector<std::vector<Vec2d>> &near_ego_polygon_vec,
    std::vector<std::vector<Vec2d>> &near_ego_polygon_transfer_vec,
    std::vector<Vec2d> &aabox2d_points) {
  // The part of the near obstacle behind ego should be ignored

  LOG_INFO("ref_s: {:.4f}, start_s: {:.4f}, end_s: {:.4f}", ref_s,
           boundary.start_s(), boundary.end_s());
  std::vector<SLPoint> obstacle_box_sl =
      obs_decision_element.obstacle_boundary.obstacle.polygon_sl();
  // before print
  std::vector<Vec2d> obstacle_box_xy{};
  for (std::size_t j = 0; j < obstacle_box_sl.size(); ++j) {
    LOG_INFO("polygon points: {}, s: {:.4f}, l: {:.4f}", j,
             obstacle_box_sl[j].s(), obstacle_box_sl[j].l());
    Vec2d xy{};
    if (!reference_line->GetPointInCartesianFrame(
            {obstacle_box_sl[j].s(), obstacle_box_sl[j].l()}, &xy)) {
      LOG_WARN("failed get xy point.");
      return false;
    }
    obstacle_box_xy.emplace_back(xy);
  }
  near_ego_polygon_vec.emplace_back(obstacle_box_xy);
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
          obs_decision_element.obstacle_boundary.obstacle.id(), first.s(),
          first.l(), second.s(), second.l(), ref_s, ref_l);
    }
  }
  for (std::size_t k = 0; k < obstacle_box_sl.size() - 1; ++k) {
    if (obstacle_box_sl[k].x() >= ref_s) {
      obstacle_box_sl_new.emplace_back(obstacle_box_sl[k]);
    }
  }
  // after print
  std::vector<Vec2d> obstacle_box_xy_new{};
  for (std::size_t j = 0; j < obstacle_box_sl_new.size(); ++j) {
    LOG_INFO("fix polygon points: {}, s: {:.4f}, l: {:.4f}", j,
             obstacle_box_sl_new[j].x(), obstacle_box_sl_new[j].y());
    Vec2d xy{};
    if (!reference_line->GetPointInCartesianFrame(
            {obstacle_box_sl_new[j].x(), obstacle_box_sl_new[j].y()}, &xy)) {
      LOG_ERROR("failed get xy point.");
      return false;
    }
    obstacle_box_xy_new.emplace_back(xy);
  }
  Polygon2d obstacle_hull;
  Polygon2d{}.compute_convex_hull(obstacle_box_xy_new,
                                  &obstacle_hull);  // get hull
  std::vector<Vec2d> obstacle_box_xy_new_after_hull = obstacle_hull.points();
  near_ego_polygon_transfer_vec.emplace_back(obstacle_box_xy_new_after_hull);
  auto compare = [](auto &a, auto &b) { return a.l() < b.l(); };
  double l_max = std::max_element(obstacle_box_sl_new.begin(),
                                  obstacle_box_sl_new.end(), compare)
                     ->l(),
         l_min = std::min_element(obstacle_box_sl_new.begin(),
                                  obstacle_box_sl_new.end(), compare)
                     ->l();
  aabox2d_points[0] = Vec2d(ref_s, l_min);
  aabox2d_points[1] = Vec2d(ref_s, l_max);
  aabox2d_points[2] = Vec2d(boundary.end_s() + back_enlarge_s, l_max);
  aabox2d_points[3] = Vec2d(boundary.end_s() + back_enlarge_s, l_min);
  LOG_INFO(
      "old start_l: {:.4f}, end_l: {:.4f}, new start_l: {:.4f}, "
      "end_l: {:.4f}",
      boundary.start_l(), boundary.end_l(), l_min, l_max);
  return true;
}

void PathRegionSearchOptimizer::BarrierGatePolygonPreProcess(
    double adc_end_s, const ReferenceLinePtr &reference_line,
    std::vector<Vec2d> &aabox2d_points) {
  double aabox2d_start_s = aabox2d_points[0].x();
  double barrier_gate_start_s =
      data_center_->master_info().barrier_gate_context().barrier_gate.start_s;
  double back_buffer = 3.0;
  if (aabox2d_start_s > adc_end_s - back_buffer &&
      aabox2d_start_s <= barrier_gate_start_s) {
    ReferencePoint ref_pt;
    double left_bound, right_bound;
    if (reference_line->GetNearestRefPoint(aabox2d_start_s, &ref_pt)) {
      left_bound = ref_pt.left_bound(), right_bound = -ref_pt.right_bound();
    } else {
      left_bound = 5.0, right_bound = -5.0;
    }
    aabox2d_points[0] = Vec2d(aabox2d_points[0].x(), right_bound);
    aabox2d_points[1] = Vec2d(aabox2d_points[1].x(), left_bound);
    aabox2d_points[2] = Vec2d(aabox2d_points[2].x(), left_bound);
    aabox2d_points[3] = Vec2d(aabox2d_points[3].x(), right_bound);
  }
}

std::vector<AABox2d> PathRegionSearchOptimizer::CreatVirtualObs(
    TaskInfo &task_info, const double &steering, const double &velocity,
    const double &heading) {
  if (std::abs(velocity) < 0.1 ||
      !config::PlanningConfig::Instance()
           ->planning_research_config()
           .path_region_search_config.enable_create_virtual_obs) {
    return {};
  }

  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &init_point = inside_data.init_point;
  const auto reference_line = task_info.reference_line();
  const auto &init_sl_point = inside_data.init_sl_point;
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data == nullptr) {
    return {};
  }
  auto vir_boxes_ptr = decision_data->mutable_lateral_virtual_boxes();

  double pre_t = config::PlanningConfig::Instance()
                     ->planning_research_config()
                     .path_region_search_config.predict_time;
  double delt_a = config::PlanningConfig::Instance()
                      ->planning_research_config()
                      .path_region_search_config.delta_a;
  double a = velocity * velocity / VehicleParam::Instance()->wheel_base() *
             std::tan(steering);
  double a1 = a + delt_a;
  double a2 = a - delt_a;

  double k1 = std::clamp(a1 / velocity / velocity, -0.4, 0.4);
  double k2 = std::clamp(a2 / velocity / velocity, -0.4, 0.4);
  double l = velocity * pre_t;
  double theta1 = std::clamp(l * k1, -M_PI / 2.0, M_PI / 2.0);
  double theta2 = std::clamp(l * k2, -M_PI / 2.0, M_PI / 2.0);

  LOG_INFO(
      "a: {:.4f}, a1: {:.4f}, a2: {:.4f}, k1: {:.4f}, k2: {:.4f}, theta1: "
      "{:.4f}, theta2: {:.4f}",
      a, a1, a2, k1, k2, theta1, theta2);
  Vec2d box_pt1{init_point.x() + velocity * std::cos(heading + theta1) * pre_t,
                init_point.y() + velocity * std::sin(heading + theta1) * pre_t};

  Vec2d box_pt2{init_point.x() + velocity * std::cos(heading + theta2) * pre_t,
                init_point.y() + velocity * std::sin(heading + theta2) * pre_t};

  SLPoint sl_pt1{}, sl_pt2{};
  reference_line->GetPointInFrenetFrame(box_pt1, &sl_pt1);
  reference_line->GetPointInFrenetFrame(box_pt2, &sl_pt2);

  auto adc_boundary = outside_data->path_obstacle_context.adc_boundary;
  auto adc_width = std::abs(adc_boundary.start_l() - adc_boundary.end_l());
  sl_pt1.set_l(
      std::max(sl_pt1.l() + adc_width / 2.0, adc_boundary.end_l() + 0.5));
  sl_pt2.set_l(
      std::min(sl_pt2.l() - adc_width / 2.0, adc_boundary.start_l() - 0.5));

  auto create_box_from_slpt = [&](const SLPoint &sl_pt, double width_offset) {
    AABox2d box{Vec2d(sl_pt.s(), sl_pt.l()),
                Vec2d(init_sl_point.s() - 1.0, sl_pt.l() + width_offset)};
    box.set_id(3);
    vir_boxes_ptr->emplace_back(box);
    LOG_INFO(
        "box x: {:.4f}, y: {:.4f}, width: {:.4f}, length: {:.4f}, sl_pt1 s: "
        "{:.4f}, sl_pt1 l: {:.4f}",
        box.center_x(), box.center_y(), box.width(), box.length(), sl_pt.s(),
        sl_pt.l());
    if ((decision_data->create_lateral_virtual_obstacle(
             box, task_info.curr_referline_pt().heading(),
             VirtualObstacle::AVOIDANCE) != ErrorCode::PLANNING_OK)) {
      LOG_ERROR("Failed to extend avoidance obstacle.");
    }
    return;
  };
  create_box_from_slpt(sl_pt1, 5.0);
  create_box_from_slpt(sl_pt2, -5.0);

  const auto &virtual_boxes = task_info.current_frame()
                                  ->planning_data()
                                  .decision_data()
                                  .lateral_virtual_boxes();
  // std::vector<AABox2d> virtual_boxes{};
  return virtual_boxes;
}
}  // namespace planning
}  // namespace neodrive

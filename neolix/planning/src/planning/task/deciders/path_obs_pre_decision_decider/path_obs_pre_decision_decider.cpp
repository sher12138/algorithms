#include "path_obs_pre_decision_decider.h"

#include "common/math/util.h"
#include "src/planning/scenario_manager/scenario_common.h"

namespace neodrive {
namespace planning {

PathObsPreDecisionDecider::PathObsPreDecisionDecider() {
  name_ = "PathObsPreDecisionDecider";
}

PathObsPreDecisionDecider::~PathObsPreDecisionDecider() {
  obstacles_boundary_.clear();
  Reset();
}

ErrorCode PathObsPreDecisionDecider::Execute(TaskInfo &task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!Init(task_info.reference_line(),
            task_info.current_frame()->inside_planner_data(),
            task_info.current_frame()->planning_data().decision_data(),
            task_info.current_frame()->mutable_outside_planner_data())) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info.reference_line(),
               task_info.current_frame()->inside_planner_data(),
               task_info.current_frame()->mutable_outside_planner_data())) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool PathObsPreDecisionDecider::Init(const ReferenceLinePtr &reference_line,
                                     const InsidePlannerData &inside_data,
                                     const DecisionData &decision_data,
                                     OutsidePlannerData *const outside_data) {
  if (reference_line == nullptr) {
    LOG_ERROR("reference_line == nullptr.");
    return false;
  }

  path_max_lateral_range_ = 30.0;
  min_corridor_width_ = VehicleParam::Instance()->width() + 0.1;
  veh_x_ = inside_data.init_point.x();
  veh_y_ = inside_data.init_point.y();
  veh_theta_ = normalize_angle(inside_data.init_point.theta());
  delta_theta_ = veh_theta_ -
                 normalize_angle(outside_data->init_point_ref_point.heading());
  init_sl_point_ = inside_data.init_sl_point;

  auto &adc_boundary = outside_data->path_obstacle_context.adc_boundary;
  BuildAdcBoundary(reference_line, inside_data.init_point, adc_boundary);

  double front_lane_boud_dis =
      std::fmax(8.0, inside_data.init_point.velocity() * 6.0);
  if (!CalcLaneBound(reference_line, adc_boundary.start_s(),
                     adc_boundary.end_s() + front_lane_boud_dis, 0.5,
                     &left_lane_bound_, &right_lane_bound_, &min_lane_width_)) {
    LOG_ERROR("CalcLaneBound failed.");
    return false;
  }
  if (min_lane_width_ < min_corridor_width_) {
    LOG_ERROR(
        "front road bound corridor less than min_corridor_width: {:.3f}, "
        "{:.3f}",
        min_lane_width_, min_corridor_width_);
    return false;
  }
  const auto adc_width = VehicleParam::Instance()->width();
  auto &passable_l_range = outside_data->path_obstacle_context.passable_l_range;
  passable_l_range.first =
      std::max(-path_max_lateral_range_,
               -right_lane_bound_ + adc_width / 2.0 +
                   FLAGS_planning_road_border_remain_threshold);
  passable_l_range.second = std::min(
      path_max_lateral_range_, left_lane_bound_ - adc_width / 2.0 -
                                   FLAGS_planning_road_border_remain_threshold);

  obstacles_boundary_.clear();
  GetObstaclesBoundary(reference_line, decision_data, &obstacles_boundary_);
  back_attention_dis_ = VehicleParam::Instance()->back_edge_to_center() + 0.5;
  back_obs_velo_thresh_ = std::fmax(FLAGS_planning_pass_by_back_obs_velo,
                                    inside_data.init_point.velocity());
  side_obs_velo_thresh_ = std::fmax(FLAGS_planning_pass_by_side_obs_velo,
                                    inside_data.init_point.velocity());
  front_reverse_obs_velo_thresh_ = std::fmax(
      FLAGS_planning_pass_by_front_obs_velo, inside_data.init_point.velocity());

  if (outside_data->road_obs_path_boundries.empty()) {
    front_attention_dis_ =
        std::fmax(30.0, inside_data.init_point.velocity() * 8.0);
  } else {
    front_attention_dis_ = outside_data->road_obs_path_boundries.back().s;
  }
  front_side_pedestrain_dis_ =
      std::fmax(6.0, inside_data.init_point.velocity() * 4.0);

  if (!CalcMaxLaneBound(reference_line, adc_boundary.start_s(),
                        adc_boundary.end_s() + front_attention_dis_, 1.0,
                        &left_max_lane_bound_, &right_max_lane_bound_)) {
    LOG_INFO("CalcLaneBound err");
    return false;
  }
  left_max_lane_bound_ = std::fmax(left_max_lane_bound_, adc_boundary.end_l());
  right_max_lane_bound_ =
      std::fabs(std::fmin(-right_max_lane_bound_, adc_boundary.start_l()));
  fast_velo_ = scenario_common::ComputeObstacleSpeedDecision(
      inside_data.init_point.velocity(),
      outside_data->init_point_ref_point.lane_type_is_pure_city_driving());
  // fast_velo_ = std::fmin(fast_velo_, inside_data.init_point.velocity());

  return true;
}

bool PathObsPreDecisionDecider::Process(
    const ReferenceLinePtr &reference_line,
    const InsidePlannerData &inside_data,
    OutsidePlannerData *const outside_data) {
  if (inside_data.is_reverse_driving) {
    LOG_INFO("Do not nudge when reverse mode.");
    return true;
  }

  const auto &adc_boundary = outside_data->path_obstacle_context.adc_boundary;
  const auto &init_sl_point = inside_data.init_sl_point;
  auto &obstacles_decision =
      outside_data->path_obstacle_context.obstacle_decision;
  obstacles_decision.clear();
  obstacles_decision.resize(obstacles_boundary_.size());
  for (std::size_t i = 0; i < obstacles_boundary_.size(); ++i) {
    bool safe_obs = true;
    auto &obstacle_boundary = obstacles_boundary_[i];
    auto &obstacle_decision = obstacles_decision[i];
    Box2d obstacle_box(obstacle_boundary.obstacle.bounding_box().center(),
                       obstacle_boundary.obstacle.bounding_box().heading(),
                       obstacle_boundary.obstacle.bounding_box().length(),
                       obstacle_boundary.obstacle.bounding_box().width());

    // create ignore decision and then check
    obstacle_decision = CreateObstacleDecision(
        adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
        Decision::DecisionType::IGNORE, obstacle_boundary, obstacle_box,
        obstacle_boundary.boundary.end_l());

    // 1. ignore, obstacle_decision is IGNORE
    if (IgnoreVirtualObs(obstacle_boundary)) continue;
    if (IgnoreOutofLateralRange(obstacle_boundary)) continue;
    if (IgnoreOutofLongitudinalRange(obstacle_boundary, adc_boundary)) continue;
    if (IgnoreFastObs(reference_line, obstacle_boundary, adc_boundary,
                      inside_data.init_point.velocity(), safe_obs))
      continue;

    // 2. near back: INGORE
    double behind_obs_dis =
        adc_boundary.start_s() - obstacle_boundary.boundary.end_s();
    if (behind_obs_dis >= 0 && behind_obs_dis < back_attention_dis_) {
      continue;
    }

    // 2. obs on adc left or right, create RIGHT or LEFT
    if (obstacle_boundary.boundary.start_s() <=
        adc_boundary.end_s() + kMathEpsilon) {
      LeftRightOnAdcObs(obstacle_boundary, adc_boundary, obstacle_box,
                        obstacle_decision);
      // check
      if (obstacle_decision.decision_type == Decision::DecisionType::IGNORE) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::YIELD_DOWN, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());
      }
      // TEST
      LOG_DEBUG(
          "Left/Right, id {}, static {}, "
          "s_s {}, e_s {}, s_l {}, e_l {}, "
          "type {}",
          obstacle_boundary.obstacle.id(),
          obstacle_boundary.obstacle.is_static(),
          obstacle_boundary.boundary.start_s(),
          obstacle_boundary.boundary.end_s(),
          obstacle_boundary.boundary.start_l(),
          obstacle_boundary.boundary.end_l(),
          int(obstacle_decision.decision_type));

      continue;
    }
    // 3. obs on adc front
    if (obstacle_boundary.boundary.start_s() > adc_boundary.end_s() &&
        obstacle_boundary.boundary.start_s() <=
            adc_boundary.end_s() + front_attention_dis_) {
      FrontObs(reference_line, obstacle_boundary, adc_boundary, obstacle_box,
               obstacle_decision, safe_obs);
      // check
      if (obstacle_decision.decision_type == Decision::DecisionType::IGNORE) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::YIELD_DOWN, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());
      }
      // TEST
      LOG_DEBUG(
          "Front, id {}, static {}, "
          "s_s {}, e_s {}, s_l {}, e_l {},"
          "type {}",
          obstacle_boundary.obstacle.id(),
          obstacle_boundary.obstacle.is_static(),
          obstacle_boundary.boundary.start_s(),
          obstacle_boundary.boundary.end_s(),
          obstacle_boundary.boundary.start_l(),
          obstacle_boundary.boundary.end_l(),
          int(obstacle_decision.decision_type));
      continue;
    }
    // 4. not in consideration
    if (obstacle_decision.decision_type == Decision::DecisionType::IGNORE) {
      obstacle_decision = CreateObstacleDecision(
          adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
          Decision::DecisionType::YIELD_DOWN, obstacle_boundary, obstacle_box,
          obstacle_boundary.boundary.end_l());
      // TEST
      LOG_DEBUG(
          "Not in Considertation, id {}, static {}, "
          "s_s {}, e_s {}, s_l {}, e_l {},"
          "type {}",
          obstacle_boundary.obstacle.id(),
          obstacle_boundary.obstacle.is_static(),
          obstacle_boundary.boundary.start_s(),
          obstacle_boundary.boundary.end_s(),
          obstacle_boundary.boundary.start_l(),
          obstacle_boundary.boundary.end_l(),
          int(obstacle_decision.decision_type));
    }
  }

  auto &path_obstacle_context = outside_data->path_obstacle_context;
  path_obstacle_context.has_data = true;
  LOG_DEBUG("obstacle decision size is {}",
            path_obstacle_context.obstacle_decision.size());
  for (std::size_t i = 0; i < path_obstacle_context.obstacle_decision.size();
       ++i) {
    const auto &obs =
        path_obstacle_context.obstacle_decision[i].obstacle_boundary.obstacle;
    const auto &expand_boundary =
        path_obstacle_context.obstacle_decision[i].boundary;
    if (obs.is_virtual()) {
      LOG_DEBUG(
          "obs id: {}, "
          "is_virtual {}, "
          "decision type: {}",
          obs.id(), obs.is_virtual(),
          int(path_obstacle_context.obstacle_decision[i].decision_type));
    } else {
      LOG_DEBUG(
          "obs id: {}, static {}, "
          "s_s {:.3f}, e_s {:.3f}, "
          "s_l {:.3f}, e_l {:.3f}, "
          "type: {}, "
          "len {:.3f}, width {:.3f}, speed {:.3f}",
          obs.id(), obs.is_static(), expand_boundary.start_s(),
          expand_boundary.end_s(), expand_boundary.start_l(),
          expand_boundary.end_l(),
          int(path_obstacle_context.obstacle_decision[i].decision_type),
          obs.length(), obs.width(), obs.speed());
    }
  }

  return true;
}

void PathObsPreDecisionDecider::BuildAdcBoundary(
    const ReferenceLinePtr &reference_line, const TrajectoryPoint &init_point,
    Boundary &adc_boundary) const {
  Box2d adc_bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
      {init_point.x(), init_point.y()}, init_point.theta(), 0.2, 0.2, 0.2);
  std::vector<Vec2d> points;
  adc_bounding_box.get_all_corners(&points);
  SLPoint sl_point;
  for (const auto &pt : points) {
    reference_line->GetPointInFrenetFrame(pt, &sl_point);
    adc_boundary.set_start_s(std::min(adc_boundary.start_s(), sl_point.s()));
    adc_boundary.set_end_s(std::max(adc_boundary.end_s(), sl_point.s()));
    adc_boundary.set_start_l(std::min(adc_boundary.start_l(), sl_point.l()));
    adc_boundary.set_end_l(std::max(adc_boundary.end_l(), sl_point.l()));
  }
}

bool PathObsPreDecisionDecider::CalcLaneBound(
    const ReferenceLinePtr &reference_line, double start_s, double end_s,
    double offset, double *left_bound, double *right_bound,
    double *bound_width) const {
  if (left_bound == nullptr || right_bound == nullptr) {
    return false;
  }
  // add length protection for reference line
  double revise_start_s = start_s;
  double revise_end_s = end_s;
  if (revise_start_s < reference_line->ref_points().front().s()) {
    revise_start_s = reference_line->ref_points().front().s();
    LOG_INFO(
        "CalcLaneBound start_s {} < reference_line->line_start_s() {}, "
        "revise into {}",
        start_s, reference_line->ref_points().front().s(), revise_start_s);
  }
  if (revise_end_s > reference_line->ref_points().back().s()) {
    revise_end_s = reference_line->ref_points().back().s();
    LOG_INFO(
        "CalcLaneBound end_s {} < reference_line->line_end_s() {}, "
        "revise into {}",
        end_s, reference_line->ref_points().back().s(), revise_end_s);
  }

  ReferencePoint ref_pt;
  if (!reference_line->GetNearestRefPoint(revise_start_s, &ref_pt)) {
    LOG_ERROR("GetNearestRefPoint fail !");
    return false;
  }
  *left_bound = ref_pt.left_bound();
  *right_bound = ref_pt.right_bound();
  *bound_width = *left_bound + *right_bound;

  while (revise_start_s < revise_end_s) {
    if (!reference_line->GetNearestRefPoint(revise_start_s, &ref_pt)) {
      LOG_ERROR("GetNearestRefPoint fail !");
    }
    if (ref_pt.left_bound() <= *left_bound) {
      *left_bound = ref_pt.left_bound();
    }  // left and right bound no bigger than
    if (ref_pt.right_bound() <= *right_bound) {
      *right_bound = ref_pt.right_bound();
    }
    if (ref_pt.left_bound() + ref_pt.right_bound() <= *bound_width) {
      *bound_width = ref_pt.left_bound() + ref_pt.right_bound();
    }
    revise_start_s += offset;
  }
  return true;
}

bool PathObsPreDecisionDecider::GetObstaclesBoundary(
    const ReferenceLinePtr &reference_line, const DecisionData &decision_data,
    std::vector<PathObstacleBoundary> *obstacles_boundary) const {
  if (obstacles_boundary == nullptr) {
    return false;
  }
  obstacles_boundary->resize(decision_data.all_obstacle().size());
  for (std::size_t i = 0; i < decision_data.all_obstacle().size(); ++i) {
    auto obstacle = decision_data.all_obstacle()[i];
    PathObstacleBoundary obstacle_boundary{
        .boundary = obstacle->PolygonBoundary(),
        .obstacle = *obstacle,
        .obstacle_ptr = obstacle};
    obstacles_boundary->at(i) = std::move(obstacle_boundary);
  }
  sort(obstacles_boundary->begin(), obstacles_boundary->end(),
       [](const PathObstacleBoundary &obstacle_boundary,
          const PathObstacleBoundary &other_obstacle_boundary) {
         if (obstacle_boundary.boundary.start_s() <
             other_obstacle_boundary.boundary.start_s()) {
           return true;
         }
         return false;
       });
  return true;
}

bool PathObsPreDecisionDecider::CalcMaxLaneBound(
    const ReferenceLinePtr &reference_line, double origin_start_s,
    double origin_end_s, double offset, double *left_bound,
    double *right_bound) const {
  if (left_bound == nullptr || right_bound == nullptr) return false;
  // add length protection for reference line
  double start_s = origin_start_s;
  double end_s = origin_end_s;
  if (start_s < reference_line->ref_points().front().s()) {
    start_s = reference_line->ref_points().front().s();
    LOG_INFO(
        "CalcMaxLaneBound start_s {} < reference_line->line_start_s() {}, "
        "revise into {}",
        origin_start_s, reference_line->ref_points().front().s(), start_s);
  }
  if (auto es = reference_line->ref_points().back().s(); end_s > es) {
    end_s = es;
    LOG_INFO(
        "CalcMaxLaneBound end_s {} < reference_line->line_end_s() {}, "
        "revise into {}",
        origin_end_s, es, end_s);
  }
  ReferencePoint ref_pt;
  if (!reference_line->GetNearestRefPoint(start_s, &ref_pt)) {
    LOG_ERROR("GetNearestRefPoint fail !");
    return false;
  }
  *left_bound = ref_pt.left_bound();
  *right_bound = ref_pt.right_bound();
  double cut_off_end_s = end_s;
  cut_off_end_s =
      std::fmin(cut_off_end_s, reference_line->ref_points().back().s());

  while (start_s < cut_off_end_s) {
    if (!reference_line->GetNearestRefPoint(start_s, &ref_pt)) {
      LOG_ERROR("GetNearestRefPoint fail !");
    }
    if (ref_pt.left_bound() > *left_bound) {
      *left_bound = ref_pt.left_bound();
    }
    if (ref_pt.right_bound() > *right_bound) {
      *right_bound = ref_pt.right_bound();
    }
    start_s += offset;
  }
  return true;
}

bool PathObsPreDecisionDecider::VirtualObstacleNeedIgnore(
    const Obstacle &virtual_obstacle) const {
  if (!virtual_obstacle.is_virtual()) {
    return false;
  }
  double proj_width = virtual_obstacle.PolygonBoundary().end_l() -
                      virtual_obstacle.PolygonBoundary().start_l();
  LOG_INFO(
      "virtual obstacle start_l:{}, end_l:{}, proj_width:{}, "
      "obstacle_width:{}",
      virtual_obstacle.PolygonBoundary().start_l(),
      virtual_obstacle.PolygonBoundary().end_l(), proj_width,
      virtual_obstacle.width());
  if (fabs(proj_width - virtual_obstacle.width()) < 0.1) {
    if (virtual_obstacle.PolygonBoundary().start_l() < 0.0 &&
        virtual_obstacle.PolygonBoundary().end_l() > 0.0) {
      return false;
    }
  }
  return true;
}

PathObstacleDecision PathObsPreDecisionDecider::CreateObstacleDecision(
    const Decision::DecisionType &decision_type,
    const PathObstacleBoundary &obstacle_boundary, const Box2d &obstacle_box,
    const double l_bound) const {
  PathObstacleDecision obstacle_decision;
  obstacle_decision.decision_type = decision_type;
  obstacle_decision.obstacle_boundary = obstacle_boundary;
  obstacle_decision.obstacle_box = obstacle_box;
  obstacle_decision.l_bound = l_bound;
  Vec2d tmp_pt;
  for (std::size_t i = 0; i < obstacle_boundary.obstacle.polygon_sl().size();
       ++i) {
    auto pt = obstacle_boundary.obstacle.polygon_sl().at(i);
    tmp_pt.set_x(pt.s());
    tmp_pt.set_y(pt.l());
    obstacle_decision.obstacle_box_sl.push_back(tmp_pt);
  }
  return obstacle_decision;
}

PathObstacleDecision PathObsPreDecisionDecider::CreateObstacleDecision(
    const Boundary &adc_boundary, const double veh_x, const double veh_y,
    const double veh_theta, const double delta_theta,
    const Decision::DecisionType &decision_type,
    const PathObstacleBoundary &obstacle_boundary, const Box2d &obstacle_box,
    const double l_bound) const {
  std::vector<Vec2d> obstacle_box_xy_expand;
  std::vector<Vec2d> obstacle_box_sl_expand;
  Boundary boundary_expand = obstacle_boundary.boundary;
  Obstacle obstacle = obstacle_boundary.obstacle;

  Box2d obstacle_box_expand(obstacle_box.center(), obstacle_box.heading(),
                            obstacle_box.length(), obstacle_box.width());
  obstacle.expand_bounding_box_corners_xy_from_origin_info(
      0.0, obstacle_box_xy_expand);
  obstacle.expand_bounding_box_corners_sl_from_origin_info(
      0.0, obstacle_box_sl_expand);

  PathObstacleDecision obstacle_decision;
  obstacle_decision.decision_type = decision_type;
  obstacle_decision.obstacle_boundary = obstacle_boundary;
  obstacle_decision.obstacle_box = obstacle_box_expand;
  obstacle_decision.l_bound = l_bound;
  obstacle_decision.boundary = boundary_expand;
  obstacle_decision.obstacle_box_xy = obstacle_box_xy_expand;
  obstacle_decision.obstacle_box_sl = obstacle_box_sl_expand;

  return obstacle_decision;
}

bool PathObsPreDecisionDecider::MergeOverlapedObs(
    const InsidePlannerData &inside_data,
    std::vector<PathObstacleDecision> &obstacles_for_path) const {
  for (std::size_t i = 0; i < obstacles_for_path.size(); ++i) {
    for (std::size_t j = i + 1; j < obstacles_for_path.size(); ++j) {
      auto &first_decision = obstacles_for_path[i];
      auto &first_obs = first_decision.obstacle_boundary.obstacle;
      auto &second_decision = obstacles_for_path[j];
      auto &second_obs = second_decision.obstacle_boundary.obstacle;

      // just process different type obs
      if (first_obs.is_static() == second_obs.is_static()) {
        continue;
      }
      if (PolygonCollision(first_obs.polygon().points(),
                           second_obs.polygon().points())) {
        if (first_obs.is_static()) {
          first_obs.set_is_static(second_obs.is_static());
          first_obs.set_speed(second_obs.speed());
          LOG_INFO(
              "obs {} overlap with obs {}, "
              "change static obs {} into dynamic, "
              "change decision {} into {}",
              first_obs.id(), second_obs.id(), first_obs.id(),
              int(first_decision.decision_type),
              int(second_decision.decision_type));
          first_decision.decision_type = second_decision.decision_type;
        } else if (second_obs.is_static()) {
          second_obs.set_is_static(first_obs.is_static());
          second_obs.set_speed(first_obs.speed());
          LOG_INFO(
              "obs {} overlap with obs {}, "
              "change static obs {} into dynamic, "
              "change decision {} into {}",
              first_obs.id(), second_obs.id(), second_obs.id(),
              int(second_decision.decision_type),
              int(first_decision.decision_type));
          second_decision.decision_type = first_decision.decision_type;
        } else {
          LOG_ERROR("out of design, err");
        }
      }
    }
  }
  return true;
}

bool PathObsPreDecisionDecider::IgnoreVirtualObs(
    const PathObstacleBoundary &obstacle_boundary) const {
  return obstacle_boundary.obstacle.is_virtual();
}

bool PathObsPreDecisionDecider::IgnoreOutofLateralRange(
    const PathObstacleBoundary &obstacle_boundary) const {
  return (
      obstacle_boundary.boundary.end_l() <=
          -(right_max_lane_bound_ +
            FLAGS_planning_road_border_remain_threshold) ||
      obstacle_boundary.boundary.start_l() >=
          (left_max_lane_bound_ + FLAGS_planning_road_border_remain_threshold));
}

bool PathObsPreDecisionDecider::IgnoreOutofLongitudinalRange(
    const PathObstacleBoundary &obstacle_boundary,
    const Boundary &adc_boundary) const {
  double behind_obs_dis =
      adc_boundary.start_s() - obstacle_boundary.boundary.end_s();
  double front_obs_dis =
      obstacle_boundary.boundary.start_s() - adc_boundary.start_s();
  return (behind_obs_dis > back_attention_dis_ ||
          front_obs_dis > front_attention_dis_);
}

bool PathObsPreDecisionDecider::IgnoreFastObs(
    const ReferenceLinePtr &reference_line,
    const PathObstacleBoundary &obstacle_boundary, const Boundary &adc_boundary,
    const double veh_v, bool &safe_check) const {
  if (obstacle_boundary.obstacle.speed() > fast_velo_ + kMathEpsilon) {
    safe_check = IsSafeDynamicObs(reference_line, obstacle_boundary,
                                  adc_boundary, veh_v);
    return safe_check;
  }
  return false;
}

bool PathObsPreDecisionDecider::BackNearObs(
    const PathObstacleBoundary &obstacle_boundary, const Boundary &adc_boundary,
    std::vector<PathObstacleDecision> &obstacles_decision) const {
  double behind_obs_dis =
      adc_boundary.start_s() - obstacle_boundary.boundary.end_s();
  return behind_obs_dis >= 0.0 && behind_obs_dis <= back_attention_dis_;
}

bool PathObsPreDecisionDecider::LeftRightOnAdcObs(
    const PathObstacleBoundary &obstacle_boundary, const Boundary &adc_boundary,
    const Box2d &obstacle_box, PathObstacleDecision &obstacle_decision) const {
  if (obstacle_boundary.boundary.start_l() > adc_boundary.end_l()) {
    obstacle_decision = CreateObstacleDecision(
        adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
        Decision::DecisionType::GO_RIGHT, obstacle_boundary, obstacle_box,
        obstacle_boundary.boundary.start_l());
  } else if (obstacle_boundary.boundary.end_l() < adc_boundary.start_l()) {
    obstacle_decision = CreateObstacleDecision(
        adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
        Decision::DecisionType::GO_LEFT, obstacle_boundary, obstacle_box,
        obstacle_boundary.boundary.end_l());
  } else if (obstacle_boundary.obstacle.center_sl().l() <= init_sl_point_.l()) {
    obstacle_decision = CreateObstacleDecision(
        adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
        Decision::DecisionType::GO_LEFT, obstacle_boundary, obstacle_box,
        obstacle_boundary.boundary.end_l());
  } else {
    obstacle_decision = CreateObstacleDecision(
        adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
        Decision::DecisionType::GO_RIGHT, obstacle_boundary, obstacle_box,
        obstacle_boundary.boundary.start_l());
  }

  return true;
}

bool PathObsPreDecisionDecider::FrontObs(
    const ReferenceLinePtr &reference_line,
    const PathObstacleBoundary &obstacle_boundary, const Boundary &adc_boundary,
    const Box2d &obstacle_box, PathObstacleDecision &obstacle_decision,
    bool saft_obs) const {
  // calc obs left and right remian dis in road
  double obs_left_lane_bound = 0.0;
  double obs_right_lane_bound = 0.0;
  double min_obs_lane_width = 0.0;
  if (!CalcLaneBound(reference_line, obstacle_boundary.boundary.start_s(),
                     obstacle_boundary.boundary.end_s(), 0.5,
                     &obs_left_lane_bound, &obs_right_lane_bound,
                     &min_obs_lane_width)) {
    LOG_INFO("CalcLaneBound err");
    return false;
  }
  double left_remain_distance =
      std::max(obs_left_lane_bound - obstacle_boundary.boundary.end_l(), 0.0);
  double right_remain_distance = std::max(
      obstacle_boundary.boundary.start_l() + obs_right_lane_bound, 0.0);
  const double REMAIN_DIS_TRESH = 0.3;
  double delta_dis = right_remain_distance - left_remain_distance;

  // static obs
  if (obstacle_boundary.obstacle.is_static() ||
      (!obstacle_boundary.obstacle.is_static() &&
       (obstacle_boundary.obstacle.speed() + 1e-10 < fast_velo_ ||
        !saft_obs))) {
    if (!FLAGS_planning_default_left_right_side) {
      if (delta_dis > REMAIN_DIS_TRESH &&
          right_remain_distance > min_corridor_width_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_RIGHT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.start_l());
      } else if (delta_dis <= REMAIN_DIS_TRESH &&
                 left_remain_distance > min_corridor_width_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_LEFT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());
      } else {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::YIELD_DOWN, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());
      }
    } else {
      if (delta_dis < -REMAIN_DIS_TRESH &&
          left_remain_distance > min_corridor_width_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_LEFT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());
      } else if (delta_dis >= -REMAIN_DIS_TRESH &&
                 right_remain_distance > min_corridor_width_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_RIGHT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.start_l());
      } else {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::YIELD_DOWN, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());
      }
    }
    return true;
  }

  // dynamic obs, divided into 3 regions by +- 60 degree line
  // 1. do not nudge dynamic vehicle
  if (obstacle_boundary.obstacle.type() == Obstacle::ObstacleType::VEHICLE) {
    obstacle_decision = CreateObstacleDecision(
        adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
        Decision::DecisionType::FOLLOW_DOWN, obstacle_boundary, obstacle_box,
        obstacle_boundary.boundary.start_l());
    return true;
  }
  // 2. other obs
  double heading_diff = 0.0;
  if (!CalcObsHeadingDiff(reference_line, obstacle_boundary, &heading_diff)) {
    LOG_INFO("CalcObsHeadingDiff err");
    return false;
  }
  std::size_t region_no = GetDynamicObsMotionRegion(heading_diff);
  if (region_no == 0) {
    if (!FLAGS_planning_default_left_right_side) {
      if (delta_dis > REMAIN_DIS_TRESH &&
          right_remain_distance > min_corridor_width_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_RIGHT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.start_l());

      } else if (delta_dis <= REMAIN_DIS_TRESH &&
                 left_remain_distance > min_corridor_width_) {
        auto obstacle_go_left_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_LEFT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l() +
                FLAGS_planning_dp_path_decision_buffer);
      }
    } else {
      if (delta_dis < -REMAIN_DIS_TRESH &&
          left_remain_distance > min_corridor_width_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_LEFT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());

      } else if (delta_dis >= -REMAIN_DIS_TRESH &&
                 right_remain_distance > min_corridor_width_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::GO_RIGHT, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.start_l());
      }
    }
  }
  // second region, obs in front and go side
  else if (region_no == 1) {
  }
  // third region, obs is go reverse
  else {
    if (obstacle_boundary.obstacle.speed() > front_reverse_obs_velo_thresh_) {
      if (obstacle_boundary.obstacle.speed() <= fast_velo_) {
        obstacle_decision = CreateObstacleDecision(
            adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
            Decision::DecisionType::YIELD_DOWN, obstacle_boundary, obstacle_box,
            obstacle_boundary.boundary.end_l());
      }
    }
  }
  // dynamic follow down
  if (obstacle_decision.decision_type == Decision::DecisionType::IGNORE) {
    obstacle_decision = CreateObstacleDecision(
        adc_boundary, veh_x_, veh_y_, veh_theta_, delta_theta_,
        Decision::DecisionType::FOLLOW_DOWN, obstacle_boundary, obstacle_box,
        obstacle_boundary.boundary.start_l());
  }

  return true;
}

bool PathObsPreDecisionDecider::CalcObsHeadingDiff(
    const ReferenceLinePtr &reference_line,
    const PathObstacleBoundary &obstacle_boundary, double *heading_diff) const {
  if (heading_diff == nullptr) {
    LOG_INFO("input nullptr");
    return false;
  }
  ReferencePoint reference_point;
  if (!reference_line->GetNearestRefPoint(
          obstacle_boundary.obstacle.center_sl().s(), &reference_point)) {
    LOG_INFO("GetNearestRefPoint fail");
  }
  *heading_diff =
      std::fabs(normalize_angle(obstacle_boundary.obstacle.velocity_heading() -
                                reference_point.heading()));
  LOG_DEBUG(
      "obs heading {}, obs velo heading {}, "
      "ref pt heading {}, heading dif {}",
      obstacle_boundary.obstacle.heading(),
      obstacle_boundary.obstacle.velocity_heading(), reference_point.heading(),
      *heading_diff);
  return true;
}

bool PathObsPreDecisionDecider::IsSafeDynamicObs(
    const ReferenceLinePtr &reference_line,
    const PathObstacleBoundary &obstacle_boundary, const Boundary &adc_boundary,
    const double veh_v) const {
  if (obstacle_boundary.boundary.end_s() + kMathEpsilon <
      adc_boundary.start_s()) {
    return true;
  }
  if (obstacle_boundary.obstacle.speed() > veh_v + kMathEpsilon) {
    return true;
  }
  double follow_dis = 0.3;
  double delta_s = obstacle_boundary.boundary.start_s() - adc_boundary.end_s();
  if (delta_s < follow_dis) {
    LOG_INFO("nearby dynamic obs [{}], dangerous!!",
             obstacle_boundary.obstacle.id());
    return false;
  }
  ReferencePoint reference_point;
  if (!reference_line->GetNearestRefPoint(
          obstacle_boundary.obstacle.center_sl().s(), &reference_point)) {
    LOG_INFO("GetNearestRefPoint fail");
    return true;
  }
  double heading_diff =
      std::abs(normalize_angle(obstacle_boundary.obstacle.velocity_heading() -
                               reference_point.heading()));
  if (heading_diff > M_PI_2 * 0.67 && M_PI - heading_diff > M_PI_2 * 0.67) {
    return true;
  }
  double deceleration = -3.0, buffer_t = 0.2;
  if (heading_diff < M_PI / 2.0) {
    // same direction
    double veh_s =
        veh_v * buffer_t +
        (std::pow(obstacle_boundary.obstacle.speed(), 2) - std::pow(veh_v, 2)) /
            (2 * deceleration);
    double obs_s =
        obstacle_boundary.obstacle.speed() *
        ((obstacle_boundary.obstacle.speed() - veh_v) / deceleration +
         buffer_t);
    if (veh_s - obs_s > delta_s - follow_dis) {
      LOG_INFO(
          "vehicle v: {:.3f}, s: {:.3f}, obstalce v: {:.3f}, s: {:.3f}, delta "
          "s: {:.3f}",
          veh_v, veh_s, obstacle_boundary.obstacle.speed(), obs_s, delta_s);
      LOG_INFO("find front slow dynamic obs [{}], dangerous!!",
               obstacle_boundary.obstacle.id());
      return false;
    }
  } else {
    // oppsite direction
    double veh_s =
        veh_v * buffer_t + (-std::pow(veh_v, 2)) / (2 * deceleration);
    double obs_s =
        obstacle_boundary.obstacle.speed() * (-veh_v / deceleration + buffer_t);
    if (veh_s + obs_s > delta_s - follow_dis) {
      LOG_INFO(
          "vehicle v: {:.3f}, s: {:.3f}, obstalce v: {:.3f}, s: {:.3f}, delta "
          "s: {:.3f}",
          veh_v, veh_s, obstacle_boundary.obstacle.speed(), obs_s, delta_s);
      LOG_INFO("find front quick reverse dynamic obs [{}], dangerous!!",
               obstacle_boundary.obstacle.id());
      return false;
    }
  }
  return true;
}

// first region 0 means obs go front,
// second region 1 means obs go side,
// third region 2 means obs go reverse
std::size_t PathObsPreDecisionDecider::GetDynamicObsMotionRegion(
    const double heading_diff) const {
  std::size_t ret = 0;
  double first_region_dif =
      FLAGS_planning_dynamic_obstacle_nudge_heading_threshold;
  double second_region_dif = M_PI - first_region_dif;
  if (heading_diff <= first_region_dif) {
    ret = 0;
  } else if (heading_diff > first_region_dif &&
             heading_diff <= second_region_dif) {
    ret = 1;
  } else {
    ret = 2;
  }
  return ret;
}

}  // namespace planning
}  // namespace neodrive

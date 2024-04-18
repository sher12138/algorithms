#include "speed_vehicle_merge_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

namespace {

constexpr double kBackAttentionDistance = 25;
constexpr double kFrontAttentionDistance = 20;

constexpr double kBuffer = 0.5;

void VisSafeBoundaryArea(ReferenceLinePtr ref_line, const Boundary& bdry) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("MergeCheckPolygon2d");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt{};
  auto polygon = event->mutable_polygon()->Add();
  ref_line->GetPointInCartesianFrame({bdry.start_s(), bdry.end_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame({bdry.start_s(), bdry.start_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame({bdry.end_s(), bdry.start_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame({bdry.end_s(), bdry.end_l()}, &pt);
  set_pt(polygon->add_point(), pt);
}

void VisSafeBoundaryArea(const Polygon2d& box) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("MergeCheckPolygon2d");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto polygon = event->mutable_polygon()->Add();
  for (const auto& pt : box.points()) {
    set_pt(polygon->add_point(), pt);
  }
}

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

// return false,
bool JudgeStraight(const std::vector<PathPoint>& points, const double length,
                   std::pair<double, bool>& straight_valid_length,
                   double curv_threshold = 0.01) {
  straight_valid_length.first = 0.0;
  for (auto& pt : points) {
    if (pt.kappa() < curv_threshold && pt.kappa() > -curv_threshold) {
      straight_valid_length.first = pt.s();
      if (pt.s() > length) return true;
    } else {
      if (pt.kappa() >= 0) {
        if (straight_valid_length.second) return false;
        straight_valid_length.second = false;
      } else {
        straight_valid_length.second = true;
      }
      return false;
    }
  }
  return false;
}
}  // namespace
SpeedVehicleMergeDecider::SpeedVehicleMergeDecider() {
  name_ = "SpeedVehicleMergeDecider";
}

SpeedVehicleMergeDecider::~SpeedVehicleMergeDecider() { Reset(); }

ErrorCode SpeedVehicleMergeDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (!Init(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  PosterioriUpdate(task_info);
  return ErrorCode::PLANNING_OK;
}

void SpeedVehicleMergeDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_speed_limit_) {
    deceleration_ =
        std::max(-3.0, std::min((speed_limit_ - adc_current_v_) * 10.0, 0.0));
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::MERGE_IN);
    internal_speed_limit.add_upper_bounds(speed_limit_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(deceleration_);
    LOG_INFO(
        "MERGE_IN {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        speed_limit_, deceleration_);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

bool SpeedVehicleMergeDecider::Init(TaskInfo& task_info) {
  const auto& adc_corner_pt_coordinate =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.adc_corner_pt_coordinate;
  if (adc_corner_pt_coordinate.size() <
      static_cast<int>(AdcCollideCornerPoint::NONE)) {
    LOG_ERROR("adc_corner_pt_coordinate size < 4");
    return false;
  }

  if (!update_speed_limit_) {
    last_limited_speed_ =
        task_info.current_frame()->inside_planner_data().vel_v;
  }
  update_speed_limit_ = false;
  deceleration_ = 0.0;
  speed_limit_ = std::numeric_limits<double>::infinity();

  auto inside_data = task_info.current_frame()->inside_planner_data();
  adc_width_ = VehicleParam::Instance()->width();
  adc_length_ = VehicleParam::Instance()->length();
  adc_back_edge_to_center_ = VehicleParam::Instance()->back_edge_to_center();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  adc_heading_ = inside_data.vel_heading;
  adc_polygon_ = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.0, 0.0,
      0.0);

  speed_vehicle_merging_decider_config_ptr_ =
      &config::PlanningConfig::Instance()
           ->planning_research_config()
           .speed_vehicle_merging_decider_config;
  if (nullptr == speed_vehicle_merging_decider_config_ptr_) {
    LOG_ERROR("get config failed.");
    return false;
  }

  auto ref_line = task_info.reference_line();
  adc_current_s_ = task_info.curr_sl().s();
  const auto& junction_list = ref_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }
    if (overlap.start_s - 1 <= adc_current_s_ &&
        overlap.end_s + 1 >= adc_current_s_ &&
        (junction_ptr->Type() ==
         static_cast<uint32_t>(JunctionType::CROSS_ROAD))) {
      adc_is_in_junction_ = true;
      adc_current_junction_type_ = junction_ptr->Type();
      adc_current_junction_start_s_ = overlap.start_s;
      adc_current_junction_end_s_ = overlap.end_s;
      break;
    }
  }

  obs_on_merging_lane_.clear();

  GenerateSafeCheckPolygon(task_info);

  return true;
}

/// from lane merging
bool SpeedVehicleMergeDecider::Process(TaskInfo& task_info) {
  if (!GetObsOnAttentionLane(task_info)) {
    return true;
  }

  GetAttentionObsCollisionInfo(task_info);
  GetSpeedLimitByAttentionObs(task_info);

  return true;
}

void SpeedVehicleMergeDecider::PosterioriUpdate(TaskInfo& task_info) {
  if (speed_limit_ > 2.0) return;

  bool flag = false;
  for (auto iter = obs_on_merging_lane_.begin();
       iter != obs_on_merging_lane_.end(); iter++) {
    if (!(iter->second.has_obs_decision)) {
      LOG_WARN("obs [{}] not has collision decision, not limit speed.",
               iter->first);
      continue;
    }

    const auto& attention_obs_info = iter->second;
    double l2_dis =
        adc_polygon_.distance_to(attention_obs_info.obs_ptr->polygon());

    if (l2_dis <
        speed_vehicle_merging_decider_config_ptr_->safe_limit_config_distance +
            attention_obs_info.obs_ptr->width() + adc_width_) {
      flag = true;
      break;
    }
  }
  // The distance between all obstacles and the car is greater than 2
  if (!flag) {
    LOG_INFO(
        "The distance between all obstacles and the car is greater than 3， "
        "limit speed must >=2");
    speed_limit_ = std::max(2.0, speed_limit_);
  }
}

bool SpeedVehicleMergeDecider::IngoreByCrossRoad(TaskInfo& task_info) {
  if (adc_is_in_junction_) {
    double diff_s = std::min(
        (adc_current_junction_end_s_ - adc_current_junction_start_s_) / 2,
        adc_current_v_ * 3);
    if (adc_current_s_ + diff_s < adc_current_junction_end_s_) {
      LOG_INFO("From the car to the crosswalk and far from the end, skip!");
      return true;
    }
  }

  return false;
}

bool SpeedVehicleMergeDecider::GetObsOnAttentionLane(TaskInfo& task_info) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if (!(speed_planner_common::InMergingArea(task_info) ||
        speed_planner_common::InDivergingArea(task_info))) {
    return false;
  }
  double dis_to_zone = traffic_conflict_zone_context.route_length_to_zone[0];
  if (dis_to_zone >
      std::max(static_cast<double>(speed_vehicle_merging_decider_config_ptr_
                                       ->attention_max_accumulate_s),
               adc_current_v_ * 3)) {
    return false;
  }
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();

  if (dynamic_obstacle.empty()) {
    return false;
  }
  const auto& merging_ids_ = traffic_conflict_zone_context.merging_ids;
  std::unordered_set<uint64_t> merging_lane_ids;
  for (auto iter = merging_ids_.begin(); iter != merging_ids_.end(); iter++) {
    merging_lane_ids.insert(iter->id);
  }

  for (const auto& obs : dynamic_obstacle) {
    double heading_diff = normalize_angle(
        obs->velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading);
    const auto& obs_boundary = obs->PolygonBoundary();
    if (std::abs(heading_diff) > M_PI_2) {
      LOG_INFO("obs [{}] heading diff {:.3f} > M_PI_2, ignore.", obs->id(),
               heading_diff);
      continue;
    }

    if ((obs_boundary.end_s() < adc_current_s_) &&
        (obs->speed() <
         adc_current_v_ * speed_vehicle_merging_decider_config_ptr_
                              ->consider_adc_back_speed_ratio)) {
      LOG_INFO("obs [{}] behind adc and slow, ignore.", obs->id());
      continue;
    }
    if (obs_boundary.start_s() >
        adc_front_edge_s_ +
            speed_vehicle_merging_decider_config_ptr_->consider_adc_front_dis) {
      LOG_INFO("obs [{}] ahead adc, ignore.", obs->id());
      continue;
    }

    if (merging_lane_ids.find(obs->matched_lane_id()) !=
        merging_lane_ids.end()) {
      obs_on_merging_lane_[obs->id()].obs_ptr = obs;
      LOG_INFO("obs [{}] on merging lane.add", obs->id());
      continue;
    }

    if ((!is_boundary_ && back_boundary_polygon_.has_overlap(obs->polygon())) ||
        (is_boundary_ && back_boudnary_.has_overlap(obs->PolygonBoundary())) &&
            (adc_current_s_ - adc_back_edge_to_center_ >
             obs->center_sl().s())) {
      LOG_INFO("if slow ,may be collision,skip obs {}", obs->id());
      continue;
    }

    if (JudgeSameLane(obs, task_info.reference_line(), adc_current_s_)) {
      if (!IgnoreObsOnCurrentLane(
              task_info, task_info.current_frame()->inside_planner_data(),
              traffic_conflict_zone_context, obs)) {
        obs_on_merging_lane_[obs->id()].obs_ptr = obs;
        LOG_INFO("obs [{}] on merging lane.add", obs->id());
      }
    }

    LOG_INFO("can't define obs {}", obs->id());
  }
  LOG_INFO("obs_on_merging_lane_ size: [{}]", obs_on_merging_lane_.size());

  return true;
}

bool SpeedVehicleMergeDecider::JudgeSameLane(
    Obstacle* const obstacle, const ReferenceLinePtr& reference_line,
    const double& adc_current_s) {
  double min_l = 0.0;
  double max_l = 0.0;
  double max_s = 0.0;
  double min_s = -1.0;
  for (const auto& ref_point : reference_line->ref_points()) {
    if (ref_point.s() < adc_current_s - kBackAttentionDistance) continue;
    if (ref_point.s() > adc_current_s + kFrontAttentionDistance) break;
    if (std::abs(min_s - -1.0) < 1e-6) {
      min_s = ref_point.s();
    }
    max_l = std::max(max_l, ref_point.left_lane_bound());
    min_l = std::max(min_l, ref_point.right_lane_bound());
    max_s = ref_point.s();
  }

  Boundary boundary{min_s, max_s, -min_l, max_l};
  LOG_INFO("find boundary: {},{},{},{}", min_s, max_s, min_l, max_l);
  if (boundary.has_overlap(obstacle->PolygonBoundary())) {
    return true;
  }
  return false;
}

bool SpeedVehicleMergeDecider::GetAttentionObsCollisionInfo(
    TaskInfo& task_info) {
  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dynamic_obstacles_decision;
  for (const auto& obs_decision : dynamic_obstacles_decision) {
    if (obs_on_merging_lane_.find(obs_decision.obstacle.id()) !=
        obs_on_merging_lane_.end()) {
      auto& attention_obs_info =
          obs_on_merging_lane_[obs_decision.obstacle.id()];
      attention_obs_info.has_obs_decision = true;
      attention_obs_info.adc_first_collide_corner_point =
          obs_decision.adc_first_collide_corner_point;
      attention_obs_info.first_collide_t =
          obs_decision.lower_points.front().first.t();
      attention_obs_info.first_collide_s =
          obs_decision.lower_points.front().first.s();
      attention_obs_info.first_collide_obs_project_v =
          obs_decision.lower_points.front().second;
      LOG_INFO(
          "obs [{}] polygon first collide t,s: {:.3f}, {:.3f}; "
          "first_collide_obs_project_v: {:.3f} ",
          obs_decision.obstacle.id(), attention_obs_info.first_collide_t,
          attention_obs_info.first_collide_s,
          attention_obs_info.first_collide_obs_project_v);
    }
  }

  for (auto iter = obs_on_merging_lane_.begin();
       iter != obs_on_merging_lane_.end(); iter++) {
    if (!(iter->second.has_obs_decision)) {
      LOG_INFO("obs [{}] new collision check ", iter->first);
      CollisionCheckWithObstaclePolyline(
          task_info.reference_line(),
          task_info.current_frame()->inside_planner_data(),
          *(iter->second.obs_ptr),
          task_info.current_frame()->mutable_outside_planner_data());
    }
  }

  return true;
}

bool SpeedVehicleMergeDecider::EgoLaneObsStraightIngore(TaskInfo& task_info) {
  double valid_length =
      std::max(speed_vehicle_merging_decider_config_ptr_
                       ->ego_lane_obs_ignore_by_straight_valid_range +
                   0.0,
               speed_vehicle_merging_decider_config_ptr_
                       ->ego_lane_obs_ignore_by_straight_valid_time_length *
                   adc_current_v_);
  const auto& path_points = task_info.current_frame()
                                ->outside_planner_data()
                                .path_data->path()
                                .path_points();
  std::pair<double, bool> straight_valid_info{valid_length, 0};
  if (!JudgeStraight(path_points, valid_length, straight_valid_info)) {
    return false;
  }

  return true;
}

void SpeedVehicleMergeDecider::GenerateSafeCheckPolygon(TaskInfo& task_info) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  auto& merging_g = traffic_conflict_zone_context.merging_geoinfo[0];
  bool ego_is_in_merging = false;
  for (auto iter = merging_g->begin(); iter != merging_g->end(); ++iter) {
    if (iter->ego_rd < 0.0) {
      ego_is_in_merging = true;
      break;
    }
  };

  if (ego_is_in_merging) {
    is_boundary_ = true;
    back_boudnary_ = Boundary(adc_current_s_ - kBackAttentionDistance,
                              adc_current_s_ - adc_back_edge_to_center_,
                              adc_start_l_ - kBuffer, adc_end_l_ + kBuffer);
    LOG_INFO("Generate back boundary ");
    VisSafeBoundaryArea(task_info.reference_line(), back_boudnary_);
  } else {
    is_boundary_ = false;
    const auto& ref_line = task_info.reference_line();
    const auto& ref_pts = ref_line->ref_points();
    ReferencePoint start_p{}, end_p{};
    int ind = ref_pts.size() - 1;
    while (ind > 0) {
      if (ref_pts[ind].s() < adc_current_s_) break;
      if (ref_pts[ind].s() > adc_current_s_ + kBackAttentionDistance) {
        ind--;
        continue;
      }
      if (std::abs(ref_pts[ind].kappa()) < 0.002) {
        end_p = ref_pts[ind];
        ind--;
      }
      if (std::abs(ref_pts[ind].kappa() - end_p.kappa()) < 0.0005) {
        start_p = ref_pts[ind];
        break;
      }
      ind--;
    }

    if (std::abs(start_p.x() - start_p.y()) < 1e-6 &&
        std::abs(end_p.x() == end_p.y()) < 1e-6 &&
        std::abs(end_p.x() - 0.0) < 1e-6 && std::abs(end_p.y() - 0.0) < 1e-6) {
      Vec2d adc_start_xy = task_info.adc_point();
      back_boundary_polygon_ =
          Polygon2d(Box2d(adc_start_xy, adc_heading_, kBackAttentionDistance,
                          adc_width_ + kBuffer));
    } else {
      double tan_theta = (end_p.y() - start_p.y()) / (end_p.x() - start_p.x());
      LOG_INFO("end_p_y:{},end_p_x:{},start_y:{},start_x:{}", end_p.y(),
               end_p.x(), start_p.x(), start_p.y());
      merge_heading_ = std::atan(-tan_theta);
      Vec2d adc_start_xy = task_info.adc_point();
      back_boundary_polygon_ =
          Polygon2d(Box2d(adc_start_xy, merge_heading_, 30, 3));
    }
    LOG_INFO("Generate back polygon ");
    VisSafeBoundaryArea(back_boundary_polygon_);
  }
}

void SpeedVehicleMergeDecider::GetSpeedLimitByAttentionObs(
    TaskInfo& task_info) {
  double deceleration{0.0};
  GetDecelerationByObs(task_info, deceleration);
  // 2. get limited speed according to max deceleration.
  if (deceleration > kApproximateEqualZero * 0.1) {
    // limit max deceleration,in case length of obs changes fast.
    deceleration = std::min(
        deceleration,
        static_cast<double>(
            speed_vehicle_merging_decider_config_ptr_->limit_max_deceleration));
    update_speed_limit_ = true;
    speed_limit_ = std::min(
        speed_limit_,
        last_limited_speed_ -
            deceleration *
                speed_vehicle_merging_decider_config_ptr_->all_delay_time);
    LOG_INFO(
        "final val: deceleration, "
        "last_limited_speed_, speed_limit_:{:.2f}, {:.2f}, {:.2f} .",
        deceleration, last_limited_speed_, speed_limit_);
    speed_limit_ = std::max(speed_limit_, 0.0);
    // if (adc_straight_) {
    //   speed_limit_ = std::max(2.0, speed_limit_);
    // }
    last_limited_speed_ = speed_limit_;
  }
}

void SpeedVehicleMergeDecider::GetDecelerationByObs(TaskInfo& task_info,
                                                    double& deceleration) {
  for (auto iter = obs_on_merging_lane_.begin();
       iter != obs_on_merging_lane_.end(); iter++) {
    if (!(iter->second.has_obs_decision)) {
      LOG_WARN("obs [{}] not has collision decision, not limit speed.",
               iter->first);
      continue;
    }

    const auto& attention_obs_info = iter->second;
    if ((attention_obs_info.adc_first_collide_corner_point ==
         AdcCollideCornerPoint::LEFT_REAR) ||
        (attention_obs_info.adc_first_collide_corner_point ==
         AdcCollideCornerPoint::RIGHT_REAR)) {
      LOG_INFO(
          "first collide point of obs [{}] is rear of adc, not limit speed.",
          iter->first);
      continue;
    }

    auto obs_ptr = iter->second.obs_ptr;
    auto lower_pts_front_s = attention_obs_info.first_collide_s;
    auto lower_pts_front_t = attention_obs_info.first_collide_t;
    double adc_just_overtake_obs_dis = lower_pts_front_s + adc_length_;
    double adc_safe_overtake_t =
        (adc_just_overtake_obs_dis +
         speed_vehicle_merging_decider_config_ptr_->merging_safe_dis) /
        adc_current_v_;
    LOG_INFO(
        "obs [{}] lower_pts_front_s {:.3f}, lower_pts_front_t {:.3f}, "
        "adc_safe_overtake_t {:.3f} ",
        iter->first, lower_pts_front_s, lower_pts_front_t, adc_safe_overtake_t);
    if (adc_safe_overtake_t <= lower_pts_front_t) {
      auto obs_lower_pt_v = attention_obs_info.first_collide_obs_project_v;
      if (obs_lower_pt_v <= adc_current_v_) {
        LOG_INFO("adc is faster and will overtake obs [{}], not limit speed.",
                 iter->first);
        continue;
      }

      double adc_obs_ttc_at_lower_pts_front =
          (lower_pts_front_t * adc_current_v_ - adc_just_overtake_obs_dis) /
          (obs_lower_pt_v - adc_current_v_);
      if (adc_obs_ttc_at_lower_pts_front >
          speed_vehicle_merging_decider_config_ptr_->merging_safe_ttc) {
        LOG_INFO(
            "adc will overtake obs [{}] and keep safe ttc {:.3f}, not limit "
            "speed.",
            iter->first, adc_obs_ttc_at_lower_pts_front);
        continue;
      }
    }

    // adc stop before lower_pt_front to yield obs.(v^2=2as)
    double adc_left_dis =
        lower_pts_front_s -
        speed_vehicle_merging_decider_config_ptr_->merging_safe_dis;
    LOG_INFO("adc_left_dis {:.3f}, adc_current_v_ {:.3f}", adc_left_dis,
             adc_current_v_);
    if (adc_left_dis <= 0.0) {
      deceleration =
          std::max(deceleration,
                   static_cast<double>(speed_vehicle_merging_decider_config_ptr_
                                           ->limit_max_deceleration));
    } else if (adc_current_v_ < kApproximateEqualZero) {
      deceleration = std::max(deceleration, kApproximateEqualZero);
    } else {
      deceleration = std::max(deceleration,
                              std::pow(adc_current_v_, 2) / 2.0 / adc_left_dis);
    }
  }
}

bool SpeedVehicleMergeDecider::CollisionCheckWithObstaclePolyline(
    const ReferenceLinePtr reference_line, const InsidePlannerData& inside_data,
    const Obstacle& obstacle, OutsidePlannerData* const outside_data) {
  auto pred_traj = obstacle.uniform_trajectory();
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
  const auto& path_points = outside_data->path_data->path().path_points();
  int path_first_collide_i = path_points.size() + 2;
  Vec2d path_first_collide_pt{};
  GetFirstCollideInfo(outside_data, obs_prediction_segment, obs_side,
                      adc_first_collide_corner_point, path_first_collide_i,
                      path_first_collide_pt);

  // 3.lower:s,t,adc_first_collide_corner_point.
  GetLaneMergingInfo(path_points, obstacle.id(), adc_first_collide_corner_point,
                     path_first_collide_i, path_first_collide_pt, obs_base_pt);

  return true;
}

void SpeedVehicleMergeDecider::GetLaneMergingInfo(
    const std::vector<PathPoint>& path_points, const int obs_id,
    const AdcCollideCornerPoint& adc_first_collide_corner_point,
    const int path_first_collide_i, const Vec2d& path_first_collide_pt,
    const Vec2d& obs_base_pt) {
  if (path_first_collide_i >= path_points.size() || path_points.empty()) {
    return;
  }

  auto& attention_obs_info = obs_on_merging_lane_[obs_id];
  auto obs_ptr = attention_obs_info.obs_ptr;
  attention_obs_info.has_obs_decision = true;
  attention_obs_info.adc_first_collide_corner_point =
      adc_first_collide_corner_point;
  attention_obs_info.first_collide_t =
      std::sqrt(std::pow(path_first_collide_pt.x() - obs_base_pt.x(), 2) +
                std::pow(path_first_collide_pt.y() - obs_base_pt.y(), 2)) /
      obs_ptr->speed();
  attention_obs_info.first_collide_s = path_points[path_first_collide_i].s();
  attention_obs_info.first_collide_obs_project_v =
      obs_ptr->speed() * std::cos(std::abs(normalize_angle(
                             obs_ptr->velocity_heading() -
                             path_points[path_first_collide_i].theta())));
  LOG_INFO(
      "obs [{}] polyline first collide t,s: {:.3f}, {:.3f}; "
      "first_collide_obs_project_v: {:.3f}",
      obs_id, attention_obs_info.first_collide_t,
      attention_obs_info.first_collide_s,
      attention_obs_info.first_collide_obs_project_v);
}

void SpeedVehicleMergeDecider::GetFirstCollideInfo(
    const OutsidePlannerData* const outside_data,
    const Segment2d& obs_prediction_segment, const OBS_SIDE& obs_side,
    AdcCollideCornerPoint& adc_first_collide_corner_point,
    int& path_first_collide_i, Vec2d& path_first_collide_pt) {
  const auto& adc_corner_pt_coordinate =
      outside_data->speed_obstacle_context.adc_corner_pt_coordinate;
  Segment2d current_adc_back_edge{adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_REAR)],
                                  adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::RIGHT_REAR)]};
  if (obs_prediction_segment.get_intersect(current_adc_back_edge,
                                           &path_first_collide_pt)) {
    path_first_collide_i = 0;
    if (OBS_SIDE::RIGHT == obs_side) {
      adc_first_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
    } else {
      adc_first_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
    }
    LOG_INFO("obs collide adc at back.");
    return;
  }

  bool find_first_collide_pt{false};
  const auto& path_points = outside_data->path_data->path().path_points();
  for (std::size_t i = 0; i < path_points.size(); ++i) {
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
    if (obs_prediction_segment.get_intersect(path_adc_back_edge,
                                             &path_first_collide_pt)) {
      find_first_collide_pt = true;
    } else if (OBS_SIDE::RIGHT == obs_side) {
      Segment2d adc_side_edge = Segment2d{
          adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_REAR)],
          adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_FRONT)]};
      if (obs_prediction_segment.get_intersect(adc_side_edge,
                                               &path_first_collide_pt)) {
        find_first_collide_pt = true;
      }
    } else {
      Segment2d adc_side_edge = Segment2d{
          adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)],
          adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_FRONT)]};
      if (obs_prediction_segment.get_intersect(adc_side_edge,
                                               &path_first_collide_pt)) {
        find_first_collide_pt = true;
      }
    }

    if (find_first_collide_pt) {
      path_first_collide_i = i;
      break;
    }
  }
}

void SpeedVehicleMergeDecider::GetObsPredictionSegment(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    OBS_SIDE& obs_side, Segment2d& obs_prediction_segment, Vec2d& obs_base_pt) {
  double obs_nearest_local_y{std::numeric_limits<double>::infinity()};
  std::vector<Vec2d> obs_bounding_box_corners;
  obstacle.bounding_box().get_all_corners(&obs_bounding_box_corners);
  for (const auto& pt : obs_bounding_box_corners) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                  pt.x(), pt.y(), obstacle.velocity_heading(), obs_local_x,
                  obs_local_y, obs_local_heading);
    if (std::abs(obs_local_y) < obs_nearest_local_y) {
      obs_nearest_local_y = std::abs(obs_local_y);
      obs_base_pt = pt;
      obs_side = obs_local_y > 0.0 ? OBS_SIDE::LEFT : OBS_SIDE::RIGHT;
    }
  }

  Vec2d segment_end_pt{
      obs_base_pt.x() +
          speed_vehicle_merging_decider_config_ptr_->obs_prediction_dis *
              std::cos(obstacle.velocity_heading()),
      obs_base_pt.y() +
          speed_vehicle_merging_decider_config_ptr_->obs_prediction_dis *
              std::sin(obstacle.velocity_heading())};
  obs_prediction_segment = std::move(Segment2d(obs_base_pt, segment_end_pt));
  VisObsPredictionSegment(obs_base_pt, segment_end_pt);
}

bool SpeedVehicleMergeDecider::IgnoreObsOnCurrentLane(
    TaskInfo& task_info, const InsidePlannerData& inside_data,
    const TrafficConflictZoneContext& traffic_conflict_zone_context,
    const Obstacle* const obstacle) {
  if (IgnoreObsOnVehicleFrame(inside_data, obstacle)) {
    return true;
  }
  double heading_diff = std::abs(
      normalize_angle(inside_data.vel_heading - obstacle->velocity_heading()));
  if ((EgoLaneObsStraightIngore(task_info) ||
       traffic_conflict_zone_context.current_lane.is_stright) &&
      IsSmallObs(obstacle) && heading_diff < 0.262) {
    return true;
  }

  return false;
}

bool SpeedVehicleMergeDecider::IgnoreObsOnVehicleFrame(
    const InsidePlannerData& inside_data, const Obstacle* const obstacle) {
  std::vector<Vec2d> obs_bounding_box_corners;
  obstacle->bounding_box().get_all_corners(&obs_bounding_box_corners);
  double obs_nearest_local_y{std::numeric_limits<double>::infinity()};
  double obs_max_local_x{std::numeric_limits<double>::lowest()};
  bool obs_has_negative_y{false};
  bool obs_has_positive_y{false};
  for (const auto& pt : obs_bounding_box_corners) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                  pt.x(), pt.y(), obstacle->velocity_heading(), obs_local_x,
                  obs_local_y, obs_local_heading);
    obs_max_local_x = std::max(obs_max_local_x, obs_local_x);
    obs_nearest_local_y = std::min(obs_nearest_local_y, std::abs(obs_local_y));
    obs_has_negative_y = obs_has_negative_y || (obs_local_y < -1e-4);
    obs_has_positive_y = obs_has_positive_y || (obs_local_y > 1e-4);
  }

  if ((obs_max_local_x < -adc_back_edge_to_center_) &&
      ((obs_nearest_local_y < adc_width_ / 2.0) ||
       (obs_has_negative_y && obs_has_positive_y))) {
    return true;
  }

  return false;
}

void SpeedVehicleMergeDecider::GetAdcCornerPointCoordinate(
    const Vec2d& adc_coordinate, const double adc_heading,
    std::vector<Vec2d>& adc_corner_pt_coordinate) {
  adc_corner_pt_coordinate.clear();
  adc_corner_pt_coordinate.resize(
      static_cast<int>(AdcCollideCornerPoint::NONE));
  double x_g = 0.0;
  double y_g = 0.0;
  double theta_g = 0.0;
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      -VehicleParam::Instance()->back_edge_to_center(),
      VehicleParam::Instance()->left_edge_to_center() +
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)] =
      std::move(Vec2d(x_g, y_g));
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      -VehicleParam::Instance()->back_edge_to_center(),
      -VehicleParam::Instance()->right_edge_to_center() -
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_REAR)] = std::move(Vec2d(x_g, y_g));
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      VehicleParam::Instance()->front_edge_to_center(),
      -VehicleParam::Instance()->right_edge_to_center() -
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_FRONT)] = std::move(Vec2d(x_g, y_g));
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      VehicleParam::Instance()->front_edge_to_center(),
      VehicleParam::Instance()->left_edge_to_center() +
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::LEFT_FRONT)] = std::move(Vec2d(x_g, y_g));
}

}  // namespace planning
}  // namespace neodrive

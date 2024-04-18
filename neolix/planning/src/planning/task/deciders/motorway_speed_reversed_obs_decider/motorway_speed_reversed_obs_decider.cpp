#include "motorway_speed_reversed_obs_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

namespace {
void VisReverseRepulsiveFieldRoadBoundary(
    const ReferenceLinePtr ref_line,
    const std::vector<Boundary>& single_road_boundaries) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(
      "ReverseRepulsiveFieldRoadBoundary");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt{};
  // display polygon of check areas
  for (const auto& bdry : single_road_boundaries) {
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
}

void VisSingleRoadBoundary(
    const ReferenceLinePtr ref_line,
    const std::vector<Boundary>& single_road_boundaries) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("SingleRoadArea");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt{};
  // display polygon of check areas
  for (const auto& bdry : single_road_boundaries) {
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
}

void VisObstacleBoundary(const ReferenceLinePtr ref_line,
                         const Boundary& obs_boundary) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("ReversedObsBoundary");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt{};
  // display polygon of check areas
  auto polygon = event->mutable_polygon()->Add();
  ref_line->GetPointInCartesianFrame(
      {obs_boundary.start_s(), obs_boundary.end_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame(
      {obs_boundary.start_s(), obs_boundary.start_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame(
      {obs_boundary.end_s(), obs_boundary.start_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame(
      {obs_boundary.end_s(), obs_boundary.end_l()}, &pt);
  set_pt(polygon->add_point(), pt);
}

bool IsSmallObs(const Obstacle& obs) {
  bool is_little_size = std::pow(std::max(obs.length(), obs.width()), 2.0) <
                        config::PlanningConfig::Instance()
                            ->plan_config()
                            .common.little_obs_area_threshold;
  return ((obs.type() == Obstacle::ObstacleType::BICYCLE) ||
          (obs.type() == Obstacle::ObstacleType::PEDESTRIAN) || is_little_size);
}

bool IsInFrontRange(double point_to_judge, double base, double range) {
  return (point_to_judge < base) && (point_to_judge > base - range);
}

}  // namespace

MotorwaySpeedReversedObsDecider::MotorwaySpeedReversedObsDecider() {
  name_ = "MotorwaySpeedReversedObsDecider";
}

MotorwaySpeedReversedObsDecider::~MotorwaySpeedReversedObsDecider() {}

ErrorCode MotorwaySpeedReversedObsDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  // 1.reset data; 2.updata data; 3.InitSingleRoadBoundaries.
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // 2.Only deal obstacles satisfied those condition, and get limited speed
  // finally. (1) small obs; (2)heading diff bigger than 3*pi/4;
  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedReversedObsDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::REVERSED_OBS);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "REVERSE_OBS {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    double deceleration = std::min(
        2.0, std::max((adc_current_v_ - limited_speed_) / all_delay_time_,
                      kApproximateEqualZero));
    double target_time = std::min(adc_current_v_ / deceleration, 1000.0);
    dynamic_obs_caution_infos_ptr_->emplace_back("Reverse", limited_speed_,
                                                 target_time, deceleration);
    LOG_INFO(
        "caution infos: adc_v {:.3f}, target_v {:.3f}, target_time {:.3f}, "
        "deceleration {:.3f}.",
        adc_current_v_, limited_speed_, target_time, deceleration);
  }
}

void MotorwaySpeedReversedObsDecider::Reset() {
  single_road_boundaries_.clear();
  reverse_use_repulsive_obs_.clear();
  acc_cmd_vecor_.clear();
  ego_speed_vector_.clear();
  relative_distance_vector_.clear();
  ego_position_vector_.clear();
  check_road_boundaries_.clear();
};

// 1.reset data; 2.updata data;
// 3.InitSingleRoadBoundaries(expect single-road in crossroads.)
bool MotorwaySpeedReversedObsDecider::Init(TaskInfo& task_info) {
  Reset();
  if (!update_limited_speed_) {
    last_limited_speed_ =
        task_info.current_frame()->inside_planner_data().vel_v;
  }
  update_limited_speed_ = false;
  single_road_boundaries_.clear();

  decltype(reversed_obs_data_.obs_last_l_dis) tmp{};
  for (auto& [id, data] : reversed_obs_data_.obs_last_l_dis) {
    if (data.updated) {
      tmp[id].l_dis = data.l_dis;
      tmp[id].updated = false;
    }
  }
  std::swap(reversed_obs_data_.obs_last_l_dis, tmp);

  dynamic_obs_caution_infos_ptr_ =
      &task_info.current_frame()
           ->mutable_outside_planner_data()
           ->motorway_speed_obstacle_context.dynamic_obs_caution_infos;
  if (!dynamic_obs_caution_infos_ptr_) {
    return false;
  }

  single_road_width_ = std::numeric_limits<double>::infinity();
  limited_speed_ = std::numeric_limits<double>::infinity();
  adc_width_ = VehicleParam::Instance()->width();
  adc_length_ = VehicleParam::Instance()->length();
  adc_back_edge_to_center_ = VehicleParam::Instance()->back_edge_to_center();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ = adc_current_s_ + adc_length_ - adc_back_edge_to_center_;
  adc_start_l_ = task_info.curr_sl().l() - adc_width_ / 2;
  adc_end_l_ = task_info.curr_sl().l() + adc_width_ / 2;
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;

  const auto& speed_reversed_obs_config =
      config::PlanningConfig::Instance()->plan_config().speed_reversed_obs;
  const auto& speed_reversed_repulsive_field_config =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .speed_reversed_repulsive_field_caution;
  auto adc_current_v = task_info.adc_point().velocity();
  double emergency_deal_min_s_len =
      speed_reversed_obs_config.emergency_deal_min_s_len;
  double slow_down_deal_min_s_len =
      speed_reversed_obs_config.slow_down_deal_min_s_len;
  emergency_s_range_ =
      std::max(adc_current_v * speed_reversed_obs_config.emergency_deal_time,
               emergency_deal_min_s_len);
  slow_down_s_range_ =
      std::max(adc_current_v * speed_reversed_obs_config.slow_down_deal_time,
               slow_down_deal_min_s_len);
  all_delay_time_ = speed_reversed_obs_config.all_delay_time;

  if (!InitSingleRoadBoundaries(task_info)) {
    LOG_INFO("InitSingleRoadBoundaries failed.");
    return false;
  }
  if (!InitRoadBoundaries(task_info)) {
    LOG_INFO("Init Reverse Repulsive method RoadBoundaries check failed.");
    return false;
  }
  return true;
}

bool MotorwaySpeedReversedObsDecider::InitRoadBoundaries(TaskInfo& task_info) {
  const auto& speed_reversed_repulsive_field_config =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .speed_reversed_repulsive_field_caution;

  // read reference
  const auto& reference_line = task_info.reference_line();
  if (reference_line == nullptr) {
    LOG_ERROR("refer line is nullptr");
    return false;
  }

  int current_s_index{0};
  for (const auto& point : reference_line->ref_points()) {
    if (point.s() < adc_current_s_) {
      current_s_index++;
      continue;
    }
  }
  int step = 3;
  for (int ref_index = current_s_index;
       ref_index < reference_line->ref_points().size() - step - 1;
       ref_index += step) {
    if (reference_line->ref_points()[ref_index].s() < adc_current_s_) {
      continue;
    }
    if ((reference_line->ref_points()[ref_index].s() - adc_current_s_) >
        speed_reversed_repulsive_field_config.filed_check_area_length) {
      break;
    }
    double right_caution_width = std::min(
        reference_line->ref_points()[ref_index].right_road_bound(),
        (reference_line->ref_points()[ref_index].right_lane_bound() +
         speed_reversed_repulsive_field_config.lane_bound_right_expend_buffer));
    double left_caution_width = std::min(
        reference_line->ref_points()[ref_index].left_road_bound(),
        (reference_line->ref_points()[ref_index].left_lane_bound() +
         speed_reversed_repulsive_field_config.lane_bound_left_expend_buffer));
    double piece_check_start_s = reference_line->ref_points()[ref_index].s();
    double piece_check_end_s =
        reference_line->ref_points()[ref_index + step].s();
    check_road_boundaries_.emplace_back(
        Boundary(piece_check_start_s, piece_check_end_s, -left_caution_width,
                 right_caution_width));
  }
  LOG_INFO("Init Reverse Repulsive method RoadBoundaries check success.");
  VisReverseRepulsiveFieldRoadBoundary(reference_line, check_road_boundaries_);
  return true;
}

bool MotorwaySpeedReversedObsDecider::InitSingleRoadBoundaries(
    TaskInfo& task_info) {
  const auto& speed_reversed_obs_config =
      config::PlanningConfig::Instance()->plan_config().speed_reversed_obs;
  bool last_point_is_in_single_road{false};
  double single_road_start_s{0.0};
  double single_road_end_s{0.0};
  const auto& reference_line = task_info.reference_line();
  if (reference_line == nullptr) {
    LOG_ERROR("refer line is nullptr");
    return false;
  }
  const auto& junction_list = reference_line->junctions();
  for (const auto& point : reference_line->ref_points()) {
    if (point.s() < adc_current_s_) {
      continue;
    }

    double road_width = point.right_road_bound() + point.left_road_bound();
    double lane_width = point.right_lane_bound() + point.left_lane_bound();
    bool is_valid_junction{true};
    for (const auto& [junction_ptr, overlap] : junction_list) {
      if (junction_ptr == nullptr) continue;
      if (junction_ptr->Type() !=
              static_cast<uint32_t>(JunctionType::IN_ROAD) &&
          overlap.start_s < point.s() && overlap.end_s > point.s()) {
        is_valid_junction = false;
        break;
      }
    }
    if ((Double::compare(lane_width, road_width, kSingleRoadEpsilon) == 0) &&
        (!point.is_in_junction() ||
         (point.is_in_junction() && is_valid_junction))) {
      if (!last_point_is_in_single_road) {
        last_point_is_in_single_road = true;
        single_road_start_s = point.s();
      }
      single_road_width_ = std::min(single_road_width_, lane_width);
    } else if (last_point_is_in_single_road) {
      last_point_is_in_single_road = false;
      single_road_end_s = point.s();
      single_road_boundaries_.emplace_back(
          Boundary(single_road_start_s, single_road_end_s,
                   -kHalf * single_road_width_, kHalf * single_road_width_));
    }
  }
  if (last_point_is_in_single_road) {
    single_road_end_s = reference_line->ref_points().back().s();
    single_road_boundaries_.emplace_back(
        Boundary(single_road_start_s, single_road_end_s,
                 -kHalf * single_road_width_, kHalf * single_road_width_));
  }
  single_road_pre_check_range_ = slow_down_s_range_;
  VisSingleRoadBoundary(reference_line, single_road_boundaries_);
  return true;
}

/**
 * @brief Only deal obstacles satisfied those condition, and get limited speed
 * finally.
 *  (1) small obs; (2)heading diff bigger than 3*pi/4;
 *
 * @param task_info
 * @return true
 * @return false
 */
bool MotorwaySpeedReversedObsDecider::Process(TaskInfo& task_info) {
  const auto& adc_front_road_boundaries =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context.adc_front_road_boundaries;
  auto& ignore_dynamic_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.ignore_dynamic_obs_id;

  // consider single road reverse obs, whatever obs has collision check or not.

  GetSpeedLimitByRepulsiveFieldForReversedObs(task_info);
  CalcSpeedLimitbyRepulsiveField(task_info);
  // if (reverse_use_repulsive_obs_.size() > 0) {
  //   SetupReverseSafeIntervalSpeedLimit(task_info);
  // }
  GetSpeedLimitByReversedObs(task_info);

  SpeedReversedObsContextInfo(adc_front_road_boundaries, ignore_dynamic_obs_id);
  return true;
}

// check obs with no decision ,use repulsive field method
void MotorwaySpeedReversedObsDecider::
    GetSpeedLimitByRepulsiveFieldForReversedObs(TaskInfo& task_info) {
  // obs has no decision
  LOG_INFO("Start Deal with obs which has no decision.");
  const auto& dynamic_obstacle = task_info.decision_data()->dynamic_obstacle();
  // read path
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();

  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& speed_reversed_repulsive_field_config =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .speed_reversed_repulsive_field_caution;

  // obs with decision
  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;
  std::unordered_set<std::size_t> obs_dealt_set{};
  for (const auto& obs_decision : dynamic_obstacles_decision) {
    obs_dealt_set.insert(obs_decision.obstacle.id());
  }
  // 1. only consider obs which reverse obs with not decision,
  for (const auto& obstacle : dynamic_obstacle) {
    if (obstacle->is_virtual() || obstacle->is_static()) continue;
    //  if (!IsSmallObs(*obstacle)) continue;
    if (obs_dealt_set.find(obstacle->id()) != obs_dealt_set.end()) continue;
    if (obstacle->PolygonBoundary().end_s() < adc_front_edge_s_) continue;
    PathPoint closest_pt{};
    // 计算heading
    double path_heading_near_obs =
        path.query_closest_point(obstacle->center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    // need consider pos is left or right
    double heading_diff =
        normalize_angle(obstacle->velocity_heading() - path_heading_near_obs);
    // only consider heading diff > 3/4 pi
    if (std::abs(heading_diff) <
        speed_reversed_repulsive_field_config.speed_reverse_heading_diff_min) {
      continue;
    }
    bool has_overlap{false};
    for (auto& road_boundary : single_road_boundaries_) {
      if (road_boundary.has_overlap(obstacle->PolygonBoundary())) {
        has_overlap = true;
        break;
      }
    }
    if (has_overlap == false) continue;
    LOG_INFO("Reverse obs id : [{}] is in repulsive method considering.",
             obstacle->id());
    // save data
    VisObstacleBoundary(task_info.reference_line(),
                        obstacle->PolygonBoundary());
    ReverseCautionObs reverse_obs;
    reverse_obs.Reset();
    //  cal lat dis between obs and adc
    double lat_dis_obs2adc = std::max(obstacle->max_l(), adc_end_l_) -
                             std::min(obstacle->min_l(), adc_start_l_) -
                             adc_width_ - obstacle->width();
    // cal lat dis between obs and closest_pt
    double lat_dis_obs2path{};
    reverse_obs.lat_distance_obs2adc =
        speed_planner_common::GetObsToPathLatDis(
            task_info.current_frame()
                ->outside_planner_data()
                .motorway_speed_obstacle_context.adc_sl_boundaries,
            *obstacle, lat_dis_obs2path)
            ? lat_dis_obs2path
            : lat_dis_obs2adc;
    // cal lon dis between obs and adc
    reverse_obs.lon_dis_obs2adc = obstacle->min_s() - adc_front_edge_s_;
    reverse_obs.obs_absule_s = obstacle->min_s();
    reverse_obs.reverse_obs_id = obstacle->id();
    reverse_obs.obs_speed_parallel_car =
        obstacle->speed() * std::cos(heading_diff);
    reverse_obs.obs_absule_speed = obstacle->speed();
    Vec2d close_point_xy{closest_pt.x(), closest_pt.y()};
    SLPoint close_point_sl{};

    if (!task_info.reference_line()->GetPointInFrenetFrame(close_point_xy,
                                                           &close_point_sl)) {
      return;
    }
    // obs_bias_pos true: left  false: right
    reverse_obs.obs_bias_pos =
        obstacle->center_sl().l() > close_point_sl.l() ? true : false;
    reverse_obs.heading_diff_obs2adc = heading_diff;
    LOG_INFO(
        "use reversed repulsive field dynamic obstacle id : [{}] ,lateral "
        "distance : {:.3f} , lon dis : {:.3f} ,heading : {:.3f}, bias_pos : "
        "{}, obs speed : {:.3f}",
        reverse_obs.reverse_obs_id, reverse_obs.lat_distance_obs2adc,
        reverse_obs.lon_dis_obs2adc, reverse_obs.heading_diff_obs2adc,
        reverse_obs.obs_bias_pos, reverse_obs.obs_speed_parallel_car);
    reverse_use_repulsive_obs_.push_back(reverse_obs);
  }
}

void MotorwaySpeedReversedObsDecider::CalcSpeedLimitbyRepulsiveField(
    TaskInfo& task_info) {
  if (reverse_use_repulsive_obs_.size() == 0) {
    return;
  }
  const auto& speed_reversed_repulsive_field_config =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .speed_reversed_repulsive_field_caution;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  adc_width_ = VehicleParam::Instance()->width();
  adc_length_ = VehicleParam::Instance()->length();
  adc_back_edge_to_center_ = VehicleParam::Instance()->back_edge_to_center();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ = adc_current_s_ + adc_length_ - adc_back_edge_to_center_;
  adc_start_l_ = task_info.curr_sl().l() - adc_width_ / 2;
  adc_end_l_ = task_info.curr_sl().l() + adc_width_ / 2;
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  for (auto& reverse_obs : reverse_use_repulsive_obs_) {
    std::vector<double> ego_position_predict{};
    std::vector<double> ego_speed_predict{};
    std::vector<double> reverse_obs_distance_predict{};
    std::vector<double> relative_lon_distancec_predict{};
    std::vector<double> relative_lat_distancec_predict{};
    std::vector<double> acc_cmd_deduction{};
    ego_position_predict.clear();
    ego_speed_predict.clear();
    reverse_obs_distance_predict.clear();
    relative_lat_distancec_predict.clear();
    acc_cmd_deduction.clear();
    relative_lon_distancec_predict.clear();
    ego_position_predict.push_back(adc_front_edge_s_);
    ego_speed_predict.push_back(inside_data.vel_v);
    relative_lon_distancec_predict.push_back(reverse_obs.lon_dis_obs2adc);
    // relative_lat_distancec_predict.push_back(reverse_obs.lat_distance_obs2adc);
    acc_cmd_deduction.push_back(inside_data.vel_a);

    for (size_t i = 0;
         i <=
         speed_reversed_repulsive_field_config.reverse_trajectory_max_point_num;
         i++) {
      reverse_obs_distance_predict.push_back(
          reverse_obs.obs_absule_s +
          speed_reversed_repulsive_field_config.planning_step_time_ * i *
              reverse_obs.obs_speed_parallel_car);
      // obs_bias_pos true: left  false: right
      if (reverse_obs.obs_bias_pos) {
        relative_lat_distancec_predict.push_back(
            reverse_obs.lat_distance_obs2adc +
            speed_reversed_repulsive_field_config.planning_step_time_ * i *
                std::sin(reverse_obs.heading_diff_obs2adc) *
                reverse_obs.obs_absule_speed);
      } else {
        relative_lat_distancec_predict.push_back(
            reverse_obs.lat_distance_obs2adc -
            speed_reversed_repulsive_field_config.planning_step_time_ * i *
                std::sin(reverse_obs.heading_diff_obs2adc) *
                reverse_obs.obs_absule_speed);
      }
    }
    for (size_t i = 0;
         i <=
         speed_reversed_repulsive_field_config.reverse_trajectory_max_point_num;
         i++) {
      if (relative_lon_distancec_predict.back() > 0) {
        double a_repulsion =
            speed_reversed_repulsive_field_config.repulsive_para *
            reverse_obs.obs_speed_parallel_car *
            (1 / (relative_lat_distancec_predict[i] +
                  std::abs(relative_lon_distancec_predict.back()))) /
            (std::abs(relative_lon_distancec_predict.back()) + 0.01);
        if (a_repulsion < -3.5) {
          a_repulsion = -3.5;
        }
        double a_cmd = acc_cmd_deduction.back() +
                       speed_reversed_repulsive_field_config.deduction_para *
                           (a_repulsion - acc_cmd_deduction.back());
        LOG_INFO("see a_repulsion : {} , a_cmd : {} , lon_dist : {} ",
                 a_repulsion, a_cmd, relative_lon_distancec_predict.back());
        acc_cmd_deduction.push_back(a_cmd);
        if (ego_speed_predict.back() +
                speed_reversed_repulsive_field_config.planning_step_time_ *
                    acc_cmd_deduction.back() <=
            0) {
          ego_speed_predict.push_back(0);
        } else {
          ego_speed_predict.push_back(
              ego_speed_predict.back() +
              speed_reversed_repulsive_field_config.planning_step_time_ *
                  acc_cmd_deduction.back());
        }
        ego_position_predict.push_back(
            ego_position_predict.back() +
            speed_reversed_repulsive_field_config.planning_step_time_ *
                ego_speed_predict.back());
        relative_lon_distancec_predict.push_back(
            reverse_obs_distance_predict[i] - ego_position_predict.back());
      }
      if (relative_lon_distancec_predict.back() <= 0) {
        break;
      }
    }
    acc_cmd_vecor_.push_back(acc_cmd_deduction);
    ego_speed_vector_.push_back(ego_speed_predict);
    ego_position_vector_.push_back(ego_position_predict);
    relative_distance_vector_.push_back(relative_lon_distancec_predict);
  }
}

/**
 * @brief 1. Calculata max deceleration by lateral distance between obs and
 * adc, obs and path. (Only consider obs which has collision check.)
 * 2. Get limited speed according to max deceleration.
 */
void MotorwaySpeedReversedObsDecider::GetSpeedLimitByReversedObs(
    TaskInfo& task_info) {
  const auto& speed_reversed_obs_config =
      config::PlanningConfig::Instance()->plan_config().speed_reversed_obs;
  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  auto& ignore_dynamic_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.ignore_dynamic_obs_id;
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  if (dynamic_obstacles_decision.empty()) {
    LOG_INFO("dynamic_obstacles_decision is empty.");
    return;
  }

  // 1. Calculata max deceleration by lateral distance between obs and adc,
  // obs and path. (Only consider obs which has collision check.)
  double deceleration{0.0};
  for (auto& obs_decision : dynamic_obstacles_decision) {
    auto& obstacle = obs_decision.obstacle;
    // reversed small obs
    if (!IsSmallObs(obstacle)) {
      continue;
    }

    if (obstacle.PolygonBoundary().end_s() < adc_front_edge_s_) {
      continue;
    }

    PathPoint closest_pt{};
    double path_heading_near_obs =
        path.query_closest_point(obstacle.center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff =
        normalize_angle(obstacle.velocity_heading() - path_heading_near_obs);
    LOG_INFO(
        "obs id[{}], heading {:.3f}, path_heading_near_obs {:.3f}, "
        "heading_diff {:.3f}",
        obstacle.id(), obstacle.velocity_heading(), path_heading_near_obs,
        heading_diff);
    if (std::abs(heading_diff) <
        speed_reversed_obs_config.reverse_heading_diff_threshold_rad) {
      continue;
    }

    VisObstacleBoundary(task_info.reference_line(), obstacle.PolygonBoundary());
    CheckBeforeComeInSingleRoad(obstacle, ignore_dynamic_obs_id);
    bool need_slow_down{false};
    double lat_dis_obs2adc = std::max(obstacle.max_l(), adc_end_l_) -
                             std::min(obstacle.min_l(), adc_start_l_) -
                             adc_width_ - obstacle.width();
    double lat_dis_obs2path{};
    double lat_dis = speed_planner_common::GetObsToPathLatDis(
                         task_info.current_frame()
                             ->outside_planner_data()
                             .motorway_speed_obstacle_context.adc_sl_boundaries,
                         obstacle, lat_dis_obs2path)
                         ? lat_dis_obs2path
                         : lat_dis_obs2adc;
    double lon_dis_to_emergency_range =
        obstacle.min_s() - adc_front_edge_s_ - emergency_s_range_;
    // ignore_dynamic_obs_id.insert(obstacle.id());

    if (lon_dis_to_emergency_range <= 0.0) {
      // get speed limit in emergency range
      double obs_speed_parallel_car =
          obstacle.speed() * std::abs(std::cos(heading_diff));
      update_limited_speed_ = true;
      limited_speed_ = std::min(
          limited_speed_,
          std::min(adc_current_v_,
                   obs_speed_parallel_car *
                       speed_reversed_obs_config.emergency_speed_limit_ratio));
      LOG_INFO(
          "reversed dynamic obstacle [{}] overlap with emergency area, "
          "obs_speed_parallel_car, limited_speed_:{:.2f}, {:.2f} .",
          obstacle.id(), obs_speed_parallel_car, limited_speed_);
    } else {
      // get max deceleration in slow down range
      LOG_INFO(
          "obs id, width, length; max_l, min_l, max_s, min_s; adc_width, "
          "adc_end_l_, adc_start_l: [{}], {:.2f}, {:.2f}; {:.2f}, {:.2f}, "
          "{:.2f}, {:.2f}; {:.2f}, {:.2f}, {:.2f} .",
          obstacle.id(), obstacle.width(), obstacle.length(), obstacle.max_l(),
          obstacle.min_l(), obstacle.max_s(), obstacle.min_s(), adc_width_,
          adc_end_l_, adc_start_l_);
      double speed_at_slowdown_to_emergeny = GetSpeedAtEmergencyPoint(
          heading_diff, obstacle, lon_dis_to_emergency_range, lat_dis);
      need_slow_down = adc_current_v_ > speed_at_slowdown_to_emergeny;
      // 2as=v0^2-v^2
      deceleration = std::max(
          deceleration, CalDecelerationByObs(lon_dis_to_emergency_range,
                                             speed_at_slowdown_to_emergeny));
    }

    if (need_slow_down && (lon_dis_to_emergency_range < 3.0)) {
      // if adc_current_v_ near 0.0 and obs is reversed worming, make speed
      // decision steady.
      update_limited_speed_ = true;
      limited_speed_ = std::min(limited_speed_,
                                std::min(last_limited_speed_, adc_current_v_));
    }

    // must put at end
    reversed_obs_data_.obs_last_l_dis[obstacle.id()].l_dis = lat_dis;
    reversed_obs_data_.obs_last_l_dis[obstacle.id()].updated = true;
  }

  // 2. get limited speed according to max deceleration.
  if (deceleration > kApproximateEqualZero) {
    update_limited_speed_ = true;
    limited_speed_ = std::min(
        limited_speed_, last_limited_speed_ - deceleration * all_delay_time_);
    LOG_INFO(
        "final val: deceleration, last_limited_speed_, "
        "limited_speed_:{:.2f}, {:.2f}, {:.2f} .",
        deceleration, last_limited_speed_, limited_speed_);
  }

  limited_speed_ = std::max(limited_speed_, 0.0);
  last_limited_speed_ = limited_speed_;
  LOG_INFO("final limited_speed_: {:.2f} .", limited_speed_);
}

// 2as=v0^2-v^2
double MotorwaySpeedReversedObsDecider::CalDecelerationByObs(
    const double lon_dis, const double speed_at_slowdown_to_emergeny) {
  const auto& speed_reversed_obs_config =
      config::PlanningConfig::Instance()->plan_config().speed_reversed_obs;
  // limited speed at emergency point is min of two methods :
  double deceleration = 0.0;
  if (speed_at_slowdown_to_emergeny <= 0.0) {
    deceleration = std::pow(adc_current_v_ - speed_at_slowdown_to_emergeny, 2) /
                   2.0 / lon_dis;
  } else if (adc_current_v_ > speed_at_slowdown_to_emergeny) {
    deceleration = (std::pow(adc_current_v_, 2) -
                    std::pow(speed_at_slowdown_to_emergeny, 2)) /
                   2.0 / lon_dis;
  }

  // limit max deceleration,in case length of obs changes fast.
  deceleration = std::min(
      deceleration,
      static_cast<double>(speed_reversed_obs_config.limit_max_deceleration));
  LOG_INFO("speed_at_slowdown_to_emergeny, deceleration:{:.2f}, {:.2f}.",
           speed_at_slowdown_to_emergeny, deceleration);
  return deceleration;
}

/**
 * @brief limited speed at emergency point is min of two methods :
 * (1) linear: speed = k_conf * l_dis + b_conf;
 * (2) ttc_lon > ttc_lat;
 *
 * @param l_dis
 * @return double
 */
double MotorwaySpeedReversedObsDecider::GetSpeedAtEmergencyPoint(
    const double heading_diff, const Obstacle& obs, const double lon_dis,
    const double lat_dis) {
  LOG_INFO(
      "heading_diff, obs_v, lon_dis, lat_dis:{:.2f}, {:.2f}, {:.2f}, {:.2f}.",
      heading_diff, obs.speed(), lon_dis, lat_dis);
  const auto& speed_reversed_obs_config =
      config::PlanningConfig::Instance()->plan_config().speed_reversed_obs;
  double obs_speed_parallel_car =
      obs.speed() * std::abs(std::cos(heading_diff));
  double obs_speed_vertical_car =
      obs.speed() * std::abs(std::sin(heading_diff));

  if (lat_dis < speed_reversed_obs_config.lateral_safe_dis) {
    LOG_INFO("lat dis little than lateral_safe_dis, set 0.0.");
    return 0.0;
  }

  double speed = speed_reversed_obs_config.linear_ratio_v_and_l_dis * lat_dis +
                 speed_reversed_obs_config.linear_offset_v_and_l_dis;

  if (reversed_obs_data_.obs_last_l_dis[obs.id()].l_dis - lat_dis >
      speed_reversed_obs_config.lateral_cutin_delta) {
    reversed_obs_data_.obs_last_l_dis[obs.id()].stop_cutin_count = 0;
    reversed_obs_data_.obs_last_l_dis[obs.id()].cutin_flag = true;
  } else {
    reversed_obs_data_.obs_last_l_dis[obs.id()].stop_cutin_count++;
    if (reversed_obs_data_.obs_last_l_dis[obs.id()].stop_cutin_count >
        speed_reversed_obs_config.stop_cutin_filter_num) {
      reversed_obs_data_.obs_last_l_dis[obs.id()].cutin_flag = false;
      reversed_obs_data_.obs_last_l_dis[obs.id()].stop_cutin_count =
          speed_reversed_obs_config.stop_cutin_filter_num;
    }
  }
  if (reversed_obs_data_.obs_last_l_dis[obs.id()].cutin_flag) {
    // consider cutting in and straight driving
    // s/v_s = l/v_l
    double v_relative = lon_dis * obs_speed_vertical_car / lat_dis;
    speed = std::min(speed, v_relative - obs_speed_parallel_car);
    LOG_INFO(
        "cutin or straight driving, last_l_dis, l_dis, "
        "v_relative,obs_speed_parallel_car, speed:{:.2f}, "
        "{:.2f}, {:.2f}, {:.2f}, {:.2f}.",
        reversed_obs_data_.obs_last_l_dis[obs.id()].l_dis, lat_dis, v_relative,
        obs_speed_parallel_car, speed);
  }
  return speed;
}

void MotorwaySpeedReversedObsDecider::CheckBeforeComeInSingleRoad(
    const Obstacle& obstacle, std::unordered_set<int>& ignore_dynamic_obs_id) {
  const auto& speed_reversed_obs_config =
      config::PlanningConfig::Instance()->plan_config().speed_reversed_obs;
  if (!speed_reversed_obs_config.is_check_before_come_in_single_road) {
    return;
  }

  ReversedObsContextInfo(obstacle);
  if (single_road_boundaries_.empty()) {
    return;
  }
  const auto& single_road_boundary = single_road_boundaries_.front();
  if (!single_road_boundary.has_overlap(obstacle.PolygonBoundary())) {
    LOG_INFO("obs and single road not overlap, ignore.");
    return;
  }
  if (!IsInFrontRange(adc_front_edge_s_, single_road_boundary.start_s(),
                      single_road_pre_check_range_)) {
    LOG_INFO("adc not in single road pre check area, ignore.");
    return;
  }
  if (adc_width_ + obstacle.width() < single_road_width_) {
    LOG_INFO("adc and obs can go through single road together, ignore.");
    return;
  }
  LOG_INFO("CheckBeforeComeInSingleRoad, adc slow down !");
  AdcSlowDown(
      std::min(single_road_boundary.start_s() - adc_front_edge_s_ -
                   speed_reversed_obs_config.single_road_stop_wait_buffer,
               obstacle.min_s() - adc_front_edge_s_ -
                   speed_reversed_obs_config.single_road_stop_wait_buffer));
  // ignore_dynamic_obs_id.insert(obstacle.id());
  return;
}

void MotorwaySpeedReversedObsDecider::AdcSlowDown(double distance_to_stop) {
  update_limited_speed_ = true;
  if (distance_to_stop < kStopWaitBufferEpsilon) {
    limited_speed_ = 0.0;
  } else {
    double deceleration = std::pow(adc_current_v_, 2) / 2.0 / distance_to_stop;
    limited_speed_ = std::min(
        limited_speed_, last_limited_speed_ - deceleration * all_delay_time_);
    limited_speed_ = std::max(limited_speed_, 0.0);
  }
  last_limited_speed_ = limited_speed_;
  LOG_INFO("adc slow down, distance_to_stop, limited_speed_: {:.2f}, {:.2f} .",
           distance_to_stop, limited_speed_);
  return;
}

void MotorwaySpeedReversedObsDecider::ReversedObsContextInfo(
    const Obstacle& obstacle) {
  LOG_INFO(
      "obs id, center_s, center_l: [{}], {:.2f}, {:.2f} ; boundary start_s, "
      "end_s, start_l, end_l:{:.2f}, {:.2f}, {:.2f}, {:.2f} .",
      obstacle.id(), obstacle.center_sl().s(), obstacle.center_sl().l(),
      obstacle.PolygonBoundary().start_s(), obstacle.PolygonBoundary().end_s(),
      obstacle.PolygonBoundary().start_l(), obstacle.PolygonBoundary().end_l());
  LOG_INFO(
      "adc_current_s_ {:.2f}, single_road_pre_check_range_ {:.2f}, "
      "adc_width_ "
      "{:.2f}, obstacle.width {:.2f}, single_road_width_ {:.2f}",
      adc_current_s_, single_road_pre_check_range_, adc_width_,
      obstacle.width(), single_road_width_);
}

void MotorwaySpeedReversedObsDecider::SpeedReversedObsContextInfo(
    const std::vector<Boundary>& adc_front_road_boundaries,
    const std::unordered_set<int>& dynamic_obs_need_ignore) {
  LOG_INFO(
      "adc_front_road_boundaries size, single_road_boundaries_, "
      "reversed_obs_data_: {}, {}, {}.",
      adc_front_road_boundaries.size(), single_road_boundaries_.size(),
      reversed_obs_data_.obs_last_l_dis.size());
  if (!adc_front_road_boundaries.empty()) {
    LOG_INFO(
        "first adc_front_road_boundaries s_s, e_s, s_l, e_l:{:.2f}, {:.2f}, "
        "{:.2f}, {:.2f}",
        adc_front_road_boundaries.front().start_s(),
        adc_front_road_boundaries.front().end_s(),
        adc_front_road_boundaries.front().start_l(),
        adc_front_road_boundaries.front().end_l());
  }
  for (const auto& single_road_boundary : single_road_boundaries_) {
    LOG_INFO(
        "single road boundary start_s, end_s, start_l, end_l:{:.2f}, {:.2f}, "
        "{:.2f}, {:.2f}",
        single_road_boundary.start_s(), single_road_boundary.end_s(),
        single_road_boundary.start_l(), single_road_boundary.end_l());
  }
  LOG_INFO("single_road_pre_check_range_: {:.2f}",
           single_road_pre_check_range_);
  LOG_INFO(
      "adc check area emergency_s_range_, slow_down_s_range_: {:.2f}, "
      "{:.2f}.",
      emergency_s_range_, slow_down_s_range_);
  LOG_INFO("need to ignore obs number: {} .", dynamic_obs_need_ignore.size());
  if (!dynamic_obs_need_ignore.empty()) {
    std::string str_obs_ids{"obs id to ignore : "};
    for (const auto& obs_id : dynamic_obs_need_ignore) {
      str_obs_ids.append("[" + std::to_string(obs_id) + "] ");
    }
    LOG_INFO("{} .", str_obs_ids);
  }
}

}  // namespace planning
}  // namespace neodrive

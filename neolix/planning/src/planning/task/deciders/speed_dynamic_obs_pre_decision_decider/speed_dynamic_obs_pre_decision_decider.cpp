#include "speed_dynamic_obs_pre_decision_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/task/deciders/speed_static_obs_pre_decision_decider/speed_static_obs_pre_decision_decider.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

SpeedDynamicObsPreDecisionDecider::SpeedDynamicObsPreDecisionDecider() {
  name_ = "SpeedDynamicObsPreDecisionDecider";
}

SpeedDynamicObsPreDecisionDecider::~SpeedDynamicObsPreDecisionDecider() {
  Reset();
}

namespace {

constexpr double kSDistance = 3.2;
constexpr std::size_t kSteps = 30;

bool GetSRangeFromBox2d(ReferenceLinePtr ref_line, const Box2d& box2d,
                        double* const max_s, double* const min_s) {
  std::vector<Vec2d> corners{};
  box2d.get_all_corners(&corners);
  double ret_max_s{std::numeric_limits<double>::lowest()};
  double ret_min_s{std::numeric_limits<double>::max()};
  for (auto& corner : corners) {
    SLPoint sl_point;
    if (!ref_line->GetPointInFrenetFrame(corner, &sl_point)) {
      return false;
    }
    ret_max_s = std::max(ret_max_s, sl_point.s());
    ret_min_s = std::min(ret_min_s, sl_point.s());
  }
  *max_s = ret_max_s;
  *min_s = ret_min_s;
  return true;
}

void VisAdcCornerPoint(const Vec2d& corner_point,
                       const std::string event_name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(event_name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto sphere = event->mutable_sphere()->Add();
  sphere->mutable_center()->set_x(corner_point.x());
  sphere->mutable_center()->set_y(corner_point.y());
  sphere->mutable_center()->set_z(0);
  sphere->set_radius(0.05);
}

void GetAdcCornerPointCoordinate(const InsidePlannerData& inside_data,
                                 std::vector<Vec2d>& adc_corner_pt_coordinate) {
  adc_corner_pt_coordinate.clear();
  adc_corner_pt_coordinate.resize(
      static_cast<int>(AdcCollideCornerPoint::NONE));
  double x_g = 0.0;
  double y_g = 0.0;
  double theta_g = 0.0;
  // std::vector<Vec2d> adc_points{};
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                -VehicleParam::Instance()->back_edge_to_center(),
                VehicleParam::Instance()->left_edge_to_center(), 0.0, x_g, y_g,
                theta_g);
  adc_corner_pt_coordinate[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)] =
      std::move(Vec2d(x_g, y_g));
  // adc_points.emplace_back(x_g, y_g);
  // VisAdcCornerPoint(Vec2d(x_g, y_g), "left_rear");
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                -VehicleParam::Instance()->back_edge_to_center(),
                -VehicleParam::Instance()->right_edge_to_center(), 0.0, x_g,
                y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_REAR)] = std::move(Vec2d(x_g, y_g));
  // adc_points.emplace_back(x_g, y_g);
  // VisAdcCornerPoint(Vec2d(x_g, y_g), "right_rear");
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                VehicleParam::Instance()->front_edge_to_center(),
                -VehicleParam::Instance()->right_edge_to_center(), 0.0, x_g,
                y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_FRONT)] = std::move(Vec2d(x_g, y_g));
  // adc_points.emplace_back(x_g, y_g);
  // VisAdcCornerPoint(Vec2d(x_g, y_g), "right_front");
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                VehicleParam::Instance()->front_edge_to_center(),
                VehicleParam::Instance()->left_edge_to_center(), 0.0, x_g, y_g,
                theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::LEFT_FRONT)] = std::move(Vec2d(x_g, y_g));
  // adc_points.emplace_back(x_g, y_g);
  // VisAdcCornerPoint(Vec2d(x_g, y_g), "left_front");
}

void VisObsPolygon2d(const Polygon2d& show_polygon) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("ObsPolygon2d");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto polygon = event->mutable_polygon()->Add();
  for (const auto& pt : show_polygon.points()) {
    set_pt(polygon->add_point(), pt);
  }
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

ErrorCode SpeedDynamicObsPreDecisionDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
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

bool SpeedDynamicObsPreDecisionDecider::Process(TaskInfo& task_info) {
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  auto& path_accumulated_s =
      outside_planner_data_ptr->speed_obstacle_context.path_accumulated_s;
  adc_current_s_ = task_info.curr_sl().s();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  GetAdcCornerPointCoordinate(task_info.current_frame()->inside_planner_data(),
                              outside_planner_data_ptr->speed_obstacle_context
                                  .adc_corner_pt_coordinate);
  if (!ComputePathAccumulatedS(
          *task_info.current_frame()->outside_planner_data().path_data,
          &path_accumulated_s)) {
    LOG_ERROR("ComputePathAccumulatedS failed.");
    return false;
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
    return false;
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
  // 3. uniform trajectory attention bounary
  const auto& speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  double uniform_trajectory_attention_lon_dis =
      std::max(15.0, speed_plan_config.uniform_trajectory_attention_lon_time *
                         adc_current_v_);
  double uniform_trajectory_attention_lat_dis =
      std::max(4.0, speed_plan_config.uniform_trajectory_attention_lat_time *
                        adc_current_v_);
  auto& uniform_trajectory_attention_boundary =
      outside_planner_data_ptr->speed_obstacle_context
          .uniform_trajectory_attention_boundary;
  uniform_trajectory_attention_boundary.set_start_s(
      task_info.adc_boundary().start_s() - 1.0);
  uniform_trajectory_attention_boundary.set_end_s(
      task_info.adc_boundary().end_s() + uniform_trajectory_attention_lon_dis);
  uniform_trajectory_attention_boundary.set_end_l(
      task_info.adc_boundary().end_l() + uniform_trajectory_attention_lat_dis);
  uniform_trajectory_attention_boundary.set_start_l(
      task_info.adc_boundary().start_l() -
      uniform_trajectory_attention_lat_dis);

  // compute AABox for path
  const auto& adc_bounding_boxes =
      outside_planner_data_ptr->speed_obstacle_context.adc_boundaries;
  constexpr double max = std::numeric_limits<double>::infinity();
  constexpr double min = -std::numeric_limits<double>::infinity();
  double max_x = min, max_y = min, min_x = max, min_y = max;
  for (std::size_t i = 0; i < adc_bounding_boxes.size(); ++i) {
    max_x = std::max(max_x, adc_bounding_boxes[i].get_aa_box().max_x());
    max_y = std::max(max_y, adc_bounding_boxes[i].get_aa_box().max_y());
    min_x = std::min(min_x, adc_bounding_boxes[i].get_aa_box().min_x());
    min_y = std::min(min_y, adc_bounding_boxes[i].get_aa_box().min_y());
  }
  path_max_polygon_ = Polygon2d(
      {{max_x, max_y}, {max_x, min_y}, {min_x, min_y}, {min_x, max_y}});

  path_small_polygons_.clear();
  for (std::size_t i = 0; i < adc_bounding_boxes.size(); i += kSteps) {
    max_x = min, max_y = min, min_x = max, min_y = max;
    bool flag = false;
    for (std::size_t j = i; j < i + kSteps; ++j) {
      if (j >= adc_bounding_boxes.size()) break;
      max_x = std::max(max_x, adc_bounding_boxes[j].get_aa_box().max_x());
      max_y = std::max(max_y, adc_bounding_boxes[j].get_aa_box().max_y());
      min_x = std::min(min_x, adc_bounding_boxes[j].get_aa_box().min_x());
      min_y = std::min(min_y, adc_bounding_boxes[j].get_aa_box().min_y());
      flag = true;
    }
    if (flag) {
      path_small_polygons_.push_back(Polygon2d(
          {{max_x, max_y}, {max_x, min_y}, {min_x, min_y}, {min_x, max_y}}));
    }
  }

  if (DynamicObstaclePreDecision(task_info) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("DynamicObstaclePreDecision failed.");
    return false;
  }
  DynamicContextInfo(outside_planner_data_ptr);

  return true;
}

bool SpeedDynamicObsPreDecisionDecider::IsTheSameLeftRoutingLine(
    const ReferenceLinePtr& reference_line) {
  if (reference_line->routing_sequence_num() !=
      unprotected_turn_left_check_info_.last_routing_seq_num) {
    LOG_INFO(
        "is new routing last_routing_seq_num, routing_sequence_num: {}, {} ",
        unprotected_turn_left_check_info_.last_routing_seq_num,
        reference_line->routing_sequence_num());
    unprotected_turn_left_check_info_.last_routing_seq_num =
        reference_line->routing_sequence_num();
    return false;
  }
  return true;
}

bool SpeedDynamicObsPreDecisionDecider::IsTheSameRightRoutingLine(
    const ReferenceLinePtr& reference_line) {
  if (reference_line->routing_sequence_num() !=
      unprotected_turn_right_check_info_.last_routing_seq_num) {
    LOG_INFO(
        "is new routing last_routing_seq_num, routing_sequence_num: {}, {} ",
        unprotected_turn_right_check_info_.last_routing_seq_num,
        reference_line->routing_sequence_num());
    unprotected_turn_right_check_info_.last_routing_seq_num =
        reference_line->routing_sequence_num();
    return false;
  }
  return true;
}

void SpeedDynamicObsPreDecisionDecider::InitUnprotectedTurnRightPolygon() {
  std::vector<Vec2d> polygon_points;
  double begin_edge_heading_cos =
      std::cos(unprotected_turn_right_check_info_.begin_edge_heading);
  double begin_edge_heading_sin =
      std::sin(unprotected_turn_right_check_info_.begin_edge_heading);
  double end_edge_heading_cos =
      std::cos(unprotected_turn_right_check_info_.end_edge_heading);
  double end_edge_heading_sin =
      std::sin(unprotected_turn_right_check_info_.end_edge_heading);
  polygon_points.push_back(unprotected_turn_right_check_info_.center_point);
  Vec2d pt_start{
      unprotected_turn_right_check_info_.center_point.x() +
          unprotected_turn_right_check_info_.radius * begin_edge_heading_cos,
      unprotected_turn_right_check_info_.center_point.y() +
          unprotected_turn_right_check_info_.radius * begin_edge_heading_sin};
  polygon_points.emplace_back(pt_start);
  double bisector_vector_x = begin_edge_heading_cos + end_edge_heading_cos;
  double bisector_vector_y = begin_edge_heading_sin + end_edge_heading_sin;
  double bisector_vector_len = std::sqrt(std::pow(bisector_vector_x, 2) +
                                         std::pow(bisector_vector_y, 2));
  double bisector_unit_vector_x = bisector_vector_x / bisector_vector_len;
  double bisector_unit_vector_y = bisector_vector_y / bisector_vector_len;
  Vec2d pt_bisector{
      unprotected_turn_right_check_info_.center_point.x() +
          unprotected_turn_right_check_info_.radius * bisector_unit_vector_x,
      unprotected_turn_right_check_info_.center_point.y() +
          unprotected_turn_right_check_info_.radius * bisector_unit_vector_y};
  polygon_points.emplace_back(pt_bisector);
  Vec2d pt_end{
      unprotected_turn_right_check_info_.center_point.x() +
          unprotected_turn_right_check_info_.radius * end_edge_heading_cos,
      unprotected_turn_right_check_info_.center_point.y() +
          unprotected_turn_right_check_info_.radius * end_edge_heading_sin};
  polygon_points.emplace_back(pt_end);
  unprotected_turn_right_check_info_.check_area_polygon =
      Polygon2d{polygon_points};
  // Use cross product from end-edge to begin-edge to check area if check area
  // is on right side.
  unprotected_turn_right_check_info_.check_area_on_right_side =
      (end_edge_heading_cos * begin_edge_heading_sin -
           end_edge_heading_sin * begin_edge_heading_cos <
       0.0);
}

void SpeedDynamicObsPreDecisionDecider::InitUnprotectedTurnLeftPolygon() {
  std::vector<Vec2d> polygon_points;
  double begin_edge_heading_cos =
      std::cos(unprotected_turn_left_check_info_.begin_edge_heading);
  double begin_edge_heading_sin =
      std::sin(unprotected_turn_left_check_info_.begin_edge_heading);
  double end_edge_heading_cos =
      std::cos(unprotected_turn_left_check_info_.end_edge_heading);
  double end_edge_heading_sin =
      std::sin(unprotected_turn_left_check_info_.end_edge_heading);
  polygon_points.push_back(unprotected_turn_left_check_info_.center_point);
  Vec2d pt_start{
      unprotected_turn_left_check_info_.center_point.x() +
          unprotected_turn_left_check_info_.radius * begin_edge_heading_cos,
      unprotected_turn_left_check_info_.center_point.y() +
          unprotected_turn_left_check_info_.radius * begin_edge_heading_sin};
  polygon_points.emplace_back(pt_start);
  double bisector_vector_x = begin_edge_heading_cos + end_edge_heading_cos;
  double bisector_vector_y = begin_edge_heading_sin + end_edge_heading_sin;
  double bisector_vector_len = std::sqrt(std::pow(bisector_vector_x, 2) +
                                         std::pow(bisector_vector_y, 2));
  double bisector_unit_vector_x = bisector_vector_x / bisector_vector_len;
  double bisector_unit_vector_y = bisector_vector_y / bisector_vector_len;
  Vec2d pt_bisector{
      unprotected_turn_left_check_info_.center_point.x() +
          unprotected_turn_left_check_info_.radius * bisector_unit_vector_x,
      unprotected_turn_left_check_info_.center_point.y() +
          unprotected_turn_left_check_info_.radius * bisector_unit_vector_y};
  polygon_points.emplace_back(pt_bisector);
  Vec2d pt_end{
      unprotected_turn_left_check_info_.center_point.x() +
          unprotected_turn_left_check_info_.radius * end_edge_heading_cos,
      unprotected_turn_left_check_info_.center_point.y() +
          unprotected_turn_left_check_info_.radius * end_edge_heading_sin};
  polygon_points.emplace_back(pt_end);
  unprotected_turn_left_check_info_.check_area_polygon =
      Polygon2d{polygon_points};
  // Use cross product from end-edge to begin-edge to check area if check area
  // is on left side.
  unprotected_turn_left_check_info_.check_area_on_left_side =
      (end_edge_heading_cos * begin_edge_heading_sin -
           end_edge_heading_sin * begin_edge_heading_cos >
       0.0);
}

void SpeedDynamicObsPreDecisionDecider::ExtendTurnRightAreaEndS(
    const ReferenceLinePtr& reference_line) {
  if (!unprotected_turn_right_check_info_.is_extend_end_s) {
    // do not need to extend end_s
    return;
  }

  const auto& speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  double last_point_kappa{-std::numeric_limits<double>::infinity()};
  bool is_extended_ok{false};
  const auto& ref_points = reference_line->ref_points();
  for (const auto& pt : ref_points) {
    if (pt.s() < unprotected_turn_right_check_info_.original_start_s) {
      continue;
    }

    if (pt.s() - unprotected_turn_right_check_info_.original_end_s >
        speed_plan_config.right_turn_back_extend) {
      // ensure extend_s is smaller than max length in config file.
      unprotected_turn_right_check_info_.extended_end_s =
          unprotected_turn_right_check_info_.original_end_s +
          speed_plan_config.right_turn_back_extend;
      is_extended_ok = true;
      break;
    }

    if ((last_point_kappa < speed_plan_config.turn_right_max_kappa) &&
        (pt.kappa() > speed_plan_config.turn_right_max_kappa)) {
      // ensure extend_s is smaller than begin_s of next right-turn-area.
      unprotected_turn_right_check_info_.extended_end_s = pt.s() - kEpsilon;
      is_extended_ok = true;
      break;
    }
    last_point_kappa = pt.kappa();
  }

  if (is_extended_ok) {
    // flag of finishing extending end_s
    unprotected_turn_right_check_info_.is_extend_end_s = false;
    LOG_INFO(
        "get complete unprotected turn right check area, center_point(x, y), "
        "radius, original_start_s, original_end_s, begin_edge_heading, "
        "extended_start_s, extended_end_s, adc_current_s_: ({:.2f}, {:.2f}), "
        "{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f} ",
        unprotected_turn_right_check_info_.center_point.x(),
        unprotected_turn_right_check_info_.center_point.y(),
        unprotected_turn_right_check_info_.radius,
        unprotected_turn_right_check_info_.original_start_s,
        unprotected_turn_right_check_info_.original_end_s,
        unprotected_turn_right_check_info_.begin_edge_heading,
        unprotected_turn_right_check_info_.extended_start_s,
        unprotected_turn_right_check_info_.extended_end_s, adc_current_s_);
  }
}

void SpeedDynamicObsPreDecisionDecider::ExtendTurnLeftAreaEndS(
    const ReferenceLinePtr& reference_line) {
  if (!unprotected_turn_left_check_info_.is_extend_end_s) {
    // do not need to extend end_s
    return;
  }

  const auto& speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  double last_point_kappa{std::numeric_limits<double>::infinity()};
  bool is_extended_ok{false};
  const auto& ref_points = reference_line->ref_points();
  for (const auto& pt : ref_points) {
    if (pt.s() < unprotected_turn_left_check_info_.original_start_s) {
      continue;
    }

    if (pt.s() - unprotected_turn_left_check_info_.original_end_s >
        speed_plan_config.left_turn_back_extend) {
      // ensure extend_s is smaller than max length in config file.
      unprotected_turn_left_check_info_.extended_end_s =
          unprotected_turn_left_check_info_.original_end_s +
          speed_plan_config.left_turn_back_extend;
      is_extended_ok = true;
      break;
    }

    if ((last_point_kappa < speed_plan_config.turn_left_min_kappa) &&
        (pt.kappa() > speed_plan_config.turn_left_min_kappa)) {
      // ensure extend_s is smaller than begin_s of next left-turn-area.
      unprotected_turn_left_check_info_.extended_end_s = pt.s() - kEpsilon;
      is_extended_ok = true;
      break;
    }
    last_point_kappa = pt.kappa();
  }

  if (is_extended_ok) {
    // flag of finishing extending end_s
    unprotected_turn_left_check_info_.is_extend_end_s = false;
    LOG_INFO(
        "get complete unprotected turn left check area, center_point(x, y), "
        "radius, original_start_s, original_end_s, begin_edge_heading, "
        "extended_start_s, extended_end_s, adc_current_s_: ({:.2f}, {:.2f}), "
        "{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f} ",
        unprotected_turn_left_check_info_.center_point.x(),
        unprotected_turn_left_check_info_.center_point.y(),
        unprotected_turn_left_check_info_.radius,
        unprotected_turn_left_check_info_.original_start_s,
        unprotected_turn_left_check_info_.original_end_s,
        unprotected_turn_left_check_info_.begin_edge_heading,
        unprotected_turn_left_check_info_.extended_start_s,
        unprotected_turn_left_check_info_.extended_end_s, adc_current_s_);
  }
}

void SpeedDynamicObsPreDecisionDecider::InitUnprotectedTurnRightCheckArea(
    TaskInfo& task_info) {
  // 1. get input info
  const auto& speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  const auto& reference_line = task_info.reference_line();
  if (!IsTheSameRightRoutingLine(reference_line)) {
    unprotected_turn_right_check_info_.original_start_s = 0.0;
    unprotected_turn_right_check_info_.extended_start_s = 0.0;
    unprotected_turn_right_check_info_.original_end_s = 0.0;
    unprotected_turn_right_check_info_.extended_end_s = 0.0;
  }
  // 2.update "center_point, radius, end_edge_heading" every
  if ((adc_current_s_ > unprotected_turn_right_check_info_.extended_start_s) &&
      (adc_current_s_ < unprotected_turn_right_check_info_.extended_end_s)) {
    unprotected_turn_right_check_info_.center_point =
        Vec2d{task_info.current_frame()->inside_planner_data().vel_x,
              task_info.current_frame()->inside_planner_data().vel_y};
    unprotected_turn_right_check_info_.radius =
        speed_plan_config.turn_left_check_sector_radius;
    ReferencePoint tmp_refer_pt;
    reference_line->GetNearestRefPoint(adc_current_s_, &tmp_refer_pt);
    unprotected_turn_right_check_info_.end_edge_heading =
        normalize_angle(tmp_refer_pt.heading());
    InitUnprotectedTurnRightPolygon();
    // Reason of initlizing left-turn-area_end_s when car_s bigger than
    // left-turn-area_start_s is to ensure left-turn-area is totally in range of
    // reference-line.
    ExtendTurnRightAreaEndS(reference_line);
  }

  // 3. if adc go through currnet left-turn-area, get next one.

  if (adc_current_s_ < unprotected_turn_right_check_info_.extended_end_s) {
    LOG_INFO(
        "adc_current_s_ {:.3f} < "
        "unprotected_turn_right_check_info_.extended_end_s {:.3f}",
        adc_current_s_, unprotected_turn_right_check_info_.extended_end_s);
    return;
  }
  // invalid,create new
  unprotected_turn_right_check_info_.is_valid = false;
  LOG_INFO(
      "get new check area, adc_current_s_ >= extended_end_s: {:.2f}, "
      "{:.2f}",
      adc_current_s_, unprotected_turn_right_check_info_.extended_end_s);

  // find next left turn area
  const auto& ref_points = reference_line->ref_points();
  int i = 0;
  bool is_finding{false};
  for (; i < ref_points.size(); i++) {
    if (ref_points[i].s() < adc_current_s_) {
      continue;
    }
    // kappa is positive when turn left
    if ((ref_points[i].kappa() < speed_plan_config.turn_right_max_kappa) &&
        (!is_finding)) {
      LOG_INFO("begin find a new right turn area. begin point kappa: {:.2f}",
               ref_points[i].kappa());
      unprotected_turn_right_check_info_.original_start_s = ref_points[i].s();
      unprotected_turn_right_check_info_.begin_edge_heading =
          normalize_angle(ref_points[i].heading() - M_PI);
      is_finding = true;
    } else if ((ref_points[i].kappa() >
                speed_plan_config.turn_right_max_kappa) &&
               is_finding) {
      unprotected_turn_right_check_info_.original_end_s = ref_points[i].s();
      is_finding = false;
      unprotected_turn_right_check_info_.is_valid = true;
      LOG_INFO(
          "find a whole right turn normally. original_start_s, original_end_s, "
          "begin_edge_heading: {:.2f}, {:.2f}, {:.2f} ",
          unprotected_turn_right_check_info_.original_start_s,
          unprotected_turn_right_check_info_.original_end_s,
          unprotected_turn_right_check_info_.begin_edge_heading);
      break;
    }
  }

  if (unprotected_turn_right_check_info_.is_valid) {
    unprotected_turn_right_check_info_.extended_start_s =
        std::max(0.0, unprotected_turn_right_check_info_.original_start_s -
                          speed_plan_config.right_turn_front_extend);
    unprotected_turn_right_check_info_.extended_end_s =
        unprotected_turn_right_check_info_.original_end_s;
    // set flag to extend end_s
    unprotected_turn_right_check_info_.is_extend_end_s = true;
    LOG_INFO("original_start_s, extended_start_s: {:.2f} {:.2f} ",
             unprotected_turn_right_check_info_.original_start_s,
             unprotected_turn_right_check_info_.extended_start_s);
  }
}

void SpeedDynamicObsPreDecisionDecider::InitUnprotectedTurnLeftCheckArea(
    TaskInfo& task_info) {
  // 1. get input info
  const auto& speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  const auto& reference_line = task_info.reference_line();
  // 2.update "center_point, radius, end_edge_heading" every cycle.
  if ((adc_current_s_ > unprotected_turn_left_check_info_.extended_start_s) &&
      (adc_current_s_ < unprotected_turn_left_check_info_.extended_end_s)) {
    unprotected_turn_left_check_info_.center_point =
        Vec2d{task_info.current_frame()->inside_planner_data().vel_x,
              task_info.current_frame()->inside_planner_data().vel_y};
    unprotected_turn_left_check_info_.radius =
        speed_plan_config.turn_left_check_sector_radius;
    ReferencePoint tmp_refer_pt;
    reference_line->GetNearestRefPoint(adc_current_s_, &tmp_refer_pt);
    unprotected_turn_left_check_info_.end_edge_heading =
        normalize_angle(tmp_refer_pt.heading());
    InitUnprotectedTurnLeftPolygon();
    // Reason of initlizing left-turn-area_end_s when car_s bigger than
    // left-turn-area_start_s is to ensure left-turn-area is totally in range of
    // reference-line.
    ExtendTurnLeftAreaEndS(reference_line);
  }

  // 3. if adc go through currnet left-turn-area, get next one.
  if (!IsTheSameLeftRoutingLine(reference_line)) {
    unprotected_turn_left_check_info_.original_start_s = 0.0;
    unprotected_turn_left_check_info_.extended_start_s = 0.0;
    unprotected_turn_left_check_info_.original_end_s = 0.0;
    unprotected_turn_left_check_info_.extended_end_s = 0.0;
  }
  if (adc_current_s_ < unprotected_turn_left_check_info_.extended_end_s) {
    return;
  }
  unprotected_turn_left_check_info_.is_valid = false;
  LOG_DEBUG(
      "get new check area, adc_current_s_ >= extended_end_s: {:.2f}, "
      "{:.2f}",
      adc_current_s_, unprotected_turn_left_check_info_.extended_end_s);

  // find next left turn area
  const auto& ref_points = reference_line->ref_points();
  int i = 0;
  bool is_finding{false};
  for (; i < ref_points.size(); i++) {
    if (ref_points[i].s() < adc_current_s_) {
      continue;
    }
    // kappa is positive when turn left
    if ((ref_points[i].kappa() > speed_plan_config.turn_left_min_kappa) &&
        (!is_finding)) {
      LOG_INFO("begin find a new left turn area. begin point kappa: {:.2f}",
               ref_points[i].kappa());
      unprotected_turn_left_check_info_.original_start_s = ref_points[i].s();
      unprotected_turn_left_check_info_.begin_edge_heading =
          normalize_angle(ref_points[i].heading() + M_PI);
      is_finding = true;
    } else if ((ref_points[i].kappa() <
                speed_plan_config.turn_left_min_kappa) &&
               is_finding) {
      unprotected_turn_left_check_info_.original_end_s = ref_points[i].s();
      is_finding = false;
      unprotected_turn_left_check_info_.is_valid = true;
      LOG_INFO(
          "find a whole left turn normally. original_start_s, original_end_s, "
          "begin_edge_heading: {:.2f}, {:.2f}, {:.2f} ",
          unprotected_turn_left_check_info_.original_start_s,
          unprotected_turn_left_check_info_.original_end_s,
          unprotected_turn_left_check_info_.begin_edge_heading);
      break;
    }
  }

  if (unprotected_turn_left_check_info_.is_valid) {
    unprotected_turn_left_check_info_.extended_start_s =
        std::max(0.0, unprotected_turn_left_check_info_.original_start_s -
                          speed_plan_config.left_turn_front_extend);
    unprotected_turn_left_check_info_.extended_end_s =
        unprotected_turn_left_check_info_.original_end_s;
    // set flag to extend end_s
    unprotected_turn_left_check_info_.is_extend_end_s = true;
    LOG_INFO(
        "extend start_s, original_start_s, extended_start_s: {:.2f} {:.2f} ",
        unprotected_turn_left_check_info_.original_start_s,
        unprotected_turn_left_check_info_.extended_start_s);
  }
}

bool SpeedDynamicObsPreDecisionDecider::BuildPathAdcBoundingBoxesAndAABoxes(
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

bool SpeedDynamicObsPreDecisionDecider::ComputePathAccumulatedS(
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

bool SpeedDynamicObsPreDecisionDecider::BuildPathAdcBoundingBoxes(
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

bool SpeedDynamicObsPreDecisionDecider::BuildPathAdcPreBoundingBoxes(
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

bool SpeedDynamicObsPreDecisionDecider::BuildPathAdcAABoxes(
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

ErrorCode SpeedDynamicObsPreDecisionDecider::DynamicObstaclePreDecision(
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
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  LOG_INFO("___dynamic_obstacle_pre_decision infos___:");
  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }
    bool ignore{false};
    bool is_in_left_turn_area{false};
    bool is_in_right_turn_area{false};
    if (!inside_data.change_lane_task_mode) {
      IsDynamicObsNeedIgnore(inside_data, *dynamic_obs_vec[i], ignore,
                             is_in_left_turn_area, is_in_right_turn_area);
    }
    if (ignore) {
      continue;
    }
    LOG_INFO("id: {}, start dynamic obstacle collision check",
             dynamic_obs_vec[i]->id());
    if (CollisionCheckObstacleWithTrajectory(
            task_info, *dynamic_obs_vec[i], is_in_left_turn_area,
            is_in_right_turn_area) != ErrorCode::PLANNING_OK) {
      LOG_ERROR("collision check failed.");
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }
  return ErrorCode::PLANNING_OK;
}

bool SpeedDynamicObsPreDecisionDecider::ObsIsInLeftTurnCheckArea(
    const Obstacle& obstacle) const {
  if (!AdcIsInValidLeftTurnSRange()) {
    return false;
  }
  return unprotected_turn_left_check_info_.check_area_polygon.has_overlap(
      obstacle.polygon());
}

bool SpeedDynamicObsPreDecisionDecider::ObsIsInRightTurnCheckArea(
    const Obstacle& obstacle) const {
  if (!AdcIsInValidRightTurnSRange()) {
    return false;
  }
  return unprotected_turn_right_check_info_.check_area_polygon.has_overlap(
      obstacle.polygon());
}

bool SpeedDynamicObsPreDecisionDecider::AdcIsInValidLeftTurnSRange() const {
  const auto& speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  // If adc is in left turn extended s range && left turn is big, return true.
  return (adc_current_s_ >
          unprotected_turn_left_check_info_.extended_start_s) &&
         (adc_current_s_ < unprotected_turn_left_check_info_.extended_end_s) &&
         ((unprotected_turn_left_check_info_.original_end_s -
           unprotected_turn_left_check_info_.original_start_s) >
          speed_plan_config.left_turn_min_len_threshold) &&
         unprotected_turn_left_check_info_.check_area_on_left_side;
}

bool SpeedDynamicObsPreDecisionDecider::AdcIsInValidRightTurnSRange() const {
  const auto& speed_plan_config =
      config::PlanningConfig::Instance()->plan_config().speed_plan;
  LOG_INFO("check unprotectRightTurn {}, {}, {}, {}, {}", adc_current_s_,
           unprotected_turn_right_check_info_.extended_start_s,
           unprotected_turn_right_check_info_.extended_end_s,
           unprotected_turn_right_check_info_.original_end_s -
               unprotected_turn_right_check_info_.original_start_s,
           unprotected_turn_right_check_info_.check_area_on_right_side);
  // If adc is in left turn extended s range && left turn is big, return true.
  return (adc_current_s_ >
          unprotected_turn_right_check_info_.extended_start_s) &&
         (adc_current_s_ < unprotected_turn_right_check_info_.extended_end_s) &&
         ((unprotected_turn_right_check_info_.original_end_s -
           unprotected_turn_right_check_info_.original_start_s) >
          speed_plan_config.right_turn_min_len_threshold) &&
         unprotected_turn_right_check_info_.check_area_on_right_side;
}

bool SpeedDynamicObsPreDecisionDecider::IsDynamicObsNeedIgnore(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    bool& is_ignore, bool& is_in_left_turn_area,
    bool& is_in_right_turn_area) const {
  auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  is_ignore = false;
  is_in_left_turn_area = false;
  is_in_right_turn_area = false;
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

  is_in_left_turn_area = ObsIsInLeftTurnCheckArea(obstacle);
  is_in_right_turn_area = ObsIsInRightTurnCheckArea(obstacle);
  if (is_in_left_turn_area || is_in_right_turn_area) {
    LOG_INFO("obs [{}] is in  turn check area, not ignore.", obstacle.id());
  } else if (std::abs(heading_diff) > M_PI_2) {
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

ErrorCode
SpeedDynamicObsPreDecisionDecider::CollisionCheckObstacleWithTrajectory(
    TaskInfo& task_info, const Obstacle& obstacle,
    const bool& is_in_left_turn_area, const bool& is_in_right_turn_area) const {
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  if (outside_data == nullptr) return ErrorCode::PLANNING_ERROR_FAILED;
  if (obstacle.length() < 1e-4 || obstacle.width() < 1e-4) {
    LOG_ERROR("Obstacle [{}] length({:.4f}) < 1e-4 || width({:.4f}) < 1e-4",
              obstacle.id(), obstacle.length(), obstacle.width());
    return ErrorCode::PLANNING_OK;
  }

  const auto& adc_bounding_boxes =
      outside_data->speed_obstacle_context.adc_boundaries;
  if (adc_bounding_boxes.size() < 3) {
    LOG_ERROR("adc bounding boxes size < 3.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (outside_data->path_data->path().path_points().size() !=
      adc_bounding_boxes.size()) {
    LOG_ERROR("path_points size != adc_bounding_boxes size.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  auto path_points = outside_data->path_data->path().path_points();
  bool adc_on_reference_line_flag =
      IsAdcOnReferenceLine(task_info.reference_line(), inside_data);
  double adc_front_theta = path_points.front().theta();

  // prediction trajectory collision check
  LOG_DEBUG("start prediction trajectory collision check:");
  CollisionCheckWithPredictionTrajectory(task_info, adc_on_reference_line_flag,
                                         obstacle, is_in_left_turn_area,
                                         is_in_right_turn_area);

  return ErrorCode::PLANNING_OK;
}

void SpeedDynamicObsPreDecisionDecider::CollisionCheckWithPredictionTrajectory(
    TaskInfo& task_info, const bool adc_on_reference_line_flag,
    const Obstacle& obstacle, const bool& is_in_left_turn_area,
    const bool& is_in_right_turn_area) const {
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto& path_points = outside_data->path_data->path().path_points();
  const auto& adc_bounding_boxes =
      outside_data->speed_obstacle_context.adc_boundaries;
  LOG_INFO("adc_front_theta is {:.4f}", path_points.front().theta());
  const auto& adc_corner_pt_coordinate =
      outside_data->speed_obstacle_context.adc_corner_pt_coordinate;
  const auto& prediction_config =
      config::PlanningConfig::Instance()->plan_config().prediction;

  bool enable_prediction = DataCenter::Instance()->enable_prediction();
  const auto& pred_trajs = obstacle.prediction_trajectories();
  if (enable_prediction && pred_trajs.empty()) {
    LOG_ERROR("pred_trajs empty.");
    return;
  }
  const auto& pred_traj =
      enable_prediction ? pred_trajs.front() : obstacle.uniform_trajectory();
  if (pred_traj.num_of_points() < 2) {
    LOG_ERROR("pred_traj nums < 2");
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
        obstacle.center(), {point.x(), point.y()}, obstacle.velocity_heading(),
        point.theta(), obstacle.polygon());

    /// Filter with path AABox2d
    if (!path_max_polygon_.has_overlap(obs_polygon)) {
      continue;
    }
    bool path_small_flag = false;
    for (const auto& small_polygon : path_small_polygons_) {
      if (small_polygon.has_overlap(obs_polygon)) {
        path_small_flag = true;
        break;
      }
    }
    if (!path_small_flag) continue;

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

  SpeedObstacleDecision decision;
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
  decision.reverse = reverse_obs;
  decision.is_in_left_turn_area = is_in_left_turn_area;
  decision.is_in_right_turn_area = is_in_right_turn_area;
  decision.lower_adc_first_index = lower_adc_first_index;
  decision.adc_first_collide_corner_point = adc_first_collide_corner_point;
  outside_data->speed_obstacle_context.dynamic_obstacles_decision.emplace_back(
      decision);
  LOG_INFO("obs[{}], decision collide {}, adc_first_collide_corner_point {}.",
           obstacle.id(), decision.collide,
           static_cast<int>(adc_first_collide_corner_point));
}

bool SpeedDynamicObsPreDecisionDecider::IsAdcOnReferenceLine(
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

void SpeedDynamicObsPreDecisionDecider::FindHighAndLowWithPolygon(
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

void SpeedDynamicObsPreDecisionDecider::DynamicContextInfo(
    OutsidePlannerData* const outside_data) {
  LOG_INFO("speed_obstacle_context_dynamic_:");
  for (const auto& context :
       outside_data->speed_obstacle_context.dynamic_obstacles_decision) {
    LOG_INFO("obstacle[{}], collided[{}], reverse[{}]", context.obstacle.id(),
             context.collide, context.reverse);
    LOG_INFO(
        "front upper_s, upper_t, upper_project_speed, upper_heading_diff: "
        "{:.4f}, {:.4f}, {:.4f}, {:.4f}",
        context.upper_points.front().first.s(),
        context.upper_points.front().first.t(),
        context.upper_points.front().second,
        context.upper_points_heading_diff.front());
    LOG_INFO(
        "back upper_s, upper_t, upper_project_speed, upper_heading_diff: "
        "{:.4f}, {:.4f}, {:.4f}, {:.4f}",
        context.upper_points.back().first.s(),
        context.upper_points.back().first.t(),
        context.upper_points.back().second,
        context.upper_points_heading_diff.back());
    LOG_INFO(
        "front lower_s, lower_t, lower_project_speed, lower_heading_diff: "
        "{:.4f}, {:.4f}, {:.4f}, {:.4f}",
        context.lower_points.front().first.s(),
        context.lower_points.front().first.t(),
        context.lower_points.front().second,
        context.lower_points_heading_diff.front());
    LOG_INFO(
        "back lower_s, lower_t, lower_project_speed, lower_heading_diff: "
        "{:.4f}, {:.4f}, {:.4f}, {:.4f}",
        context.lower_points.back().first.s(),
        context.lower_points.back().first.t(),
        context.lower_points.back().second,
        context.lower_points_heading_diff.back());
  }
}

void SpeedDynamicObsPreDecisionDecider::VisUnprotectedLeftTurnCheckArea() {
  if (!FLAGS_planning_enable_vis_event) return;
  if (!AdcIsInValidLeftTurnSRange()) {
    LOG_DEBUG(
        "do not vis check area. adc_current_s_, extended_start_s, "
        "extended_end_s: {:.2f}, {:.2f}, {:.2f}",
        adc_current_s_, unprotected_turn_left_check_info_.extended_start_s,
        unprotected_turn_left_check_info_.extended_end_s);
    return;
  };

  LOG_DEBUG(
      "vis check area. adc_current_s_, extended_start_s, extended_end_s: "
      "{:.2f}, {:.2f}, {:.2f}",
      adc_current_s_, unprotected_turn_left_check_info_.extended_start_s,
      unprotected_turn_left_check_info_.extended_end_s);

  auto event = vis::EventSender::Instance()->GetEvent("LeftTurnCheckArea");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto polygon = event->mutable_polygon()->Add();
  for (const auto& pt :
       unprotected_turn_left_check_info_.check_area_polygon.points()) {
    set_pt(polygon->add_point(), pt);
  }
}

void SpeedDynamicObsPreDecisionDecider::VisUnprotectedRightTurnCheckArea() {
  if (!FLAGS_planning_enable_vis_event) return;
  if (!AdcIsInValidRightTurnSRange()) {
    LOG_DEBUG(
        "do not vis check area. adc_current_s_, extended_start_s, "
        "extended_end_s: {:.2f}, {:.2f}, {:.2f}",
        adc_current_s_, unprotected_turn_right_check_info_.extended_start_s,
        unprotected_turn_right_check_info_.extended_end_s);
    return;
  };

  LOG_DEBUG(
      "vis check area. adc_current_s_, extended_start_s, extended_end_s: "
      "{:.2f}, {:.2f}, {:.2f}",
      adc_current_s_, unprotected_turn_right_check_info_.extended_start_s,
      unprotected_turn_right_check_info_.extended_end_s);

  auto event = vis::EventSender::Instance()->GetEvent("RightTurnCheckArea");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto polygon = event->mutable_polygon()->Add();
  for (const auto& pt :
       unprotected_turn_right_check_info_.check_area_polygon.points()) {
    set_pt(polygon->add_point(), pt);
  }
}

}  // namespace planning
}  // namespace neodrive

#include "path_original_solve_bound_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/data_center/data_center.h"

namespace neodrive {
namespace planning {

void VisRefLineBoundInfo(const ReferenceLinePtr ref_line,
                         const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  std::vector<ReferencePoint> ref_points = ref_line->ref_points();
  for (std::size_t i = 0; i < ref_points.size(); i += 10) {
    // ref point
    auto pt = ref_points.at(i);
    Vec2d xy{pt.x(), pt.y()};
    auto sphere = event->mutable_sphere()->Add();
    set_pt(sphere->mutable_center(), xy);
    sphere->set_radius(0.05);

    // left/right bound point
    SLPoint left_sl{pt.s(), pt.left_bound()},
        right_sl{pt.s(), -pt.right_bound()};
    Vec2d left_xy, right_xy;

    ref_line->GetPointInCartesianFrame(left_sl, &left_xy);
    auto sphere_l = event->mutable_sphere()->Add();
    set_pt(sphere_l->mutable_center(), left_xy);
    sphere_l->set_radius(0.1);

    ref_line->GetPointInCartesianFrame(right_sl, &right_xy);
    auto sphere_r = event->mutable_sphere()->Add();
    set_pt(sphere_r->mutable_center(), right_xy);
    sphere_r->set_radius(0.1);
  }
}

void VisRefLineReversRightBoundInfo(const ReferenceLinePtr ref_line,
                                    const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  std::vector<ReferencePoint> ref_points = ref_line->ref_points();
  for (std::size_t i = 0; i < ref_points.size(); i += 10) {
    // ref point
    auto pt = ref_points.at(i);
    Vec2d xy{pt.x(), pt.y()};
    auto sphere = event->mutable_sphere()->Add();
    set_pt(sphere->mutable_center(), xy);
    sphere->set_radius(0.05);

    // reverse road right bound point
    SLPoint reverse_right_sl{pt.s(), pt.left_reverse_road_bound()};
    Vec2d reverse_right_xy;

    ref_line->GetPointInCartesianFrame(reverse_right_sl, &reverse_right_xy);
    auto sphere_reverse = event->mutable_sphere()->Add();
    set_pt(sphere_reverse->mutable_center(), reverse_right_xy);
    sphere_reverse->set_radius(0.1);
  }
}

PathOriginalSolveBoundDecider::PathOriginalSolveBoundDecider() {
  name_ = "PathOriginalSolveBoundDecider";
}

PathOriginalSolveBoundDecider::~PathOriginalSolveBoundDecider() { Reset(); }

ErrorCode PathOriginalSolveBoundDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>> start execute: {}", name_);

  if (!Init(task_info)) {
    LOG_ERROR("Init Failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool PathOriginalSolveBoundDecider::Init(TaskInfo& task_info) {
  if (task_info.current_frame()->mutable_outside_planner_data() == nullptr) {
    LOG_ERROR("outside_data == nullptr");
    return false;
  }
  adc_start_l_ = task_info.adc_boundary_origin().start_l();
  adc_end_l_ = task_info.adc_boundary_origin().end_l();

  return true;
}

bool PathOriginalSolveBoundDecider::Process(TaskInfo& task_info) {
  if (data_center_->master_info().curr_scenario() ==
      ScenarioState::BARRIER_GATE) {
    LOG_INFO("current scenario is barrier gate, no bound extend!");
    return true;
  }

  motorway_detour_ = data_center_->master_info().curr_scenario() ==
                     ScenarioState::MOTORWAY_DETOUR;
  detour_ =
      data_center_->master_info().curr_scenario() == ScenarioState::DETOUR;

  VisRefLineBoundInfo(task_info.reference_line(), "bound info before extend");
  VisRefLineReversRightBoundInfo(task_info.reference_line(),
                                 "reverse right bound info");

  if (!ExtendLaneBound(task_info)) {
    LOG_ERROR("Extend lane bound failed.");
    return false;
  }

  VisRefLineBoundInfo(task_info.reference_line(), "bound info after extend");

  return true;
}

bool PathOriginalSolveBoundDecider::ExtendLaneBound(TaskInfo& task_info) {
  bool extend_flag{false};
  bool outside_lane{false};
  double extend_ratio{1.0};
  double ego_outside_l{0.0};
  /// In non-motor/motor detour scenario
  if (detour_) {
    auto& lane_borrow_context =
        data_center_->master_info().lane_borrow_context();
    bool borrow_left =
        lane_borrow_context.borrow_side == LaneBorrowContext::BorrowSide::Left;
    bool borrow_right =
        lane_borrow_context.borrow_side == LaneBorrowContext::BorrowSide::Right;
    extend_flag = borrow_left || borrow_right;
    extend_direction_ =
        (!borrow_left && !borrow_right)
            ? ExtendDirection::NONE
            : (borrow_left ? ExtendDirection::LEFT : ExtendDirection::RIGHT);
    extend_ratio = lane_borrow_context.lane_borrow_extend_ratio;
  } else if (motorway_detour_) {
    auto& motorway_lane_borrow_context =
        data_center_->master_info().motorway_lane_borrow_context();
    bool borrow_left = motorway_lane_borrow_context.borrow_side ==
                       MotorwayLaneBorrowContext::BorrowSide::Left;
    bool borrow_right = motorway_lane_borrow_context.borrow_side ==
                        MotorwayLaneBorrowContext::BorrowSide::Right;
    extend_flag = borrow_left || borrow_right;
    extend_direction_ =
        (!borrow_left && !borrow_right)
            ? ExtendDirection::NONE
            : (borrow_left ? ExtendDirection::LEFT : ExtendDirection::RIGHT);
    extend_ratio = motorway_lane_borrow_context.lane_borrow_extend_ratio;
  }
  /// In other scenario
  if (!extend_flag) {
    ReferencePoint pt = task_info.curr_referline_pt();
    if (adc_end_l_ > pt.left_lane_bound()) {
      extend_flag = true;
      outside_lane = true;
      extend_direction_ = ExtendDirection::LEFT;
      ego_outside_l = adc_end_l_ - pt.left_lane_bound();
      LOG_INFO("adc is outside current lane, extend left bound");
    } else if (adc_start_l_ < -pt.right_lane_bound()) {
      extend_flag = true;
      outside_lane = true;
      extend_direction_ = ExtendDirection::RIGHT;
      ego_outside_l = -pt.right_lane_bound() - adc_start_l_;
      LOG_INFO("adc is outside current lane, extend right bound");
    }
    extend_ratio = 0.0;
  }
  if (!extend_flag) {
    LOG_INFO("not should be extend solve bound.");
    return true;
  }

  LOG_INFO("modify lane bound: left({}), right({})",
           extend_direction_ == ExtendDirection::LEFT,
           extend_direction_ == ExtendDirection::RIGHT);
  LOG_INFO("lane_extend_weight: {:.4f}", extend_ratio);

  std::size_t ref_points_size =
      task_info.reference_line_raw()->ref_points().size();
  for (std::size_t i = 0; i < ref_points_size; ++i) {
    const auto& pt_utm = task_info.reference_line_raw()->ref_points().at(i);
    double left_bound{pt_utm.left_lane_bound()};
    double right_bound{pt_utm.right_lane_bound()};
    double nearby_width{0.0};
    uint64_t nearby_lane_id;
    bool left_lane_borrow_flag{false}, right_lane_borrow_flag{false};
    std::pair<double, double> near_lane_width{0.0, 0.0};
    auto& plan_config = config::PlanningConfig::Instance()->plan_config();
    if (outside_lane) {
      if (extend_direction_ != ExtendDirection::NONE) {
        nearby_width = ego_outside_l + 0.6;
        LOG_DEBUG("adc is outside current lane");
      }
    } else if (extend_direction_ == ExtendDirection::LEFT &&
               PlanningMap::Instance()->GetNearestLeftLane(
                   task_info.curr_referline_pt().hd_map_lane_id(),
                   {pt_utm.x(), pt_utm.y()}, nearby_lane_id)) {
      const double& left_min_dis =
          task_info.curr_referline_pt().lane_type_is_pure_city_driving()
              ? plan_config.motorway_detour_scenario.left_min_lane_borrow_dis
              : plan_config.detour_scenario.left_min_lane_borrow_dis;
      const double& left_max_dis =
          task_info.curr_referline_pt().lane_type_is_pure_city_driving()
              ? plan_config.motorway_detour_scenario.left_max_lane_borrow_dis
              : plan_config.detour_scenario.left_max_lane_borrow_dis;
      double lane_s, lane_l;
      if (!PlanningMap::Instance()->GetSLWithLane(nearby_lane_id, pt_utm.x(),
                                                  pt_utm.y(), lane_s, lane_l)) {
        lane_s = task_info.curr_sl().s();
      }
      near_lane_width =
          PlanningMap::Instance()->GetLaneDistanceWidth(nearby_lane_id, lane_s);
      nearby_width = std::clamp(near_lane_width.first + near_lane_width.second,
                                left_min_dis, left_max_dis);
    } else if (extend_direction_ == ExtendDirection::RIGHT &&
               PlanningMap::Instance()->GetNearestRightLane(
                   task_info.curr_referline_pt().hd_map_lane_id(),
                   {pt_utm.x(), pt_utm.y()}, nearby_lane_id)) {
      const double& right_min_dis =
          task_info.curr_referline_pt().lane_type_is_pure_city_driving()
              ? plan_config.motorway_detour_scenario.right_min_lane_borrow_dis
              : plan_config.detour_scenario.right_min_lane_borrow_dis;
      const double& right_max_dis =
          task_info.curr_referline_pt().lane_type_is_pure_city_driving()
              ? plan_config.motorway_detour_scenario.right_max_lane_borrow_dis
              : plan_config.detour_scenario.right_max_lane_borrow_dis;
      double lane_s, lane_l;
      if (!PlanningMap::Instance()->GetSLWithLane(nearby_lane_id, pt_utm.x(),
                                                  pt_utm.y(), lane_s, lane_l)) {
        lane_s = task_info.curr_sl().s();
      }
      near_lane_width =
          PlanningMap::Instance()->GetLaneDistanceWidth(nearby_lane_id, lane_s);
      nearby_width = std::clamp(near_lane_width.first + near_lane_width.second,
                                right_min_dis, right_max_dis);
    } else if (!PlanningMap::Instance()->GetNearestLeftLane(
                   task_info.curr_referline_pt().hd_map_lane_id(),
                   {pt_utm.x(), pt_utm.y()}, nearby_lane_id) ||
               !PlanningMap::Instance()->GetNearestRightLane(
                   task_info.curr_referline_pt().hd_map_lane_id(),
                   {pt_utm.x(), pt_utm.y()}, nearby_lane_id)) {
      nearby_width = 3.5;
    }
    if (ExtendSolveBound(pt_utm, extend_direction_ == ExtendDirection::LEFT,
                         extend_direction_ == ExtendDirection::RIGHT,
                         extend_ratio, &nearby_width, &left_bound, &right_bound,
                         &left_lane_borrow_flag, &right_lane_borrow_flag)) {
      task_info.reference_line()->SetIndexBound(i, left_bound, right_bound);
      task_info.reference_line()->SetIndexLaneBorrowFlag(
          i, left_lane_borrow_flag, right_lane_borrow_flag);
    }

    const auto& pt = task_info.reference_line()->ref_points().at(i);
    if (i % 5 == 0) {
      LOG_DEBUG(
          "s, l_r_b, l_l_b, l_b, r_r_b, r_l_b, r_b: {:.3f}, {:.3f}, "
          "{:.3f}, "
          "{:.3f}, {:.3f}, {:.3f}, {:.3f}",
          pt.s(), pt.left_road_bound(), pt.left_lane_bound(), pt.left_bound(),
          pt.right_road_bound(), pt.right_lane_bound(), pt.right_bound());
    }
  }

  return true;
}

bool PathOriginalSolveBoundDecider::ExtendSolveBound(
    const ReferencePoint& point, const bool left, const bool right,
    const double ratio, const double* nearby_lane_width, double* left_bound,
    double* right_bound, bool* left_lane_borrow_flag,
    bool* right_lane_borrow_flag) {
  if (!FLAGS_planning_lane_borrow_enable_flag) {
    return false;
  }

  *left_lane_borrow_flag = false;
  *right_lane_borrow_flag = false;
  double left_lane_bound = point.left_lane_bound();
  double left_road_bound = point.left_road_bound();
  double right_lane_bound = point.right_lane_bound();
  double right_road_bound = point.right_road_bound();
  double left_reverse_road_bound = point.left_reverse_road_bound();

  if (IsAllowedExtendToReverseLane(point)) {
    left_road_bound = left_reverse_road_bound;
  }

  bool extend_left =
      left && (std::abs(left_road_bound - left_lane_bound) >= 0.3);
  bool extend_right =
      right && (std::abs(right_road_bound - right_lane_bound) >= 0.3);
  if (!extend_left && !extend_right) {
    return false;
  }

  if (extend_left) {
    double left_bound_by_weight =
        left_lane_bound * ratio +
        (1 - ratio) * (left_lane_bound + *nearby_lane_width);
    *left_bound = std::abs(std::clamp(
        left_bound_by_weight,
        adc_end_l_ + 0.5 * VehicleParam::Instance()->width(), left_road_bound));

    *left_lane_borrow_flag = true;
  }
  if (extend_right) {
    double right_bound_by_weight =
        right_lane_bound * ratio +
        (1 - ratio) * (right_lane_bound + *nearby_lane_width);
    *right_bound = std::abs(
        std::clamp(-right_bound_by_weight, -right_road_bound,
                   adc_start_l_ - 0.5 * VehicleParam::Instance()->width()));

    *right_lane_borrow_flag = true;
  }
  LOG_DEBUG(
      "extend lat solve bound, extend l/r: {}, {}, extend width: "
      "{:.4f}",
      extend_left, extend_right, *nearby_lane_width);

  return true;
}

bool PathOriginalSolveBoundDecider::IsAllowedExtendToReverseLane(
    const ReferencePoint& ref_pt) {
  /// 1.Read the information of the reference point
  BoundaryEdgeType left_boundary_edge_type = ref_pt.left_boundary_edge_type();
  std::vector<DividerFeature> left_divider_feature =
      ref_pt.left_divider_feature();
  double left_road_bound = ref_pt.left_road_bound(),
         left_reverse_road_bound = ref_pt.left_reverse_road_bound();

  /// 2.Information updates
  bool is_left_bound_allowed_cross =
      (left_boundary_edge_type == BoundaryEdgeType::MARKING);
  int cross_cnt = 0;
  for (int i = 0; i < left_divider_feature.size(); ++i) {
    DividerFeature divider_feature = left_divider_feature[i];
    if (scenario_common::CanCrossLane(divider_feature.divider_type_,
                                      divider_feature.divider_color_))
      cross_cnt++;
  }
  bool is_left_divider_allowed_cross =
      (cross_cnt == left_divider_feature.size());
  bool is_reverse_lane_allowed_cross =
      (std::abs(left_road_bound - left_reverse_road_bound) >= 1e-5);

  LOG_DEBUG(
      "ref_pt infos x:{:.4f}, y:{:.4f}, left_boundary_edge_type:{}, "
      "left_road_bound:{:.4f}, left_reverse_road_bound:{:.4f}",
      ref_pt.x(), ref_pt.y(), ref_pt.left_boundary_edge_type(),
      ref_pt.left_road_bound(), ref_pt.left_reverse_road_bound());

  /// 3.Excluded logic
  if (!is_left_bound_allowed_cross) return false;
  if (!is_left_divider_allowed_cross) return false;
  if (!is_reverse_lane_allowed_cross) return false;

  return true;
}

}  // namespace planning
}  // namespace neodrive

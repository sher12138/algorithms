#include "observe_ref_for_lane_borrow.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/math/math_utils.h"
#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

ObserveRefForLaneBorrow::ObserveRefForLaneBorrow()
    : ObserveRef("ObserveRefForLaneBorrow") {}

bool ObserveRefForLaneBorrow::ComputePathObserveRefLInfo(
    TaskInfo& task_info, PathObserveRefLInfo* path_observe_info) {
  if (path_observe_info == nullptr ||
      !observe_ref_config().enable_lane_borrow) {
    return false;
  }
  auto& observe_l = path_observe_info->observe_ref_l;
  auto& attention_dynamic_obstacles =
      path_observe_info->attention_dynamic_obstacles;
  auto& observe_type = path_observe_info->type;
  attention_dynamic_obstacles.clear();
  observe_type = PathObserveRefLInfo::RefLType::NONE;

  const auto& bounds_info =
      task_info.current_frame()
          ->outside_planner_data()
          .path_context.original_path_boundary.path_boundary;
  if (bounds_info.empty()) {
    LOG_WARN("path bounds_info is empty.");
    return false;
  }

  adc_original_boundary_ = task_info.adc_boundary_origin();
  LOG_INFO("adc_original_boundary: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
           adc_original_boundary_.start_s(), adc_original_boundary_.end_s(),
           adc_original_boundary_.start_l(), adc_original_boundary_.end_l());
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  auto ref_ptr = task_info.reference_line();
  ReferencePoint ref_pt{};
  if (!ref_ptr->GetNearestRefPoint(inside_data.init_sl_point.s(), &ref_pt)) {
    LOG_WARN("could not find nearest ref point.");
    return false;
  }
  LOG_INFO(
      "closest left/right_bound, left/right_lane_bound, heading: {:.3f}, "
      "{:.3f}, {:.3f}, {:.3f}, {:.3f}",
      ref_pt.left_bound(), ref_pt.right_bound(), ref_pt.left_lane_bound(),
      ref_pt.right_lane_bound(), ref_pt.heading());

  double width_threshold = VehicleParam::Instance()->width() * 0.5;
  bool adc_on_refer_lane = ((inside_data.init_sl_point.l() >
                             ref_pt.left_lane_bound() - width_threshold) ||
                            (inside_data.init_sl_point.l() <
                             -ref_pt.right_lane_bound() + width_threshold))
                               ? false
                               : true;
  if (adc_on_refer_lane ||
      inside_data.lane_borrow_side == LaneBorrowContext::BorrowSide::None) {
    LOG_INFO(
        "adc_on_refer_lane: [{}] or lane_borrow_side_none: [{}], skip.",
        adc_on_refer_lane,
        inside_data.lane_borrow_side == LaneBorrowContext::BorrowSide::None);
    return false;
  }
  if (!ComputeObserveRegions(inside_data, ref_ptr)) {
    return false;
  }
  ComputeAttentionDynamicObstacles(task_info.decision_data(), ref_pt,
                                   inside_data.lane_borrow_side);

  return ComputeObserveRef(inside_data, bounds_info, ref_pt,
                           inside_data.lane_borrow_side, &observe_l,
                           &attention_dynamic_obstacles, &observe_type);
}

bool ObserveRefForLaneBorrow::ComputeObserveRegions(
    const InsidePlannerData& inside_data, ReferenceLinePtr ref_ptr) {
  const auto& borrow_config = observe_ref_config().lane_borrow;
  borrow_attention_s_vec_.clear();
  for (std::size_t i = 0; i < ref_ptr->ref_points().size(); i += 10) {
    const auto& point = ref_ptr->ref_points()[i];
    if (point.s() < inside_data.init_sl_point.s() -
                        borrow_config.lane_borrow_back_attention_dis) {
      continue;
    }
    if (point.s() > inside_data.init_sl_point.s() +
                        borrow_config.lane_borrow_front_attention_dis) {
      break;
    }
    borrow_attention_s_vec_.push_back(std::make_pair(
        point.s(),
        std::make_pair(point.left_lane_bound(), point.right_lane_bound())));
  }

  attention_lane_bounds_.clear();
  for (std::size_t i = 0; i + 1 < borrow_attention_s_vec_.size(); ++i) {
    std::vector<Vec2d> aabox2d_points{
        {borrow_attention_s_vec_[i].first,
         borrow_attention_s_vec_[i].second.first},
        {borrow_attention_s_vec_[i + 1].first,
         borrow_attention_s_vec_[i + 1].second.first},
        {borrow_attention_s_vec_[i + 1].first,
         -borrow_attention_s_vec_[i + 1].second.second},
        {borrow_attention_s_vec_[i].first,
         -borrow_attention_s_vec_[i].second.second}};
    attention_lane_bounds_.push_back(AABox2d(aabox2d_points));
  }

  return !attention_lane_bounds_.empty();
}

void ObserveRefForLaneBorrow::ComputeAttentionDynamicObstacles(
    const std::shared_ptr<DecisionData>& decision_data_ptr,
    const ReferencePoint& ref_pt,
    const LaneBorrowContext::BorrowSide& borrow_side) {
  if (decision_data_ptr == nullptr) return;

  const auto& borrow_config = observe_ref_config().lane_borrow;
  attention_dynamic_obstacles_.clear();
  for (const auto& obs : decision_data_ptr->dynamic_obstacle()) {
    double heading_diff =
        std::abs(normalize_angle(obs->velocity_heading() - ref_pt.heading()));

    if (obs->speed() > borrow_config.lane_borrow_attention_dynamic_speed) {
      continue;
    }
    if (heading_diff > borrow_config.dynamic_obs_heading_diff) {
      continue;
    }
    if (obs->PolygonBoundary().start_s() -
            borrow_config.lane_borrow_front_attention_dis >
        adc_original_boundary_.end_s()) {
      continue;
    }
    if (obs->PolygonBoundary().end_s() +
            borrow_config.lane_borrow_back_attention_dis <
        adc_original_boundary_.start_s()) {
      continue;
    }
    if (obs->PolygonBoundary().start_l() -
            borrow_config.dynamic_obs_lateral_attention_dis >
        adc_original_boundary_.end_l()) {
      continue;
    }
    if (obs->PolygonBoundary().end_l() +
            borrow_config.dynamic_obs_lateral_attention_dis <
        adc_original_boundary_.start_l()) {
      continue;
    }
    if (borrow_side == LaneBorrowContext::BorrowSide::Left &&
        obs->PolygonBoundary().start_l() > adc_original_boundary_.start_l()) {
      continue;
    }
    if (borrow_side == LaneBorrowContext::BorrowSide::Right &&
        obs->PolygonBoundary().end_l() < adc_original_boundary_.end_l()) {
      continue;
    }
    std::vector<Vec2d> aabox2d_points{
        {obs->PolygonBoundary().start_s(), obs->PolygonBoundary().start_l()},
        {obs->PolygonBoundary().end_s(), obs->PolygonBoundary().start_l()},
        {obs->PolygonBoundary().end_s(), obs->PolygonBoundary().end_l()},
        {obs->PolygonBoundary().start_s(), obs->PolygonBoundary().end_l()}};
    AABox2d box(aabox2d_points);
    box.set_id(obs->id());

    for (const auto& bound : attention_lane_bounds_) {
      if (box.has_overlap(bound)) {
        attention_dynamic_obstacles_.push_back(*obs);
        LOG_INFO(
            "attention_dynamic_obs: id[{}], h_diff: {:.3f}, v: {:.3f}, s_s: "
            "{:.3f}, e_s: {:.3f}, s_l: "
            "{:.3f}, e_l: {:.3f}",
            obs->id(), heading_diff, obs->speed(),
            obs->PolygonBoundary().start_s(), obs->PolygonBoundary().end_s(),
            obs->PolygonBoundary().start_l(), obs->PolygonBoundary().end_l());
        break;
      }
    }
  }
  LOG_INFO("attention_dynamic_obstacles size: {}",
           attention_dynamic_obstacles_.size());
}

bool ObserveRefForLaneBorrow::ComputeObserveRef(
    const InsidePlannerData& inside_data,
    const std::vector<PathRegion::Bound>& bounds_info,
    const ReferencePoint& ref_pt,
    const LaneBorrowContext::BorrowSide& borrow_side, double* observe_l,
    std::vector<Obstacle>* attention_dynamic_obstacles,
    PathObserveRefLInfo::RefLType* observe_type) {
  // left bound min value; right bound max value
  double min_left_static_bound{1000.}, max_right_static_bound{-1000.};
  for (const auto& bound_info : bounds_info) {
    min_left_static_bound =
        std::min(min_left_static_bound, bound_info.upper_point.l());
    max_right_static_bound =
        std::max(max_right_static_bound, bound_info.lower_point.l());
  }
  LOG_INFO("min_left_static_bound: {:.3f}, max_right_static_bound: {:.3f}",
           min_left_static_bound, max_right_static_bound);

  // freespace left_min_bound or right_max_bound
  double min_left_freespace_bound{1000.}, max_right_freespace_bound{-1000.};
  const auto& freespace_upper_bound =
      DataCenter::Instance()->frenet_upper_bound();
  for (const auto& bound : freespace_upper_bound) {
    min_left_freespace_bound = (min_left_freespace_bound > bound.at(1))
                                   ? bound.at(1)
                                   : min_left_freespace_bound;
  }
  const auto& freespace_lower_bound =
      DataCenter::Instance()->frenet_lower_bound();
  for (const auto& bound : freespace_lower_bound) {
    max_right_freespace_bound = (max_right_freespace_bound < bound.at(1))
                                    ? bound.at(1)
                                    : max_right_freespace_bound;
  }
  LOG_INFO(
      "min_left_freespace_bound: {:.3f}, max_right_freespace_bound: {:.3f}",
      min_left_freespace_bound, max_right_freespace_bound);

  // dynamic static min or max value
  double max_left_dynamic_bound{-1000.}, min_right_dynamic_bound{1000.};
  for (const auto& box : attention_dynamic_obstacles_) {
    max_left_dynamic_bound = std::max(max_left_dynamic_bound, box.max_l());
    min_right_dynamic_bound = std::min(min_right_dynamic_bound, box.min_l());
  }
  LOG_INFO("max_left_dynamic_bound: {:.3f}, min_right_dynamic_bound: {:.3f}",
           max_left_dynamic_bound, min_right_dynamic_bound);

  double attention_space_value =
      (borrow_side == LaneBorrowContext::BorrowSide::Left)
          ? std::max(
                std::max(max_right_static_bound, max_right_freespace_bound),
                max_left_dynamic_bound)
          : std::min(std::min(min_left_static_bound, min_left_freespace_bound),
                     min_right_dynamic_bound);
  LOG_INFO("attention_space_value: {:.3f}", attention_space_value);

  double lane_borrow_width_threshold =
      observe_ref_config().lane_borrow.lane_borrow_width_threshold;
  bool observe_dynamic_obstacles =
      (borrow_side == LaneBorrowContext::BorrowSide::Left)
          ? (ref_pt.left_lane_bound() - attention_space_value <
             lane_borrow_width_threshold)
          : (attention_space_value - ref_pt.right_lane_bound() <
             lane_borrow_width_threshold);
  LOG_INFO("observe_dynamic_obstacles: {}", observe_dynamic_obstacles);
  if (!observe_dynamic_obstacles) return false;

  // decision
  double curr_observe_l =
      (borrow_side == LaneBorrowContext::BorrowSide::Left)
          ? attention_space_value +
                observe_ref_config()
                    .lane_borrow.lane_borrow_observe_l_safe_width
          : attention_space_value -
                observe_ref_config()
                    .lane_borrow.lane_borrow_observe_l_safe_width;
  LOG_INFO("last observe_l: {:.3f}, curr_observe_l: {:.3f}", *observe_l,
           curr_observe_l);
  const double& k =
      observe_ref_config().lane_borrow.lane_borrow_observe_l_filter_ratio;
  *observe_l = *observe_l * (1.0 - k) + k * curr_observe_l;

  *attention_dynamic_obstacles = attention_dynamic_obstacles_;
  *observe_type = PathObserveRefLInfo::RefLType::LANEBORROW;

  LOG_INFO("observe_l: {:.3f}, BORROW", *observe_l);

  return true;
}

}  // namespace planning
}  // namespace neodrive

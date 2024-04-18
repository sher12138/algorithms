#include "path_bound_shrinker_decider.h"

#include "common/math/util.h"
#include "math/curve1d/spline.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/proxy/proxy_type.h"
using neodrive::global::perception::camera::LaneLineType;

namespace neodrive {
namespace planning {

namespace {}  // namespace

PathBoundShrinkerDecider::PathBoundShrinkerDecider() {
  name_ = "PathBoundShrinkerDecider";
}

PathBoundShrinkerDecider::~PathBoundShrinkerDecider() { Reset(); }

ErrorCode PathBoundShrinkerDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  path_shrink_config_ = config::PlanningConfig::Instance()
                            ->planning_research_config()
                            .path_road_graph_config.path_shrink;
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  const auto& inside_data_ = task_info.current_frame()->inside_planner_data();
  auto reference_line = task_info.reference_line();
  const auto& bounds_info =
      outside_data->path_context.original_path_boundary.path_boundary;

  const auto& camera_lanes =
      data_center_->environment().perception_lanes_proxy().camera_lanes_line();
  VisShrink(reference_line, camera_lanes, "camera_lanes");
  SetCurbLines(reference_line, task_info);
  ShrinkByCurbLane(task_info);
  VisShrink(reference_line, path_boundary_, "ShrinkByCurbLane");

  if (!ShrinkByEgoVehicleHalfWidth(task_info)) {
    LOG_ERROR("shrink bounds failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  auto& shrink_path_boundries_first = outside_data->shrink_half_ego_boundaries;
  shrink_path_boundries_first = shrink_bounds_info_;

  VisShrink(reference_line, shrink_bounds_info_, "ShrinkByEgoVehicleHalfWidth");

  ShrinkByFullSpeed(task_info);
  VisShrink(reference_line, shrink_bounds_info_, "ShrinkByFullSpeed");

  ShrinkByKappa(task_info);
  VisShrink(reference_line, shrink_bounds_info_, "ShrinkByKappa");

  // ShrinkByEgoVehicle(task_info);
  // VisShrink(reference_line, shrink_bounds_info_, "ShrinkByEgoVehicle");

  ShrinkFinalCheck(task_info);

  std::vector<Vec2d> shrink_right_xy_boundary{};
  std::vector<Vec2d> shrink_left_xy_boundary{};
  for (const auto& bound : shrink_bounds_info_) {
    Vec2d right_xy_point{};
    if (!reference_line->GetPointInCartesianFrame(
            {bound.lower_point.s(), bound.lower_point.l()}, &right_xy_point)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    Vec2d left_xy_point{};
    if (!reference_line->GetPointInCartesianFrame(
            {bound.upper_point.s(), bound.upper_point.l()}, &left_xy_point)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    shrink_right_xy_boundary.emplace_back(right_xy_point);
    shrink_left_xy_boundary.emplace_back(left_xy_point);
  }
  VisShrink(shrink_left_xy_boundary, shrink_right_xy_boundary, "shrink pts");
  outside_data->path_context.shrink_path_boundary.left_xy_boundary =
      std::move(shrink_left_xy_boundary);
  outside_data->path_context.shrink_path_boundary.right_xy_boundary =
      std::move(shrink_right_xy_boundary);

  // TEST
  LOG_DEBUG("shrink_bounds res:");
  for (std::size_t i = 0; i < shrink_bounds_info_.size(); ++i) {
    LOG_DEBUG(
        "l_type[{}], r_type[{}], orig: [{:.3f}, {:.3f}], res[{:.3f}, {:.3f}]",
        int(bounds_info[i].upper_type), int(bounds_info[i].lower_type),
        bounds_info[i].upper_point.l(), bounds_info[i].lower_point.l(),
        shrink_bounds_info_[i].upper_point.l(),
        shrink_bounds_info_[i].lower_point.l());
  }
  outside_data->path_context.shrink_path_boundary.path_boundary =
      std::move(shrink_bounds_info_);

  /// compute lat_boundaries
  auto& lat_boundaries = outside_data->road_obs_path_shrink_boundries;
  lat_boundaries.clear();
  for (const auto& bound :
       outside_data->path_context.shrink_path_boundary.path_boundary) {
    PieceBoundary bound_infos{
        .left_is_obs = bound.upper_id >= 0,
        .right_is_obs = bound.lower_id >= 0,
        .s = bound.lower_point.s(),
        .left_bound = bound.upper_point.l(),
        .right_bound = bound.lower_point.l(),
        .left_lane_bound = bound.upper_lane_bound_point.l(),
        .right_lane_bound = bound.lower_lane_bound_point.l(),
        .left_road_bound = bound.upper_road_bound_point.l(),
        .right_road_bound = bound.lower_road_bound_point.l()};
    lat_boundaries.emplace_back(bound_infos);
  }

  return ErrorCode::PLANNING_OK;
}

void PathBoundShrinkerDecider::ShrinkByCurbLane(TaskInfo& task_info) {
  path_boundary_ = task_info.current_frame()
                       ->outside_planner_data()
                       .path_context.original_path_boundary.path_boundary;
  if (path_boundary_.empty()) {
    LOG_INFO("path_boundary is empty");
    return;
  }

  if (!config::PlanningConfig::Instance()
           ->plan_config()
           .common.enable_camera_lines) {
    LOG_INFO("config not ShrinkByCurbLane");
    return;
  }

  if (task_info.current_frame()->inside_planner_data().is_in_the_park) {
    LOG_INFO("Not enable camera lines in the park");
    return;
  }

  double overlap_start_s = task_info.curr_sl().s() + 100.0;
  if (!config::PlanningConfig::Instance()
           ->plan_config()
           .common.enable_camera_lines_in_junction) {
    LOG_INFO("not enable_camera_lines in junction");
    const auto& junction_list = task_info.reference_line()->junctions();
    for (const auto& [junction_ptr, overlap] : junction_list) {
      if (!junction_ptr) {
        continue;
      }
      if (task_info.curr_sl().s() > overlap.start_s - 100.0 &&
          task_info.curr_sl().s() < overlap.end_s) {
        overlap_start_s = overlap.start_s;
        LOG_INFO("junction found, overlap_start_s: {:.4f}", overlap_start_s);
        break;
      }
    }
  }

  LOG_INFO("right_curb_lane_ size: {}, path_boundary_ size: {}",
           right_curb_lane_.size(), path_boundary_.size());
  size_t i = 0, j = 0;
  while (i < right_curb_lane_.size() && j < path_boundary_.size()) {
    LOG_DEBUG(
        "i: {}, j: {}, right_curb_lane_ s: {:.4f}, l: {:.4f}, lower_point s: "
        "{:.4f}, l: {:.4f}",
        i, j, right_curb_lane_[i].s(), right_curb_lane_[i].l(),
        path_boundary_[j].lower_point.s(), path_boundary_[j].lower_point.l());
    if (right_curb_lane_[i].s() > overlap_start_s) {
      LOG_INFO("right_curb_lane_[i].s: {:.4f}, overlap_start_s: {:.4f}",
               right_curb_lane_[i].s(), overlap_start_s);
      break;
    }
    if (path_boundary_[j].lower_point.s() < right_curb_lane_[i].s()) {
      if (path_boundary_[j].lower_point.l() - right_curb_lane_[i].l() <
          kMathEpsilon) {
        path_boundary_[j].lower_point.set_l(right_curb_lane_[i].l());
      }
      ++j;
    } else {
      ++i;
    }
  }

  LOG_INFO("left_curb_lane_ size: {}, path_boundary_ size: {}",
           left_curb_lane_.size(), path_boundary_.size());
  i = 0;
  j = 0;
  while (i < left_curb_lane_.size() && j < path_boundary_.size()) {
    LOG_DEBUG(
        "i: {}, j: {}, left_curb_lane s: {:.4f}, l: {:.4f}, upper_point s: "
        "{:.4f}, l: {:.4f}",
        i, j, left_curb_lane_[i].s(), left_curb_lane_[i].l(),
        path_boundary_[j].upper_point.s(), path_boundary_[j].upper_point.l());
    if (left_curb_lane_[i].s() > overlap_start_s) {
      LOG_INFO("left_curb_lane_[i].s: {:.4f}, overlap_start_s: {:.4f}",
               left_curb_lane_[i].s(), overlap_start_s);
      break;
    }
    if (path_boundary_[j].upper_point.s() < left_curb_lane_[i].s()) {
      if (path_boundary_[j].upper_point.l() - left_curb_lane_[i].l() >
          kMathEpsilon) {
        path_boundary_[j].upper_point.set_l(left_curb_lane_[i].l());
      }
      ++j;
    } else {
      ++i;
    }
  }
}

bool PathBoundShrinkerDecider::ShrinkByEgoVehicleHalfWidth(
    TaskInfo& task_info) {
  half_car_width_ = VehicleParam::Instance()->width() / 2.;
  shrink_bounds_info_.clear();
  for (const auto& bound_info : path_boundary_) {
    double left_bound = bound_info.upper_point.l() - half_car_width_;
    double right_bound = bound_info.lower_point.l() + half_car_width_;
    if (right_bound - left_bound > kMathEpsilon) {
      break;
    }
    shrink_bounds_info_.emplace_back(bound_info);
    shrink_bounds_info_.back().lower_point.set_l(right_bound);
    shrink_bounds_info_.back().upper_point.set_l(left_bound);
  }
  return !shrink_bounds_info_.empty();
}

void PathBoundShrinkerDecider::ShrinkByKappa(TaskInfo& task_info) {
  double length = path_shrink_config_.kappa_length;
  auto calculate_buff = [&length](double kappa) {
    double r1 = std::abs(1.0 / kappa);
    double r2 = r1 + VehicleParam::Instance()->width();
    return std::sqrt(length * length + r2 * r2) - r2;
  };

  auto& reference_line = task_info.reference_line();
  ReferencePoint reference_point;
  for (auto& bound_info : shrink_bounds_info_) {
    reference_line->GetNearestRefPoint(bound_info.lower_point.s(),
                                       &reference_point);
    LOG_DEBUG("kappa: {:.4f}", reference_point.kappa());
    if (std::abs(reference_point.kappa()) < 0.001) continue;
    double buff = calculate_buff(reference_point.kappa());
    LOG_DEBUG("buff: {:.4f}", buff);
    if (buff < kMathEpsilon) {
      continue;
    }
    if (reference_point.kappa() > 0.0) {
      bound_info.lower_point.set_l(std::min(bound_info.lower_point.l() + buff,
                                            bound_info.upper_point.l() - 0.1));
    } else {
      bound_info.upper_point.set_l(std::max(bound_info.upper_point.l() - buff,
                                            bound_info.lower_point.l() + 0.1));
    }
  }
}

void PathBoundShrinkerDecider::ShrinkByHeading(TaskInfo& task_info) {
  auto& reference_line = task_info.reference_line();
  auto& adc_boundary = task_info.adc_boundary();
  task_info.current_frame()->inside_planner_data().init_point.angle();
  for (auto& bound_info : shrink_bounds_info_) {
    if (bound_info.lower_point.s() >
        adc_boundary.start_s() + VehicleParam::Instance()->length() * 0.5)
      break;
    if (bound_info.lower_point.s() < adc_boundary.start_s()) continue;

    ReferencePoint reference_point;
    reference_line->GetNearestRefPoint(bound_info.lower_point.s(),
                                       &reference_point);
    double delt_heading =
        reference_point.heading() -
        task_info.current_frame()->inside_planner_data().init_point.theta();
    LOG_DEBUG(
        "XXXdelt_heading: {:.4f}, ref heading: {:.4f}, init_point theta: "
        "{:.4f}, adc_point theta: {:.4f}",
        delt_heading, reference_point.heading(),
        task_info.current_frame()->inside_planner_data().init_point.theta(),
        task_info.adc_point().theta());
    if (std::abs(delt_heading) < 0.01) continue;

    double buff = VehicleParam::Instance()->length() *
                      std::sin(std::min(std::abs(delt_heading), M_PI / 4.0)) +
                  VehicleParam::Instance()->width() / 2.0 *
                      (std::cos(delt_heading) - 1.0);
    LOG_DEBUG("buff: {:.4f}", buff);

    if (buff < kMathEpsilon) {
      continue;
    }
    if (delt_heading > 0) {
      bound_info.lower_point.set_l(bound_info.lower_point.l() + buff);
    } else {
      bound_info.upper_point.set_l(bound_info.upper_point.l() - buff);
    }
  }
}

void PathBoundShrinkerDecider::ShrinkByFullSpeed(TaskInfo& task_info) {
  // 设计思路:
  // 1. 车道缩减: 考虑 Location、Map、Control 的误差分别为:
  // Location + Map + Control -> 30cm
  // 2. 障碍物缩减: 障碍物最小误差 25cm，最大误差 40cm

  const double middle_velo = FLAGS_planning_piecewise_middle_speed;
  const auto velo = task_info.current_frame()->inside_planner_data().vel_v;
  longi_dis_thresh_ = velo > middle_velo ? 3.0 : 2.0;

  enum class ShrinkType { Zero = 0, Single, Double };
  double narrow_mode_left_l{shrink_bounds_info_.front().upper_point.l()};
  double narrow_mode_right_l{shrink_bounds_info_.front().lower_point.l()};
  for (std::size_t i = 1; i < shrink_bounds_info_.size(); ++i) {
    // 1. closed ego vehicle shrink should by other ways
    if ((shrink_bounds_info_[i].lower_point.s() -
         shrink_bounds_info_.front().lower_point.s()) <= longi_dis_thresh_)
      continue;

    // 2. original bound and id
    const double origin_left_bound = shrink_bounds_info_[i].upper_point.l();
    const double origin_right_bound = shrink_bounds_info_[i].lower_point.l();
    const auto left_id = shrink_bounds_info_[i].upper_id;
    const auto right_id = shrink_bounds_info_[i].lower_id;
    const auto upper_type = shrink_bounds_info_[i].upper_type;
    const auto lower_type = shrink_bounds_info_[i].lower_type;

    // 3. shrink
    double left_bound{origin_left_bound}, right_bound{origin_right_bound};
    // 3.1 double lane
    ShrinkType shrink_type = ShrinkType::Double;
    if (upper_type == PathRegion::Bound::BoundType::LANE) {
      shrink_type = lower_type == PathRegion::Bound::BoundType::LANE
                        ? ShrinkType::Zero
                        : ShrinkType::Single;
    } else if (upper_type == PathRegion::Bound::BoundType::ROAD) {
      shrink_type = (lower_type == PathRegion::Bound::BoundType::OBS ||
                     lower_type == PathRegion::Bound::BoundType::ROAD)
                        ? ShrinkType::Double
                        : ShrinkType::Single;
    } else if (upper_type == PathRegion::Bound::BoundType::OBS) {
      shrink_type = (lower_type == PathRegion::Bound::BoundType::OBS ||
                     lower_type == PathRegion::Bound::BoundType::ROAD)
                        ? ShrinkType::Double
                        : ShrinkType::Single;
    }

    bool is_narrow = false;
    switch (shrink_type) {
      case ShrinkType::Double:
        is_narrow = !ShrinkDoubleLane(origin_left_bound, origin_right_bound,
                                      &left_bound, &right_bound);
        break;
      case ShrinkType::Single:
        is_narrow = !ShrinkSingleLane(
            task_info, origin_left_bound, origin_right_bound, left_id, right_id,
            upper_type, lower_type, &left_bound, &right_bound);
        break;
      case ShrinkType::Zero:
        is_narrow =
            !ShrinkZeroLane(task_info, origin_left_bound, origin_right_bound,
                            upper_type, lower_type, &left_bound, &right_bound);
        break;
      default:
        LOG_ERROR("invalid shrink type.");
        break;
    }
    LOG_DEBUG(
        "i, l_id, r_id, l_t, r_t, d: {}, {}, {}, {}, {}, {}, {:.3f}, {:.3f}, "
        "{:.3f}, {:.3f}",
        i, left_id, right_id, int(upper_type), int(lower_type),
        int(shrink_type), origin_left_bound, left_bound, origin_right_bound,
        right_bound);

    narrow_mode_left_l = left_bound;
    narrow_mode_right_l = right_bound;

    // 4. narrow mode
    if (is_narrow) {
      left_bound = narrow_mode_left_l;
      right_bound = narrow_mode_right_l;
      LOG_WARN("index: {} is narrow mode", i);
    }

    if (left_bound < right_bound + kDeltL / 5.0) {
      double max_bound = std::max(left_bound, right_bound);
      double min_bound = std::min(left_bound, right_bound);
      left_bound = max_bound + kDeltL / 5.;
      right_bound = min_bound - kDeltL / 5.;
    }
    shrink_bounds_info_[i].upper_point.set_l(left_bound);
    shrink_bounds_info_[i].lower_point.set_l(right_bound);
  }
}

bool PathBoundShrinkerDecider::ShrinkDoubleLane(const double origin_left_bound,
                                                const double origin_right_bound,
                                                double* left_bound,
                                                double* right_bound) const {
  const double tmp_width = origin_left_bound - origin_right_bound;
  if (tmp_width + kMathEpsilon < kDeltL) {
    LOG_INFO("exit shrink bound, tmp_width < kDeltL.");
    *left_bound = origin_left_bound - kDeltL / 3.;
    *right_bound = origin_right_bound + kDeltL / 3.;
    return false;
  }
  double shrink_lane_bound = std::fmin(tmp_width * kDoubleLaneOffsetCoef,
                                       path_shrink_config_.min_shrink_dis);
  *left_bound = origin_left_bound - shrink_lane_bound;
  *right_bound = origin_right_bound + shrink_lane_bound;
  return true;
}

bool PathBoundShrinkerDecider::ShrinkSingleLane(
    TaskInfo& task_info, const double origin_left_bound,
    const double origin_right_bound, const int left_id, const int right_id,
    const PathRegion::Bound::BoundType& upper_type,
    const PathRegion::Bound::BoundType& lower_type, double* left_bound,
    double* right_bound) const {
  const double tmp_width = origin_left_bound - origin_right_bound;
  if (tmp_width + kMathEpsilon < kDeltL) {
    LOG_INFO("exit shrink bound, tmp_width < kDeltL");
    *left_bound = origin_left_bound - kDeltL / 3.;
    *right_bound = origin_right_bound + kDeltL / 3.;
    return false;
  }

  double obs_dis =
      task_info.current_frame()->inside_planner_data().is_lane_borrowing
          ? kObsMeanDis
          : kObsMinDis;
  double obs_shrink_dis =
      (tmp_width > obs_dis) ? obs_dis : tmp_width * kObsOffsetCoef;
  double lane_shrink_dis =
      (tmp_width - obs_shrink_dis < kDeltL + path_shrink_config_.min_shrink_dis)
          ? (tmp_width - obs_shrink_dis) * kSingleLaneOffsetCoef
          : path_shrink_config_.min_shrink_dis;

  double shrink_dis = std::min(lane_shrink_dis, obs_shrink_dis);
  if (upper_type == PathRegion::Bound::BoundType::OBS ||
      upper_type == PathRegion::Bound::BoundType::ROAD) {
    *left_bound = origin_left_bound - shrink_dis;
    *right_bound = origin_right_bound + kDeltL / 3.;
  } else {
    *left_bound = origin_left_bound - kDeltL / 3.;
    *right_bound = origin_right_bound + shrink_dis;
  }
  return true;
}

bool PathBoundShrinkerDecider::ShrinkZeroLane(
    TaskInfo& task_info, const double origin_left_bound,
    const double origin_right_bound,
    const PathRegion::Bound::BoundType& upper_type,
    const PathRegion::Bound::BoundType& lower_type, double* left_bound,
    double* right_bound) const {
  const double tmp_width = origin_left_bound - origin_right_bound;
  if (tmp_width + kMathEpsilon < kDeltL) {
    LOG_INFO("exit shrink bound, tmp_width < kDeltL");
    *left_bound = origin_left_bound - kDeltL / 3.;
    *right_bound = origin_right_bound + kDeltL / 3.;
    return false;
  }

  *left_bound = origin_left_bound - kDeltL / 3.;
  *right_bound = origin_right_bound + kDeltL / 3.;
  return true;
}

void PathBoundShrinkerDecider::ShrinkByEgoVehicle(TaskInfo& task_info) {
  // 2. cal ego vehicle box xy point
  double lateral_buffer{0.0};
  double front_buffer{0.0};
  double end_buffer{0.0};
  const auto& init_point =
      task_info.current_frame()->inside_planner_data().init_point;
  const auto adc_bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
      {init_point.x(), init_point.y()}, init_point.theta(), lateral_buffer,
      front_buffer, end_buffer);
  std::vector<Vec2d> adc_box_xy_pts{};
  adc_bounding_box.get_all_corners(&adc_box_xy_pts);

  const auto& reference_point =
      task_info.current_frame()->outside_planner_data().init_point_ref_point;
  double loc_x{0.0}, loc_y{0.0}, loc_theta{0.0};
  std::vector<Vec2d> adc_box_xy_pts_loc;
  // TEST
  LOG_DEBUG("adc_box_xy_pts_loc:");
  for (auto pt : adc_box_xy_pts) {
    if (!earth2vehicle(reference_point.x(), reference_point.y(),
                       reference_point.heading(), pt.x(), pt.y(),
                       init_point.theta(), loc_x, loc_y, loc_theta)) {
      LOG_INFO("earth2vehicle err!");
    }
    double revise_s = reference_point.s() + loc_x;
    double revise_l = loc_y;
    Vec2d tmp_pt(revise_s, revise_l);
    adc_box_xy_pts_loc.push_back(tmp_pt);
    // TEST
    LOG_DEBUG("revise_s {:.3f}, revise_l {:.3f}", revise_s, revise_l);
  }
  Vec2d right_front_pt_xy = adc_box_xy_pts_loc[0];
  Vec2d left_front_pt_xy = adc_box_xy_pts_loc[1];
  Vec2d left_back_pt_xy = adc_box_xy_pts_loc[2];
  Vec2d right_back_pt_xy = adc_box_xy_pts_loc[3];

  // 3. expand
  const double bound_expand_buffer =
      FLAGS_planning_road_border_remain_threshold;
  const std::size_t SKIP = 0;
  // 3.1 right side
  double delta_x = right_front_pt_xy.x() - right_back_pt_xy.x();
  double delta_y = right_front_pt_xy.y() - right_back_pt_xy.y();
  // TEST
  LOG_DEBUG("right line xy delta y {:.3f} / delta x {:.3f} = {:.3f}", delta_y,
            delta_x, delta_y / delta_x);
  for (std::size_t i = 0; i < shrink_bounds_info_.size(); ++i) {
    if (shrink_bounds_info_[i].lower_point.s() < right_back_pt_xy.x()) continue;
    if (i >= SKIP &&
        shrink_bounds_info_[i - SKIP].lower_point.s() > right_front_pt_xy.x())
      break;
    double right_l =
        right_back_pt_xy.y() +
        delta_y / delta_x *
            (shrink_bounds_info_[i].lower_point.s() - right_back_pt_xy.x());
    double l =
        (right_l > (kMathEpsilon + shrink_bounds_info_[i].lower_point.l()))
            ? shrink_bounds_info_[i].lower_point.l()
            : right_l;

    // TEST
    LOG_DEBUG("xy i {}, right bound {:.3f} change into {:.3f}, {:.3f}", i,
              shrink_bounds_info_[i].lower_point.l(), right_l, l);

    shrink_bounds_info_[i].lower_point.set_l(l);
  }

  // 3.2 left side.
  delta_x = left_front_pt_xy.x() - left_back_pt_xy.x();
  delta_y = left_front_pt_xy.y() - left_back_pt_xy.y();
  // TEST
  LOG_DEBUG("left line xy delta y {:.3f} / delta x {:.3f} = {:.3f}", delta_y,
            delta_x, delta_y / delta_x);
  for (std::size_t i = 0; i < shrink_bounds_info_.size(); ++i) {
    if (shrink_bounds_info_[i].upper_point.s() < left_back_pt_xy.x()) continue;
    if (i >= SKIP &&
        shrink_bounds_info_[i - SKIP].upper_point.s() > left_front_pt_xy.x())
      break;
    double left_l =
        left_back_pt_xy.y() +
        delta_y / delta_x *
            (shrink_bounds_info_[i].upper_point.s() - left_back_pt_xy.x());
    double l =
        (left_l > (kMathEpsilon + shrink_bounds_info_[i].upper_point.l()))
            ? left_l
            : kMathEpsilon + shrink_bounds_info_[i].upper_point.l();
    // TEST
    LOG_INFO("xy i {}, left bound {:.3f} change into {:.3f}, {:.3f}", i,
             shrink_bounds_info_[i].upper_point.l(), left_l, l);
    shrink_bounds_info_[i].upper_point.set_l(l);
  }

  // 4. check
  for (std::size_t i = 0; i < shrink_bounds_info_.size(); ++i) {
    if (shrink_bounds_info_[i].lower_point.s() > right_front_pt_xy.x()) break;
    if (shrink_bounds_info_[i].lower_point.s() < right_back_pt_xy.x()) continue;
    if (shrink_bounds_info_[i].upper_point.l() -
            shrink_bounds_info_[i].lower_point.l() <
        1e-5) {
      LOG_ERROR(
          "i {} s {:.3f}, lat_boundaries ({:.3f}, {:.3f})"
          "corridor is too narrow!",
          i, shrink_bounds_info_[i].lower_point.s(),
          shrink_bounds_info_[i].upper_point.l(),
          shrink_bounds_info_[i].lower_point.l());
    }
  }
}

void PathBoundShrinkerDecider::VisShrink(
    const neodrive::planning::ReferenceLinePtr reference_line,
    const std::vector<neodrive::global::perception::camera::CameraLaneLine>&
        curb_lines,
    const std::string& name) {
  if (!FLAGS_planning_enable_vis_event || curb_lines.empty()) {
    LOG_INFO("curb_lines size: {}", curb_lines.size());
    return;
  }
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, const auto& p) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(p.x());
    sphere->mutable_center()->set_y(p.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.05);
  };

  for (auto& line : curb_lines) {
    if (line.type() != LaneLineType::ROAD_CURB) continue;
    for (auto& point : line.odometry_point_set()) {
      set_pts(event, point);
    }
  }
}

void PathBoundShrinkerDecider::SetCurbLines(
    const neodrive::planning::ReferenceLinePtr reference_line,
    TaskInfo& task_info) {
  const auto& perception_lanes =
      data_center_->environment().perception_lanes_proxy().camera_lanes_line();
  LOG_INFO("perception_lanes size: {}", perception_lanes.size());
  std::vector<Vec2d> left_interpolate_curbLine{};
  std::vector<Vec2d> right_interpolate_curbLine{};

  for (const auto& lane : perception_lanes) {
    LOG_INFO("lane type:{} , lane pos type:{} ", lane.type(), lane.pos_type());
    if (lane.type() != LaneLineType::ROAD_CURB) {
      continue;
    }
    if (lane.odometry_point_set_size() < 3) {
      LOG_INFO("odometry_point_set size < 3, do not check.");
      continue;
    }
    std::vector<Vec2d> curb_lane{};
    Vec2d start_point{lane.odometry_point_set(0).x(),
                      lane.odometry_point_set(0).y()};
    Vec2d end_point;

    for (int index = 1; index < lane.odometry_point_set_size(); index++) {
      const auto& odom_curb_point = lane.odometry_point_set(index);
      end_point.set_x(odom_curb_point.x());
      end_point.set_y(odom_curb_point.y());

      double length = std::sqrt((end_point.y() - start_point.y()));
      start_point.distance_sqr_to(end_point);
      double interpolate_length = 0.1;
      if (length < interpolate_length / 2.0) {
        continue;
      } else if (length > interpolate_length) {
        while (length > interpolate_length) {
          LOG_DEBUG("XXXlength: {:.4f}, i: {}", length, index);
          curb_lane.push_back(start_point);
          double heading = std::atan2(end_point.y() - start_point.y(),
                                      end_point.x() - start_point.x());

          Vec2d interpolate_point{
              start_point.x() + interpolate_length * std::cos(heading),
              start_point.y() + interpolate_length * std::sin(heading)};
          start_point.set_x(interpolate_point.x());
          start_point.set_y(interpolate_point.y());
          length = start_point.distance_sqr_to(end_point);
        }
      } else {
        curb_lane.push_back(start_point);
        start_point.set_x(end_point.x());
        start_point.set_y(end_point.y());
      }
    }
    if (lane.pos_type() == CameraLaneLinePositionType::EGO_LEFT) {
      left_interpolate_curbLine = curb_lane;
    }
    if (lane.pos_type() == CameraLaneLinePositionType::EGO_RIGHT) {
      right_interpolate_curbLine = curb_lane;
    }
  }
  if (left_interpolate_curbLine.empty()) {
    LOG_INFO("XXX left_curb_lane empty");
  }
  if (right_interpolate_curbLine.empty()) {
    LOG_INFO("XXX right_curb_lane empty");
  }
  VisShrink(left_interpolate_curbLine, right_interpolate_curbLine,
            "Interpolate CurbLines");
  left_curb_lane_.clear();
  right_curb_lane_.clear();
  for (auto& point : left_interpolate_curbLine) {
    SLPoint sl_pt{};
    reference_line->GetPointInFrenetFrame(point, &sl_pt);
    left_curb_lane_.push_back(sl_pt);
  }
  for (auto& point : right_interpolate_curbLine) {
    SLPoint sl_pt{};
    reference_line->GetPointInFrenetFrame(point, &sl_pt);
    right_curb_lane_.push_back(sl_pt);
  }
}

void PathBoundShrinkerDecider::VisShrink(
    const neodrive::planning::ReferenceLinePtr reference_line,
    const std::vector<neodrive::planning::PathRegion::Bound>&
        shrink_bounds_info,
    const std::string& name) {
  if (!FLAGS_planning_enable_vis_event || shrink_bounds_info.empty()) {
    LOG_INFO("shrink_bounds_info size: {}", shrink_bounds_info.size());
    return;
  }

  vis_shrink_right_xy_boundary_.clear();
  vis_shrink_left_xy_boundary_.clear();
  for (const auto& bound : shrink_bounds_info) {
    Vec2d right_xy_point{};
    if (!reference_line->GetPointInCartesianFrame(
            {bound.lower_point.s(), bound.lower_point.l()}, &right_xy_point)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    Vec2d left_xy_point{};
    if (!reference_line->GetPointInCartesianFrame(
            {bound.upper_point.s(), bound.upper_point.l()}, &left_xy_point)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    vis_shrink_right_xy_boundary_.emplace_back(
        std::make_pair(right_xy_point, bound.lower_type));
    vis_shrink_left_xy_boundary_.emplace_back(
        std::make_pair(left_xy_point, bound.upper_type));
  }
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pts) {
    for (auto& p : pts) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->mutable_center()->set_x(p.first.x());
      sphere->mutable_center()->set_y(p.first.y());
      sphere->mutable_center()->set_z(0);
      sphere->set_radius(0.05);

      auto text = event->mutable_text()->Add();
      text->mutable_position()->set_x(p.first.x());
      text->mutable_position()->set_y(p.first.y());
      text->mutable_position()->set_z(0);
      text->set_text(std::to_string(int(p.second)));
    }
  };

  set_pts(event, vis_shrink_right_xy_boundary_);
  set_pts(event, vis_shrink_left_xy_boundary_);
}

void PathBoundShrinkerDecider::VisShrink(const std::vector<Vec2d>& left,
                                         const std::vector<Vec2d>& right,
                                         const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pts) {
    for (auto& p : pts) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->mutable_center()->set_x(p.x());
      sphere->mutable_center()->set_y(p.y());
      sphere->mutable_center()->set_z(0);
      sphere->set_radius(0.05);
    }
  };

  set_pts(event, left);
  set_pts(event, right);
}

void PathBoundShrinkerDecider::ShrinkFinalCheck(TaskInfo& task_info) {
  for (std::size_t i = 0; i < shrink_bounds_info_.size(); ++i) {
    double cur_left_bound = shrink_bounds_info_[i].upper_point.l(),
           cur_right_bound = shrink_bounds_info_[i].lower_point.l();
    if (cur_left_bound < cur_right_bound) {
      // cut off shrink
      shrink_bounds_info_.erase(shrink_bounds_info_.begin() + i,
                                shrink_bounds_info_.end());
      // update end_s
      auto outside_data =
          task_info.current_frame()->mutable_outside_planner_data();
      outside_data->path_context.valid_region_end_s =
          shrink_bounds_info_.back().upper_point.s();
      outside_data->path_context.valid_backup_end_s =
          shrink_bounds_info_.back().upper_point.s();
      return;
    }
  }
}

}  // namespace planning
}  // namespace neodrive

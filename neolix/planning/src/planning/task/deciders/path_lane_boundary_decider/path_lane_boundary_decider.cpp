#include "path_lane_boundary_decider.h"

namespace neodrive {
namespace planning {

PathLaneBoundaryDecider::PathLaneBoundaryDecider() {
  name_ = "PathLaneBoundaryDecider";
}

PathLaneBoundaryDecider::~PathLaneBoundaryDecider() { Reset(); }

ErrorCode PathLaneBoundaryDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!Init(task_info.current_frame()->mutable_outside_planner_data())) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool PathLaneBoundaryDecider::Init(OutsidePlannerData* const outside_data) {
  if (outside_data == nullptr) {
    LOG_ERROR("outside_data == nullptr.");
    return false;
  }

  return true;
}

bool PathLaneBoundaryDecider::Process(TaskInfo& task_info) {
  // 1. prefer_valid_length
  prefer_valid_length_ = std::fmax(
      std::fabs(task_info.current_frame()->inside_planner_data().vel_v * 4),
      6.0);
  LOG_INFO("prefer_valid_length: {:.3f}", prefer_valid_length_);

  if (!SampleLaneBoundary(
          task_info, kDeltaS,
          task_info.current_frame()->mutable_outside_planner_data())) {
    LOG_ERROR("SampleLaneBoundary failed.");
    return false;
  }

  return true;
}

void PathLaneBoundaryDecider::SaveTaskResults(TaskInfo& task_info) {
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();

  outside_data->path_context.prefer_valid_length = prefer_valid_length_;
}

bool PathLaneBoundaryDecider::DynamicDeltaS(
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data) const {
  const double low_velo = FLAGS_planning_piecewise_low_speed;
  const double middle_velo = FLAGS_planning_piecewise_middle_speed;
  const double middle_high_velo = FLAGS_planning_piecewise_middle_high_speed;
  const double high_velo = FLAGS_planning_piecewise_high_speed;
  const double high_speed_delta_s = FLAGS_planning_piecewise_high_speed_delta_s;
  const double low_speed_delta_s = FLAGS_planning_piecewise_low_speed_delta_s;
  double& delta_s = outside_data->path_context.piecewise_delta_s;
  if (inside_data.vel_v <= low_velo) {
    delta_s = low_speed_delta_s;
  } else if (inside_data.vel_v > low_velo && inside_data.vel_v <= middle_velo) {
    delta_s = low_speed_delta_s;
  } else if (inside_data.vel_v > middle_velo &&
             inside_data.vel_v <= middle_high_velo) {
    delta_s = high_speed_delta_s;
  } else if (inside_data.vel_v > middle_high_velo &&
             inside_data.vel_v <= high_velo) {
    delta_s = high_speed_delta_s;
  } else if (inside_data.vel_v > high_velo) {
    delta_s = high_speed_delta_s;
  } else {
    LOG_ERROR("out of design, err");
  }
  outside_data->path_context.piecewise_delta_s = delta_s;
  LOG_INFO("inside_data.vel_v {}, piecewise_delta_s {}", inside_data.vel_v,
           outside_data->path_context.piecewise_delta_s);
  return true;
}

bool PathLaneBoundaryDecider::SampleLaneBoundary(
    const TaskInfo& task_info, const double delta_s,
    OutsidePlannerData* const outside_data) const {
  const auto& reference_line = task_info.reference_line();
  const auto& init_sl_point =
      task_info.frame()->inside_planner_data().init_sl_point;
  const auto& vel_v = task_info.frame()->inside_planner_data().vel_v;
  double end_s = fmin(
      reference_line->ref_points().back().s(),
      init_sl_point.s() + fmax(vel_v * FLAGS_planning_trajectory_time_length,
                               FLAGS_planning_trajectory_min_length));
  ModifyExtendS(task_info, end_s);
  double start_s =
      fmax(init_sl_point.s(), reference_line->ref_points().front().s());
  LOG_INFO("get lat_boundary from start_s [{:.4f}] to end_s [{:.4f}]", start_s,
           end_s);
  if (start_s >= end_s - delta_s) {
    LOG_ERROR("start_s[{:.4f}] and end_s[{:.4f}] too close, invalid", start_s,
              end_s);
    return false;
  }

  outside_data->road_obs_path_boundries.clear();
  ReferencePoint tmp_point;
  PieceBoundary tmp_bound;
  for (; start_s < end_s - delta_s; start_s += delta_s) {
    if (!reference_line->GetNearestRefPoint(start_s, &tmp_point)) {
      LOG_ERROR("get reference point failed, s: {:.4f}", start_s);
      return false;
    }
    if (!outside_data->road_obs_path_boundries.empty() &&
        outside_data->road_obs_path_boundries.back().s >
            tmp_point.s() + kMathEpsilon) {
      continue;
    }
    tmp_bound.s = tmp_point.s();
    tmp_bound.left_bound = tmp_point.left_bound();
    tmp_bound.left_lane_bound = tmp_point.left_lane_bound();
    tmp_bound.left_road_bound = tmp_point.left_road_bound();
    tmp_bound.right_bound = -tmp_point.right_bound();
    tmp_bound.right_lane_bound = -tmp_point.right_lane_bound();
    tmp_bound.right_road_bound = -tmp_point.right_road_bound();
    outside_data->road_obs_path_boundries.emplace_back(tmp_bound);

    LOG_DEBUG(
        "s, l_r_b, l_l_b, r_l_b, r_r_b: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
        tmp_point.s(), tmp_point.left_road_bound(), tmp_point.left_lane_bound(),
        -tmp_point.right_lane_bound(), -tmp_point.right_road_bound());
  }
  reference_line->GetNearestRefPoint(end_s, &tmp_point);
  tmp_bound.s = tmp_point.s();
  tmp_bound.left_bound = tmp_point.left_bound();
  tmp_bound.left_lane_bound = tmp_point.left_lane_bound();
  tmp_bound.left_road_bound = tmp_point.left_road_bound();
  tmp_bound.right_bound = -tmp_point.right_bound();
  tmp_bound.right_lane_bound = -tmp_point.right_lane_bound();
  tmp_bound.right_road_bound = -tmp_point.right_road_bound();
  outside_data->road_obs_path_boundries.emplace_back(tmp_bound);
  return true;
}

void PathLaneBoundaryDecider::ModifyExtendS(const TaskInfo& task_info,
                                            double& end_s) const {
  auto& decision_data = task_info.decision_data();
  auto& reference_line = task_info.reference_line();
  std::vector<std::pair<double, double>> obstacle_segments;
  for (auto& obs_ptr : decision_data->static_obstacle()) {
    std::vector<Vec2d> tmp_corners;
    obs_ptr->bounding_box().get_all_corners(&tmp_corners);
    for (auto& corner_pt : tmp_corners) {
      SLPoint sl_pt{};
      if (!reference_line->GetPointInFrenetFrame({corner_pt.x(), corner_pt.y()},
                                                 &sl_pt)) {
        LOG_ERROR("get point to frenet coordinate failed");
        continue;
      }
      ReferencePoint tmp_refer_pt;
      reference_line->GetNearestRefPoint(sl_pt.s(), &tmp_refer_pt);
      if (sl_pt.l() < tmp_refer_pt.left_bound() &&
          sl_pt.l() > -tmp_refer_pt.right_bound()) {
        LOG_INFO("Obs [{}] is in bound", obs_ptr->id());
        obstacle_segments.push_back(
            std::make_pair(obs_ptr->PolygonBoundary().start_s(),
                           obs_ptr->PolygonBoundary().end_s()));
        break;
      }
    }
  }
  LOG_INFO("filter obstacle size: {}", obstacle_segments.size());
  std::sort(obstacle_segments.begin(), obstacle_segments.end(),
            [&](const auto& a, const auto& b) { return a.first < b.first; });
  std::vector<std::pair<double, double>> combined_obstacle_segments;
  for (size_t i = 0; i < obstacle_segments.size(); i++) {
    while (i < obstacle_segments.size() - 1 &&
           obstacle_segments[i + 1].first < obstacle_segments[i].second) {
      obstacle_segments[i + 1].first = obstacle_segments[i].first;
      obstacle_segments[i + 1].second = std::max(
          obstacle_segments[i].second, obstacle_segments[i + 1].second);
      i++;
    }
    combined_obstacle_segments.push_back(obstacle_segments[i]);
  }
  for (auto& obstacle_segment : combined_obstacle_segments) {
    if (end_s > obstacle_segment.first && end_s < obstacle_segment.second) {
      LOG_INFO("Extend planning end s from {:.4f} to {:.4f}", end_s,
               obstacle_segment.second);
      end_s = obstacle_segment.second;
      break;
    }
  }
  return;
}

}  // namespace planning
}  // namespace neodrive

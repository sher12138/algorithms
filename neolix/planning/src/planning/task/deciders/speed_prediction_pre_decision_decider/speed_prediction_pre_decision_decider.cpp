#include "speed_prediction_pre_decision_decider.h"

#include "planning_map/planning_map.h"
#include "reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {

namespace {

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using neodrive::global::prediction::Trajectory_PredictorType_FreeMove;
std::pair<double, double> protected_junction_{0.0, 0.0};
std::pair<double, double> unprotected_junction_{0.0, 0.0};

const config::AutoPlanningResearchConfig::
    SpeedPredictionPreDecisionDeciderConfig*
        speed_prediction_pre_decision_decider_config_{nullptr};
double adc_front_edge_s_{0.0};
Polygon2d adc_polygon_{};
std::unordered_set<int> obs_not_ignore_{};

bool ReverseIgnoreWithHeadingDiff(const Obstacle& obstacle,
                                  const double ego_heading) {
  return std::abs(normalize_angle(ego_heading - obstacle.velocity_heading())) >
         speed_prediction_pre_decision_decider_config_->reverse_heading_diff *
             M_PI;
}

bool ReverseIgnoreWithPose(const Obstacle& obstacle,
                           const std::array<double, 3>& ego_pose) {
  double min_x{std::numeric_limits<double>::infinity()};
  for (const auto& pt : obstacle.polygon().points()) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(ego_pose[0], ego_pose[1], ego_pose[2], pt.x(), pt.y(),
                  obstacle.velocity_heading(), obs_local_x, obs_local_y,
                  obs_local_heading);
    min_x = std::min(min_x, obs_local_x);
  }

  return min_x < VehicleParam::Instance()->front_edge_to_center();
}

bool ReverseIgnoreWithPolygon(const Obstacle& obstacle,
                              const Polygon2d& ego_polygon) {
  return !obstacle.polygon().has_overlap(ego_polygon);
}

bool ObsOnLeft(const Obstacle& obstacle,
               const std::array<double, 3>& ego_pose) {
  // all point of obs on left,then return true
  double min_y{std::numeric_limits<double>::infinity()};
  for (const auto& pt : obstacle.polygon().points()) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(ego_pose[0], ego_pose[1], ego_pose[2], pt.x(), pt.y(),
                  obstacle.velocity_heading(), obs_local_x, obs_local_y,
                  obs_local_heading);
    min_y = std::min(min_y, obs_local_y);
  }

  return min_y > VehicleParam::Instance()->width() * 0.5;
}

bool ObsOnRight(const Obstacle& obstacle,
                const std::array<double, 3>& ego_pose) {
  // any point of obs on right,then return true
  double max_y{std::numeric_limits<double>::infinity()};
  for (const auto& pt : obstacle.polygon().points()) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(ego_pose[0], ego_pose[1], ego_pose[2], pt.x(), pt.y(),
                  obstacle.velocity_heading(), obs_local_x, obs_local_y,
                  obs_local_heading);
    max_y = std::min(max_y, obs_local_y);
  }

  return max_y < -VehicleParam::Instance()->width() * 0.5;
}

std::unordered_set<size_t> GetRoadIdsExceptJunction(ReferenceLinePtr ref_line,
                                                    const double curr_s) {
  auto& rpts = ref_line->ref_points();
  auto idx = ref_line_util::BinarySearchIndex(rpts, curr_s);

  std::unordered_set<size_t> lane_ids{};
  while (idx < rpts.size() && rpts[idx].s() - curr_s < 100.0) {
    if (!rpts[idx].is_in_junction()) {
      lane_ids.insert(rpts[idx].hd_map_lane_id());
    }
    idx++;
  }

  std::unordered_set<size_t> ans{};
  for (auto id : lane_ids) {
    ans.insert(PlanningMap::Instance()->GetLaneRoadId(id));
  }

  return ans;
}  // namespace

bool GetRoadBound(const ReferenceLinePtr& reference_line,
                  const Boundary& boundary, double* left_bound,
                  double* right_bound) {
  if (left_bound == nullptr || right_bound == nullptr) return false;
  *left_bound = 100.0;
  *right_bound = -100.0;
  const auto& ref_points = reference_line->ref_points();
  auto bs_idx = [&ref_points](auto t) -> size_t {
    if (t < ref_points.front().s()) return 0;
    size_t m = ref_points.size();
    if (t > ref_points.back().s()) return m - 1;

    size_t l = 0, r = m - 1;
    while (l < r) {
      int mid = l + (r - l) / 2;
      if (ref_points[mid].s() < t) {
        l = mid + 1;
      } else {
        r = mid;
      }
    }
    return l;
  };
  std::size_t start_index = bs_idx(boundary.start_s());
  std::size_t end_index = bs_idx(boundary.end_s());
  for (std::size_t index = start_index; index <= end_index; ++index) {
    *left_bound = std::fmin(*left_bound, ref_points[index].left_road_bound());
    *right_bound =
        std::fmax(*right_bound, -ref_points[index].right_road_bound());
  }
  return true;
}

bool CheckIfObsInReferLineRoad(const ReferenceLinePtr& ref_line,
                               const Obstacle& obs) {
  const auto& obs_boundary = obs.PolygonBoundary();
  double left_road_bound{100.0};
  double right_road_bound{-100.0};
  GetRoadBound(ref_line, obs_boundary, &left_road_bound, &right_road_bound);
  LOG_INFO(
      "ref line road bound at obs [{}], left_road_bound "
      "right_road_bound: {:.3f}, {:.3f} ",
      left_road_bound, right_road_bound);
  return !(obs_boundary.start_l() > left_road_bound ||
           obs_boundary.end_l() < right_road_bound);
}

ErrorCode StraightRoadFilterErrPrediction(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  auto ego_roads = GetRoadIdsExceptJunction(task_info.reference_line(),
                                            task_info.curr_sl().s());

  if (ego_roads.empty()) return ErrorCode::PLANNING_OK;  // in junction
  std::string str_log = "Ego road id : ";
  for (auto id : ego_roads) {
    str_log += std::to_string(id) + ", ";
  }
  LOG_INFO("{}", str_log);

  auto& speed_obstacle_context = task_info.current_frame()
                                     ->mutable_outside_planner_data()
                                     ->speed_obstacle_context;
  using ObsType = Obstacle::ObstacleType;

  for (auto& decision : speed_obstacle_context.dynamic_obstacles_decision) {
    auto& obs = decision.obstacle;
    LOG_INFO(
        "obs {} collision {}, type veh {}, obs.prediction_trajectories "
        "size {}",
        obs.id(), decision.collide, obs.type() == ObsType::VEHICLE,
        obs.prediction_trajectories().size());
    if (!decision.collide) continue;
    if (auto& traj = obs.prediction_trajectories();
        !(obs.type() == ObsType::VEHICLE || obs.type() == ObsType::BICYCLE) ||
        traj.empty()) {
      continue;
    }

    auto IsInSRange = [](double s, std::pair<double, double> s_range,
                         double buffer) {
      buffer = buffer > 0.0 ? buffer : 0.0;
      return (s > s_range.first - buffer) && (s < s_range.second + buffer);
    };
    double safe_buffer = 0.5 * std::max(obs.length(), obs.width());

    LOG_INFO(
        "check if obs is in unprotected_junction_: obs_s {:.3f}, "
        "unprotected_junction_.start_s, end_s {:.3f}, {:.3f}",
        obs.center_sl().s(), unprotected_junction_.first,
        unprotected_junction_.second);
    if (IsInSRange(obs.center_sl().s(), unprotected_junction_, safe_buffer)) {
      LOG_INFO(
          "adc is not in junction, but obs [{}] is in unprotected junction, "
          "not ignore",
          obs.id());
      continue;
    }

    LOG_INFO(
        "check obs_in_refer_line_road protected_junction_.start_s, end_s "
        "{:.3f}, {:.3f}",
        protected_junction_.first, protected_junction_.second);
    auto obs_road_id =
        PlanningMap::Instance()->GetLaneRoadId(obs.matched_lane_id());
    bool obs_in_refer_line_road =
        (IsInSRange(obs.center_sl().s(), unprotected_junction_, safe_buffer) ||
         IsInSRange(obs.center_sl().s(), protected_junction_, safe_buffer))
            ? false
            : CheckIfObsInReferLineRoad(task_info.reference_line(), obs);
    LOG_INFO(
        "obs [{}] matched_lane_id [{}], obs_road_id [{}], "
        "obs_in_refer_line_road [{}]",
        obs.id(), obs.matched_lane_id(), obs_road_id, obs_in_refer_line_road);
    if (ego_roads.count(obs_road_id) || obs_in_refer_line_road) {
      LOG_INFO("ego_roads.count(obs_road_id) {},obs_in_refer_line_road {}",
               ego_roads.count(obs_road_id), obs_in_refer_line_road);
      continue;
    }

    double obs_adc_cartisian_dis = adc_polygon_.distance_to_2(obs.polygon());
    bool on_right =
        speed_prediction_pre_decision_decider_config_->right_consider
            ? ObsOnRight(obs, {inside_data.vel_x, inside_data.vel_y,
                               inside_data.vel_heading})
            : false;
    LOG_INFO("obs [{}] , on_right [{}], obs_adc_cartisian_dis {:.3f}", obs.id(),
             on_right, obs_adc_cartisian_dis);
    if (on_right ||
        obs_adc_cartisian_dis < speed_prediction_pre_decision_decider_config_
                                    ->obs_adc_safe_dis_straight) {
      continue;
    }

    if (IsInSRange(obs.center_sl().s(), protected_junction_, safe_buffer)) {
      double obs_adc_heading_diff = normalize_angle(
          obs.velocity_heading() -
          task_info.current_frame()->inside_planner_data().vel_heading);
      bool on_left = ObsOnLeft(
          obs, {inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading});
      LOG_INFO("obs [{}] , on_left [{}], obs_adc_heading_diff {:.3f}", obs.id(),
               on_left, obs_adc_heading_diff);
      if (!(on_left && (std::abs(obs_adc_heading_diff) > 0.70 * M_PI ||
                        obs_adc_cartisian_dis >
                            speed_prediction_pre_decision_decider_config_
                                ->obs_adc_safe_dis_cross_road))) {
        LOG_INFO(
            "obs [{}] is in protected junction, but not safe enough, not "
            "ignore.",
            obs.id());
        continue;
      }
    }

    if (obs_not_ignore_.find(obs.id()) != obs_not_ignore_.end()) {
      continue;
    }

    LOG_INFO("Ignore obs {}.", obs.id());
    speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id.insert(obs.id());
  }

  return ErrorCode::PLANNING_OK;
}

bool GetObsToPathInfo(TaskInfo& task_info, const Obstacle& obstacle,
                      double* obs_path_heading_diff, double& obs_path_dis) {
  PathPoint closest_pt{};
  if (!task_info.current_frame()
           ->outside_planner_data()
           .path_data->path()
           .query_closest_point(obstacle.center(), closest_pt)) {
    return false;
  }
  *obs_path_heading_diff =
      normalize_angle(obstacle.velocity_heading() - closest_pt.theta());

  if (!speed_planner_common::GetObsToPathLatDis(
          task_info.current_frame()
              ->outside_planner_data()
              .speed_obstacle_context.adc_sl_boundaries,
          obstacle, obs_path_dis)) {
    return false;
  }

  return true;
}

// ignore obs which is reverse, parallel adc, and keep safe distance.
ErrorCode CorssRoadFilterErrPrediction(TaskInfo& task_info) {
  auto& speed_obstacle_context = task_info.current_frame()
                                     ->mutable_outside_planner_data()
                                     ->speed_obstacle_context;
  using ObsType = Obstacle::ObstacleType;

  for (auto& decision : speed_obstacle_context.dynamic_obstacles_decision) {
    auto& obs = decision.obstacle;
    LOG_INFO("obs [{}] collision [{}], type veh or bic [{}]", obs.id(),
             decision.collide,
             obs.type() == ObsType::VEHICLE || obs.type() == ObsType::BICYCLE);
    if (!decision.collide) {
      LOG_INFO("not collide.");
      continue;
    }
    if (auto& traj = obs.prediction_trajectories();
        !(obs.type() == ObsType::VEHICLE || obs.type() == ObsType::BICYCLE) ||
        traj.empty()) {
      LOG_INFO("obs type [{}], traj is empty [{}].",
               static_cast<int>(obs.type()), traj.empty());
      continue;
    }

    double obs_adc_heading_diff = std::abs(normalize_angle(
        decision.obstacle.velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading));
    double obs_adc_cartisian_dis = adc_polygon_.distance_to_2(obs.polygon());
    double obs_path_heading_diff;
    double obs_path_dis{0.0};
    if (!GetObsToPathInfo(task_info, decision.obstacle, &obs_path_heading_diff,
                          obs_path_dis)) {
      LOG_INFO("GetObsToPathInfo fail.");
      return ErrorCode::PLANNING_OK;
    }
    obs_path_heading_diff = std::abs(obs_path_heading_diff);
    LOG_INFO(
        "obs_adc_heading_diff, obs_path_heading_diff, "
        "obs_adc_cartisian_dis, obs_path_dis: {:.3f}, {:.3f}, {:.3f}, {:.3f}. ",
        obs_adc_heading_diff, obs_path_heading_diff, obs_adc_cartisian_dis,
        obs_path_dis);
    if ((obs_adc_cartisian_dis <

         static_cast<double>(speed_prediction_pre_decision_decider_config_
                                 ->obs_adc_safe_dis_cross_road)) ||
        (obs_path_dis <
         speed_prediction_pre_decision_decider_config_->obs_path_safe_dis)) {
      LOG_INFO("dis not satisfy.");
      continue;
    }

    if (std::min(decision.lower_points.front().first.s(),
                 decision.lower_points.back().first.s()) >
        speed_prediction_pre_decision_decider_config_->obs_lower_point_mis_s) {
      continue;
    }

    if ((obs_adc_heading_diff < speed_prediction_pre_decision_decider_config_
                                    ->obs_adc_reverse_heading_diff) ||
        (obs_path_heading_diff < speed_prediction_pre_decision_decider_config_
                                     ->obs_path_reverse_heading_diff)) {
      LOG_INFO("head-diff not satisfy.");
      continue;
    }

    LOG_INFO("obs {} is reverse, parallel adc, and keep safe distance, ignore.",
             obs.id());
    speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id.insert(obs.id());
  }

  return ErrorCode::PLANNING_OK;
}

void GetObsOnPath(TaskInfo& task_info) {
  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dynamic_obstacles_decision;

  for (auto& decision : dynamic_obstacles_decision) {
    if (decision.lower_points.empty()) {
      continue;
    }

    if (decision.lower_points.front().first.t() < 0.2) {
      obs_not_ignore_.insert(decision.obstacle.id());
    }
  }
}

// filter obs: collide && reverse && >0.75pai && obs-behind-front-edge && obs
// not on path
void ReverseObsIgnore(TaskInfo& task_info) {
  auto& speed_obstacle_context = task_info.current_frame()
                                     ->mutable_outside_planner_data()
                                     ->speed_obstacle_context;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& adc_boundaries = task_info.current_frame()
                                   ->outside_planner_data()
                                   .speed_obstacle_context.adc_boundaries;
  for (auto& decision : speed_obstacle_context.dynamic_obstacles_decision) {
    auto& obs = decision.obstacle;
    if (!decision.collide) {
      continue;
    }
    if (!decision.reverse) {
      continue;
    }
    if (!ReverseIgnoreWithHeadingDiff(obs, inside_data.vel_heading)) {
      LOG_INFO("heading diff not satify.");
      continue;
    }
    if (!ReverseIgnoreWithPose(obs, {inside_data.vel_x, inside_data.vel_y,
                                     inside_data.vel_heading})) {
      LOG_INFO("pos not satify.");
      continue;
    }
    bool collide{false};
    for (const auto& ego_box : adc_boundaries) {
      collide = ReverseIgnoreWithPolygon(obs, Polygon2d(ego_box));
      if (collide) break;
    }
    if (collide) {
      LOG_INFO("polygon collide not satify.");
      continue;
    }
    LOG_INFO("obs {} is reverse, ignore with heading/perception polygon/pose",
             obs.id());
    speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id.insert(obs.id());
  }
}

// ignore obs,which is behind and will collide from behind
void IgnoreObsAccordingAdcCollidePoint(TaskInfo& task_info) {
  auto& speed_obstacle_context = task_info.current_frame()
                                     ->mutable_outside_planner_data()
                                     ->speed_obstacle_context;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();

  auto ObsHasPointBehindAdc = [](const Obstacle& obs,
                                 const std::array<double, 3>& ego_pose) {
    double min_x{std::numeric_limits<double>::infinity()};
    for (const auto& pt : obs.polygon().points()) {
      double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
      earth2vehicle(ego_pose[0], ego_pose[1], ego_pose[2], pt.x(), pt.y(),
                    obs.velocity_heading(), obs_local_x, obs_local_y,
                    obs_local_heading);
      min_x = std::min(min_x, obs_local_x);
    }

    return min_x < -VehicleParam::Instance()->back_edge_to_center();
  };

  for (auto& decision : speed_obstacle_context.dynamic_obstacles_decision) {
    auto& obs = decision.obstacle;
    if (!decision.collide) {
      continue;
    }
    if (ObsHasPointBehindAdc(obs, {inside_data.vel_x, inside_data.vel_y,
                                   inside_data.vel_heading}) &&
        ((decision.adc_first_collide_corner_point ==
          AdcCollideCornerPoint::LEFT_REAR) ||
         (decision.adc_first_collide_corner_point ==
          AdcCollideCornerPoint::RIGHT_REAR))) {
      LOG_INFO("ignore obs {}, behind and will collide from behind.", obs.id());
      speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id.insert(obs.id());
    }
  }
}

bool Init(TaskInfo& task_info) {
  speed_prediction_pre_decision_decider_config_ =
      &config::PlanningConfig::Instance()
           ->planning_research_config()
           .speed_prediction_pre_decision_decider_config;
  if (!speed_prediction_pre_decision_decider_config_) {
    return false;
  }

  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  adc_polygon_ = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.0, 0.0,
      0.0);

  adc_front_edge_s_ = task_info.curr_sl().s() +
                      VehicleParam::Instance()->front_edge_to_center();

  // find first junction in front of adc.
  protected_junction_ = {0.0, 0.0};
  unprotected_junction_ = {0.0, 0.0};
  for (auto [id, s, e] : task_info.reference_line()->junction_overlaps()) {
    if (e > adc_front_edge_s_) {
      unprotected_junction_ = {s, e};
      LOG_INFO("get first junction before adc: start_s, end_s: {:.3f}, {:.3f}",
               s, e);
      break;
    }
  }

  if (unprotected_junction_.second > adc_front_edge_s_) {
    for (auto [id, s, e] : task_info.reference_line()->signal_overlaps()) {
      // signal_overlaps is stopline
      if ((s > unprotected_junction_.first) &&
          (e < unprotected_junction_.second)) {
        protected_junction_ = unprotected_junction_;
        unprotected_junction_ = {0.0, 0.0};
        LOG_INFO(
            "end of signal is before adc and junction is overlap with signal, "
            "get protected junction: start_s, end_s: {:.3f}, {:.3f}",
            protected_junction_.first, protected_junction_.second);
        break;
      }
    }
  }

  const auto& adc_corner_pt_coordinate =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.adc_corner_pt_coordinate;
  if (adc_corner_pt_coordinate.size() <
      static_cast<int>(AdcCollideCornerPoint::NONE)) {
    LOG_ERROR("adc_corner_pt_coordinate size < 4");
    return false;
  }
  obs_not_ignore_.clear();

  return true;
}

}  // namespace

SpeedPredictionPreDecisionDecider::SpeedPredictionPreDecisionDecider() {
  name_ = "SpeedPredictionPreDecisionDecider";
}

void SpeedPredictionPreDecisionDecider::SaveTaskResults(TaskInfo& task_info) {}

void SpeedPredictionPreDecisionDecider::Reset() {}

ErrorCode SpeedPredictionPreDecisionDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Init(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  GetObsOnPath(task_info);
  ReverseObsIgnore(task_info);
  IgnoreObsAccordingAdcCollidePoint(task_info);

  if (frame->outside_planner_data().consider_reverse_lane_detour_obs) {
    LOG_INFO(
        "adc will borrow lane to reverse road, not filter err prediction.");
    return ErrorCode::PLANNING_OK;
  }
  if (!ref_line_util::SIsInJunction(task_info.reference_line(),
                                    adc_front_edge_s_)) {
    return StraightRoadFilterErrPrediction(task_info);
  } else {
    return CorssRoadFilterErrPrediction(task_info);
  }
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive

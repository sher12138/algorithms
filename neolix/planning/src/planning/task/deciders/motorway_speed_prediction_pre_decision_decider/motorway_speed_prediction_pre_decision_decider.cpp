#include "motorway_speed_prediction_pre_decision_decider.h"

#include "planning_map/planning_map.h"
#include "reference_line/reference_line_util.h"
namespace neodrive {
namespace planning {

namespace {

constexpr double kReverseHeadingDiff = 0.75 * M_PI;
constexpr double kAttenTime = 1.0;
constexpr double kGap = 2.5;
constexpr double kProtectedDis = 2.5;

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using neodrive::global::prediction::Trajectory_PredictorType_FreeMove;
std::pair<double, double> protected_junction_{0.0, 0.0};
std::pair<double, double> unprotected_junction_{0.0, 0.0};

const config::AutoPlanningResearchConfig::
    SpeedPredictionPreDecisionDeciderConfig*
        speed_prediction_pre_decision_decider_config_{nullptr};
double adc_front_edge_s_{0.0};
Polygon2d adc_polygon_{};

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

ErrorCode StraightRoadFilterErrPrediction(
    TaskInfo& task_info, std::unordered_set<int> obs_not_ignore) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  auto ego_roads = GetRoadIdsExceptJunction(task_info.reference_line(),
                                            task_info.curr_sl().s());

  if (ego_roads.empty()) return ErrorCode::PLANNING_OK;  // in junction
  std::string str_log = "Ego road id : ";
  for (auto id : ego_roads) {
    str_log += std::to_string(id) + ", ";
  }
  LOG_INFO("{}", str_log);

  auto& motorway_speed_obstacle_context = task_info.current_frame()
                                              ->mutable_outside_planner_data()
                                              ->motorway_speed_obstacle_context;
  using ObsType = Obstacle::ObstacleType;

  for (auto& decision :
       motorway_speed_obstacle_context.multi_cipv_dynamic_obstacles_decision) {
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

    if (IsInSRange(obs.center_sl().s(), unprotected_junction_, safe_buffer)) {
      LOG_INFO(
          "adc is not in junction, but obs [{}] is in unprotected junction, "
          "not ignore",
          obs.id());
      continue;
    }

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
    if (obs_not_ignore.find(obs.id()) == obs_not_ignore.end()) {
      LOG_INFO("Ignore obs {}.", obs.id());
      motorway_speed_obstacle_context.ignore_dynamic_obs_id.insert(obs.id());
    }
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
              .motorway_speed_obstacle_context.adc_sl_boundaries,
          obstacle, obs_path_dis)) {
    return false;
  }

  return true;
}

// ignore obs which is reverse, parallel adc, and keep safe distance.
ErrorCode CorssRoadFilterErrPrediction(TaskInfo& task_info,
                                       std::unordered_set<int> obs_not_ignore) {
  auto& motorway_speed_obstacle_context = task_info.current_frame()
                                              ->mutable_outside_planner_data()
                                              ->motorway_speed_obstacle_context;
  using ObsType = Obstacle::ObstacleType;

  for (auto& decision :
       motorway_speed_obstacle_context.multi_cipv_dynamic_obstacles_decision) {
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

    if (obs_not_ignore.find(obs.id()) == obs_not_ignore.end()) {
      LOG_INFO(
          "obs {} is reverse, parallel adc, and keep safe distance, ignore.",
          obs.id());
      motorway_speed_obstacle_context.ignore_dynamic_obs_id.insert(obs.id());
    }
  }

  return ErrorCode::PLANNING_OK;
}

// filter obs: collide && reverse && >0.75pai && obs-behind-front-edge && obs
// not on path
void ReverseObsIgnore(TaskInfo& task_info,
                      std::unordered_set<int> obs_not_ignore) {
  auto& motorway_speed_obstacle_context = task_info.current_frame()
                                              ->mutable_outside_planner_data()
                                              ->motorway_speed_obstacle_context;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& adc_boundaries =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context.adc_boundaries;
  for (auto& decision :
       motorway_speed_obstacle_context.multi_cipv_dynamic_obstacles_decision) {
    if (motorway_speed_obstacle_context.ignore_dynamic_obs_id.find(
            decision.obstacle.id()) !=
        motorway_speed_obstacle_context.ignore_dynamic_obs_id.end()) {
      LOG_INFO("obs {}  has ignored by other reason.", decision.obstacle.id());
      continue;
    }
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

    if (obs_not_ignore.find(obs.id()) == obs_not_ignore.end()) {
      LOG_INFO("obs {} is reverse, ignore with heading/perception polygon/pose",
               obs.id());
      motorway_speed_obstacle_context.ignore_dynamic_obs_id.insert(obs.id());
    }
  }
}

// ignore obs,which is behind and will collide from behind
void IgnoreObsAccordingAdcCollidePoint(TaskInfo& task_info,
                                       std::unordered_set<int> obs_not_ignore) {
  auto& motorway_speed_obstacle_context = task_info.current_frame()
                                              ->mutable_outside_planner_data()
                                              ->motorway_speed_obstacle_context;
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

  for (auto& decision :
       motorway_speed_obstacle_context.multi_cipv_dynamic_obstacles_decision) {
    auto& obs = decision.obstacle;
    if (motorway_speed_obstacle_context.ignore_dynamic_obs_id.find(
            decision.obstacle.id()) !=
        motorway_speed_obstacle_context.ignore_dynamic_obs_id.end()) {
      LOG_INFO("obs {}  has ignored by other reason.", decision.obstacle.id());
      continue;
    }
    if (!decision.collide) {
      continue;
    }
    if (ObsHasPointBehindAdc(obs, {inside_data.vel_x, inside_data.vel_y,
                                   inside_data.vel_heading}) &&
        ((decision.adc_first_collide_corner_point ==
          AdcCollideCornerPoint::LEFT_REAR) ||
         (decision.adc_first_collide_corner_point ==
          AdcCollideCornerPoint::RIGHT_REAR))) {
      if (obs_not_ignore.find(obs.id()) == obs_not_ignore.end()) {
        LOG_INFO("ignore obs {}, behind and will collide from behind.",
                 obs.id());
        motorway_speed_obstacle_context.ignore_dynamic_obs_id.insert(obs.id());
      }
    }
  }
}

bool Init(TaskInfo& task_info, std::unordered_set<int>& obs_not_ignore) {
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
        break;
      }
    }
  }
  const auto& adc_corner_pt_coordinate =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context.adc_corner_pt_coordinate;
  if (adc_corner_pt_coordinate.size() <
      static_cast<int>(AdcCollideCornerPoint::NONE)) {
    LOG_ERROR("adc_corner_pt_coordinate size < 4");
    return false;
  }

  obs_not_ignore.clear();

  return true;
}

}  // namespace

bool FenceFilterReverseOBs::EgoBePortectedByFence(
    double s, double speed, std::vector<MatrixInterval>& combine_data) {
  if (combine_data.empty()) {
    return false;
  }

  if (combine_data.front().Longitudinal().start() > s) {
    return false;
  }

  if (combine_data.back().Longitudinal().end() < s) {
    return false;
  }
  if (combine_data.size() == 1) {
    return true;
  }

  auto iterator = combine_data.begin();
  auto next_iter = iterator;
  LOG_INFO("start  find  int");
  while (iterator != combine_data.end()) {
    next_iter = std::next(iterator);
    if (next_iter->Longitudinal().start() - iterator->Longitudinal().end() >
        kGap) {
      return false;
    }
    if (next_iter->Longitudinal().end() > s + speed * kAttenTime) {
      LOG_INFO("ego be protected!");
      return true;
    }
    iterator = next_iter;
  }
  return false;
}

bool FenceFilterReverseOBs::ObsBePortectedByFence(
    const std::vector<MatrixInterval>& combine_data, Obstacle* const& obstacle,
    const bool ego_be_protected_by_right_fence) {
  if (combine_data.empty()) {
    return false;
  }
  // -> solution 1 ,
  if (ego_be_protected_by_right_fence &&
      combine_data.back().Lateral().start() > obstacle->center_sl().l()) {
    LOG_INFO("ego be protected and obs {} out fence,ignore", obstacle->id());
    return true;
  }

  if (obstacle->min_s() > combine_data.back().Longitudinal().end()) {
    return false;
  }
  if (obstacle->max_s() < combine_data.front().Longitudinal().start()) {
    return false;
  }

  if (combine_data.size() == 1) {
    return true;
  }
  for (const auto& data : combine_data) {
    LOG_INFO("longitudianl start s{:.3f}, end s{:.3f}",
             data.Longitudinal().start(), data.Longitudinal().end());
    LOG_INFO("lateral start {:.3f}, end {:.3f}", data.Lateral().start(),
             data.Lateral().end());
  }
  LOG_INFO("obs s {:.3f},", obstacle->center_sl().s());
  // the first can insert position
  auto iterator = std::lower_bound(combine_data.begin(), combine_data.end(),
                                   obstacle->center_sl().s(),
                                   [](const MatrixInterval& lhs, double rhs) {
                                     return lhs.Longitudinal().end() < rhs;
                                   });

  if (iterator == combine_data.end()) {
    LOG_INFO("obs {} over fence,", obstacle->id());
    return true;
  }
  if (iterator->Lateral().start() * obstacle->center_sl().l() < 0.0) {
    LOG_INFO("obs {} not in same side with fence", obstacle->id());
    return true;
  }

  if ((std::abs(iterator->Lateral().start()) +
       std::abs(iterator->Lateral().end())) /
          2. >
      std::abs(obstacle->center_sl().l())) {
    LOG_INFO("obs {} in fence, can't ignore.", obstacle->id());
    return true;
  }

  double s_start = obstacle->center_sl().s() - obstacle->speed() * kAttenTime;
  if (iterator->Lateral().start() < s_start) {
    LOG_INFO("obs {} can't pass fence, ignore", obstacle->id());
    return true;
  }
  // judge weather exist a hole, what can let  obs across.
  auto prev_iter = iterator;
  LOG_INFO("start  find  int");
  while (iterator != combine_data.begin()) {
    prev_iter = std::prev(iterator);
    if (iterator->Longitudinal().start() - prev_iter->Longitudinal().end() >
        kGap) {
      return false;
    }
    if (prev_iter->Longitudinal().start() < s_start) {
      LOG_INFO("can ignore obs {}", obstacle->id());
      return true;
    }
    iterator = prev_iter;
  }
  return false;
}

void FenceFilterReverseOBs::IgnoreOBsByPerceptionRightFence(
    TaskInfo& task_info) {
  auto& motorway_speed_obstacle_context = task_info.current_frame()
                                              ->mutable_outside_planner_data()
                                              ->motorway_speed_obstacle_context;
  const auto& all_obstacles = task_info.decision_data()->all_obstacle();
  const auto& dynamic_obstacles = task_info.decision_data()->dynamic_obstacle();
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data;
  const double ego_heading =
      task_info.current_frame()->inside_planner_data().vel_heading;
  const double ego_l = task_info.curr_sl().l();
  const double ego_s = task_info.curr_sl().s();
  const double ego_speed =
      task_info.current_frame()->inside_planner_data().vel_v;
  auto& ignore_dynamic_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.ignore_dynamic_obs_id;

  const auto& curr_scenario =
      DataCenter::Instance()->master_info().curr_scenario();
  const auto& curr_stage = DataCenter::Instance()
                               ->master_info()
                               .motorway_intersection_context()
                               .stage;
  // need divide with  scenario
  if (curr_scenario == ScenarioState::MOTORWAY_INTERSECTION) {
    LOG_INFO("ego in intersection, skip fence filter");
    return;
  }

  if (path == nullptr) return;
  const auto& ref_line = task_info.reference_line();
  std::vector<const Obstacle*> right_fence_obstacles;
  for (const auto& obstacle : all_obstacles) {
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->sub_type() !=
        PerceptionObstacle_SubType::PerceptionObstacle_SubType_ST_FENCE) {
      continue;
    }

    if (obstacle->center_sl().s() < ego_s) {
      continue;
    }

    if (obstacle->center_sl().l() - ego_l > 0.0) {
      continue;
    } else {
      right_fence_obstacles.push_back(obstacle);
    }
  }

  if (right_fence_obstacles.empty()) {
    return;
  }
  // lat, long
  std::vector<MatrixInterval> data{};
  for (const auto& obs : right_fence_obstacles) {
    data.push_back(
        std::move(MatrixInterval(VecInterval(obs->min_l(), obs->max_l()),
                                 VecInterval(obs->min_s(), obs->max_s()))));
  }
  for (const auto& cc : data) {
    LOG_INFO("Long ss {:.3f},Long ss {:.3f}, Later ss{:.3f}, Later se {:.3f}",
             cc.Longitudinal().start(), cc.Longitudinal().end(),
             cc.Lateral().start(), cc.Lateral().end());
  }
  auto combined_data = std::move(MatrixInternalCombine(data));
  bool ego_be_protected_by_right_fence =
      EgoBePortectedByFence(ego_s, ego_speed, combined_data);
  // 1. construct fence boundary -->
  // 1.0 --> assume the fence is a continue mulit-line
  // 2.0 --> fence is continue internal line --> internal combine and
  // internal find 3.0 --> 区间碰撞结果的区间存在性查询，区间的交集查询。
  // if(!) find adverse obs
  for (const auto& obstacle : dynamic_obstacles) {
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->center_sl().s() < ego_s) {
      continue;
    }
    if (obstacle->center_sl().l() > ego_l) {
      continue;
    }
    double heading_diff = 0.0;
    if (!speed_planner_common::ObsEgoAlongPathHeadingDiffBaseSpeedHeading(
            obstacle, *path, ego_heading, heading_diff)) {
      LOG_INFO("can't get heading diff with obs {}, skip!", obstacle->id());
      continue;
    };
    // reverse get
    if (std::abs(heading_diff) > kReverseHeadingDiff) {
      LOG_INFO("prosss obs {}", obstacle->id());
      // find weather con pass fence,
      if (ObsBePortectedByFence(combined_data, obstacle,
                                ego_be_protected_by_right_fence)) {
        LOG_INFO("obs {} can't pass fence ,ignore", obstacle->id());
        ignore_dynamic_obs_id.insert(obstacle->id());
      }
    }
  }
}

void MotorwaySpeedPredictionPreDecisionDecider::GetObsOnPath(
    TaskInfo& task_info) {
  const auto& multi_cipv_dynamic_obstacles_decision =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;

  for (auto& decision : multi_cipv_dynamic_obstacles_decision) {
    if (decision.lower_points.empty()) {
      continue;
    }

    if (decision.lower_points.front().first.t() < 0.2) {
      obs_not_ignore_.insert(decision.obstacle.id());
    }
  }
}

void MotorwaySpeedPredictionPreDecisionDecider::IgnoreObsByPerceptionRoadBound(
    TaskInfo& task_info) {
  auto& motorway_speed_obstacle_context = task_info.current_frame()
                                              ->mutable_outside_planner_data()
                                              ->motorway_speed_obstacle_context;
  const auto& curb_lines = DataCenter::Instance()
                               ->environment()
                               .perception_lanes_proxy()
                               .camera_curb_lines();

  LOG_INFO("curb line size : {}", curb_lines.size());
  const auto& inside_data = task_info.current_frame()->inside_planner_data();

  Box2d ego_box = VehicleParam::Instance()->get_adc_bounding_box(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.1, 0.1,
      0.1);

  double ego_s = task_info.curr_sl().s();

  double forward_check_dis = VehicleParam::Instance()->front_edge_to_center() +
                             inside_data.vel_v * 1.5 + 2.0;
  auto& path_data =
      task_info.current_frame()->mutable_outside_planner_data()->path_data;

  const auto& path_pts = path_data->path().path_points();
  const auto& ref_line = task_info.reference_line();

  auto GetPointPosition = [](SLPoint line_start, SLPoint line_end,
                             SLPoint check_point) {
    double cross_product =
        (check_point.s() - line_start.s()) * (line_end.l() - line_start.l()) -
        (check_point.l() - line_start.l()) * (line_end.s() - line_start.s());
    if (cross_product == 0) {
      return kOnLine;
    } else if (cross_product > 0) {
      return kRight;
    } else {
      return kLeft;
    }
  };

  auto JudgeInSegment = [](SLPoint line_start, SLPoint line_end,
                           SLPoint check_point) {
    if (line_start.s() <= line_end.s()) {
      return line_end.s() <= check_point.s() &&
             check_point.s() <= line_start.s();
    } else {
      return line_start.s() <= check_point.s() &&
             check_point.s() <= line_end.s();
    }
  };

  curb_area_.clear();
  std::vector<std::pair<SLPoint, SLPoint>> left_curb_line;

  for (const auto& curb_line : curb_lines) {
    for (const auto& curb_segment : curb_line) {
      SLPoint road_start_sl;
      SLPoint road_end_sl;
      ref_line->GetPointInFrenetFrame(
          {curb_segment.start().x(), curb_segment.start().y()}, &road_start_sl);
      ref_line->GetPointInFrenetFrame(
          {curb_segment.end().x(), curb_segment.end().y()}, &road_end_sl);
      if (road_start_sl.l() < 0) continue;
      left_curb_line.emplace_back(std::make_pair(road_start_sl, road_end_sl));
      curb_area_.emplace_back(Boundary(road_start_sl.s(), road_end_sl.s(),
                                       road_start_sl.l(),
                                       road_end_sl.l() + 0.3));
    }
  }
  std::sort(left_curb_line.begin(), left_curb_line.end(),
            [](const std::pair<SLPoint, SLPoint>& a,
               const std::pair<SLPoint, SLPoint>& b) {
              if (a.first.s() < b.first.s())
                return true;
              else
                return false;
            });

  LOG_INFO("ego s : {}", ego_s);
  if (!left_curb_line.empty()) {
    LOG_INFO("curb  s  - ego s --fist  : {} --second  : {}",
             left_curb_line.front().first.s() - ego_s,
             left_curb_line.back().second.s() - ego_s);
  }

  for (size_t i = 0; i < left_curb_line.size(); i++) {
    if ((i + 1) % 3 == 0 || i == 0) {
      LOG_INFO("curb area start s : {} ,end s : {} ",
               left_curb_line[i].first.s() - ego_s,
               left_curb_line[i].second.s() - ego_s);
    }
  }
  using ObsType = Obstacle::ObstacleType;
  for (auto& decision :
       motorway_speed_obstacle_context.multi_cipv_dynamic_obstacles_decision) {
    auto& obs = decision.obstacle;

    if (motorway_speed_obstacle_context.ignore_dynamic_obs_id.find(
            decision.obstacle.id()) !=
        motorway_speed_obstacle_context.ignore_dynamic_obs_id.end()) {
      LOG_INFO("obs {}  has ignored by other reason.", decision.obstacle.id());
      continue;
    }

    if (obs.type() != ObsType::VEHICLE) {
      continue;
    }

    auto heading_diff = std::abs(
        normalize_angle(inside_data.vel_heading - obs.velocity_heading()));
    if (heading_diff < 0.25 * M_PI) {
      obs_not_ignore_.insert(obs.id());
      continue;
    }
    SLPoint obs_point{obs.center_sl().s(), obs.center_sl().l()};

    if (heading_diff > kReverseHeadingDiff && !left_curb_line.empty()) {
      if ((left_curb_line.back().second.s() - ego_s) >= 3.0 &&
          (left_curb_line.front().first.s() - ego_s) < 1.0) {
        auto judge_result =
            GetPointPosition(left_curb_line.front().first,
                             left_curb_line.back().second, obs_point);
        if (judge_result == JudgeObsInSamePos::kLeft) {
          LOG_INFO("obs id : {} is in curb left, need ignore", obs.id());
          motorway_speed_obstacle_context.ignore_dynamic_obs_id.insert(
              obs.id());
        }
      }
    }
  }
  VisCerbBoundary(task_info.reference_line(), curb_area_, "curb_area");
}

void MotorwaySpeedPredictionPreDecisionDecider::VisCerbBoundary(
    const ReferenceLinePtr ref_line, const std::vector<Boundary>& check_area,
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

  for (const auto& bdry : check_area) {
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
}

MotorwaySpeedPredictionPreDecisionDecider::
    MotorwaySpeedPredictionPreDecisionDecider() {
  name_ = "MotorwaySpeedPredictionPreDecisionDecider";
}

void MotorwaySpeedPredictionPreDecisionDecider::SaveTaskResults(
    TaskInfo& task_info) {}

void MotorwaySpeedPredictionPreDecisionDecider::Reset() {}

ErrorCode MotorwaySpeedPredictionPreDecisionDecider::Execute(
    TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Init(task_info, obs_not_ignore_)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  GetObsOnPath(task_info);
  FenceFilterReverseOBs::IgnoreOBsByPerceptionRightFence(task_info);
  IgnoreObsByPerceptionRoadBound(task_info);

  ReverseObsIgnore(task_info, obs_not_ignore_);
  IgnoreObsAccordingAdcCollidePoint(task_info, obs_not_ignore_);

  if (frame->outside_planner_data().consider_reverse_lane_detour_obs) {
    LOG_INFO(
        "adc will borrow lane to reverse road, not filter err prediction.");
    return ErrorCode::PLANNING_OK;
  }
  if (!JudgeEgoInJunction(task_info, adc_front_edge_s_)) {
    LOG_INFO("ego is not in junction ");
    return StraightRoadFilterErrPrediction(task_info, obs_not_ignore_);
  } else {
    LOG_INFO("ego is in junction");
    return CorssRoadFilterErrPrediction(task_info, obs_not_ignore_);
  }

  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedPredictionPreDecisionDecider::JudgeEgoInJunction(
    TaskInfo& task_info, const double& ego_s) {
  auto ref_line = task_info.reference_line();
  const auto& junction_list = ref_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }

    double deal_area_start_s =
        overlap.start_s - VehicleParam::Instance()->length() - 20.0;
    double deal_area_end_s = overlap.end_s;
    if (deal_area_start_s <= ego_s && deal_area_end_s >= ego_s &&
        (junction_ptr->Type() !=
         static_cast<uint32_t>(JunctionType::IN_ROAD))) {
      return true;
    } else if (deal_area_start_s > ego_s) {
      break;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace neodrive

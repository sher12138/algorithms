#include "obstacles_intention_decider.h"

#include <stdio.h>

#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_manager.h"

namespace neodrive {
namespace planning {
using cyberverse::LaneInfoConstPtr;

ObstaclesIntentionDecider::ObstaclesIntentionDecider() {
  name_ = "ObstaclesIntentionDecider";
  ResetTable();
}

ObstaclesIntentionDecider::~ObstaclesIntentionDecider() { Reset(); }

bool ObstaclesIntentionDecider::ObstaclesIntentionProcess(TaskInfo &task_info) {
  std::unordered_map<int, std::vector<ObstaclesIntention>>
      cur_frame_obs_intention{};
  if (!OtherSideObstaclesIntentionProcess(task_info, cur_frame_obs_intention)) {
    LOG_ERROR("OtherSideObstaclesIntentionProcess ERROR");
  }
  task_info.current_frame()->mutable_outside_planner_data()->obs_intention =
      cur_frame_obs_intention;
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->obs_intention_context = cur_obs_intention_context_;
  SaveObsIntentionContextStr(task_info, cur_frame_obs_intention);
  last_frame_obs_intention_ = cur_frame_obs_intention;
  FilterObsIntentionsProcess(task_info);
  return true;
}

bool ObstaclesIntentionDecider::OtherSideObstaclesIntentionProcess(
    TaskInfo &task_info,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  for (auto &obs : other_side_obstacles_table_) {
    LOG_INFO("Process obs {}", obs.first);
    OtherSideObstacleIntentionProcess(task_info, obs, cur_frame_obs_intention);
  }
  return true;
}

bool ObstaclesIntentionDecider::OtherSideObstacleIntentionProcess(
    TaskInfo &task_info,
    std::pair<const int, std::vector<Obstacle *>> &obs_pair,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  is_right_lane_tmp_ = false;
  is_left_lane_tmp_ = false;
  auto &current_obstacle = obs_pair.second.back();
  auto &earliest_obstacle = obs_pair.second.front();
  if (obs_pair.second.size() > 2) {
    earliest_obstacle = obs_pair.second[2];
  }
  // get utm obs
  Obstacle *obs_utm{nullptr};
  if (!data_center_->mutable_object_table_utm()->get_obstacle(obs_pair.first,
                                                              &obs_utm)) {
    LOG_ERROR("Get utm obstacle from object table failed");
    return false;
  }
  State predicted_state = GetNextState(*obs_utm, obs_pair.second);
  current_predicted_state_ = predicted_state;
  double obs_heading_diff;
  double current_obstacle_heading;
  double earliest_obstacle_heading;
  if (current_obstacle->speed() < kStaticSpeedThreshold ||
      earliest_obstacle->speed() < kStaticSpeedThreshold) {
    current_obstacle_heading = current_obstacle->heading();
    earliest_obstacle_heading = earliest_obstacle->heading();
  } else {
    current_obstacle_heading = current_obstacle->velocity_heading();
    earliest_obstacle_heading = earliest_obstacle->velocity_heading();
  }
  obs_heading_diff =
      normalize_angle(current_obstacle_heading - earliest_obstacle_heading);

  std::vector<ObsIntentionWeight> obs_intention_weight_vector;
  obs_intention_weight_vector.clear();
  Vec3d relative_obs;
  GetObsRelativeCoordinate(current_obstacle, relative_obs);
  cyberverse::LaneInfoConstPtr lane = GetObsCurrentLane(obs_heading_diff);
  if (lane != nullptr && !current_obstacle->is_static() &&
      current_obstacle->speed() > kStaticSpeedThreshold) {
    // GOforward or turn intention check
    bool current_lane_turn_check = false;
    if (cur_obs_intention_context_.find(current_obstacle->id()) ==
        cur_obs_intention_context_.end()) {
      ObstacleIntentionContext obs_context;
      obs_context.lane_id = lane->Id();
      obs_context.hash_lane_id =
          cyberverse::HDMap::Instance()->GetIdHashString(lane->Id());
      obs_context.x = relative_obs.x();
      obs_context.y = relative_obs.y();
      obs_context.heading = relative_obs.z();
      obs_context.speed = current_obstacle->speed();
      obs_context.length = current_obstacle->length();
      obs_context.width = current_obstacle->width();
      cur_obs_intention_context_[current_obstacle->id()] = obs_context;
    } else {
      cur_obs_intention_context_[current_obstacle->id()].lane_id = lane->Id();
      cur_obs_intention_context_[current_obstacle->id()].hash_lane_id =
          cyberverse::HDMap::Instance()->GetIdHashString(lane->Id());
      cur_obs_intention_context_[current_obstacle->id()].x = relative_obs.x();
      cur_obs_intention_context_[current_obstacle->id()].y = relative_obs.y();
      cur_obs_intention_context_[current_obstacle->id()].heading =
          relative_obs.z();
      cur_obs_intention_context_[current_obstacle->id()].speed =
          current_obstacle->speed();
      cur_obs_intention_context_[current_obstacle->id()].length =
          current_obstacle->length();
      cur_obs_intention_context_[current_obstacle->id()].width =
          current_obstacle->width();
    }
    std::vector<uint64_t> right_lanes;
    if (!PlanningMap::Instance()->GetRightLanes(lane->Id(), right_lanes)) {
      LOG_ERROR("GetRightLanes failed");
    }
    if (right_lanes.empty()) {
      is_right_lane_tmp_ = true;
    }
    double current_obstacle_s, current_obstacle_l;
    lane->GetProjection(
        common::math::Vec2d(obs_utm->center().x(), obs_utm->center().y()),
        &current_obstacle_s, &current_obstacle_l);
    cur_obs_intention_context_[current_obstacle->id()].s_in_lane =
        current_obstacle_s;
    cur_obs_intention_context_[current_obstacle->id()].l_in_lane =
        current_obstacle_l;
    if (!ObsCurrentLaneTurnIntentionProcess(
            lane, *current_obstacle, obs_heading_diff,
            obs_intention_weight_vector, cur_frame_obs_intention)) {
      LOG_INFO("ObsCurrentLaneIntentionProcess Done skip!");
      current_lane_turn_check = true;
    }
    if (!current_lane_turn_check) {
      ObsSuccessorLanesTurnIntentionProcess(
          lane, *current_obstacle, *obs_utm, obs_heading_diff,
          obs_intention_weight_vector, cur_frame_obs_intention);
    }
  }
  ObsCutinIntentionProcess(task_info, obs_pair, obs_intention_weight_vector,
                           cur_frame_obs_intention);
  ObsIntentionWeightsProcess(*current_obstacle, obs_intention_weight_vector,
                             cur_frame_obs_intention);
  return true;
}

bool ObstaclesIntentionDecider::CurrentLaneObstaclesIntentionProcess(
    TaskInfo &task_info,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  return true;
}

bool ObstaclesIntentionDecider::ObsSuccessorLanesTurnIntentionProcess(
    const cyberverse::LaneInfoConstPtr lane, const Obstacle &obs,
    const Obstacle &obs_utm, const double &obs_heading_diff,
    std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  // Get obs SuccessorLanes
  std::vector<uint64_t> lanes;
  if (!PlanningMap::Instance()->GetSuccessorLanes(lane->Id(), lanes)) {
    LOG_ERROR("GetSuccessorLanes failed");
    obs_intention_weight_vector.emplace_back(
        ObstacleIntentionType::GO_FORWARD_INTENTION, kWeightMax);
    return true;
  }
  double current_obstacle_s, current_obstacle_l;
  lane->GetProjection(
      common::math::Vec2d(obs_utm.center().x(), obs_utm.center().y()),
      &current_obstacle_s, &current_obstacle_l);
  cur_obs_intention_context_[obs.id()].s_in_lane = current_obstacle_s;
  cur_obs_intention_context_[obs.id()].l_in_lane = current_obstacle_l;
  // LOG_ERROR(" utm heading {}  velocity_heading {}, current_obstacle_s: {},
  // current_obstacle_l: {}, lane length: {}", obs_utm.heading(),
  // obs_utm.velocity_heading(), current_obstacle_s, current_obstacle_l,
  // lane->TotalLength());

  double lane_heading_obs_diff_1 = normalize_angle(
      obs_utm.velocity_heading() - lane->Heading(current_obstacle_s));
  double lane_heading_obs_diff_2 =
      normalize_angle(obs_utm.heading() - lane->Heading(current_obstacle_s));
  double lane_heading_obs_diff =
      fabs(lane_heading_obs_diff_1) < fabs(lane_heading_obs_diff_2)
          ? lane_heading_obs_diff_1
          : lane_heading_obs_diff_2;
  LOG_ERROR(
      "lane_heading_obs_diff: {} {} {} lane_heading_obs_diff_1 {} "
      "lane_heading_obs_diff_2 {}",
      lane_heading_obs_diff, obs_utm.velocity_heading(),
      lane->Heading(current_obstacle_s), lane_heading_obs_diff_1,
      lane_heading_obs_diff_2);
  if (fabs(lane_heading_obs_diff) > kReverseLaneThreshold) {
    double reverse_lane_heading_obs_diff = normalize_angle(
        lane->Heading(current_obstacle_s) - obs_utm.velocity_heading());
    LOG_ERROR("reverse_lane_heading_obs_diff: {}",
              reverse_lane_heading_obs_diff);
    if (reverse_lane_heading_obs_diff < 0) {
      ObstacleIntentionType intention =
          ObstacleIntentionType::TURN_LEFT_INTENTION;
      ObsTurnCheck(obs_heading_diff, intention);
      if (intention != ObstacleIntentionType::TURN_LEFT_INTENTION) {
        cur_obs_intention_context_[obs.id()].is_rectified = true;
        LOG_INFO("obs.id {} is {} rectified to {}", obs.id(),
                 intention_map_[ObstacleIntentionType::TURN_LEFT_INTENTION],
                 intention_map_[intention]);
      }
      obs_intention_weight_vector.emplace_back(intention, kWeightMax);
      return true;
    }
    ObstacleIntentionType intention =
        ObstacleIntentionType::TURN_RIGHT_INTENTION;
    ObsTurnCheck(obs_heading_diff, intention);
    if (intention != ObstacleIntentionType::TURN_RIGHT_INTENTION) {
      cur_obs_intention_context_[obs.id()].is_rectified = true;
      LOG_INFO("obs.id {} is {} rectified to {}", obs.id(),
               intention_map_[ObstacleIntentionType::TURN_RIGHT_INTENTION],
               intention_map_[intention]);
    }
    obs_intention_weight_vector.emplace_back(intention, kWeightMax);
    return true;
  }
  for (auto &lane_id : lanes) {
    auto successor_lane = cyberverse::HDMap::Instance()->GetLaneById(lane_id);
    if (!successor_lane) {
      LOG_ERROR("successor_lane not found");
      return false;
    }
    double end_s = kEndSThreshold < successor_lane->TotalLength()
                       ? kEndSThreshold
                       : successor_lane->TotalLength();
    double start_s = current_obstacle_s - kStartSThreshold > 0
                         ? current_obstacle_s - kStartSThreshold
                         : 0;
    double successor_heading_obs_diff = normalize_angle(
        successor_lane->Heading(end_s) - lane->Heading(start_s));
    LOG_ERROR("successor_heading_obs_diff: {}", successor_heading_obs_diff);
    // LOG_ERROR("current_obstacle SuccessorLane lane id: {}, lane start
    // heading: {} successor_heading_obs_diff: {}
    // lane->Heading(current_obstacle_s): {}",
    // cyberverse::HDMap::Instance()->GetIdHashString(lane_id),
    // successor_lane->end_heading(), successor_heading_obs_diff,
    // lane->Heading(current_obstacle_s));
    if (fabs(successor_heading_obs_diff) < kTurnClassifyRadLimit) {
      // GO_FORWARD weight default 1.0
      ObstacleIntentionType intention =
          ObstacleIntentionType::GO_FORWARD_INTENTION;
      double turn_weight = 0.0;
      if (!ObsGoForwardCheck(intention, obs_heading_diff, turn_weight)) {
        obs_intention_weight_vector.emplace_back(intention, turn_weight);
      }
      if (intention != ObstacleIntentionType::GO_FORWARD_INTENTION) {
        cur_obs_intention_context_[obs.id()].is_rectified = true;
        LOG_INFO("obs.id {} is {} rectified to {}", obs.id(),
                 intention_map_[ObstacleIntentionType::GO_FORWARD_INTENTION],
                 intention_map_[intention]);
      }
      obs_intention_weight_vector.emplace_back(
          ObstacleIntentionType::GO_FORWARD_INTENTION, kWeightMax);
      continue;
    }
    // if (fabs(successor_heading_obs_diff) > kStaticUTurnThreshold) {
    //   // U turn default 1.0
    //   obs_intention_weight_vector.emplace_back(
    //       ObstacleIntentionType::U_TURN_INTENTION, kWeightMax);
    //   continue;
    // }
    LOG_ERROR(
        "right turn lane_heading_obs_diff: {}, current_obstacle_l: {}, "
        "lane->TotalLength(): {}, current_obstacle_s: {}, "
        "lane->TotalLength() - current_obstacle_s: {}",
        lane_heading_obs_diff, current_obstacle_l, lane->TotalLength(),
        current_obstacle_s, lane->TotalLength() - current_obstacle_s);
    if (successor_heading_obs_diff < 0) {
      // right turn
      cur_obs_intention_context_[obs.id()].right_lane_id = lane_id;
      if (lane_heading_obs_diff > 0.0 ||
          (current_obstacle_l >= 0.0 &&
           lane->TotalLength() - current_obstacle_s > kLaneLengthThreshold)) {
        // Get obs SuccessorLanes
        if (is_right_lane_tmp_) {
          obs_intention_weight_vector.emplace_back(
              ObstacleIntentionType::TURN_RIGHT_INTENTION, kWeightMax);
          continue;
        }
        obs_intention_weight_vector.emplace_back(
            ObstacleIntentionType::TURN_RIGHT_INTENTION, kWeightMin);
        continue;
      }
      double weight = fabs(lane_heading_obs_diff) * kTurnHeadingDiffWeight;
      if (current_obstacle_l < 0) {
        weight += fabs(current_obstacle_l) * kTurnLDiffWeight;
      }
      obs_intention_weight_vector.emplace_back(
          ObstacleIntentionType::TURN_RIGHT_INTENTION, weight);
      continue;
    }
    // left turn
    if (lane_heading_obs_diff < 0.0 ||
        (current_obstacle_l < 0.0 &&
         lane->TotalLength() - current_obstacle_s > kLaneLengthThreshold)) {
      ObstacleIntentionType intention =
          ObstacleIntentionType::TURN_LEFT_INTENTION;
      ObsTurnCheck(obs_heading_diff, intention);
      obs_intention_weight_vector.emplace_back(intention, kWeightMin);
      continue;
    }
    double weight = lane_heading_obs_diff * kTurnHeadingDiffWeight;
    if (current_obstacle_l > 0) {
      weight += current_obstacle_l * kTurnLDiffWeight;
    }
    ObstacleIntentionType intention =
        ObstacleIntentionType::TURN_LEFT_INTENTION;
    ObsTurnCheck(obs_heading_diff, intention);
    obs_intention_weight_vector.emplace_back(intention, weight);
    continue;
  }
  return true;
}

bool ObstaclesIntentionDecider::ObsCurrentLaneTurnIntentionProcess(
    const cyberverse::LaneInfoConstPtr lane, const Obstacle &obs,
    const double &obs_heading_diff,
    std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  double lane_heading_diff = GetLaneHeadingDiff(
      current_predicted_state_.x, current_predicted_state_.y, lane);
  // auto points = lane->Points();
  // for (int i=0; i<points.size();i++) {
  //   LOG_ERROR("lane point x: {} y: {}", points[i].x(), points[i].y());
  // }
  if (fabs(lane_heading_diff) >= kTurnClassifyRadLimit) {
    // lane turn
    if (lane_heading_diff < 0) {
      ObstacleIntentionType intention =
          ObstacleIntentionType::TURN_RIGHT_INTENTION;
      ObsTurnCheck(obs_heading_diff, intention);
      if (intention != ObstacleIntentionType::TURN_RIGHT_INTENTION) {
        cur_obs_intention_context_[obs.id()].is_rectified = true;
        LOG_INFO("obs.id {} is {} rectified to {}", obs.id(),
                 intention_map_[ObstacleIntentionType::TURN_RIGHT_INTENTION],
                 intention_map_[intention]);
      }
      cur_obs_intention_context_[obs.id()].right_lane_id = lane->Id();
      obs_intention_weight_vector.emplace_back(intention, kWeightMax);
      return false;
    }
    ObstacleIntentionType intention =
        ObstacleIntentionType::TURN_LEFT_INTENTION;
    ObsTurnCheck(obs_heading_diff, intention);
    if (intention != ObstacleIntentionType::TURN_LEFT_INTENTION) {
      cur_obs_intention_context_[obs.id()].is_rectified = true;
      LOG_INFO("obs.id {} is {} rectified to {}", obs.id(),
               intention_map_[ObstacleIntentionType::TURN_LEFT_INTENTION],
               intention_map_[intention]);
    }
    obs_intention_weight_vector.emplace_back(intention, kWeightMax);
    return false;
  }
  return true;
}

void ObstaclesIntentionDecider::FilterObsIntentionsProcess(
    TaskInfo &task_info) {
  std::unordered_map<int, std::vector<ObstaclesIntention>>
      filter_obs_intentions;
  if (task_info.current_frame()
          ->mutable_outside_planner_data()
          ->obs_intention.size() < 1) {
    task_info.current_frame()
        ->mutable_outside_planner_data()
        ->filter_obs_intentions = filter_obs_intentions;
    return;
  }
  for (auto &obs_intention : task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->obs_intention) {
    if (!ObsRiskCheck(obs_intention)) {
      filter_obs_intentions.insert(obs_intention);
      LOG_INFO("FilterObsIntentionsProcess obstacle id: {}",
               obs_intention.first);
    }
  }
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->filter_obs_intentions = filter_obs_intentions;
}

bool ObstaclesIntentionDecider::ObsRiskCheck(
    const std::pair<int,
                    std::vector<neodrive::global::planning::ObstaclesIntention>>
        &obs_intention) {
  auto it = other_side_obstacles_table_.find(obs_intention.first);
  if (it == other_side_obstacles_table_.end()) {
    return true;
  }
  auto &obs = it->second.back();
  Vec3d relative_obs;
  GetObsRelativeCoordinate(obs, relative_obs);
  // LOG_ERROR("ego_position x: {} y:{} heading:{}, obs_position x: {} y:{}
  // heading:{} velocity_heading:{}, relative_obs x: {} y:{} heading:{}, obs_id:
  // {}", ego_position.x(), ego_position.y(), ego_position.z(),
  // obs_position.x(), obs_position.y(), obs_position.z(),
  // obs->velocity_heading(), relative_obs.x(), relative_obs.y(),
  // relative_obs.z(), obs_intention.first);
  for (auto &intention : obs_intention.second) {
    if (intention.type() == ObstacleIntentionType::GO_FORWARD_INTENTION) {
      if (!ObsGoFowardRiskCheck(relative_obs)) {
        return false;
      }
      continue;
    }
    if (intention.type() == ObstacleIntentionType::TURN_LEFT_INTENTION) {
      if (!ObsTurnLeftRiskCheck(relative_obs)) {
        return false;
      }
      continue;
    }
    if (intention.type() == ObstacleIntentionType::TURN_RIGHT_INTENTION) {
      if (!ObsTurnRightRiskCheck(relative_obs)) {
        return false;
      }
      continue;
    }
  }

  return true;
}
bool ObstaclesIntentionDecider::ObsGoFowardRiskCheck(
    const Vec3d &relative_obs) {
  // Straight-through can be filtered
  return true;
}

bool ObstaclesIntentionDecider::ObsTurnLeftRiskCheck(
    const Vec3d &relative_obs) {
  if (fabs(relative_obs.z()) >= 2.4) {
    // Reverse left turn
    return false;
  }
  if (relative_obs.z() >= 0.785 && relative_obs.z() <= 2.35 &&
      relative_obs.y() < 0) {
    // Right-side left turn
    return false;
  }
  return true;
}

bool ObstaclesIntentionDecider::ObsTurnRightRiskCheck(
    const Vec3d &relative_obs) {
  if (fabs(relative_obs.z()) <= 0.8 && relative_obs.y() > 0 &&
      relative_obs.x() < 10) {
    // Adjacent lane right turn
    return false;
  }
  return true;
}

bool ObstaclesIntentionDecider::ObsCutinIntentionProcess(
    TaskInfo &task_info,
    std::pair<const int, std::vector<Obstacle *>> &obs_pair,
    std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  LOG_INFO("---ObsCutinIntentionProcess---");
  auto &current_obstacle = obs_pair.second.back();
  if (task_info.last_frame() == nullptr) {
    LOG_INFO("task_info.last_frame() == nullptr");
    return true;
  }
  last_path_data_ = task_info.last_frame()->outside_planner_data().path_data;
  if (current_obstacle->max_s() < adc_boundary_.end_s()) {
    LOG_INFO("obs id {} current_obstacle->max_s() < adc_boundary_.end_s() skip",
             current_obstacle->id());
    return true;
  }
  double max_s;
  double max_l;
  double min_s;
  double min_l;
  std::vector<PathPoint> index_vc;
  if (!GetObsMaxMinSLInTrajectory(current_obstacle, min_s, max_s, min_l, max_l,
                                  index_vc)) {
    LOG_INFO("GetObsMaxMinSLInTrajectory current_obstacle fail");
    return false;
  }
  if (index_vc.size() < 4) {
    LOG_INFO("index_vc.size() < 4 process failed");
    return false;
  }
  LOG_INFO("current_obstacle min_s {} max_s {} min_l {} max_l {}", min_s, max_s,
           min_l, max_l);
  if (min_s > last_path_data_->path().path_points().back().s()) {
    LOG_INFO("min_s > s  skip");
  }
  double s_speed, l_speed;
  bool is_dynamic = false;
  double bound = KMaxNumber;
  PathPoint path_point;
  int index;
  double near_x, near_y;
  if (min_l > 0.0) {
    path_point = index_vc[2];
  } else if (max_l < 0.0) {
    path_point = index_vc[3];
  } else {
    path_point = index_vc[1];
  }

  // if
  // (!last_path_data_->path().QueryClosestPointWithIndex({current_obstacle->center().x(),
  // current_obstacle->center().y()}, path_point, index)) {
  //   LOG_INFO("Cannot find closest point");
  //   return true;
  // }
  const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
      {path_point.s(), 0.0}, 0.0, 0.2, 0.2, 0.0);
  LOG_INFO("adc_box center_x {} center_y {} half_width {}", adc_box.center_x(),
           adc_box.center_y(), adc_box.half_width());
  GetObsAvgSLSpeedInTrajectory(obs_pair.second, s_speed, l_speed, is_dynamic);
  LOG_INFO("s_speed {}, l_speed {}, is_dynamic {}", s_speed, l_speed,
           is_dynamic);
  if (!is_dynamic) {
    if (current_obstacle->speed() > kStaticSpeedThreshold) {
      is_dynamic = true;
      double obs_heading_diff = normalize_angle(
          current_predicted_state_odom_.heading - path_point.theta());
      LOG_INFO(
          "current_predicted_state_odom_.heading {} "
          "current_obstacle->velocity_heading() {}, path_point.theta() {} ego "
          "heading {}",
          current_predicted_state_odom_.heading,
          current_obstacle->velocity_heading(), path_point.theta(),
          vehicle_state_.Heading());
      s_speed = current_obstacle->speed() * std::cos(obs_heading_diff);
      if (obs_heading_diff < 0.0) {
        l_speed = fabs(current_obstacle->speed() * std::sin(obs_heading_diff));
      } else {
        l_speed = -current_obstacle->speed() * std::sin(obs_heading_diff);
      }
      LOG_INFO(
          "dynamic update obs_heading_diff {} s_speed {}, l_speed {}, "
          "is_dynamic {}",
          obs_heading_diff, s_speed, l_speed, is_dynamic);
    }
  }
  double time_to_lane_bound = KMaxNumber;
  double speed_heading = 0.0;
  if (fabs(l_speed) > KMinNumber && fabs(s_speed) > KMinNumber) {
    speed_heading = std::atan2(l_speed, s_speed);
  }
  LOG_INFO("speed_heading: {} current_obstacle type {}", speed_heading,
           obs_map_[current_obstacle->type()]);
  if (fabs(speed_heading) > kCutinHeadingThreshold && is_dynamic) {
    LOG_INFO("filter Reverse!");
    return true;
  }
  if (is_dynamic) {
    if (min_l > adc_box.center_y()) {
      // obs on left
      bound = min_l - adc_box.half_width();
      LOG_INFO("bound {}", bound);
      time_to_lane_bound = bound / l_speed;
    } else if (max_l < adc_box.center_y()) {
      // obs on right
      bound = max_l + adc_box.half_width();
      LOG_INFO("bound {}", bound);
      time_to_lane_bound = -bound / l_speed;
    } else {
      LOG_INFO(" else skip");
      return true;
    }
    if (time_to_lane_bound < KMinNumber) {
      LOG_INFO("time_to_lane_bound < KMinNumber {}", time_to_lane_bound);
      return true;
    }
    LOG_INFO("time_to_lane_bound {}", time_to_lane_bound);
    if (time_to_lane_bound < kCutinTimeThreshold) {
      obs_intention_weight_vector.emplace_back(
          ObstacleIntentionType::CUT_IN_INTENTION, kWeightMax);
      if (cur_obs_intention_context_.find(current_obstacle->id()) ==
          cur_obs_intention_context_.end()) {
        ObstacleIntentionContext obs_context;
        obs_context.min_l = min_l;
        obs_context.max_l = max_l;
        obs_context.min_s = min_s;
        obs_context.max_s = max_s;
        obs_context.s_speed = s_speed;
        obs_context.l_speed = l_speed;
        obs_context.sl_speed_heading = speed_heading;
        obs_context.time_to_lane_bound = time_to_lane_bound;
        cur_obs_intention_context_[current_obstacle->id()] = obs_context;
      } else {
        cur_obs_intention_context_[current_obstacle->id()].min_l = min_l;
        cur_obs_intention_context_[current_obstacle->id()].max_l = max_l;
        cur_obs_intention_context_[current_obstacle->id()].min_s = min_s;
        cur_obs_intention_context_[current_obstacle->id()].max_s = max_s;
        cur_obs_intention_context_[current_obstacle->id()].s_speed = s_speed;
        cur_obs_intention_context_[current_obstacle->id()].l_speed = l_speed;
        cur_obs_intention_context_[current_obstacle->id()].sl_speed_heading =
            speed_heading;
        cur_obs_intention_context_[current_obstacle->id()].time_to_lane_bound =
            time_to_lane_bound;
      }
      LOG_INFO(
          "current_obstacle->id() {} min_l {}, max_l {}, min_s {}, max_s {}, "
          "s_speed {}, l_speed {}, speed_heading {}, time_to_lane_bound {}",
          current_obstacle->id(),
          cur_obs_intention_context_[current_obstacle->id()].min_l,
          cur_obs_intention_context_[current_obstacle->id()].max_l,
          cur_obs_intention_context_[current_obstacle->id()].min_s,
          cur_obs_intention_context_[current_obstacle->id()].max_s,
          cur_obs_intention_context_[current_obstacle->id()].s_speed,
          cur_obs_intention_context_[current_obstacle->id()].l_speed,
          cur_obs_intention_context_[current_obstacle->id()].sl_speed_heading,
          cur_obs_intention_context_[current_obstacle->id()]
              .time_to_lane_bound);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive

#include "obstacles_intention_base_decider.h"

#include <stdio.h>

#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_manager.h"

namespace neodrive {
namespace planning {
using cyberverse::LaneInfoConstPtr;

ObstaclesIntentionBaseDecider::~ObstaclesIntentionBaseDecider() { Reset(); }

void ObstaclesIntentionBaseDecider::ResetTable() {
  vehicle_state_list_.clear();
  perception_obstacles_list_.clear();
  obstacles_table_.clear();
  other_side_obstacles_table_.clear();
  current_lane_obstacles_table_.clear();
  time_list_.clear();
  cur_obs_intention_context_.clear();
}

void ObstaclesIntentionBaseDecider::ResetFrame() {
  obstacles_table_.clear();
  other_side_obstacles_table_.clear();
  current_lane_obstacles_table_.clear();
  bicycle_obstacles_table_.clear();
  cur_obs_intention_context_.clear();
}

ErrorCode ObstaclesIntentionBaseDecider::Execute(TaskInfo &task_info) {
  if (task_info.decision_data()->dynamic_obstacle().size() == 0) {
    LOG_INFO("dynamic_obstacle_size 0, SKIP");
    return ErrorCode::PLANNING_OK;
  }
  LOG_INFO(">>>> start execute {}", name_);
  if (!Init(task_info)) {
    LOG_ERROR("Init failed. current_frame == nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool ObstaclesIntentionBaseDecider::Init(TaskInfo &task_info) {
  return task_info.current_frame() != nullptr;
}

bool ObstaclesIntentionBaseDecider::Process(TaskInfo &task_info) {
  // maintain need obs list.
  DynamicPerceptionObstaclesPreProcess(task_info);
  ObstaclesIntentionProcess(task_info);
  return true;
}

bool ObstaclesIntentionBaseDecider::AddFrame(TaskInfo &task_info) {
  auto current_frame_obstacles = task_info.decision_data()->all_obstacle();
  if (current_frame_obstacles.size() == 0) {
    LOG_ERROR("dynamic_obstacle.size() == 0 Skip");
    return false;
  }
  double time = cyber::Time::Now().ToSecond();
  time_list_.emplace_back(time);
  perception_obstacles_list_.emplace_back(current_frame_obstacles);
  vehicle_state_list_.emplace_back(vehicle_state_);

  // for (auto obs: current_frame_obstacles){
  //   LOG_ERROR("ObstaclesIntentionBaseDecider dynamic_obstacle Get obstacle
  //   id:
  //   {}", obs->id());
  // }

  if (perception_obstacles_list_.size() > kPerObsListTimestampNums) {
    perception_obstacles_list_.pop_front();
    time_list_.pop_front();
    vehicle_state_list_.pop_front();
  } else if (perception_obstacles_list_.size() < kPerObsListTimestampMinNums) {
    LOG_INFO("perception_obstacles_list_ size:{}  skip!",
             perception_obstacles_list_.size());
    return false;
  }

  return true;
}
bool ObstaclesIntentionBaseDecider::CheckObs(
    const std::vector<Obstacle *> &obs_table, const std::size_t &j) {
  if (obs_table[j] == nullptr) {
    LOG_INFO("obs is nullptr!");
    return false;
  }
  return true;
}

void ObstaclesIntentionBaseDecider::PerceptionObstaclesTableProcess(
    TaskInfo &task_info) {
  ResetFrame();
  for (std::size_t i = 0; i < perception_obstacles_list_.size(); i++) {
    if (perception_obstacles_list_[i].empty()) {
      continue;
    }
    std::size_t cur_frame_per_obs_size = perception_obstacles_list_[i].size();
    for (std::size_t j = 0; j < cur_frame_per_obs_size; j++) {
      if (!CheckObs(perception_obstacles_list_[i], j) ||
          perception_obstacles_list_[i][j]->is_virtual() ||
          perception_obstacles_list_[i][j]->id() <= 0) {
        continue;
      }
      if (obstacles_table_.find(perception_obstacles_list_[i][j]->id()) ==
          obstacles_table_.end()) {
        // LOG_ERROR("Get new obs: {}", perception_obstacles_list_[i][j]->id());
        obstacles_table_[perception_obstacles_list_[i][j]->id()] = {
            perception_obstacles_list_[i][j]};
        continue;
      }
      // LOG_ERROR("Add old obs: {}", perception_obstacles_list_[i][j]->id());
      obstacles_table_[perception_obstacles_list_[i][j]->id()].emplace_back(
          perception_obstacles_list_[i][j]);
    }
  }
}
bool ObstaclesIntentionBaseDecider::EgoStatePreProcess(TaskInfo &task_info) {
  ego_vehicle_sl_ = task_info.curr_sl();
  adc_boundary_ = task_info.adc_boundary();
  return true;
}

bool ObstaclesIntentionBaseDecider::ClassifyDynamicObstacles(
    TaskInfo &task_info) {
  for (auto &it : obstacles_table_) {
    auto &current_obs = it.second.back();
    if (it.second.size() < 3) {
      continue;
    }
    int key = it.first;
    std::vector<Obstacle *> &obstacleVector = it.second;
    if (current_obs->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO("Get obs: {}, ObstacleType PEDESTRIAN skip ", it.first);
      continue;
    }
    if (current_obs->type() == Obstacle::ObstacleType::UNKNOWN_UNMOVABLE ||
        current_obs->type() == Obstacle::ObstacleType::UNKNOWN) {
      LOG_INFO("Get obs: {}, ObstacleType UNKNOWN_UNMOVABLE or UNKNOWN skip ",
               it.first);
      continue;
    }

    // if (current_obs->speed() < kStaticSpeedThreshold) {
    //   LOG_ERROR("Get obs: {} speed is too small", it.first);
    //   continue;
    // }
    // if (current_obs->type() == Obstacle::ObstacleType::BICYCLE) {
    //   LOG_INFO("Get obs: {}, ObstacleType BICYCLE add ", it.first);
    //   bicycle_obstacles_table_[key] = obstacleVector;
    //   continue;
    // }
    // if (current_obs->type() != Obstacle::ObstacleType::VEHICLE) {
    //   LOG_INFO("Get obs: {}, ObstacleType Not VEHICLE Skip ", it.first);
    //   continue;
    // }
    if (current_obs->speed() > kStaticSpeedThreshold) {
      if (!FilterObs(current_obs)) {
        LOG_INFO("Filter obs: {}, Skip ", it.first);
        continue;
      }
    }
    // LOG_ERROR("obstacleVector size: {} current_obs->center().x(): {},
    // current_obs->center().y(): {}, current_obs->heading(): {}",
    // obstacleVector.size(), current_obs->center().x(),
    // current_obs->center().y(), current_obs->heading());
    ReferencePoint cur_obs_in_ref;
    if (!task_info.reference_line()->GetNearestRefPoint(
            current_obs->center_sl().s(), &cur_obs_in_ref)) {
      LOG_ERROR(
          "fail to get ref point from obstacle {} current_obs->center().x(): "
          "{}, current_obs->center().y(): {}, current_obs->heading(): {}",
          key, current_obs->center().x(), current_obs->center().y(),
          current_obs->heading());
      continue;
    }
    // LOG_ERROR("Get ref point from obstacle {} left_lane_bound(): {}
    // right_lane_bound: {}, cur_obs_sl.l {}", key,
    // cur_obs_in_ref.left_lane_bound(), cur_obs_in_ref.right_lane_bound(),
    // current_obs->center_sl().l());
    double obs_ego_s_diff = current_obs->center_sl().s() - ego_vehicle_sl_.s();
    double obs_ego_l_diff =
        fabs(current_obs->center_sl().l() - ego_vehicle_sl_.l());
    // LOG_ERROR("obs_ego_l_diff: {} obs_ego_s_diff: {}", obs_ego_l_diff,
    // obs_ego_s_diff);
    if (current_obs->center_sl().l() <=
            cur_obs_in_ref.left_lane_bound() - KCurrentLaneBoundBuffer &&
        current_obs->center_sl().l() + cur_obs_in_ref.right_lane_bound() -
                KCurrentLaneBoundBuffer >=
            0) {
      // current lane obstacle
      if (obs_ego_s_diff > kCurrentLaneObsSLimit) {
        // obs front of ego
        // current_lane_obstacles_table_[key] = obstacleVector;
        LOG_INFO("current_lane_obstacles_table_ add {} obstacle", key);
      } else {
        LOG_INFO("current_lane_obstacles_table_ skip {} obstacle  s {} l {}",
                 key, current_obs->center_sl().s(),
                 current_obs->center_sl().l());
      }
      continue;
    }
    // // cutin other side obs
    // if ((current_obs->center_sl().l() < cur_obs_in_ref.left_lane_bound()
    // + 4.0 || current_obs->center_sl().l() + cur_obs_in_ref.right_lane_bound()
    // + 4.0 > 0.0)) {
    //   if (obs_ego_s_diff > 0.0 && obs_ego_s_diff < kOtherLaneObsSMaxLimit){
    //     other_side_cutin_all_obstacles_table_[key] = obstacleVector;
    //   }
    // }
    // other side obstalce
    if (obs_ego_s_diff > kOtherLaneObsSMinLimit &&
        obs_ego_s_diff < kOtherLaneObsSMaxLimit &&
        obs_ego_l_diff < kOtherMaxLObsLimit) {
      // if (!current_obs->is_static() && current_obs->speed() >
      // kStaticSpeedThreshold){
      //   other_side_obstacles_table_[key] = obstacleVector;
      // }
      other_side_obstacles_table_[key] = obstacleVector;
      // LOG_INFO("other_side_obstacles_table_ add {} obstacle , s: {} l: {}",
      // key,
      //          current_obs->center_sl().s(), current_obs->center_sl().l());
    } else {
      LOG_INFO("other_side_obstacles_table_ skip {} obstacle  s {} l {}", key,
               current_obs->center_sl().s(), current_obs->center_sl().l());
    }
    continue;
  }
  return true;
}

bool ObstaclesIntentionBaseDecider::DynamicPerceptionObstaclesPreProcess(
    TaskInfo &task_info) {
  if (!AddFrame(task_info)) return false;
  PerceptionObstaclesTableProcess(task_info);
  if (!EgoStatePreProcess(task_info)) return false;
  if (!ClassifyDynamicObstacles(task_info)) return false;
  return true;
}

bool ObstaclesIntentionBaseDecider::ObstaclesIntentionProcess(
    TaskInfo &task_info) {
  LOG_ERROR("use virtual ERROR");
  return true;
}

bool ObstaclesIntentionBaseDecider::ObsIntentionWeightsProcess(
    const Obstacle &obs,
    std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  if (obs_intention_weight_vector.size() < 1) {
    LOG_ERROR("obstacle intention weight vector is empty");
    return true;
  }
  double totalWeight = 0.0;
  for (const auto &obs_intention_weight : obs_intention_weight_vector) {
    totalWeight += obs_intention_weight.weight;
  }
  auto timestamp = obs.get_time_stamp();
  // LOG_ERROR("obs {} timestamp : {}", obs.id(), timestamp);
  bool sign = false;
  for (auto &obs_intention_weight : obs_intention_weight_vector) {
    sign = false;
    ObstaclesIntention obs_intention;
    obs_intention.set_type(obs_intention_weight.type);
    obs_intention.set_probability(
        (totalWeight > 0) ? obs_intention_weight.weight / totalWeight : 0);
    obs_intention.set_time(timestamp);
    if (cur_frame_obs_intention.find(obs.id()) ==
        cur_frame_obs_intention.end()) {
      // not exit
      cur_frame_obs_intention[obs.id()].emplace_back(obs_intention);
      LOG_INFO("obstacle intention: id {} type {} probability {} time {}",
               obs.id(), intention_map_[obs_intention.type()],
               obs_intention.probability(), obs_intention.time());
      continue;
    }
    for (auto &cur_obs_intention : cur_frame_obs_intention[obs.id()]) {
      if (obs_intention.type() == cur_obs_intention.type()) {
        double probability =
            cur_obs_intention.probability() + obs_intention.probability();
        cur_obs_intention.set_probability(probability);
        LOG_INFO(
            "update obstacle intention: id {} type {} probability {} time {}",
            obs.id(), intention_map_[obs_intention.type()],
            cur_obs_intention.probability(), cur_obs_intention.time());
        sign = true;
        break;
      }
    }
    if (!sign) {
      cur_frame_obs_intention[obs.id()].emplace_back(obs_intention);
      LOG_INFO("!!obstacle intention: id {} type {} probability {} time {}",
               obs.id(), intention_map_[obs_intention.type()],
               obs_intention.probability(), obs_intention.time());
    }
  }
  if (cur_frame_obs_intention.find(obs.id()) != cur_frame_obs_intention.end()) {
    std::vector<ObstaclesIntention> &obstacles =
        cur_frame_obs_intention[obs.id()];
    std::sort(obstacles.begin(), obstacles.end(), CompareByProbability);
  }
  return true;
}
void ObstaclesIntentionBaseDecider::SaveTaskResults(TaskInfo &task_info) {
  return;
}

bool ObstaclesIntentionBaseDecider::CompareByProbability(
    const ObstaclesIntention &a, const ObstaclesIntention &b) {
  return a.probability() > b.probability();
}

void ObstaclesIntentionBaseDecider::SaveObsIntentionContextStr(
    TaskInfo &task_info,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->obs_intention_context_str.clear();
  // LOG_INFO("SaveObsIntentionContextStr, cur_frame_obs_intention size: {}",
  // cur_frame_obs_intention.size());
  for (auto &obs_intention : cur_frame_obs_intention) {
    std::string context_str = "[" + std::to_string(obs_intention.first) + ":";
    for (auto &intention : obs_intention.second) {
      context_str += " " + intention_map_[intention.type()] + ":" +
                     std::to_string(intention.probability());
    }
    context_str += "]";
    task_info.current_frame()
        ->mutable_outside_planner_data()
        ->obs_intention_context_str[obs_intention.first] = context_str;
    // LOG_INFO("sTring: {}", context_str);
  }
}

bool ObstaclesIntentionBaseDecider::ObsIntentionAddAndProcess(
    ObstacleIntentionType intention_type, double weight, const Obstacle &obs,
    std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  obs_intention_weight_vector.emplace_back(intention_type, weight);
  ObsIntentionWeightsProcess(obs, obs_intention_weight_vector,
                             cur_frame_obs_intention);
  return true;
}

void ObstaclesIntentionBaseDecider::FilterObsIntentionsProcess(
    TaskInfo &task_info) {
  LOG_INFO("FilterObsIntentionsProcess is in virtual Error!");
}

bool ObstaclesIntentionBaseDecider::EstimateAccelerationAndTurnRate(
    const std::vector<Obstacle *> &history, double &a, double &turn_rate) {
  double total_acceleration = 0.0;
  double total_turn_rate = 0.0;
  int count = 0;
  a = total_acceleration;
  turn_rate = total_turn_rate;
  if (history.size() < 3) {
    LOG_INFO("history.size() < 3 skip EstimateAccelerationAndTurnRate");
    return true;
  }
  for (int i = history.size() - 1; i >= history.size() - 2; --i) {
    if (history[i] == nullptr || history[i - 1] == nullptr) continue;
    double dt = history[i]->get_time_stamp() - history[i - 1]->get_time_stamp();
    if (dt <= 0 || dt > 1) continue;
    double dv = history[i]->speed() - history[i - 1]->speed();
    double dheading;
    if (history[i]->speed() > kStaticSpeedThreshold) {
      dheading =
          history[i]->velocity_heading() - history[i - 1]->velocity_heading();
    } else {
      dheading = history[i]->heading() - history[i - 1]->heading();
    }

    total_acceleration += dv / dt;
    total_turn_rate += dheading / dt;
    ++count;
  }
  double avg_acceleration = count > 0 ? total_acceleration / count : 0;
  double avg_turn_rate = count > 0 ? total_turn_rate / count : 0;
  a = total_acceleration;
  turn_rate = total_turn_rate;
  return true;
}

ObstaclesIntentionBaseDecider::State ObstaclesIntentionBaseDecider::UpdateState(
    const Obstacle *current_state, double a, double turn_rate, double dt) {
  State new_state;
  new_state.a = a;
  new_state.turn_rate = turn_rate;
  new_state.x =
      current_state->center().x() +
      current_state->speed() * cos(current_state->velocity_heading()) * dt;
  new_state.y =
      current_state->center().y() +
      current_state->speed() * sin(current_state->velocity_heading()) * dt;
  new_state.v = current_state->speed() + a * dt;
  if (current_state->speed() > kStaticSpeedThreshold) {
    new_state.heading =
        normalize_angle(current_state->velocity_heading() + turn_rate * dt);
  } else {
    new_state.heading =
        normalize_angle(current_state->heading() + turn_rate * dt);
  }

  new_state.time = current_state->get_time_stamp() + dt;
  return new_state;
}
ObstaclesIntentionBaseDecider::State
ObstaclesIntentionBaseDecider::GetNextState(
    const Obstacle &obs_utm, const std::vector<Obstacle *> &history) {
  double a, turn_rate;
  EstimateAccelerationAndTurnRate(history, a, turn_rate);
  LOG_ERROR("a: {}, turn_rate: {}", a, turn_rate);
  State predicted_state = UpdateState(&obs_utm, a, turn_rate, KDt);
  current_predicted_state_odom_ =
      UpdateState(history.back(), a, turn_rate, KDt);
  LOG_INFO("tmp_state: x: {}, y: {}, v: {} v heading: {} heading: {}",
           obs_utm.center().x(), obs_utm.center().y(), obs_utm.speed(),
           obs_utm.velocity_heading(), obs_utm.heading());
  LOG_INFO("predicted_state: x: {}, y: {}, v: {} heading: {}",
           predicted_state.x, predicted_state.y, predicted_state.v,
           predicted_state.heading);
  LOG_INFO("predicted_state odom: x: {}, y: {}, v: {} heading: {}",
           current_predicted_state_odom_.x, current_predicted_state_odom_.y,
           current_predicted_state_odom_.v,
           current_predicted_state_odom_.heading);
  return predicted_state;
}
cyberverse::LaneInfoConstPtr ObstaclesIntentionBaseDecider::GetObsCurrentLane(
    const double &obs_heading_diff) {
  cyberverse::LaneInfoConstPtr lane;
  if (!GetNearestLaneWithHeadingForObs(
          current_predicted_state_.x, current_predicted_state_.y, kSearchRadius,
          current_predicted_state_.heading, kSearchAngle, obs_heading_diff,
          lane)) {
    LOG_ERROR("obs: could not match cloest lane.");
    return nullptr;
  }
  LOG_ERROR("obs: [{}] match cloest lane.",
            cyberverse::HDMap::Instance()->GetIdHashString(lane->Id()));
  return lane;
}

cyberverse::LaneInfoConstPtr ObstaclesIntentionBaseDecider::GetObsCurrentLane(
    Obstacle &obs_utm, const double &obs_heading_diff) {
  cyberverse::LaneInfoConstPtr lane;
  if (!GetNearestLaneWithHeadingForObs(
          obs_utm.center().x(), obs_utm.center().y(), kSearchRadius,
          obs_utm.velocity_heading(), kSearchAngle, obs_heading_diff, lane)) {
    LOG_ERROR("obs: could not match cloest lane.");
    return nullptr;
  }
  LOG_ERROR("obs: [{}] match cloest lane.",
            cyberverse::HDMap::Instance()->GetIdHashString(lane->Id()));
  return lane;
}

bool ObstaclesIntentionBaseDecider::GetNearestLaneWithHeadingForObs(
    const double &x, const double &y, const double &radius,
    const double &heading, const double &heading_diff,
    const double &obs_heading_diff, cyberverse::LaneInfoConstPtr &lane) {
  common::math::Vec2d point_2D(x, y);
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  std::size_t lane_size = cyberverse::HDMap::Instance()->GetLanesWithHeading(
      point_2D, radius, heading, heading_diff, &lanes);
  if (lane_size == 0 || lanes.front() == nullptr) {
    return false;
  }
  // if (lane_size == 1) {
  //   lane = lanes.front();
  //   return true;
  // }
  std::vector<cyberverse::LaneInfoConstPtr> lanes_selected;
  lanes_selected.clear();
  LOG_INFO("obs_heading_diff :{}", obs_heading_diff);
  for (const auto &lane : lanes) {
    double lane_heading_diff = GetLaneHeadingDiff(x, y, lane);
    if (obs_heading_diff < 0 && lane_heading_diff >= kTurnClassifyRadLimit) {
      // skip left lane
      LOG_ERROR("skip left lane {}",
                cyberverse::HDMap::Instance()->GetIdHashString(lane->Id()));
      continue;
    }
    // if (obs_heading_diff > 0 && lane_heading_diff <=
    // -kTurnClassifyRadLimit) {
    //   // skip right lane
    //   LOG_ERROR("skip right lane {}",
    //   cyberverse::HDMap::Instance()->GetIdHashString(lane->Id())); continue;
    // }
    lanes_selected.emplace_back(lane);
  }
  if (lanes_selected.size() == 0) {
    return false;
  }
  if (lanes_selected.size() == 1) {
    lane = lanes_selected.front();
    return true;
  }
  uint64_t best_matched_lane;
  double offset = 0.0;
  double heading_deviation = 0.0;
  constexpr double max_vertical_projection = 7.0;
  constexpr double max_horizontal_projection = 3.0;
  auto Cmp = [&](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) -> bool {
    return math::Sign(a.second - b.second) == 0 ? a.first < b.first
                                                : a.second < b.second;
  };
  auto Cost = [&max_vertical_projection](auto dis, auto heading_cost,
                                         double w = 0.5) {
    return w * dis / max_vertical_projection + (1 - w) * heading_cost;
  };
  math::Point ego_pos{x, y};
  math::AD2 ego_heading{std::cos(heading), std::sin(heading)};
  std::pair<uint64_t, double> res{0, std::numeric_limits<double>::infinity()};
  for (auto &lane : lanes_selected) {
    auto &pts = lane->Points();
    auto &segments = lane->Segments();
    std::map<std::pair<double, double>, int, decltype(Cmp)> seg_distance_to_ego(
        Cmp);

    for (int i = 0; i < segments.size(); i++) {
      math::AD2 seg_dir{segments[i].unit_direction().x(),
                        segments[i].unit_direction().y()};
      math::AD2 seg_to_ego{ego_pos.x() - segments[i].start().x(),
                           ego_pos.y() - segments[i].start().y()};
      double dis_y = std::abs(math::Cross(seg_dir, seg_to_ego));
      double dis_x = math::Dot(seg_dir, seg_to_ego);
      if (dis_x > 1e-3) {
        dis_x = std::max(0.0, dis_x - segments[i].length());
      } else if (dis_x < -1e-3) {
        dis_x = -dis_x;
      }
      double dis = math::Length({dis_x, dis_y});
      if (dis_x > max_horizontal_projection ||
          dis_y > max_vertical_projection) {
        LOG_DEBUG("[{}](drop): dis_x->{}, dis_y->{}", i, dis_x, dis_y);
        continue;
      }
      double heading_cost = 0.5 - 0.5 * math::Dot(ego_heading, seg_dir);
      seg_distance_to_ego.insert(
          std::make_pair(std::make_pair(dis, heading_cost), i));
    }
    if (seg_distance_to_ego.empty()) {
      continue;
    }
    int cnt = 0;
    constexpr int max_matched_line_num = 3;
    std::pair<std::pair<int, double>, std::pair<double, double>>
        best_matched_seg{{seg_distance_to_ego.begin()->second,
                          Cost(seg_distance_to_ego.begin()->first.first,
                               seg_distance_to_ego.begin()->first.second)},
                         {seg_distance_to_ego.begin()->first.first,
                          seg_distance_to_ego.begin()->first.second}};
    for (auto &item : seg_distance_to_ego) {
      double cost = Cost(item.first.first, item.first.second);
      double best_cost = best_matched_seg.first.second;
      if (math::Sign(cost - best_matched_seg.first.second, 1e-5) == 0) {
        cost = Cost(item.first.first, item.first.second, 0.4);
        best_cost = Cost(best_matched_seg.second.first,
                         best_matched_seg.second.second, 0.4);
      }
      if (cost < best_cost) {
        best_matched_seg.first.second =
            Cost(item.first.first, item.first.second);
        best_matched_seg.first.first = item.second;
        best_matched_seg.second.first = item.first.first;
        best_matched_seg.second.second = item.first.second;
      }
      if (++cnt > max_matched_line_num) {
        break;
      }
    }
    if (res.second > best_matched_seg.first.second) {
      res.first = lane->Id();
      res.second = best_matched_seg.first.second;
      heading_deviation =
          math::Sign(math::Cross(
              ego_heading,
              math::AD2{
                  segments[best_matched_seg.first.first].unit_direction().x(),
                  segments[best_matched_seg.first.first]
                      .unit_direction()
                      .y()})) *
          std::acos(1 - best_matched_seg.second.second * 2);
      offset = math::Distance(
          ego_pos, math::LineSegment{
                       {segments[best_matched_seg.first.first].start().x(),
                        segments[best_matched_seg.first.first].start().y()},
                       {segments[best_matched_seg.first.first].end().x(),
                        segments[best_matched_seg.first.first].end().y()}});
    }
  }
  if (res.first == 0 && res.second == std::numeric_limits<double>::infinity()) {
    return false;
  }
  best_matched_lane = res.first;
  lane = cyberverse::HDMap::Instance()->GetLaneById(best_matched_lane);
  return true;
}

double ObstaclesIntentionBaseDecider::GetLaneHeadingDiff(
    const double &x, const double &y,
    const cyberverse::LaneInfoConstPtr &lane) {
  double total_length = lane->TotalLength();
  double s, l;
  lane->GetProjection(common::math::Vec2d(x, y), &s, &l);
  double end_heading = s + kEndSThreshold < lane->TotalLength()
                           ? lane->Heading(s + kEndSThreshold)
                           : lane->end_heading();
  double start_heading = s - kStartSThreshold > 0
                             ? lane->Heading(s - kStartSThreshold)
                             : lane->heading(0);
  double lane_heading_diff = normalize_angle(end_heading - start_heading);
  // LOG_INFO("current s:{}, lane->TotalLength():{},  lane_heading_diff: {},
  // lane->heading(0):{} , lane->end_heading():{}, lane->Heading(s +
  // kEndSThreshold):{}, lane->Heading(s - kStartSThreshold):{},
  // kTurnClassifyRadLimit: {}, lane_id: {}", s, lane->TotalLength(),
  // lane_heading_diff, lane->heading(0), lane->end_heading(), lane->Heading(s +
  // kEndSThreshold), lane->Heading(s - kStartSThreshold),
  // kTurnClassifyRadLimit,
  // cyberverse::HDMap::Instance()->GetIdHashString(lane->Id()));
  return lane_heading_diff;
}

bool ObstaclesIntentionBaseDecider::ObsTurnCheck(
    const double &obs_heading_diff, ObstacleIntentionType &intention_type) {
  // check obs_heading_diff is same with intention_type
  // and modify when it is the wrong lane
  LOG_INFO("Intention: {}, obs_heading_diff: {}",
           intention_map_[intention_type], obs_heading_diff);
  if (intention_type == ObstacleIntentionType::TURN_RIGHT_INTENTION) {
    intention_type = obs_heading_diff > 0.3
                         ? ObstacleIntentionType::TURN_LEFT_INTENTION
                         : ObstacleIntentionType::TURN_RIGHT_INTENTION;
    LOG_INFO("After Intention: {}", intention_map_[intention_type]);
    return true;
  }

  if (intention_type == ObstacleIntentionType::TURN_LEFT_INTENTION) {
    intention_type = obs_heading_diff < -0.3
                         ? ObstacleIntentionType::TURN_RIGHT_INTENTION
                         : ObstacleIntentionType::TURN_LEFT_INTENTION;
    LOG_INFO("After Intention: {}", intention_map_[intention_type]);
    return true;
  }
  return true;
}

bool ObstaclesIntentionBaseDecider::ObsGoForwardCheck(
    ObstacleIntentionType &intention_type, const double &obs_heading_diff,
    double &weight) {
  LOG_INFO("Intention: GO {} is_left_lane_tmp_ {} is_right_lane_tmp_ {}",
           current_predicted_state_.turn_rate);
  static constexpr double kObsHeadingDiffLimit = 0.2;
  static constexpr double kObsHeadingDiffLeftTurnLimit = 0.785;
  static constexpr double kObsHeadingDiffRightTurnLimit = -0.785;
  static constexpr double KLeftTurnRateThreshold_2 =
      KLeftTurnRateThreshold * 0.5;
  static constexpr double KRightTurnRateThreshold_2 =
      KRightTurnRateThreshold * 0.5;

  if (fabs(obs_heading_diff) < kObsHeadingDiffLimit) {
    return true;
  } else if (obs_heading_diff > kObsHeadingDiffLeftTurnLimit) {
    intention_type = ObstacleIntentionType::TURN_LEFT_INTENTION;
    weight = kWeightMax;
    LOG_INFO("obs_heading_diff > 0.785 After Intention: {}",
             intention_map_[intention_type]);
    return false;
  } else if (obs_heading_diff < kObsHeadingDiffRightTurnLimit) {
    intention_type = ObstacleIntentionType::TURN_RIGHT_INTENTION;
    weight = kWeightMax;
    LOG_INFO("obs_heading_diff < -0.785 After Intention: {}",
             intention_map_[intention_type]);
    return false;
  }

  if (is_left_lane_tmp_ &&
      current_predicted_state_.turn_rate > KLeftTurnRateThreshold_2 &&
      obs_heading_diff > 0) {
    intention_type = ObstacleIntentionType::TURN_LEFT_INTENTION;
    weight = current_predicted_state_.turn_rate * KLeftTurnRateWeight;
    LOG_INFO("After Intention: {}", intention_map_[intention_type]);
    return false;
  }
  if (is_right_lane_tmp_ &&
      current_predicted_state_.turn_rate <= KRightTurnRateThreshold_2 &&
      obs_heading_diff < 0) {
    intention_type = ObstacleIntentionType::TURN_RIGHT_INTENTION;
    weight = current_predicted_state_.turn_rate * KRightTurnRateWeight;
    LOG_INFO("After Intention: {}", intention_map_[intention_type]);
    return false;
  }
  return true;
}

bool ObstaclesIntentionBaseDecider::GetObsRelativeCoordinate(
    const Obstacle *obs, Vec3d &relative_obs) {
  Vec3d obs_position{obs->center().x(), obs->center().y(),
                     obs->velocity_heading()};
  Vec3d ego_position{vehicle_state_.X(), vehicle_state_.Y(),
                     vehicle_state_.Heading()};
  common::ConvertToRelativeCoordinate(obs_position, ego_position, relative_obs);
  return true;
}

bool ObstaclesIntentionBaseDecider::FilterObs(const Obstacle *obs) {
  Vec3d relative_obs;
  GetObsRelativeCoordinate(obs, relative_obs);
  // back filter
  if (relative_obs.x() < 0 && fabs(relative_obs.z()) > kBackFilterThreshold) {
    return false;
  }
  // left 2m filter
  if (relative_obs.y() > kLeftFilterDistanceThreshold &&
      relative_obs.z() > kForwardLeftFilterThreshold) {
    return false;
  }
  // right 2m filter
  if (relative_obs.y() < kRightFilterDistanceThreshold &&
      relative_obs.z() < kForwardRightFilterThreshold) {
    return false;
  }
  // // obs type and position right filter
  // if (obs->type() != Obstacle::ObstacleType::VEHICLE && relative_obs.x() < 0)
  // {
  //   return false;
  // }
  return true;
}
bool ObstaclesIntentionBaseDecider::GetObsAvgSLSpeedInTrajectory(
    const std::vector<Obstacle *> &history, double &s_speed, double &l_speed,
    bool &is_dynamic) {
  s_speed = 0.0;
  l_speed = 0.0;
  is_dynamic = false;
  if (history.size() < 5) {
    LOG_INFO("history.size() < 5");
    return true;
  }
  if (last_path_data_ == nullptr) {
    LOG_INFO("last_path_data_ nullptr Skip!");
    return true;
  }
  if (last_path_data_->path().path_points().size() < 3) {
    LOG_INFO("last_path_data_->frenet_path().size() < 3");
    return true;
  }
  double sum_l = 0.0;
  double sum_s = 0.0;
  double dv_l = 0.0;
  double dv_s = 0.0;
  int count = 0;
  for (int i = history.size() - 1; i >= history.size() - 3; --i) {
    if (history[i] == nullptr || history[i - 2] == nullptr) {
      LOG_INFO("history nullptr!");
      return true;
    }
    double dt = history[i]->get_time_stamp() - history[i - 2]->get_time_stamp();
    if (dt <= 0.001 || dt > 1.0) {
      LOG_INFO("dt <= 0.0 || dt > 1.0 Skip!");
      return true;
    }
    double max_s_i;
    double max_l_i;
    double min_s_i;
    double min_l_i;
    std::vector<PathPoint> index_vc_i;
    if (!GetObsMaxMinSLInTrajectory(history[i], min_s_i, max_s_i, min_l_i,
                                    max_l_i, index_vc_i)) {
      LOG_INFO("GetObsMaxMinSLInTrajectory obs {} fail", i);
      return false;
    }
    LOG_INFO("GET obs {} min_s_i {} max_s_i {} min_l_i {} max_l_i {}", i,
             min_s_i, max_s_i, min_l_i, max_l_i);
    double max_s_i_1;
    double max_l_i_1;
    double min_s_i_1;
    double min_l_i_1;
    std::vector<PathPoint> index_vc_i_1;
    if (!GetObsMaxMinSLInTrajectory(history[i - 2], min_s_i_1, max_s_i_1,
                                    min_l_i_1, max_l_i_1, index_vc_i_1)) {
      LOG_INFO("GetObsMaxMinSLInTrajectory obs {} fail", i);
      return false;
    }
    LOG_INFO("GET obs {} min_s_i_1 {} max_s_i_1 {} min_l_i_1 {} max_l_i_1 {}",
             i - 2, min_s_i_1, max_s_i_1, min_l_i_1, max_l_i_1);

    if (min_l_i > 0.0) {
      // obs on left
      dv_l = (min_l_i_1 - min_l_i) / dt;

    } else if (max_l_i < 0.0) {
      // obs on right
      dv_l = (max_l_i - max_l_i_1) / dt;
    } else {
      LOG_INFO("obs on forward");
      return true;
    }
    dv_s = (min_s_i - min_s_i_1) / dt;
    // if(fabs(dv_l) < KLDiffThreshold){
    //   continue;
    // }
    if (dv_l < 0.0) {
      LOG_INFO("dv_l < 0.0 Skip!");
      return true;
    }
    sum_l += dv_l;
    sum_s += dv_s;
    count++;
    LOG_INFO("dv_l {} dv_s {} sum_l {} sum_s {}", dv_l, dv_s, sum_l, sum_s);
  }
  if (count <= 1) {
    return true;
  }
  s_speed = sum_s / count;
  l_speed = sum_l / count;
  if (sum_l > 0.0) {
    is_dynamic = true;
  } else {
    is_dynamic = false;
  }
  return true;
}

bool ObstaclesIntentionBaseDecider::GetObsMaxMinSLInTrajectory(
    const Obstacle *obs, double &min_s, double &max_s, double &min_l,
    double &max_l, std::vector<PathPoint> &index_vc) {
  auto boundary = obs->PolygonBoundary();
  auto box = obs->bounding_box();
  if (index_vc.empty()) {
    for (int i = 0; i < 4; ++i) {
      index_vc.push_back(PathPoint());
    }
  }
  std::vector<Vec2d> corners;
  max_s = std::numeric_limits<double>::lowest();
  max_l = std::numeric_limits<double>::lowest();
  min_s = std::numeric_limits<double>::max();
  min_l = std::numeric_limits<double>::max();
  box.get_all_corners(&corners);
  if (corners.empty() || last_path_data_ == nullptr ||
      last_path_data_->path().path_points().size() < 3) {
    LOG_INFO("Empty corners or path data");
    return false;
  }
  for (auto &pt : corners) {
    PathPoint path_point;
    int index = 0;
    if (!last_path_data_->path().QueryClosestPointWithIndex(pt, path_point,
                                                            index)) {
      LOG_INFO("Cannot find closest point");
      return true;
    }
    // LOG_INFO("Get near path_point x {} y {} theta {} s {}", path_point.x(),
    // path_point.y(), path_point.theta(), path_point.s());
    Vec3d pt_position{pt.x(), pt.y(), obs->heading()};
    Vec3d path_point_position{path_point.x(), path_point.y(),
                              path_point.theta()};
    Vec3d relative_pt;
    common::ConvertToRelativeCoordinate(pt_position, path_point_position,
                                        relative_pt);
    if (min_s > relative_pt.x() + path_point.s()) {
      min_s = relative_pt.x() + path_point.s();
      index_vc[0] = path_point;
    }
    if (max_s < relative_pt.x() + path_point.s()) {
      max_s = relative_pt.x() + path_point.s();
      index_vc[1] = path_point;
    }
    if (min_l > relative_pt.y()) {
      min_l = relative_pt.y();
      index_vc[2] = path_point;
    }
    if (max_l < relative_pt.y()) {
      max_l = relative_pt.y();
      index_vc[3] = path_point;
    }
  }
  return true;
}

bool ObstaclesIntentionBaseDecider::GetObsAvgSLSpeed(
    const std::vector<Obstacle *> &history, double &s_speed, double &l_speed,
    bool &is_dynamic) {
  s_speed = 0.0;
  l_speed = 0.0;
  is_dynamic = false;
  if (history.size() < 4) {
    LOG_INFO("history.size() < 3");
    return true;
  }
  double sum_l = 0.0;
  double sum_s = 0.0;
  double dv_l = 0.0;
  double dv_s = 0.0;
  for (int i = history.size() - 1; i >= history.size() - 3; --i) {
    if (history[i] == nullptr || history[i - 1] == nullptr) {
      LOG_INFO("history nullptr!");
      return true;
    }
    double dt = history[i]->get_time_stamp() - history[i - 1]->get_time_stamp();
    if (dt <= 0.001 || dt > 1.0) {
      LOG_INFO("dt <= 0.0 || dt > 1.0 Skip!");
      return true;
    }
    if (history[i]->min_l() > ego_vehicle_sl_.l()) {
      // obs on left
      dv_l = (history[i - 1]->min_l() - history[i]->min_l()) / dt;

    } else if (history[i]->max_l() < ego_vehicle_sl_.l()) {
      // obs on right
      dv_l = (history[i]->max_l() - history[i - 1]->max_l()) / dt;
    } else {
      LOG_INFO("obs on forward");
      return true;
    }
    dv_s = (history[i]->min_s() - history[i - 1]->min_s()) / dt;
    if (dv_l < 0.0) {
      LOG_INFO("dv_l < 0.0 Skip!");
      return true;
    }
    sum_l += dv_l;
    sum_s += dv_s;
    LOG_INFO("dv_l {} dv_s {} sum_l {} sum_s {}", dv_l, dv_s, sum_l, sum_s);
  }
  s_speed = sum_s / 3.0;
  l_speed = sum_l / 3.0;
  if (sum_l > 0.0) {
    is_dynamic = true;
  } else {
    is_dynamic = false;
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive

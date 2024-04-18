#include "obstacles_intention_particle_decider.h"

#include <stdio.h>

#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_manager.h"

namespace neodrive {
namespace planning {
using cyberverse::LaneInfoConstPtr;

ObstaclesIntentionParticleDecider::ObstaclesIntentionParticleDecider() {
  name_ = "ObstaclesIntentionParticleDecider";
  ResetTable();
}

ObstaclesIntentionParticleDecider::~ObstaclesIntentionParticleDecider() {
  Reset();
}

bool ObstaclesIntentionParticleDecider::ObstaclesIntentionProcess(
    TaskInfo &task_info) {
  std::unordered_map<int, std::vector<ObstaclesIntention>>
      cur_frame_obs_intention{};
  for (auto &obs : other_side_obstacles_table_) {
    LOG_INFO("Process obs {}", obs.first);
    ObstaclesIntentionParticleProcess(obs, cur_frame_obs_intention);
  }
  task_info.current_frame()->mutable_outside_planner_data()->obs_intention =
      cur_frame_obs_intention;
  last_frame_obs_intention_ = cur_frame_obs_intention;
  FilterObsIntentionsProcess(task_info);
  return true;
}

bool ObstaclesIntentionParticleDecider::ObstaclesIntentionParticleProcess(
    std::pair<const int, std::vector<Obstacle *>> &obs,
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  if (obs.second.empty()) {
    return false;
  }
  // get utm obs
  Obstacle *obs_utm{nullptr};
  if (!data_center_->mutable_object_table_utm()->get_obstacle(obs.first,
                                                              &obs_utm)) {
    LOG_ERROR("Get utm obstacle from object table failed");
    return false;
  }
  auto &current_obstacle = obs.second.back();
  auto &earliest_obstacle = obs.second.front();
  double obs_heading_diff =
      normalize_angle(current_obstacle->velocity_heading() -
                      earliest_obstacle->velocity_heading());
  State predicted_state = GetNextState(*obs_utm, obs.second);
  cyberverse::LaneInfoConstPtr lane =
      GetObsCurrentLane(*obs_utm, obs_heading_diff);
  ObsParticleFilter obs_particle_filter(obs.first, obs.second, *obs_utm, lane,
                                        predicted_state);
  if (!obs_particle_filter.Process()) {
    return false;
  }
  obs_particle_filter.PostProcess(cur_frame_obs_intention);
  for (auto &obs_intention : cur_frame_obs_intention[obs.first]) {
    LOG_INFO("Obstacle id: {}, obstacle intention type: {}, probability: {}",
             obs.first, intention_map_[obs_intention.type()],
             obs_intention.probability());
  }
  return true;
}

void ObstaclesIntentionParticleDecider::FilterObsIntentionsProcess(
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

bool ObstaclesIntentionParticleDecider::ObsRiskCheck(
    const std::pair<int,
                    std::vector<neodrive::global::planning::ObstaclesIntention>>
        &obs_intention) {
  auto it = other_side_obstacles_table_.find(obs_intention.first);
  if (it == other_side_obstacles_table_.end()) {
    return true;
  }
  auto &obs = it->second.back();
  Point3D obs_position;
  obs_position.set_x(obs->center().x());
  obs_position.set_y(obs->center().y());
  obs_position.set_z(obs->velocity_heading());
  Point3D ego_position;
  ego_position.set_x(vehicle_state_.X());
  ego_position.set_y(vehicle_state_.Y());
  ego_position.set_z(vehicle_state_.Heading());
  Point3D relative_obs;
  neodrive::common::ConvertToRelativeCoordinate(obs_position, ego_position,
                                                relative_obs);
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
bool ObstaclesIntentionParticleDecider::ObsGoFowardRiskCheck(
    const Point3D &relative_obs) {
  // Straight-through can be filtered
  return true;
}

bool ObstaclesIntentionParticleDecider::ObsTurnLeftRiskCheck(
    const Point3D &relative_obs) {
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

bool ObstaclesIntentionParticleDecider::ObsTurnRightRiskCheck(
    const Point3D &relative_obs) {
  if (fabs(relative_obs.z()) <= 0.8 && relative_obs.y() > 0 &&
      relative_obs.x() < 10) {
    // Adjacent lane right turn
    return false;
  }
  return true;
}

ParticleFilter::ParticleFilter() {}

ParticleFilter::~ParticleFilter() { particles_.clear(); }
ParticleFilter::ParticleFilter(
    const int &numParticles, const Obstacle &obs_utm,
    std::vector<cyberverse::LaneInfoConstPtr> &lane_sq, const double &dt,
    const ObstaclesIntentionBaseDecider::State &predict_state,
    const double &current_obstacle_s, const double &current_obstacle_l) {
  time_step_predict_ = dt;
  lane_sq_ = lane_sq;
  obs_utm_ = obs_utm;
  current_obstacle_s_ = current_obstacle_s;
  current_obstacle_l_ = current_obstacle_l;
  predict_state_ = predict_state;
  GetLaneSquencesIntention();
  initParticles(particles_, numParticles, obs_utm.center().x(),
                obs_utm.center().y(), obs_utm.velocity_heading(),
                obs_utm.speed());
  predict(particles_, time_step_predict_);
  updateWeights(particles_, predict_state.x, predict_state.y,
                predict_state.heading, predict_state.v);
  // resample(particles_);
}

bool ParticleFilter::GetLaneSquencesIntention() {
  auto &current_lane = lane_sq_.front();
  if (lane_sq_.size() == 1) {
    double lane_heading_diff =
        ObstaclesIntentionBaseDecider::GetLaneHeadingDiff(
            predict_state_.x, predict_state_.y, current_lane);
    // auto points = lane->Points();
    // for (int i=0; i<points.size();i++) {
    //   LOG_ERROR("lane point x: {} y: {}", points[i].x(), points[i].y());
    // }
    if (fabs(lane_heading_diff) >=
        ObstaclesIntentionBaseDecider::kTurnClassifyRadLimit) {
      // lane turn
      if (lane_heading_diff < 0) {
        intention_ = ObstacleIntentionType::TURN_RIGHT_INTENTION;
        return true;
      }
      intention_ = ObstacleIntentionType::TURN_LEFT_INTENTION;
      return true;
    }
    intention_ = ObstacleIntentionType::GO_FORWARD_INTENTION;
    return true;
  }
  double total_length = lane_sq_.back()->TotalLength();
  double end_s = ObstaclesIntentionBaseDecider::kEndSThreshold < total_length
                     ? ObstaclesIntentionBaseDecider::kEndSThreshold
                     : total_length;
  double start_s =
      current_obstacle_s_ - ObstaclesIntentionBaseDecider::kStartSThreshold > 0
          ? current_obstacle_s_ -
                ObstaclesIntentionBaseDecider::kStartSThreshold
          : 0;
  double successor_heading_obs_diff = normalize_angle(
      lane_sq_.back()->Heading(end_s) - current_lane->Heading(start_s));
  LOG_ERROR("successor_heading_obs_diff: {}", successor_heading_obs_diff);
  if (fabs(successor_heading_obs_diff) >=
      ObstaclesIntentionBaseDecider::kTurnClassifyRadLimit) {
    // lane turn
    if (successor_heading_obs_diff < 0) {
      intention_ = ObstacleIntentionType::TURN_RIGHT_INTENTION;
      return true;
    }
    intention_ = ObstacleIntentionType::TURN_LEFT_INTENTION;
    return true;
  }
  intention_ = ObstacleIntentionType::GO_FORWARD_INTENTION;
  return true;
}

void ParticleFilter::initParticles(ParticleSet &particles,
                                   const int &numParticles, const double &x,
                                   const double &y, const double &heading,
                                   const double &speed) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(0, 0.5);
  std::normal_distribution<> d1(0, 0.3);

  for (int i = 0; i < numParticles; ++i) {
    particles.emplace_back(x + d(gen), y + d(gen), heading + d1(gen),
                           speed + d(gen), 1.0 / numParticles);
  }
}

void ParticleFilter::predict(ParticleSet &particles, double dt) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(0, 0.3);
  global::common::PointENU point;
  GetLaneSquencesTargetPoint(point);
  for (auto &particle : particles) {
    double headiing_diff = normalize_angle(point.z() - particle.heading);
    double heading_bias;
    if (headiing_diff >= 0) {
      heading_bias = std::min(kMaxTurnRate * dt, headiing_diff);
    } else {
      heading_bias = -std::min(kMaxTurnRate * dt, -headiing_diff);
    }
    particle.heading += heading_bias * 0.5 + d(gen);
    particle.x += particle.speed * dt * cos(particle.heading) + d(gen);
    particle.y += particle.speed * dt * sin(particle.heading) + d(gen);
  }
}
bool ParticleFilter::GetLaneSquencesTargetPoint(
    global::common::PointENU &point) {
  double target_s = current_obstacle_s_ + 10;
  if (target_s < lane_sq_[0]->TotalLength()) {
    point = lane_sq_[0]->GetSmoothPoint(target_s);
    auto point_end = lane_sq_.back()->GetSmoothPoint(10);
    double heading =
        atan2(point_end.y() - point.y(), point_end.x() - point.x());
    double heading_diff = (4 * lane_sq_[0]->Heading(target_s) + heading) / 5;
    point.set_z(heading_diff);
    return true;
  }
  auto point_2 = lane_sq_[0]->GetSmoothPoint(target_s);
  point = lane_sq_.back()->GetSmoothPoint(10);
  double heading = atan2(point.y() - point_2.y(), point.x() - point_2.x());
  double heading_diff = (4 * lane_sq_[0]->end_heading() + heading) / 5;
  point.set_z(heading_diff);
  return true;
}

void ParticleFilter::updateWeights(ParticleSet &particles,
                                   const double &observed_x,
                                   const double &observed_y,
                                   const double &observed_speed,
                                   const double &observed_heading) {
  double sigma_pos = 0.5;
  double sigma_speed = 0.5;
  double sigma_heading = 0.5;
  weights_sum_ = 0;
  for (auto &particle : particles) {
    double dx = observed_x - particle.x;
    double dy = observed_y - particle.y;
    double position_error =
        exp(-(dx * dx + dy * dy) / (2 * sigma_pos * sigma_pos));
    double speed_error = exp(-((observed_speed - particle.speed) *
                               (observed_speed - particle.speed)) /
                             (2 * sigma_speed * sigma_speed));
    double heading_error =
        exp(-fabs(normalize_angle(particle.heading - observed_heading)) /
            (2 * sigma_heading * sigma_heading));
    particle.weight = position_error * speed_error * heading_error;
    weights_sum_ += particle.weight;
  }
}

void ParticleFilter::resample(ParticleSet &particles) {
  std::vector<double> cumulative_weights(particles.size());
  double sum = 0;
  for (int i = 0; i < particles.size(); ++i) {
    sum += particles[i].weight;
    cumulative_weights[i] = sum;
  }
  weights_sum_ = sum;
  ParticleSet new_particles;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, sum);
  for (int i = 0; i < particles.size(); ++i) {
    double sample = dis(gen);
    int index = std::lower_bound(cumulative_weights.begin(),
                                 cumulative_weights.end(), sample) -
                cumulative_weights.begin();
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
}

std::tuple<double, double, double, double> ParticleFilter::estimateState(
    const ParticleSet &particles) {
  double x_sum = 0, y_sum = 0, heading_sum = 0, speed_sum = 0, weight_sum = 0;
  for (const auto &particle : particles) {
    x_sum += particle.x * particle.weight;
    y_sum += particle.y * particle.weight;
    heading_sum += particle.heading * particle.weight;
    speed_sum += particle.speed * particle.weight;
    weight_sum += particle.weight;
  }

  double x_avg = x_sum / weight_sum;
  double y_avg = y_sum / weight_sum;
  double heading_avg = heading_sum / weight_sum;
  double speed_avg = speed_sum / weight_sum;

  return std::make_tuple(x_avg, y_avg, heading_avg, speed_avg);
}

ObsParticleFilter::ObsParticleFilter() {}
ObsParticleFilter::~ObsParticleFilter() { obs_particle_filterset_.clear(); }
ObsParticleFilter::ObsParticleFilter(
    const int &obs_id, const std::vector<Obstacle *> &obs_table,
    const Obstacle &obs_utm, const cyberverse::LaneInfoConstPtr &current_lane,
    const ObstaclesIntentionBaseDecider::State &state) {
  obs_particle_filterset_.clear();
  obs_table_ = obs_table;
  obs_id_ = obs_id;
  obs_utm_ = obs_utm;
  predict_state_ = state;
  current_lane_ = current_lane;
}

bool ObsParticleFilter::Process() {
  std::vector<std::vector<cyberverse::LaneInfoConstPtr>> lane_squences_2d;
  double current_obstacle_s, current_obstacle_l;
  if (!LaneMatchingProcess(lane_squences_2d, current_obstacle_s,
                           current_obstacle_l)) {
    LOG_ERROR("LaneMatchingProcess Error");
    return false;
  }
  // auto current_obs_ = obs_table_.back();
  for (auto &lane_sq : lane_squences_2d) {
    ParticleFilter obs_pf(numParticles_, obs_utm_, lane_sq, time_step_predict_,
                          predict_state_, current_obstacle_s,
                          current_obstacle_l);
    obs_particle_filterset_[obs_pf.intention_].emplace_back(obs_pf);
  }
  return true;
}

bool ObsParticleFilter::PostProcess(
    std::unordered_map<int, std::vector<ObstaclesIntention>>
        &cur_frame_obs_intention) {
  double totalWeight = 0.0;
  std::unordered_map<neodrive::global::planning::ObstacleIntentionType,
                     neodrive::global::planning::ObstaclesIntention>
      obs_intention_set;
  for (auto &filterset : obs_particle_filterset_) {
    for (auto &obs_particle_filter : filterset.second) {
      if (obs_intention_set.find(filterset.first) == obs_intention_set.end()) {
        ObstaclesIntention obs_intention;
        obs_intention.set_type(filterset.first);
        obs_intention.set_probability(obs_particle_filter.weights_sum_);
        obs_intention.set_time(obs_utm_.get_time_stamp());
        obs_intention_set[filterset.first] = obs_intention;
      } else {
        double probability = obs_intention_set[filterset.first].probability() +
                             obs_particle_filter.weights_sum_;
        obs_intention_set[filterset.first].set_probability(probability);
      }
      totalWeight += obs_particle_filter.weights_sum_;
    }
  }
  for (const auto &intention : intention_map_) {
    if (obs_intention_set.find(intention.first) == obs_intention_set.end()) {
      ObstaclesIntention obs_intention_2;
      obs_intention_2.set_type(intention.first);
      obs_intention_2.set_probability(0);
      obs_intention_2.set_time(obs_utm_.get_time_stamp());
      obs_intention_set[intention.first] = obs_intention_2;
      cur_frame_obs_intention[obs_id_].emplace_back(
          obs_intention_set[intention.first]);
    } else {
      obs_intention_set[intention.first].set_probability(
          obs_intention_set[intention.first].probability() / totalWeight);
      cur_frame_obs_intention[obs_id_].emplace_back(
          obs_intention_set[intention.first]);
    }
  }
  return true;
}

bool ObsParticleFilter::LaneMatchingProcess(
    std::vector<std::vector<cyberverse::LaneInfoConstPtr>> &lane_squences_2d,
    double &current_obstacle_s, double &current_obstacle_l) {
  if (current_lane_ == nullptr) {
    LOG_ERROR("current_lane_ is null");
    return false;
  }
  current_lane_->GetProjection(
      common::math::Vec2d(obs_utm_.center().x(), obs_utm_.center().y()),
      &current_obstacle_s, &current_obstacle_l);
  // double l0 = current_lane_->EndS() - current_obstacle_s;
  double lane_current_s_heading = current_lane_->Heading(current_obstacle_s);
  double obs_lane_heading_diff =
      normalize_angle(obs_utm_.velocity_heading() - lane_current_s_heading);
  if (fabs(obs_lane_heading_diff) > M_PI / 4.0) {
    LOG_ERROR("Not find lane! fabs(obs_lane_heading_diff) > M_PI / 4.0");
    return false;
  }

  // Get obs SuccessorLanes
  std::vector<uint64_t> lanes;
  if (!PlanningMap::Instance()->GetSuccessorLanes(current_lane_->Id(), lanes)) {
    LOG_ERROR("GetSuccessorLanes failed");
    return false;
  }
  // if (lanes.size() == 1)
  // {
  //   auto successor_lane =
  //   cyberverse::HDMap::Instance()->GetLaneById(lanes.back()); if
  //   (successor_lane == nullptr) {
  //     LOG_ERROR("GetLaneById failed");
  //     return false;
  //   }
  //   lane_squences.push_back(successor_lane);
  //   l1 = successor_lane->TotalLength();
  //   total_lenght = l1 + l0;
  //   if (total_lenght >= 15){
  //     return true;
  //   }
  //   lanes.clear();
  //   if (!PlanningMap::Instance()->GetSuccessorLanes(successor_lane->Id(),
  //   lanes)) {
  //     LOG_ERROR("GetSuccessorLanes failed");
  //     return false;
  //   }
  // }
  for (auto &laneid : lanes) {
    auto successor_lane = cyberverse::HDMap::Instance()->GetLaneById(laneid);
    if (successor_lane == nullptr) {
      continue;
    }
    std::vector<cyberverse::LaneInfoConstPtr> lane_sq{current_lane_,
                                                      successor_lane};
    lane_squences_2d.emplace_back(lane_sq);
  }
  if (lane_squences_2d.size() == 0) {
    return false;
  }
  // auto successor_lane = cyberverse::HDMap::Instance()->GetLaneById(lane_id);
  // double lane_heading_diff = normalize_angle(current_lane_->end_heading() -
  // current_lane_->heading(0));
  return true;
}

}  // namespace planning
}  // namespace neodrive

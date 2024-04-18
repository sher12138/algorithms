#include "speed_obs_intention_slow_down_decider.h"

namespace neodrive {
namespace planning {

SpeedObsIntentionSlowDownDecider::SpeedObsIntentionSlowDownDecider() {
  name_ = "SpeedObsIntentionSlowDownDecider";
}

SpeedObsIntentionSlowDownDecider::~SpeedObsIntentionSlowDownDecider() {
  Reset();
}

ErrorCode SpeedObsIntentionSlowDownDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  cur_frame_obs_intention_ =
      task_info.current_frame()->mutable_outside_planner_data()->obs_intention;
  cur_obs_intention_context_ = task_info.current_frame()
                                   ->mutable_outside_planner_data()
                                   ->obs_intention_context;
  speed_limit_ = kMaxSpeed;
  intention_slow_down_ = false;

  LOG_INFO(
      "cur_frame_obs_intention_ size: {}, cur_obs_intention_context_ size {}",
      cur_frame_obs_intention_.size(), cur_obs_intention_context_.size());
  if (cur_frame_obs_intention_.empty()) {
    LOG_INFO(
        "cur_frame_obs_intention_.empty() skip task, "
        "SpeedObsIntentionSlowDown");
    return ErrorCode::PLANNING_OK;
  }
  const auto& vehicle_state = DataCenter::Instance()->vehicle_state_utm();
  cur_speed_ = vehicle_state.LinearVelocity();
  if (cur_speed_ <= kEgoCarStopVelThreshold) {
    LOG_INFO("speed is too low skip task, SpeedObsIntentionSlowDown");
    return ErrorCode::PLANNING_OK;
  }
  // auto bike_driving = task_info.curr_referline_pt().lane_type_is_biking();
  // if (!bike_driving) {
  //   LOG_INFO("Not in bike_driving Skip!");
  //   return ErrorCode::PLANNING_OK;
  // }
  ObsIntentionProcess(task_info);
  return ErrorCode::PLANNING_OK;
}

bool SpeedObsIntentionSlowDownDecider::SelectIntentionObs(
    std::unordered_map<int, double>& turn_right_obs_ids_probs,
    std::unordered_set<int>& agent_ids,
    std::unordered_set<int>& cutin_agent_ids) {
  for (const auto& obs_intention_map : cur_frame_obs_intention_) {
    for (const auto& obs_intention : obs_intention_map.second) {
      if (obs_intention.type() ==
          neodrive::global::planning::ObstacleIntentionType::
              TURN_RIGHT_INTENTION) {
        LOG_INFO(
            "Get agent_id {} lane id :{} hash lane id : {}",
            obs_intention_map.first,
            cur_obs_intention_context_[obs_intention_map.first].lane_id,
            cur_obs_intention_context_[obs_intention_map.first].hash_lane_id);
        agent_ids.insert(obs_intention_map.first);
        turn_right_obs_ids_probs[obs_intention_map.first] =
            obs_intention.probability();
      }
      if (obs_intention.type() ==
          neodrive::global::planning::ObstacleIntentionType::CUT_IN_INTENTION) {
        cutin_agent_ids.insert(obs_intention_map.first);
        LOG_INFO("cutin_agent_ids insert {}", obs_intention_map.first);
      }
    }
  }
  LOG_INFO("agent_ids size : {} cutin_agent_ids.size {}", agent_ids.size(),
           cutin_agent_ids.size());
  if (agent_ids.empty() && cutin_agent_ids.empty()) return false;
  return true;
}

bool SpeedObsIntentionSlowDownDecider::GetRightConnectionConflictInfos(
    TaskInfo& task_info, const std::unordered_set<int>& agent_ids) {
  cyberverse::JunctionInfoConstPtr junction_ptr;
  double distance = kMaxDistance;
  if (!PlanningMap::Instance()->IsJunctionWithinRadiusMetersAhead(
          vehicle_state_.X(), vehicle_state_.Y(), vehicle_state_.Heading(),
          kJunctionRadius, distance, junction_ptr)) {
    LOG_INFO("Not IsJunctionWithinRadiusMetersAhead Skip!");
    is_in_junction_ = false;
    return false;
  }
  if (junction_ptr == nullptr || distance > kJunctionRadius) {
    LOG_INFO("Not found junction_ptr Skip!");
    is_in_junction_ = false;
    return false;
  }
  is_in_junction_ = true;
  LOG_INFO("distance to juction {}", distance);
  cur_right_connection_conflict_infos_.clear();
  is_strong_slow_ = false;
  conflict_data_base_map_.set_interactive_agent_ids(
      agent_ids, MotorwayInteractiveType::CUSTOM);
  cur_right_connection_conflict_infos_ =
      conflict_data_base_map_.ComputeConflictMeetingData(task_info);
  LOG_INFO("cur_right_connection_conflict_infos_ size: {}",
           cur_right_connection_conflict_infos_.size());
  if (cur_right_connection_conflict_infos_.empty()) {
    LOG_INFO("cur_right_connection_conflict_infos_ empty skip!");
    return false;
  }
  for (auto& right_connection_conflict_info :
       cur_right_connection_conflict_infos_) {
    if (right_connection_conflict_info.conflict_area_bound.size() < 4) {
      cur_right_connection_conflict_infos_.clear();
      LOG_INFO("right_connection_conflict_info.conflict_area_bound.size() < 4");
      return false;
    }
    LOG_INFO("Process agent {}", right_connection_conflict_info.agent.id);
    // if
    // (cur_obs_intention_context_.find(right_connection_conflict_info.agent.id)
    // == cur_obs_intention_context_.end()){
    //   LOG_INFO("id not in {}", right_connection_conflict_info.agent.id);
    // }
    LOG_INFO("right_connection_conflict_info lane_id {} obs lane id {}",
             right_connection_conflict_info.lane_id,
             cur_obs_intention_context_[right_connection_conflict_info.agent.id]
                 .lane_id);
    LOG_INFO("conflict_area_bound {} {} {} {}",
             right_connection_conflict_info.conflict_area_bound[0],
             right_connection_conflict_info.conflict_area_bound[1],
             right_connection_conflict_info.conflict_area_bound[2],
             right_connection_conflict_info.conflict_area_bound[3]);
  }
  return true;
}

bool SpeedObsIntentionSlowDownDecider::ProcessRightConnectionConflictInfos(
    std::unordered_set<int>& agent_ids,
    std::unordered_map<int, double>& turn_right_obs_ids_probs) {
  if (cur_right_connection_conflict_infos_.empty()) {
    return true;
  }
  for (auto& right_connection_conflict_info :
       cur_right_connection_conflict_infos_) {
    if (right_connection_conflict_info.conflict_area_bound.size() < 4) {
      cur_right_connection_conflict_infos_.clear();
      LOG_INFO("conflict_area_bound.size() < 4");
      return false;
    }
    double ego_dis = right_connection_conflict_info.conflict_area_bound[3] -
                     kEgoLengthBuffer;
    double ego_center_dis =
        (right_connection_conflict_info.conflict_area_bound[2] +
         right_connection_conflict_info.conflict_area_bound[3]) *
            0.5 -
        kEgoLengthBuffer;
    auto& obs_intention_context =
        cur_obs_intention_context_[right_connection_conflict_info.agent.id];
    double agent_dis = right_connection_conflict_info.conflict_area_bound[0] -
                       obs_intention_context.length * 0.5;
    LOG_INFO("ego_dis {} ego_center_dis {} agent_dis {}", ego_dis,
             ego_center_dis, agent_dis);
    if (ego_dis <= 0.0) {
      ego_dis = ego_center_dis;
      agent_dis = (right_connection_conflict_info.conflict_area_bound[0] +
                   right_connection_conflict_info.conflict_area_bound[1]) *
                      0.5 -
                  obs_intention_context.length * 0.5;
    }
    if (agent_dis <= 0.0) {
      ego_dis = ego_center_dis;
      agent_dis = (right_connection_conflict_info.conflict_area_bound[0] +
                   right_connection_conflict_info.conflict_area_bound[1]) *
                      0.5 -
                  obs_intention_context.length * 0.5;
    }
    if (agent_dis <= 0.0 || ego_dis <= 0.0) {
      LOG_INFO("agent_dis <= 0.0 || ego_dis <= 0.0");
      continue;
    }
    auto& prob =
        turn_right_obs_ids_probs[right_connection_conflict_info.agent.id];
    LOG_INFO("obs id {} prob {}", right_connection_conflict_info.agent.id,
             prob);
    double target_speed =
        std::max(cur_speed_, data_center_->master_info().adc_target_speed());
    if (target_speed <= kEgoCarStopVelThreshold){
      LOG_INFO("target_speed <= kEgoCarStopVelThreshold Error!");
      return true;
    }
    auto obs_tts = agent_dis / obs_intention_context.speed;
    auto ego_tts = ego_dis / target_speed;
    auto diff_tts = obs_tts - ego_tts;
    LOG_INFO("master info target speed {} acc {}",
             data_center_->master_info().adc_target_speed(),
             data_center_->master_info().adc_target_accel());
    LOG_INFO("obs tts {} ego tts {} diff_tts {} cur_speed_ {} obs speed: {}",
             obs_tts, ego_tts, diff_tts, cur_speed_,
             obs_intention_context.speed);
    agent_ids.erase(right_connection_conflict_info.agent.id);
    if (ego_tts > kEgoSlowTimeBuffer) {
      LOG_INFO("kEgoSlowBuffer Skip");
      continue;
    }
    if (fabs(diff_tts) > kTTSGo) {
      LOG_INFO("kTTSGo Skip");
      continue;
    }
    if (ego_tts < kEgoStrongSlowTimeBuffer) {
      is_strong_slow_ = true;
    }
    double t = ego_tts + kTTSGo + diff_tts + kTimeToAreaBuffer;
    if (t <= 0.0) {
      LOG_INFO("t <= 0 Skip");
      continue;
    }
    intention_slow_down_ = true;
    prob = std::min(prob / kProbThreshold, 1.0);
    if (diff_tts > 0.0) {
      double speed_limit = ego_dis / t;
      double speed_prob_limit = (speed_limit - cur_speed_) * prob + cur_speed_;
      LOG_INFO(
          "t {} diff_tts > 0 speed_limit {} cur_speed_ {} speed_prob_limit {}",
          t, speed_limit, cur_speed_, speed_prob_limit);
      speed_limit_ = std::min(speed_prob_limit, speed_limit_);
    } else {
      double speed_limit = ego_dis / t;
      double speed_prob_limit = (speed_limit - cur_speed_) * prob + cur_speed_;
      LOG_INFO(
          "t {} diff_tts > 0 speed_limit {} cur_speed_ {} speed_prob_limit {}",
          t, speed_limit, cur_speed_, speed_prob_limit);
      speed_limit_ = std::min(speed_prob_limit, speed_limit_);
    }
  }
  return true;
}

bool SpeedObsIntentionSlowDownDecider::ObsIntentionProcess(
    TaskInfo& task_info) {
  LOG_INFO("ObsIntentionProcess Processing!");
  // process intention turn right
  std::unordered_map<int, double> turn_right_obs_ids_probs;
  std::unordered_set<int> agent_ids;
  std::unordered_set<int> cutin_agent_ids;
  last_path_data_ = task_info.last_frame()->outside_planner_data().path_data;
  if (last_path_data_ == nullptr) {
    LOG_INFO("last_path_data_ nullptr Skip!");
    return true;
  }
  // auto traffic_conflict_zone_context =
  // task_info.current_frame()->outside_planner_data().traffic_conflict_zone_context;
  // auto meeting_geoinfo = traffic_conflict_zone_context.meeting_geoinfo;
  // for (int i=0; i < 2; ++i){
  //   if (meeting_geoinfo[i] == nullptr) continue;
  //   for (auto &m:*meeting_geoinfo[i]){
  //     double dis = sqrt(pow((DataCenter::Instance()->vehicle_state_utm().X()
  //     - m.first.outlet_left[0]), 2) +
  //     pow((DataCenter::Instance()->vehicle_state_utm().Y() -
  //     m.first.outlet_left[1]), 2)); LOG_INFO("meeting_geoinfo size {},
  //     meeting_geoinfo. vehicle_state_utm().X() {}, vehicle_state_utm().Y() {}
  //     outlet_left:x {} y {} dis:{} orientation:{} lane id {}",
  //       meeting_geoinfo[i]->size(),
  //       DataCenter::Instance()->vehicle_state_utm().X(),
  //       DataCenter::Instance()->vehicle_state_utm().Y(),
  //       m.first.outlet_left[0], m.first.outlet_left[1], dis,
  //       static_cast<int>(m.second.orientation),
  //       m.first.lane_id);
  //   }
  // }
  if (!SelectIntentionObs(turn_right_obs_ids_probs, agent_ids, cutin_agent_ids))
    return false;
  is_in_junction_ = false;
  if (!agent_ids.empty()) {
    if (GetRightConnectionConflictInfos(task_info, agent_ids)) {
      ProcessRightConnectionConflictInfos(agent_ids, turn_right_obs_ids_probs);
    }
  }
  ProcessNoneRightConflictInfos(task_info, agent_ids, turn_right_obs_ids_probs);
  if (!cutin_agent_ids.empty()) {
    ProcessCutinObs(task_info, cutin_agent_ids);
  }

  return true;
}

bool SpeedObsIntentionSlowDownDecider::ProcessNoneRightConflictInfos(
    TaskInfo& task_info, const std::unordered_set<int>& agent_ids,
    std::unordered_map<int, double>& turn_right_obs_ids_probs) {
  std::unordered_set<int> rectified_agent_ids;
  std::unordered_set<int> normal_agent_ids;
  if (!is_in_junction_) {
    LOG_INFO("not in junction skip!");
    return false;
  }
  for (auto& agent_id : agent_ids) {
    LOG_INFO("Process None conflict agent {} x {} y {} heading {}", agent_id,
             cur_obs_intention_context_[agent_id].x,
             cur_obs_intention_context_[agent_id].y,
             cur_obs_intention_context_[agent_id].heading);
    if (cur_obs_intention_context_[agent_id].y <= 0.0) {
      LOG_INFO("[agent_id].y <=0 skip");
      continue;
    }
    if (cur_obs_intention_context_[agent_id].is_rectified) {
      rectified_agent_ids.insert(agent_id);
    } else {
      normal_agent_ids.insert(agent_id);
    }
  }
  ProcessNormalTurnRightObs(task_info, normal_agent_ids,
                            turn_right_obs_ids_probs);
  return true;
}
bool SpeedObsIntentionSlowDownDecider::ProcessNormalTurnRightObs(
    TaskInfo& task_info, std::unordered_set<int>& agent_ids,
    std::unordered_map<int, double>& turn_right_obs_ids_probs) {
  if (agent_ids.empty()) {
    LOG_INFO("ProcessNormalTurnRightObs empty skip!");
    return true;
  }
  std::unordered_set<int> agent_ids_new;
  for (auto agent_id : agent_ids) {
    if (ProcessOneNormalTurnRightObs(task_info, agent_id,
                                     turn_right_obs_ids_probs)) {
      agent_ids_new.insert(agent_id);
    }
  }
  return true;
}

bool SpeedObsIntentionSlowDownDecider::ProcessOneNormalTurnRightObs(
    TaskInfo& task_info, int& agent_id,
    std::unordered_map<int, double>& turn_right_obs_ids_probs) {
  LOG_INFO("ProcessNormalTurnRightObs Processing! {}", agent_id);
  LOG_INFO("agent id {} lane id {} right_lane_id {}", agent_id,
           cur_obs_intention_context_[agent_id].lane_id,
           cur_obs_intention_context_[agent_id].right_lane_id);
  if (cur_obs_intention_context_[agent_id].lane_id == 0 ||
      cur_obs_intention_context_[agent_id].right_lane_id == 0) {
    LOG_INFO("lane is none! skip");
  }
  cyberverse::LaneInfoConstPtr right_lane = nullptr;
  auto lane = cyberverse::HDMap::Instance()->GetLaneById(
      cur_obs_intention_context_[agent_id].lane_id);
  if (cur_obs_intention_context_[agent_id].right_lane_id ==
      cur_obs_intention_context_[agent_id].lane_id) {
    right_lane = lane;
  } else {
    right_lane = cyberverse::HDMap::Instance()->GetLaneById(
        cur_obs_intention_context_[agent_id].right_lane_id);
  }
  if (lane == nullptr || right_lane == nullptr) {
    LOG_INFO("lane or right_lane is nullptr skip!");
    return false;
  }
  LOG_INFO("hash lane id {} right lane hash id {}",
           cur_obs_intention_context_[agent_id].hash_lane_id,
           cyberverse::HDMap::Instance()->GetIdHashString(
               cur_obs_intention_context_[agent_id].right_lane_id));
  std::vector<Vec3d> right_center_points;
  std::vector<neodrive::planning::SLPoint> right_center_sl_points;
  neodrive::planning::SLPoint last_sl_pt{0.0, 0.0};
  ReferencePoint* conflict_ref_pt = nullptr;
  for (size_t i = 0; i < right_lane->Points().size(); i++) {
    auto& pt = right_lane->Points()[i];
    // LOG_INFO("right lane point x {} y {}", pt.x(), pt.y());
    Vec3d relative_pt;
    Vec3d odom_pt{pt.x(), pt.y(), 0.0};
    Vec3d ego_utm_position{data_center_->vehicle_state_utm().X(),
                           data_center_->vehicle_state_utm().Y(),
                           data_center_->vehicle_state_utm().Heading()};
    Vec3d ego_odom_position{data_center_->vehicle_state_odometry().X(),
                            data_center_->vehicle_state_odometry().Y(),
                            data_center_->vehicle_state_odometry().Heading()};
    common::ConvertToRelativeCoordinate(odom_pt, ego_utm_position, relative_pt);
    common::ConvertToWorldCoordinate(relative_pt, ego_odom_position, odom_pt);
    neodrive::planning::SLPoint sl_pt;
    if (!task_info.reference_line()->GetPointInFrenetFrame(
            {odom_pt.x(), odom_pt.y()}, &sl_pt)) {
      LOG_INFO("GetPointInFrenetFrame failed!");
      return false;
    }
    // task_info.reference_line()->GetNearestRefPoint
    right_center_points.emplace_back(odom_pt);
    right_center_sl_points.emplace_back(sl_pt);
    if (last_sl_pt.s() == 0.0) {
      last_sl_pt.set_s(sl_pt.s());
      last_sl_pt.set_l(sl_pt.l());
      continue;
    }
    ReferencePoint conflict_in_ref;
    // LOG_INFO("last_sl_pt->l {} sl_pt.l {} last_sl_pt->s {} sl_pt.s {}",
    // last_sl_pt.l(), sl_pt.l(), last_sl_pt.s(), sl_pt.s());
    if (last_sl_pt.l() > 0.0 && sl_pt.l() < 0) {
      if (!task_info.reference_line()->GetNearestRefPoint(
              {odom_pt.x(), odom_pt.y()}, &conflict_in_ref)) {
        LOG_INFO("GetNearestRefPoint failed");
        return false;
      }
      conflict_ref_pt = &conflict_in_ref;
      last_sl_pt.set_s(sl_pt.s());
      last_sl_pt.set_l(sl_pt.l());
      break;
    } else if (i == right_lane->Points().size() - 1) {
      if (!task_info.reference_line()->GetNearestRefPoint(
              {odom_pt.x(), odom_pt.y()}, &conflict_in_ref)) {
        LOG_INFO("GetNearestRefPoint failed");
        return false;
      }
      if (conflict_in_ref.distance_sqr_to({odom_pt.x(), odom_pt.y()}) <
          kSqrtSeachDisBuffer) {
        LOG_INFO(
            "conflict_in_ref.distance_sqr_to({odom_pt.x(), odom_pt.y()}) < 36 "
            "skip");
        return false;
      }
    }
    last_sl_pt.set_s(sl_pt.s());
    last_sl_pt.set_l(sl_pt.l());
  }

  if (conflict_ref_pt == nullptr) {
    return false;
  }
  double pt_in_lane_s, pt_in_lane_l;
  Vec3d relative_pt;
  Vec3d utm_ref_pt{conflict_ref_pt->x(), conflict_ref_pt->y(), 0.0};
  Vec3d ego_utm_position{data_center_->vehicle_state_utm().X(),
                         data_center_->vehicle_state_utm().Y(),
                         data_center_->vehicle_state_utm().Heading()};
  Vec3d ego_odom_position{data_center_->vehicle_state_odometry().X(),
                          data_center_->vehicle_state_odometry().Y(),
                          data_center_->vehicle_state_odometry().Heading()};
  common::ConvertToRelativeCoordinate(utm_ref_pt, ego_odom_position,
                                      relative_pt);
  common::ConvertToWorldCoordinate(relative_pt, ego_utm_position, utm_ref_pt);
  right_lane->GetProjection({utm_ref_pt.x(), utm_ref_pt.y()}, &pt_in_lane_s,
                            &pt_in_lane_l);
  double agent_dis;
  double agent_dis_end;
  LOG_INFO(
      "pt_in_lane_s {}, pt_in_lane_l {} s_in_lane {} l_in_lane {}, length {} "
      "width {}",
      pt_in_lane_s, pt_in_lane_l,
      cur_obs_intention_context_[agent_id].s_in_lane,
      cur_obs_intention_context_[agent_id].l_in_lane,
      cur_obs_intention_context_[agent_id].length,
      cur_obs_intention_context_[agent_id].width);
  if (cur_obs_intention_context_[agent_id].right_lane_id ==
      cur_obs_intention_context_[agent_id].lane_id) {
    agent_dis = pt_in_lane_s - cur_obs_intention_context_[agent_id].s_in_lane -
                cur_obs_intention_context_[agent_id].length * 0.5 +
                cur_obs_intention_context_[agent_id].l_in_lane -
                cur_obs_intention_context_[agent_id].width * 0.5;
    agent_dis_end = pt_in_lane_s -
                    cur_obs_intention_context_[agent_id].s_in_lane +
                    cur_obs_intention_context_[agent_id].length * 0.5 +
                    cur_obs_intention_context_[agent_id].l_in_lane -
                    cur_obs_intention_context_[agent_id].width * 0.5;
  } else {
    agent_dis = pt_in_lane_s + lane->TotalLength() -
                cur_obs_intention_context_[agent_id].s_in_lane -
                cur_obs_intention_context_[agent_id].length * 0.5 +
                cur_obs_intention_context_[agent_id].l_in_lane -
                cur_obs_intention_context_[agent_id].width * 0.5;
    agent_dis_end = pt_in_lane_s + lane->TotalLength() -
                    cur_obs_intention_context_[agent_id].s_in_lane +
                    cur_obs_intention_context_[agent_id].length * 0.5 +
                    cur_obs_intention_context_[agent_id].l_in_lane -
                    cur_obs_intention_context_[agent_id].width * 0.5;
  }
  double ego_dis = conflict_ref_pt->s() - task_info.adc_boundary().end_s() +
                   cur_obs_intention_context_[agent_id].l_in_lane -
                   cur_obs_intention_context_[agent_id].width * 0.5;
  LOG_INFO("agent_dis {} agent_dis_end {} ego_dis {}", agent_dis, agent_dis_end,
           ego_dis);
  if ((agent_dis < 0.0 && agent_dis_end < 0.0) || ego_dis < 0.0) {
    LOG_INFO("agent_dis agent_dis_end ego_dis < 0 skip!");
    return false;
  }
  std::vector<double> conflict_area_bound{agent_dis, ego_dis};
  auto& prob = turn_right_obs_ids_probs[agent_id];
  LOG_INFO("obs id {} prob {}", agent_id, prob);
  double target_speed =
      std::max(cur_speed_, data_center_->master_info().adc_target_speed());
  auto obs_tts = agent_dis / cur_obs_intention_context_[agent_id].speed;
  auto obs_tts_end = agent_dis_end / cur_obs_intention_context_[agent_id].speed;
  auto ego_tts = ego_dis / target_speed;
  auto diff_tts = obs_tts - ego_tts;
  auto diff_tts_end = obs_tts_end - ego_tts;
  LOG_INFO("master info target speed {} acc {}",
           data_center_->master_info().adc_target_speed(),
           data_center_->master_info().adc_target_accel());
  LOG_INFO(
      "obs tts {} ego tts {} obs_tts_end {} diff_tts {} diff_tts_end {} "
      "cur_speed_ {} obs speed: {}",
      obs_tts, ego_tts, obs_tts_end, diff_tts, diff_tts_end, cur_speed_,
      cur_obs_intention_context_[agent_id].speed);

  if (ego_tts > kEgoSlowTimeBuffer && ego_dis > kEgoDisStartBuffer) {
    LOG_INFO("kEgoSlowBuffer Skip");
    return true;
  }
  if (diff_tts > kTTSGo) {
    LOG_INFO("kTTSGo Skip");
    return true;
  } else if (-diff_tts_end > kTTSGo) {
    LOG_INFO("-diff_tts_end kTTSGo Skip");
    return true;
  }
  if (-diff_tts > kTTSGo && -diff_tts_end < kTTSGo) {
    diff_tts = diff_tts_end;
    LOG_INFO("diff_tts = diff_tts_end");
  }
  if (obs_tts < ego_tts && ego_tts < obs_tts_end &&
      ego_dis < kEgoDisSlowBuffer) {
    LOG_INFO("obs_tts < ego_tts && ego_tts < obs_tts_end speed to 0");
    intention_slow_down_ = true;
    speed_limit_ = 0.0;
    return true;
  }
  if (ego_tts < kEgoStrongSlowTimeBuffer) {
    is_strong_slow_ = true;
  }
  double t = ego_tts + kTTSGo + diff_tts + kTimeToAreaBuffer;
  if (t <= 0.0) {
    LOG_INFO("t <= 0 Skip");
    return true;
  }
  intention_slow_down_ = true;
  prob = std::min(prob / kProbThreshold, 1.0);
  if (diff_tts > 0.0) {
    double speed_limit = ego_dis / t;
    double speed_prob_limit = (speed_limit - cur_speed_) * prob + cur_speed_;
    LOG_INFO(
        "t {} diff_tts > 0 speed_limit {} cur_speed_ {} speed_prob_limit {}", t,
        speed_limit, cur_speed_, speed_prob_limit);
    speed_limit_ = std::min(speed_prob_limit, speed_limit_);
  } else {
    double speed_limit = ego_dis / t;
    double speed_prob_limit = (speed_limit - cur_speed_) * prob + cur_speed_;
    LOG_INFO(
        "t {} diff_tts > 0 speed_limit {} cur_speed_ {} speed_prob_limit {}", t,
        speed_limit, cur_speed_, speed_prob_limit);
    speed_limit_ = std::min(speed_prob_limit, speed_limit_);
  }
  return true;
}

bool SpeedObsIntentionSlowDownDecider::ProcessRectifiedTurnRightObs(
    std::unordered_set<int>& agent_ids,
    std::unordered_map<int, double>& turn_right_obs_ids_probs) {
  if (agent_ids.empty()) {
    LOG_INFO("ProcessRectifiedTurnRightObs empty skip!");
    return true;
  }
  return true;
}

void SpeedObsIntentionSlowDownDecider::SaveTaskResults(TaskInfo& task_info) {
  if (intention_slow_down_) {
    double acc = (speed_limit_ - cur_speed_) * kACCSlowWeight;
    LOG_INFO("Before acc {}", acc);
    acc = std::max(acc, kACCSlowLimit);
    LOG_INFO("After acc {}", acc);
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::OBS_INTENTION);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    if (cur_speed_ >= kEnableACCSlowThreshold && is_strong_slow_) {
      if (acc < 0.0) {
        internal_speed_limit.set_acceleration(acc);
        internal_speed_limit.add_upper_bounds(0.0);
        LOG_INFO(
            "SpeedObsIntentionSlowDownDecider {} limit speed: speed = {:.2f}, "
            "acc "
            "= {:.2f}",
            SpeedLimit_ConstraintType_Name(
                internal_speed_limit.constraint_type()),
            0.0, acc);
      } else {
        internal_speed_limit.set_acceleration(0.0);
        internal_speed_limit.add_upper_bounds(speed_limit_);
        LOG_INFO(
            "SpeedObsIntentionSlowDownDecider {} limit speed: speed = {:.2f}, "
            "acc "
            "= {:.2f}",
            SpeedLimit_ConstraintType_Name(
                internal_speed_limit.constraint_type()),
            speed_limit_, 0.0);
      }
    } else {
      if (fabs(acc) < 0.5 && acc < 0.0) {
        internal_speed_limit.add_upper_bounds(0.0);
        internal_speed_limit.set_acceleration(acc);
        LOG_INFO(
            "SpeedObsIntentionSlowDownDecider {} limit speed: speed = {:.2f}, "
            "acc "
            "= {:.2f}",
            SpeedLimit_ConstraintType_Name(
                internal_speed_limit.constraint_type()),
            0.0, acc);
      } else {
        internal_speed_limit.add_upper_bounds(speed_limit_);
        internal_speed_limit.set_acceleration(0.0);
        LOG_INFO(
            "SpeedObsIntentionSlowDownDecider {} limit speed: speed = {:.2f}, "
            "acc "
            "= {:.2f}",
            SpeedLimit_ConstraintType_Name(
                internal_speed_limit.constraint_type()),
            speed_limit_, 0.0);
      }
    }

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}
bool SpeedObsIntentionSlowDownDecider::ProcessCutinObs(
    TaskInfo& task_info, std::unordered_set<int>& cutin_agent_ids) {
  if (cutin_agent_ids.empty()) {
    LOG_INFO("ProcessCutinObs empty skip!");
    return true;
  }
  PathPoint path_point;
  int index = 0;
  if (!last_path_data_->path().QueryClosestPointWithIndex(
          {vehicle_state_.X(), vehicle_state_.Y()}, path_point, index)) {
    LOG_INFO("Cannot find closest point");
    return true;
  }
  ego_path_point_ = &path_point;
  if (ego_path_point_ == nullptr) {
    LOG_INFO("ego_path_point_ nullptr skip!");
  }
  ego_current_s_in_trajectort_ =
      ego_path_point_->s() + VehicleParam::Instance()->length() * 0.5;
  LOG_INFO("ego_path_point_ s {} ego_current_s_in_trajectort_ {}",
           ego_path_point_->s(), ego_current_s_in_trajectort_);
  for (auto agent_id : cutin_agent_ids) {
    if (ProcessOneCutinObs(task_info, agent_id)) {
    }
  }

  return true;
}

bool SpeedObsIntentionSlowDownDecider::ProcessOneCutinObs(TaskInfo& task_info,
                                                          int& agent_id) {
  const double& time_to_lane_bound =
      cur_obs_intention_context_[agent_id].time_to_lane_bound;
  const double& min_s = cur_obs_intention_context_[agent_id].min_s;
  const double& max_s = cur_obs_intention_context_[agent_id].max_s;
  const double& s_speed = cur_obs_intention_context_[agent_id].s_speed;
  const double& l_speed = cur_obs_intention_context_[agent_id].l_speed;
  const double& min_l = cur_obs_intention_context_[agent_id].min_l;
  const double& max_l = cur_obs_intention_context_[agent_id].max_l;
  const double& sl_speed_heading =
      cur_obs_intention_context_[agent_id].sl_speed_heading;
  LOG_INFO(
      "Process cutin id {} info: min_l {} max_l {} min_s {} max_s {} l_speed "
      "{} s_speed {} sl_speed_heading {} time_to_lane_bound {}",
      agent_id, min_l, max_l, min_s, max_s, l_speed, s_speed, sl_speed_heading,
      time_to_lane_bound);
  double current_s_min_diff = min_s - ego_current_s_in_trajectort_;
  double speed_s_diff = s_speed - cur_speed_;
  LOG_INFO("current_s_min_diff {}, speed_s_diff {} cur_speed_ {}",
           current_s_min_diff, speed_s_diff, cur_speed_);
  double obs_end_s = s_speed * time_to_lane_bound + current_s_min_diff;
  double obs_start_s =
      s_speed * time_to_lane_bound + max_s - ego_current_s_in_trajectort_;
  LOG_INFO("obs_end_s {} obs_start_s {}", obs_end_s, obs_start_s);
  if (speed_s_diff > 0.0 && current_s_min_diff > 0.0) {
    LOG_INFO("obs Speed > ego Skip!");
    return true;
  } else if (speed_s_diff > 0.0) {
    intention_slow_down_ = true;
    double next_s_min_diff =
        min_s + s_speed * time_to_lane_bound - ego_current_s_in_trajectort_ -
        s_speed * kHighSpeedCutinSlowHighRatio * time_to_lane_bound;
    LOG_INFO("next_s_min_diff {}", next_s_min_diff);
    if (next_s_min_diff < 0.0) {
      // is_strong_slow_ = true;
      speed_limit_ = s_speed * kHighSpeedCutinSlowMediRatio;
      LOG_INFO("next_s_min_diff < 0.0 speed_limit_ {}", speed_limit_);
    } else {
      speed_limit_ = s_speed * kHighSpeedCutinSlowHighRatio;
      LOG_INFO("next_s_min_diff > 0.0 speed_limit_ {}", speed_limit_);
    }
    LOG_INFO("cur_speed_ < s_speed slow {}", speed_limit_);
  } else if (speed_s_diff <= 0.0) {
    double t_end = obs_end_s / cur_speed_;
    double t_start = obs_start_s / cur_speed_ + kCutinTimeBuffer;
    LOG_INFO("t_end {} t_start {}", t_end, t_start);
    if (time_to_lane_bound >= t_end && time_to_lane_bound <= t_start) {
      if (s_speed < 0) {
        LOG_INFO("Ego is First");
      } else {
        intention_slow_down_ = true;
        // is_strong_slow_ = true;
        speed_limit_ = std::max(s_speed - kCutinSpeedBuffer, 0.0);
        LOG_INFO(
            "time_to_lane_bound >= t_end && time_to_lane_bound <= t_start "
            "speed_limit_ {}",
            speed_limit_);
      }
    } else if (time_to_lane_bound >= t_start) {
      LOG_INFO("time_to_lane_bound >= t_start skip");
      return true;
    } else if (t_end > time_to_lane_bound + kCutinTimeSlowBuffer &&
               t_end > kMinNum) {
      LOG_INFO("t_end > time_to_lane_bound + kCutinTimeSlowBuffer skip");
      return true;
    } else if (t_end > time_to_lane_bound &&
               t_end <= time_to_lane_bound + kCutinTimeSlowBuffer) {
      intention_slow_down_ = true;
      // is_strong_slow_ = true;
      double diff_time = time_to_lane_bound + kCutinTimeSlowBuffer;
      speed_limit_ = obs_end_s / diff_time;
      LOG_INFO(
          "t_end > time_to_lane_bound && t_end <= time_to_lane_bound + "
          "kCutinTimeSlowBuffer speed_limit_ {}",
          speed_limit_);
      return true;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
#include "sim_lateral_planner.h"

#include <ctime>

namespace neodrive {
namespace planning {
namespace sim_planner {

namespace {
bool getVehicleVertices(const VehicleParam& param, const State& state,
                        std::vector<Vec2d>* vertices) {
  vertices->clear();

  double angle = state.angle;
  double cos_theta = cos(angle);
  double sin_theta = sin(angle);
  double c_x = state.vec_position.x() + param.d_cr() * cos_theta;
  double c_y = state.vec_position.y() + param.d_cr() * sin_theta;
  double d_wx = param.width() / 2 * sin_theta;
  double d_wy = param.width() / 2 * cos_theta;
  double d_lx = param.length() / 2 * cos_theta;
  double d_ly = param.length() / 2 * sin_theta;

  // Counterclockwise from left-front vertex
  vertices->emplace_back(Vec2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
  vertices->emplace_back(Vec2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
  vertices->emplace_back(Vec2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
  vertices->emplace_back(Vec2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy));

  return true;
}

}  // namespace

bool SimLateralPlanner::runEudm(
    const std::vector<std::vector<DcpAction>>& action_script) {
  int n_sequence = action_script.size();
  prepareMultiThreadContainers(n_sequence);
  // loop
  neodrive::planning::Boundary leading_boundary_cur;
  bool ambiguity_cur = false;
  SimMapPoint sm_pt_cur;
  if (!SimMap::Instance()->getNearestPoint(ego_vehicle_.state().vec_position,
                                           &sm_pt_cur)) {
    LOG_ERROR("get nearest point failed");
    return false;
  }

  SLPoint sl_pt_cur(sm_pt_cur.s, sm_pt_cur.l);
  if (!SimMap::Instance()->searchBoundary(
          sl_pt_cur, std::max(ego_vehicle_.state().velocity * 12.0, 30.0),
          &leading_boundary_cur, &ambiguity_cur) &&
      !ambiguity_cur) {
    LOG_ERROR("search front obs failed");
    return false;
  }

  if (last_leading_boundary_.start_s() < 1e+8 &&
      (sm_pt_cur.s <= abs(last_leading_boundary_.end_s()) ||
       leading_boundary_cur.start_s() > 1e+8)) {
    leading_boundary_cur = last_leading_boundary_;
  } else {
    last_leading_boundary_ = leading_boundary_cur;
  }

  neodrive::planning::Boundary center_boundary;
  bool ambiguity_center = false;
  SLPoint sl_pt_center(sm_pt_cur.s, 0.0);

  if (!SimMap::Instance()->searchBoundary(
          sl_pt_center, std::max(ego_vehicle_.state().velocity * 12.0, 30.0),
          &center_boundary, &ambiguity_center) &&
      !ambiguity_center) {
    LOG_ERROR("search front obs failed");
    return false;
  }

  if (leading_boundary_cur.start_s() < 1e+8) {
    SLPoint sl_pt_left_tmp(
        std::max(leading_boundary_cur.start_s(), sm_pt_cur.s),
        leading_boundary_cur.end_l()),
        sl_pt_right_tmp(std::max(leading_boundary_cur.start_s(), sm_pt_cur.s),
                        leading_boundary_cur.start_l());

    SimMapPoint pt_left_tmp, pt_right_tmp;
    int real_left_extend_lane_nums = 1e+8, real_right_extend_lane_nums = 1e+8;
    if (!SimMap::Instance()->getNearestPoint(sl_pt_left_tmp, &pt_left_tmp) ||
        !SimMap::Instance()->getNearestPoint(sl_pt_right_tmp, &pt_right_tmp)) {
      LOG_ERROR("get nearest point failed!");
      return false;
    }
    potential_left_id_ = (fmod(leading_boundary_cur.end_l(), kIntervalL) == 0.0)
                             ? (leading_boundary_cur.end_l() / kIntervalL)
                             : (leading_boundary_cur.end_l() / kIntervalL + 1);
    potential_right_id_ =
        (fmod(leading_boundary_cur.start_l(), kIntervalL) == 0.0)
            ? (leading_boundary_cur.start_l() / kIntervalL)
            : (leading_boundary_cur.start_l() / kIntervalL - 1);
    Vec2d goal_left_pt, goal_right_pt;
    double real_left_extend_length, real_right_extend_length;
    bool is_search_left_pt_failed = false, is_search_right_pt_failed = false;
    if (!SimMap::Instance()->searchPoint(
            Vec2d(pt_left_tmp.x, pt_left_tmp.y), 0.0, 2, &goal_left_pt,
            &real_left_extend_length, &real_left_extend_lane_nums)) {
      LOG_ERROR("search point failed!");
      is_search_left_pt_failed = true;
    }
    if (!SimMap::Instance()->searchPoint(
            Vec2d(pt_right_tmp.x, pt_right_tmp.y), 0.0, -2, &goal_right_pt,
            &real_right_extend_length, &real_right_extend_lane_nums)) {
      LOG_ERROR("search point failed!");
      is_search_right_pt_failed = true;
    }
    if (!is_search_left_pt_failed) {
      potential_left_id_ =
          (potential_left_id_ + real_left_extend_lane_nums < 0)
              ? 0
              : (potential_left_id_ + real_left_extend_lane_nums);
    } else {
      potential_left_id_ = 0;
    }
    if (!is_search_right_pt_failed) {
      potential_right_id_ =
          (potential_right_id_ + real_right_extend_lane_nums > 0)
              ? 0
              : (potential_right_id_ + real_right_extend_lane_nums);
    } else {
      potential_right_id_ = 0;
    }
  }

  for (int i = 0; i < n_sequence; ++i) {
    if (pruningTwoLanes(sm_pt_cur.l, potential_center_id_, potential_left_id_,
                        potential_right_id_, action_script[i])) {
      LOG_INFO("Reduce Branch Action Set:{}", i);
      continue;
    }
    simulateActionSequence(ego_vehicle_, action_script[i], i, center_boundary,
                           sm_pt_cur.s);
  }
  LOG_INFO("[SimLateralPlanner][Process]loop forward simulation finished!");
  bool sim_success = false;
  int num_valid_behaviors = 0;
  for (int i = 0; i < static_cast<int>(sim_res_.size()); ++i) {
    if (sim_res_[i] == 1) {
      sim_success = true;
      num_valid_behaviors++;
    }
  }

  for (int i = 0; i < n_sequence; ++i) {
    std::ostringstream line_info;
    line_info << "[SimLateralPlanner][Result]" << i << " [";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::retLonActionName(a.lon);
    }
    line_info << "|";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::retLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << final_cost_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    if (sim_res_[i]) {
      line_info << "[SimLateralPlanner][Result][e;s;n;w:";
      for (const auto& c : progress_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << "init Has Obs:" << c.has_leading_obs_center << ";"
                  << "potential left id:" << c.potential_left_id << ";"
                  << "potential center id:" << c.potential_center_id << ";"
                  << "potential right id:" << c.potential_right_id << ";"
                  << " Lane ID:" << c.lane_id << ";"
                  << " Current Lane ID:" << c.cur_id << ";"
                  << " Target Lane ID:" << c.target_id << ";"
                  << " Efficiency (expected speed):"
                  << c.efficiency.ego_to_desired_vel
                  << " Efficiency (difference in front vehicle speed):"
                  << c.efficiency.leading_to_desired_vel << ";"
                  << " Center Driving preference:"
                  << c.center_driving.center_driving_preference << ";"
                  << " Weight (reduction):" << c.weight;
        line_info << "|";
      }
      line_info << "]";
    }
    LOG_INFO("{}", line_info.str());
  }

  if (!sim_success) {
    LOG_ERROR("all sim failed!");
    return false;
  }
  if (!evaluateMultiThreadSimResults(action_script, &winner_id_,
                                     &winner_score_)) {
    LOG_ERROR("fail to evaluate multi-thread sim results. Exit!");
    return false;
  }

  return true;
}

bool SimLateralPlanner::init() {
  dcp_tree_ptr_ =
      new DcpTree(cfg_.sim_.duration_.tree_height, cfg_.sim_.duration_.layer,
                  cfg_.sim_.duration_.last_layer);
  getEgoSimParam(cfg_, &ego_sim_param_);

  return true;
}

bool SimLateralPlanner::getEgoSimParam(
    const Cfg& cfg, OnLaneForwardSimulation::Param* sim_param) {
  sim_param->idm_param.kMinimumSpacing =
      cfg.sim_.ego_.ego_lon_.ego_lon_idm_.min_spacing;
  sim_param->idm_param.kDesiredHeadwayTime =
      cfg.sim_.ego_.ego_lon_.ego_lon_idm_.head_time;
  sim_param->idm_param.kAcceleration =
      cfg.sim_.ego_.ego_lon_.ego_lon_limit_.acc;
  sim_param->idm_param.kComfortableBrakingDeceleration =
      cfg.sim_.ego_.ego_lon_.ego_lon_limit_.soft_brake;
  sim_param->idm_param.kHardBrakingDeceleration =
      cfg.sim_.ego_.ego_lon_.ego_lon_limit_.hard_brake;
  sim_param->idm_param.kExponent = cfg.sim_.ego_.ego_lon_.ego_lon_idm_.exponent;
  sim_param->max_lon_acc_jerk = cfg.sim_.ego_.ego_lon_.ego_lon_limit_.acc_jerk;
  sim_param->max_lon_brake_jerk =
      cfg.sim_.ego_.ego_lon_.ego_lon_limit_.brake_jerk;
  sim_param->steer_control_gain =
      cfg.sim_.ego_.ego_lat_.ego_lat_pure_pursuit_.gain;
  sim_param->steer_control_max_lookahead_dist =
      cfg.sim_.ego_.ego_lat_.ego_lat_pure_pursuit_.max_lookahead_dist;
  sim_param->steer_control_min_lookahead_dist =
      cfg.sim_.ego_.ego_lat_.ego_lat_pure_pursuit_.min_lookahead_dist;
  sim_param->max_lat_acceleration_abs =
      cfg.sim_.ego_.ego_lat_.ego_lat_limit_.acc;
  sim_param->max_lat_jerk_abs = cfg.sim_.ego_.ego_lat_.ego_lat_limit_.jerk;
  sim_param->max_curvature_abs =
      cfg.sim_.ego_.ego_lat_.ego_lat_limit_.curvature;
  sim_param->max_steer_angle_abs =
      cfg.sim_.ego_.ego_lat_.ego_lat_limit_.steer_angle;
  sim_param->max_steer_rate = cfg.sim_.ego_.ego_lat_.ego_lat_limit_.steer_rate;
  sim_param->auto_decelerate_if_lat_failed =
      cfg.sim_.ego_.auto_dec_if_lat_failed;

  return true;
}

void SimLateralPlanner::updateDcpTree(const DcpAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->updateScript();
  sim_time_total_ = dcp_tree_ptr_->planning_horizon();
}

bool SimLateralPlanner::prepareMultiThreadContainers(const int n_sequence) {
  sim_res_.clear();
  sim_res_.resize(n_sequence, 0);

  risky_res_.clear();
  risky_res_.resize(n_sequence, 0);

  sim_info_.clear();
  sim_info_.resize(n_sequence, std::string(""));

  final_cost_.clear();
  final_cost_.resize(n_sequence, 0.0);

  progress_cost_.clear();
  progress_cost_.resize(n_sequence);

  forward_trajs_.clear();
  forward_trajs_.resize(n_sequence);

  forward_lat_behaviors_.clear();
  forward_lat_behaviors_.resize(n_sequence);

  forward_lon_behaviors_.clear();
  forward_lon_behaviors_.resize(n_sequence);

  return true;
}

bool SimLateralPlanner::simulateActionSequence(
    const Vehicle& ego_vehicle, const std::vector<DcpAction>& action_seq,
    const int seq_id, const neodrive::planning::Boundary& center_boundary,
    const double cur_s) {
  if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = std::string("(Pre-deleted)");
    return false;
  }
  // Forward simulation result container
  std::vector<int> sub_sim_res(1);
  std::vector<int> sub_risky_res(1);
  std::vector<std::string> sub_sim_info(1);
  std::vector<std::vector<CostStructure>> sub_progress_cost(1);
  std::vector<std::vector<Vehicle>> sub_forward_trajs(1);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(1);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(1);

  simulateScenario(ego_vehicle, action_seq, seq_id, 0, center_boundary, cur_s,
                   &sub_sim_res, &sub_risky_res, &sub_sim_info,
                   &sub_progress_cost, &sub_forward_trajs,
                   &sub_forward_lat_behaviors, &sub_forward_lon_behaviors);

  if (sub_sim_res.front() == 0) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = sub_sim_info.front();
    LOG_ERROR("sub sim res empty!");
    return false;
  }

  sim_res_[seq_id] = 1;
  risky_res_[seq_id] = sub_risky_res.front();
  sim_info_[seq_id] = sub_sim_info.front();
  progress_cost_[seq_id] = sub_progress_cost.front();
  forward_trajs_[seq_id] = sub_forward_trajs.front();
  forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors.front();
  forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors.front();

  return true;
}

bool SimLateralPlanner::simulateScenario(
    const Vehicle& ego_vehicle, const std::vector<DcpAction>& action_seq,
    const int seq_id, const int sub_seq_id,
    const neodrive::planning::Boundary& center_boundary, const double cur_s,
    std::vector<int>* sub_sim_res, std::vector<int>* sub_risky_res,
    std::vector<std::string>* sub_sim_info,
    std::vector<std::vector<CostStructure>>* sub_progress_cost,
    std::vector<std::vector<Vehicle>>* sub_forward_trajs,
    std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
    std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors) {
  // declare variables which will be used to track traces from multiple layers
  std::vector<Vehicle> ego_traj_multilayers{ego_vehicle};

  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructure> cost_multilayers;

  ForwardSimEgoAgent ego_fsagent_this_layer;
  ego_fsagent_this_layer.vehicle = ego_vehicle;
  updateSimSetupForScenario(action_seq, &ego_fsagent_this_layer);

  std::vector<DcpAction> action_seq_sim = action_seq;

  bool has_leading_obs_center =
      (center_boundary.start_s() > 1e+8) ? false : true;
  double leading_obs_center_start_s = center_boundary.start_s();
  // Start forward simulation on an action sequence of ego
  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    auto action_this_layer = action_seq_sim[i];
    if (!updateSimSetupForLayer(action_this_layer, &ego_fsagent_this_layer)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      LOG_ERROR("update sim setup for layer failed");
      return false;
    }
    // Forward simulation of this action (layer)
    std::vector<Vehicle> ego_traj_multisteps;
    // (note: a single "action (lane change/lane
    // keeping/acceleration/deceleration)" may execute multiple time steps)
    if (!simulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                              &ego_traj_multisteps)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      LOG_ERROR("sim single action failed");
      return false;
    }

    // Update forward autonomous vehicle and simulated intelligent agent status
    // (one action may execute multiple time steps)
    ego_fsagent_this_layer.vehicle.setState(ego_traj_multisteps.back().state());

    bool is_strictly_safe = false;
    if (!strictSafetyCheck(ego_traj_multisteps, &is_strictly_safe)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Check F)");
      LOG_ERROR("strict safety check failed");
      return false;
    }
    if (!is_strictly_safe) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Strict F)");
      LOG_ERROR("is no safe");
      return false;
    }

    // Strict check overstep bound
    bool is_overstep_bound = false;
    if (!checkOverStepBound(ego_fsagent_this_layer.vehicle.state(), 0.1,
                            &is_overstep_bound)) {
      LOG_ERROR("bound check failed");
      return false;
    }
    if (is_overstep_bound) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string(" fix crash issues");
      LOG_ERROR("overstep bound");
      return false;
      // LOG_WARN("over step bound!");
    }
    int current_lane_id;
    if (int road_id; !SimMap::Instance()->getRoadLaneId(
            ego_fsagent_this_layer.vehicle.state().vec_position, &road_id,
            &current_lane_id)) {
      LOG_ERROR("get road lane Id failed!");
    }

    if (checkIfLateralActionFinished(ego_fsagent_this_layer.vehicle.state(),
                                     ego_fsagent_this_layer.target_lane,
                                     ego_fsagent_this_layer.current_lane,
                                     ego_fsagent_this_layer.lat_behavior)) {
      LOG_INFO("[Loop]lateral action finished!");
      if (!updateLateralActionSequence(i, &action_seq_sim)) {
        (*sub_sim_res)[sub_seq_id] = 0;
        (*sub_sim_info)[sub_seq_id] += std::string("(Update Lat F)");
        LOG_ERROR("check lateral action finished failed");
        return false;
      }
    }
    // * trace
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.emplace_back(
        ego_fsagent_this_layer.lat_behavior);
    ego_lon_behavior_multilayers.emplace_back(
        ego_fsagent_this_layer.lon_behavior);

    CostStructure cost;
    bool verbose = false;
    if (!costFunction(action_this_layer, ego_fsagent_this_layer, verbose,
                      current_lane_id, &cost)) {
      LOG_ERROR("Cost calculation failed!");
      return false;
    }
    if ((i < action_seq_sim.size() - 1) ||
        ((leading_obs_center_start_s < 1e+8) &&
         (cur_s < leading_obs_center_start_s))) {
      cost.center_driving.center_driving_preference = 0.0;
    }
    cost.has_leading_obs_center = has_leading_obs_center;
    cost.potential_center_id = potential_center_id_;
    cost.potential_left_id = potential_left_id_;
    cost.potential_right_id = potential_right_id_;
    cost.weight = cost.weight * pow(cfg_.cost_.discount_factor, i);
    cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_multilayers.emplace_back(cost);
  }  // Ending a loop on an action sequence

  (*sub_sim_res)[sub_seq_id] = 1;
  (*sub_risky_res)[sub_seq_id] = 0;
  (*sub_progress_cost)[sub_seq_id] = cost_multilayers;
  (*sub_forward_trajs)[sub_seq_id] = ego_traj_multilayers;
  (*sub_forward_lat_behaviors)[sub_seq_id] = ego_lat_behavior_multilayers;
  (*sub_forward_lon_behaviors)[sub_seq_id] = ego_lon_behavior_multilayers;
  return true;
}

bool SimLateralPlanner::updateSimSetupForScenario(
    const std::vector<DcpAction>& action_seq,
    ForwardSimEgoAgent* ego_fsagent) const {
  LateralBehavior seq_lat_behavior;
  double operation_at_seconds;
  bool is_cancel_behavior;
  classifyActionSeq(action_seq, &operation_at_seconds, &seq_lat_behavior,
                    &is_cancel_behavior);
  ego_fsagent->operation_at_seconds = operation_at_seconds;
  ego_fsagent->is_cancel_behavior = is_cancel_behavior;
  ego_fsagent->seq_lat_behavior = seq_lat_behavior;

  // * get action sequence type
  if (is_cancel_behavior) {  // Turn first and then maintain the lane
    ego_fsagent->seq_lat_mode = LatSimMode::kChangeThenCancel;
  } else {
    if (seq_lat_behavior ==
        LateralBehavior::kLaneKeeping) {  // Always maintain lane
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneKeep;
    } else if (action_seq.front().lat ==
               DcpLatAction::kLaneKeeping) {  // Keep the lane before turning
      ego_fsagent->seq_lat_mode = LatSimMode::kKeepThenChange;
    } else {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneChange;  // Always turn
    }
  }

  // * lon
  double desired_vel = std::floor(ego_fsagent->vehicle.state().velocity);
  neodrive::planning::sim_planner::control::IntelligentDriverModel::Param
      idm_param_tmp;
  idm_param_tmp = ego_sim_param_.idm_param;

  switch (action_seq[1].lon) {
    case DcpLonAction::kAccelerate: {
      idm_param_tmp.kDesiredVelocity =
          std::min(desired_vel + cfg_.sim_.acc_cmd_vel_gap, desired_velocity_);
      idm_param_tmp.kMinimumSpacing *=
          (1.0 - cfg_.sim_.ego_.lon_aggressive_ratio);
      idm_param_tmp.kDesiredHeadwayTime *=
          (1.0 - cfg_.sim_.ego_.lon_aggressive_ratio);
      break;
    }
    case DcpLonAction::kDecelerate: {
      idm_param_tmp.kDesiredVelocity =
          std::min(std::max(desired_vel - cfg_.sim_.dec_cmd_vel_gap, 0.0),
                   desired_velocity_);
      break;
    }
    case DcpLonAction::kMaintain: {
      idm_param_tmp.kDesiredVelocity = std::min(desired_vel, desired_velocity_);
      break;
    }
    default: {
      assert(false);
    }
  }
  ego_fsagent->sim_param = ego_sim_param_;
  ego_fsagent->sim_param.idm_param = idm_param_tmp;

  return true;
}

bool SimLateralPlanner::classifyActionSeq(
    const std::vector<DcpAction>& action_seq, double* operation_at_seconds,
    LateralBehavior* lat_behavior, bool* is_cancel_operation) const {
  double duration = 0.0;
  bool find_lat_active_behavior = false;
  *is_cancel_operation = false;
  for (const auto& action : action_seq) {
    if (!find_lat_active_behavior) {
      if (action.lat == DcpLatAction::kLaneChangeLeft) {
        *operation_at_seconds = duration;
        *lat_behavior = LateralBehavior::kLaneChangeLeft;
        find_lat_active_behavior = true;
      }
      if (action.lat == DcpLatAction::kLaneChangeRight) {
        *operation_at_seconds = duration;
        *lat_behavior = LateralBehavior::kLaneChangeRight;
        find_lat_active_behavior = true;
      }
    } else {
      if (action.lat == DcpLatAction::kLaneKeeping) {
        *is_cancel_operation = true;
      }
    }
    duration += action.t;
  }
  if (!find_lat_active_behavior) {
    *operation_at_seconds = duration + cfg_.sim_.duration_.layer;
    *lat_behavior = LateralBehavior::kLaneKeeping;
    *is_cancel_operation = false;
  }

  return true;
}

bool SimLateralPlanner::updateSimSetupForLayer(
    const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const {
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;

  if (!translateDcpActionToLonLatBehavior(action, &lat_behavior,
                                          &lon_behavior)) {
    LOG_ERROR("trans dcp action to lon lat behavior failed");
    return false;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;

  auto state = ego_fsagent->vehicle.state();

  int lane_current = 1e+8, lane_target = 1e+8;

  SimMapPoint sm_pt_this_layer;
  if (!SimMap::Instance()->getNearestPoint(state.vec_position,
                                           &sm_pt_this_layer)) {
    LOG_ERROR("get nearest point failed!");
    return false;
  }
  if (sm_pt_this_layer.l >= potential_center_id_ * kIntervalL) {
    lane_current =
        (sm_pt_this_layer.l >=
         0.5 * (potential_center_id_ + potential_left_id_) * kIntervalL)
            ? potential_left_id_
            : potential_center_id_;
  } else {
    lane_current =
        (sm_pt_this_layer.l >=
         0.5 * (potential_center_id_ + potential_right_id_) * kIntervalL)
            ? potential_center_id_
            : potential_right_id_;
  }

  if (ego_fsagent->lat_behavior == LateralBehavior::kLaneChangeLeft) {
    lane_target = potential_left_id_;
  } else if (ego_fsagent->lat_behavior == LateralBehavior::kLaneChangeRight) {
    lane_target = potential_right_id_;
  } else {
    lane_target = lane_current;
  }
  ego_fsagent->current_lane = lane_current;
  ego_fsagent->target_lane = lane_target;

  if (ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping) {
    neodrive::planning::Boundary front_boundary, rear_boundary;
    // P:The case of obstacle boundaries as virtual obstacles is not currently
    // considered here
    ego_fsagent->obs_boundary.clear();
    ego_fsagent->obs_boundary.insert(
        std::pair<std::string, neodrive::planning::Boundary>("front",
                                                             front_boundary));
    ego_fsagent->obs_boundary.insert(
        std::pair<std::string, neodrive::planning::Boundary>("rear",
                                                             rear_boundary));
  }
  return true;
}

bool SimLateralPlanner::translateDcpActionToLonLatBehavior(
    const DcpAction& action, LateralBehavior* lat,
    LongitudinalBehavior* lon) const {
  switch (action.lat) {
    case DcpLatAction::kLaneKeeping: {
      *lat = LateralBehavior::kLaneKeeping;
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      *lat = LateralBehavior::kLaneChangeLeft;
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      *lat = LateralBehavior::kLaneChangeRight;
      break;
    }
    default: {
      return false;
    }
  }

  switch (action.lon) {
    case DcpLonAction::kMaintain: {
      *lon = LongitudinalBehavior::kMaintain;
      break;
    }
    case DcpLonAction::kAccelerate: {
      *lon = LongitudinalBehavior::kAccelerate;
      break;
    }
    case DcpLonAction::kDecelerate: {
      *lon = LongitudinalBehavior::kDecelerate;
      break;
    }
    default: {
      return false;
    }
  }

  return true;
}

bool SimLateralPlanner::simulateSingleAction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
    std::vector<Vehicle>* ego_traj) {
  // prepare the container for storage in the vehicle and simulate the forward
  // trajectory of other vehicles around it
  ego_traj->clear();

  std::vector<double> dt_steps;
  SimLateralPlanner::getSimTimeSteps(action, &dt_steps);

  ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;

  // Forward simulation dt_ Steps
  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    double sim_time_step = dt_steps[i];

    State ego_state_cache_this_step;

    {
      State state_output;
      if (!SimLateralPlanner::egoAgentForwardSim(
              ego_fsagent_this_step, sim_time_step, &state_output)) {
        LOG_ERROR("ego forward sim failed");
        return false;
      }

      Vehicle v_tmp = ego_fsagent_this_step.vehicle;
      v_tmp.setState(state_output);
      ego_traj->emplace_back(v_tmp);
      ego_state_cache_this_step = state_output;
    }

    ego_fsagent_this_step.vehicle.setState(ego_state_cache_this_step);
  }

  return true;
}

bool SimLateralPlanner::getSimTimeSteps(const DcpAction& action,
                                        std::vector<double>* dt_steps) const {
  double sim_time_resolution = cfg_.sim_.duration_.step;
  double sim_time_total = action.t;
  int n_1 = std::floor(sim_time_total / sim_time_resolution);
  double dt_remain = sim_time_total - n_1 * sim_time_resolution;
  std::vector<double> steps(n_1, sim_time_resolution);
  if (fabs(dt_remain) > kEPS) {
    steps.insert(steps.begin(), dt_remain);
  }
  *dt_steps = steps;
  return true;
}

bool SimLateralPlanner::egoAgentForwardSim(ForwardSimEgoAgent& ego_fsagent,
                                           const double sim_time_step,
                                           State* state_out) const {
  State state_output;
  bool is_safe = false;
  neodrive::planning::Boundary leading_boundary;
  bool ambiguity = false;
  if (!SimMap::Instance()->searchBoundary(
          ego_fsagent.vehicle.state().vec_position,
          std::max(ego_fsagent.vehicle.state().velocity * 3.0, 15.0),
          &leading_boundary, &ambiguity) &&
      !ambiguity) {
    LOG_WARN("search front obs failed");
    // return false;
  }
  if (ego_fsagent.lat_behavior == LateralBehavior::kLaneKeeping) {
    // Lane keeping, only forward facing vehicles in the current lane need to be
    // considered
    if (!strictSingleSafetyCheck(ego_fsagent.vehicle, &is_safe) || !is_safe) {
      LOG_ERROR("strict single safety check failed");
      return false;
    }
    double lat_track_offset = 0.0;
    if (!OnLaneForwardSimulation::propagateOnceAdvancedLK(
            ego_fsagent.current_lane, ego_fsagent.vehicle, leading_boundary,
            lat_track_offset, sim_time_step, ego_fsagent.sim_param,
            &state_output)) {
      LOG_ERROR("LK failed");
      return false;
    }
  } else {
    double look_ahead_dist = 1e+8;
    // The current lane is ahead of the previous car Cl
    if (!SimLateralPlanner::getLookAheadDistOnLC(ego_fsagent, leading_boundary,
                                                 0.8, 3.0, &look_ahead_dist)) {
      LOG_ERROR("get look ahead dist failed");
      return false;
    }
    if (!strictSingleSafetyCheck(ego_fsagent.vehicle, &is_safe) || !is_safe) {
      LOG_ERROR("strict single safe check failed");
      return false;
    }
    double lat_track_offset = 0.0;
    auto sim_param = ego_fsagent.sim_param;
    if (!OnLaneForwardSimulation::propagateOnceAdvancedLC(
            ego_fsagent.current_lane, ego_fsagent.target_lane,
            ego_fsagent.vehicle, ego_fsagent.obs_boundary, leading_boundary,
            lat_track_offset, look_ahead_dist, sim_time_step, sim_param,
            &state_output)) {
      LOG_ERROR("LC failed");
      return false;
    }
  }

  // int ego_lane_id;
  if (SimLateralPlanner::checkIfLateralActionFinished(
          state_output, ego_fsagent.target_lane, ego_fsagent.current_lane,
          ego_fsagent.lat_behavior)) {
    LOG_INFO("[EAFS]lateral action finished!");
    ego_fsagent.lat_behavior = LateralBehavior::kLaneKeeping;
  }

  *state_out = state_output;
  return true;
}

bool SimLateralPlanner::strictSafetyCheck(const std::vector<Vehicle>& ego_traj,
                                          bool* is_safe) {
  if (!cfg_.Safety_.strict_check_enable) {
    *is_safe = true;
    return true;
  }

  int num_points_ego = ego_traj.size();
  if (num_points_ego == 0) {
    *is_safe = true;
    return true;
  }

  for (const auto& entry : ego_traj) {
    bool is_single_safe;
    if (strictSingleSafetyCheck(entry, &is_single_safe)) {
      if (is_single_safe) continue;
      *is_safe = false;
      return true;
    }
    LOG_ERROR("strict safe check failed");
    return false;
  }
  *is_safe = true;
  return true;
}

bool SimLateralPlanner::strictSingleSafetyCheck(const Vehicle& vehicle,
                                                bool* is_safe) const {
  std::vector<Vec2d> vertices;
  *is_safe = false;
  if (!getVehicleVertices(vehicle.param(), vehicle.state(), &vertices)) {
    LOG_ERROR("get vertices failed");
    return false;
  }
  if (!SimMap::Instance()->collisionCheck(
          math::Polygon{{{vertices[0].x(), vertices[0].y()},
                         {vertices[1].x(), vertices[1].y()},
                         {vertices[2].x(), vertices[2].y()},
                         {vertices[3].x(), vertices[3].y()}}})) {
    *is_safe = true;
  }
  return true;
}

bool SimLateralPlanner::checkIfLateralActionFinished(
    const State& cur_state, const int target_lane_id, const int current_lane_id,
    const LateralBehavior& lat_behavior) const {
  if (lat_behavior == LateralBehavior::kLaneKeeping) {
    return false;
  }

  SimMapPoint sm_pt;
  if (!SimMap::Instance()->getNearestPoint(cur_state.vec_position, &sm_pt)) {
    LOG_ERROR("get nearest point failed!");
    return false;
  }

  if ((lat_behavior == LateralBehavior::kLaneChangeLeft) &&
      (sm_pt.l >= (0.5 * (target_lane_id + current_lane_id) * kIntervalL))) {
    return true;
  }
  if ((lat_behavior == LateralBehavior::kLaneChangeRight) &&
      (sm_pt.l <= (0.5 * (target_lane_id + current_lane_id) * kIntervalL))) {
    return true;
  }
  return false;
}

bool SimLateralPlanner::updateLateralActionSequence(
    const int cur_idx, std::vector<DcpAction>* action_seq) const {
  if (cur_idx == static_cast<int>(action_seq->size()) - 1) {
    return true;
  }

  switch ((*action_seq)[cur_idx].lat) {
    case DcpLatAction::kLaneKeeping: {
      // * no need to update
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeRight;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // * LLRRR -> x
          LOG_WARN("LLRRR -> x");
          return false;
        }
      }
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeLeft;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // * RRLLL -> x
          LOG_WARN("RRLLL -> x");
          return false;
        }
      }
      break;
    }
    default: {
      LOG_ERROR("[SimLateralPlanner]Error - Invalid lateral behavior");
      assert(false);
    }
  }

  return true;
}

bool SimLateralPlanner::costFunction(const DcpAction& action,
                                     const ForwardSimEgoAgent& ego_fsagent,
                                     const bool verbose,
                                     const int current_lane_id,
                                     CostStructure* cost) {
  double duration = action.t;

  auto seq_lat_behavior = ego_fsagent.seq_lat_behavior;
  auto is_cancel_behavior = ego_fsagent.is_cancel_behavior;

  double ego_velocity = ego_fsagent.vehicle.state().velocity;
  CostStructure cost_tmp;
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_) {
    cost_tmp.efficiency.ego_to_desired_vel =
        cfg_.cost_.effciency_.ego_lack_speed_to_desired_unit_cost *
        fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_);
  } else {
    if (ego_fsagent.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost_.effciency_.ego_desired_speed_tolerate_gap) {
      cost_tmp.efficiency.ego_to_desired_vel =
          cfg_.cost_.effciency_.ego_over_speed_to_desired_unit_cost *
          fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost_.effciency_.ego_desired_speed_tolerate_gap);
    }
  }
  neodrive::planning::Boundary leading_obs_tar;
  SimMapPoint sm_pt_cur;
  if (!SimMap::Instance()->getNearestPoint(
          ego_fsagent.vehicle.state().vec_position, &sm_pt_cur)) {
    LOG_ERROR("get nearest point failed");
    return false;
  }
  SLPoint sl_pt_tar(sm_pt_cur.s,
                    (ego_fsagent.target_lane - (ego_fsagent.target_lane > 0.0) +
                     (ego_fsagent.target_lane < 0.0)) *
                        kIntervalL);
  bool ambiguity = false;
  if (!SimMap::Instance()->searchBoundary(
          sl_pt_tar, std::max(ego_fsagent.vehicle.state().velocity * 2.0, 15.0),
          &leading_obs_tar, &ambiguity) &&
      !ambiguity) {
    LOG_ERROR("search front obs failed");
    return false;
  }
  double distance_to_leading_obs =
      sqrt(pow(leading_obs_tar.start_s() - sm_pt_cur.s, 2) +
           pow(sl_pt_tar.l() - sm_pt_cur.l, 2));
  double distance_residual_ratio =
      std::max(leading_obs_tar.start_s() - sm_pt_cur.s, 0.0);
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_ &&
      distance_to_leading_obs < cfg_.cost_.effciency_.leading_distance_th) {
    double ego_blocked_by_leading_velocity =
        ego_fsagent.vehicle.state()
            .velocity;  // Speed at which the self driving vehicle is blocked by
                        // the front vehicle: [Self driving speed - front
                        // vehicle speed, 0]+
    double leading_to_desired_velocity =
        desired_velocity_;  // Expected speed from front vehicle: [Expected
                            // speed - Front vehicle speed, 0]+
    cost_tmp.efficiency.leading_to_desired_vel =
        std::max(cfg_.cost_.effciency_.min_distance_ratio,
                 distance_residual_ratio) *
        (cfg_.cost_.effciency_.ego_speed_blocked_by_leading_unit_cost *
             ego_blocked_by_leading_velocity +
         cfg_.cost_.effciency_.leading_speed_blocked_desired_vel_unit_cost *
             leading_to_desired_velocity);  // Efficiency c. Cost of efficiency
                                            // that will be blocked by the front
                                            // car
  }

  // (3) User preference cost
  if (seq_lat_behavior == LateralBehavior::kLaneChangeLeft ||
      seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
    if (is_cancel_behavior) {
      cost_tmp.navigation.lane_change_preference = 0;
    } else {
      cost_tmp.navigation.lane_change_preference = 0.0;
      if (lc_info_.recommend_lc_left &&
          seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost_.navigation_.lane_change_unit_cost_vel_lb,
                      ego_velocity) *
            cfg_.cost_.navigation_.lane_change_left_recommendation_reward;
        if (action.lat != DcpLatAction::kLaneChangeLeft) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost_.navigation_.lane_change_unit_cost_vel_lb,
                       ego_velocity) *
              cfg_.cost_.user_.late_operate_unit_cost;
        }
      } else if (lc_info_.recommend_lc_right &&
                 seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost_.navigation_.lane_change_unit_cost_vel_lb,
                      ego_velocity) *
            cfg_.cost_.navigation_.lane_change_right_recommendation_reward;
        if (action.lat != DcpLatAction::kLaneChangeRight) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost_.navigation_.lane_change_unit_cost_vel_lb,
                       ego_velocity) *
              cfg_.cost_.user_.late_operate_unit_cost;
        }
      }
    }
  }

  cost_tmp.lane_id = current_lane_id;
  cost_tmp.cur_id = ego_fsagent.current_lane;
  cost_tmp.target_id = ego_fsagent.target_lane;
  cost_tmp.center_driving.center_driving_preference =
      abs(current_lane_id) *
      cfg_.cost_.offsetdriving_.lane_center_driving_coeff;
  cost_tmp.weight = duration;
  *cost = cost_tmp;

  return true;
}

bool SimLateralPlanner::evaluateMultiThreadSimResults(
    const std::vector<std::vector<DcpAction>>& action_script, int* winner_id,
    double* winner_cost) {
  double min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();
  for (int i = 0; i < num_sequences; ++i) {
    if (sim_res_[i] == 0) {
      continue;
    }
    double cost = 0.0;
    auto action_seq = action_script[i];
    SimLateralPlanner::evaluateSinglePolicyTrajs(progress_cost_[i], action_seq,
                                                 &cost);
    final_cost_[i] = cost;
    LOG_INFO("cost id: {} cost: {:.3f}", i, cost);
    if (cost < min_cost) {
      min_cost = cost;
      best_id = i;
    }
  }
  *winner_cost = min_cost;
  *winner_id = best_id;

  return true;
}

bool SimLateralPlanner::evaluateSinglePolicyTrajs(
    const std::vector<CostStructure>& progress_cost,
    const std::vector<DcpAction>& action_seq, double* score) {
  double score_tmp = 0.0;
  for (const auto& c : progress_cost) {
    score_tmp += c.ave();
  }
  *score = score_tmp;
  return true;
}

// run_once
bool SimLateralPlanner::runOnce() {
  // Obtain the current nearest lane ID
  if (!SimMap::Instance()) {
    LOG_ERROR("map interface not initialized. Exit!");
    return false;
  }

  ego_id_ = ego_vehicle_.id();
  time_stamp_ = ego_vehicle_.state().time_stamp;
  LOG_INFO(
      "[SimLateralPlanner]------ SimLateralPlanner Cycle Begins (stamp): "
      "{:.4f}",
      time_stamp_);

  int ego_lane_id_by_pos;
  // Obtain the current lane of the vehicle through position
  if (int road_id; !SimMap::Instance()->getRoadLaneId(
          ego_vehicle_.state().vec_position, &road_id, &ego_lane_id_by_pos)) {
    LOG_ERROR("get road & lane id failed");
    return false;
  }
  ego_lane_id_ = ego_lane_id_by_pos;

  LOG_INFO(
      "[SimLateralPlanner][Input]Ego plan state (x,y,theta,v,a,k): {:.3f}, "
      "{:.3f}, {:.3f}, "
      "{:.3f},{:.3f}, {:.3f}",
      ego_vehicle_.state().vec_position.x(),
      ego_vehicle_.state().vec_position.y(), ego_vehicle_.state().angle,
      ego_vehicle_.state().velocity, ego_vehicle_.state().acceleration,
      ego_vehicle_.state().curvature);
  LOG_INFO(
      "[SimLateralPlanner][Setup]Desired vel: {:.3f}, sim_time total: {:.3f}, ",
      desired_velocity_, sim_time_total_);

  /*
  Early branch reduction: Continuous changes from right to left and from left to
  right are unreasonable and should be reduced
  */
  int road_id, lane_id;
  bool cruise_reduction = false;
  if (!SimMap::Instance()->getRoadLaneId(ego_vehicle_.state().vec_position,
                                         &road_id, &lane_id)) {
    LOG_ERROR("get road lane id failed!");
  }
  if (lane_id == 0) {
    bool ambiguity = false;
    neodrive::planning::Boundary leading_boundary;
    if (!SimMap::Instance()->searchBoundary(
            ego_vehicle_.state().vec_position,
            std::max(ego_vehicle_.state().velocity * 15.0, 30.0),
            &leading_boundary, &ambiguity) &&
        !ambiguity) {
      LOG_ERROR("search front obs failed");
      return false;
    }
    if (leading_boundary.start_s() > 1e+8) {
      LOG_INFO("Cruise along the middle lane and reduce branches in advance");
      cruise_reduction = true;
    }
  }

  pre_deleted_seq_ids_.clear();
  // cruise_reduction = false;
  auto action_script = dcp_tree_ptr_->action_script();
  action_script.emplace_back(std::vector<DcpAction>(
      5, DcpAction(DcpLonAction(1), DcpLatAction(1), 1.0)));
  action_script.emplace_back(std::vector<DcpAction>(
      5, DcpAction(DcpLonAction(1), DcpLatAction(2), 1.0)));

  int n_sequence = action_script.size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = action_script[i];
    int num_actions = action_seq.size();
    if (cruise_reduction) {
      LOG_INFO(
          "The current vehicle is located in the middle lane and there are no "
          "obstacles ahead, so the branches are reduced");
      for (int j = 0; j < num_actions; j++) {
        if (action_seq[j].lat != DcpLatAction::kLaneKeeping) {
          pre_deleted_seq_ids_.insert(i);
          break;
        }
      }
    } else {
      for (int j = 1; j < num_actions; j++) {
        if ((action_seq[j - 1].lat == DcpLatAction::kLaneChangeLeft &&
             action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
            (action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
             action_seq[j].lat == DcpLatAction::kLaneChangeLeft)) {
          pre_deleted_seq_ids_.insert(i);
          break;
        }
      }
    }
  }

  // Execute runonce
  if (!runEudm(action_script)) {
    LOG_ERROR("run SimLateralPlanner failed");
    return false;
  }

  std::ostringstream line_info;
  line_info << "[SimLateralPlanner]SUCCESS id:" << winner_id_ << " [";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::retLonActionName(a.lon);
  }
  line_info << "|";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::retLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3)
            << winner_score_;
  LOG_INFO("{}", line_info.str());

  return true;
}

void SimLateralPlanner::setLaneChangeInfo(const LaneChangeInfo& lc_info) {
  lc_info_ = lc_info;
}

void SimLateralPlanner::setVehicleState(const State& state) {
  ego_vehicle_.setState(state);
}

int SimLateralPlanner::winner_id() const { return winner_id_; }

double SimLateralPlanner::time_cost() const { return time_cost_; }

void SimLateralPlanner::setDesiredVelocity(const double desired_vel) {
  desired_velocity_ = std::max(0.0, desired_vel);
}

bool SimLateralPlanner::getLookAheadDistOnLC(
    const ForwardSimEgoAgent& ego,
    const neodrive::planning::Boundary& leading_boundary, const double alpha,
    const double delta, double* look_ahead_dist) const {
  double look_ahead_dist_;
  SimMapPoint sm_pt_ego;
  if (!SimMap::Instance()->getNearestPoint(ego.vehicle.state().vec_position,
                                           &sm_pt_ego)) {
    LOG_ERROR("cor trans failed");
    return false;
  }
  look_ahead_dist_ = leading_boundary.start_s() - sm_pt_ego.s -
                     ego.vehicle.param().front_suspension();
  if (look_ahead_dist_ < 200.0)
    // LOG_INFO("Distance from forward obstacles:{:.3f}", look_ahead_dist_);
    look_ahead_dist_ = std::exp(alpha * (look_ahead_dist_ - delta));
  // LOG_INFO("Distance from forward obstacles:{:.3f}", look_ahead_dist_);
  *look_ahead_dist = look_ahead_dist_;

  return true;
}

bool SimLateralPlanner::checkOverStepBound(const State& state,
                                           const double threshold, bool* res) {
  // int cur_lane_id;
  // if (int road_id; !SimMap::Instance()->getRoadLaneId(state.vec_position,
  //                                                     &road_id,
  //                                                     &cur_lane_id)) {
  //   *res = true;
  //   return true;
  // }
  Vec2d goal_pt;
  double real_extend_length;
  int real_extend_lane_nums;
  double l_bound, r_bound;
  SimMapPoint sm_pt;
  if (!SimMap::Instance()->getNearestPoint(state.vec_position, &sm_pt)) {
    LOG_ERROR("cor trans failed");
    return false;
  }
  if (!SimMap::Instance()->searchRoadBound(state.vec_position, &l_bound,
                                           &r_bound)) {
    LOG_ERROR("Road Bound Search Failed");
    return false;
  }
  // if (cur_lane_id > 0) {
  //   *res = (sm_pt.l > l_bound + threshold) ? true : false;
  // } else {
  //   *res = (sm_pt.l < -r_bound - threshold) ? true : false;
  // }

  *res = (sm_pt.l > l_bound + threshold || sm_pt.l < -r_bound - threshold)
             ? true
             : false;

  return true;
}

bool SimLateralPlanner::pruningTwoLanes(
    const double cur_l, const int center_id, const int left_id,
    const int right_id, const std::vector<DcpAction>& action_seq) {
  int current_id = 999;
  if (cur_l >= center_id * kIntervalL) {
    current_id = (cur_l >= 0.5 * (center_id + left_id) * kIntervalL)
                     ? left_id
                     : center_id;
  } else {
    current_id = (cur_l >= 0.5 * (center_id + right_id) * kIntervalL)
                     ? center_id
                     : right_id;
  }
  for (const auto& entry : action_seq) {
    if (current_id == left_id && entry.lat == DcpLatAction::kLaneChangeLeft) {
      return true;
    } else if (current_id == right_id &&
               entry.lat == DcpLatAction::kLaneChangeRight) {
      return true;
    }
  }
  return false;
}
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive

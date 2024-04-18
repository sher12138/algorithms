#include "sim_lateral_track_planner.h"
namespace neodrive {
namespace planning {
namespace sim_planner {
bool SimLateralTrackPlanner::runOnce(
    const std::vector<SimMapPoint>& goal_sl_pts,
    const std::unordered_map<std::string, int>& all_lanes_id) {
  if (!SimMap::Instance()) {
    LOG_ERROR("map interface not initialized. Exit!");
    return false;
  }
  if (goal_sl_pts.empty()) {
    LOG_ERROR("goal sl pts empty. Exit!");
    return false;
  }

  if (all_lanes_id.size() != 3) {
    LOG_WARN("All Lanes ID Possible Missing!");
  }
  for (const auto& entry : all_lanes_id) {
    std::string lane_key = entry.first;
    if (lane_key == "left") {
      potential_left_id_ = entry.second;
    } else if (lane_key == "center") {
      potential_center_id_ = entry.second;
    } else if (lane_key == "right") {
      potential_right_id_ = entry.second;
    } else {
      LOG_WARN("Unknown Lane Present!");
    }
  }

  set_goal_sl_pts(goal_sl_pts);

  int ego_lane_id_by_pos, road_id;
  if (!SimMap::Instance()->getRoadLaneId(ego_vehicle_.state().vec_position,
                                         &road_id, &ego_lane_id_by_pos)) {
    LOG_ERROR("get road & lane id failed");
    return false;
  }
  ego_lane_id_ = ego_lane_id_by_pos;

  sim_planner::SimMapPoint sl_pt_cur;
  if (!SimMap::Instance()->getNearestPoint(ego_vehicle_.state().vec_position,
                                           &sl_pt_cur)) {
    LOG_ERROR("get nearest point failed");
    return false;
  }
  std::sort(
      goal_sl_pts_.begin(), goal_sl_pts_.end(),
      [sl_pt_cur](const SimMapPoint& sm_pt_1, const SimMapPoint& sm_pt_2) {
        return abs(sm_pt_1.l - sl_pt_cur.l) < abs(sm_pt_2.l - sl_pt_cur.l);
      });
  goal_sl_pt_ = goal_sl_pts_.back();

  SimMapPoint goal_xy_pt;
  double goal_l_bound, goal_r_bound;
  if (!SimMap::Instance()->getNearestPoint(SLPoint(goal_sl_pt_.s, 0.0),
                                           &goal_xy_pt)) {
    LOG_ERROR("get nearest pt failed!");
    return false;
  }
  if (!SimMap::Instance()->searchRoadBound(Vec2d(goal_xy_pt.x, goal_xy_pt.y),
                                           &goal_l_bound, &goal_r_bound)) {
    LOG_ERROR("search road bound failed!");
    return false;
  }

  bool is_left_forbid = false, is_right_forbid = false;
  if (goal_sl_pt_.l < sl_pt_cur.l) {
    is_left_forbid = true;
    goal_sl_pt_.l =
        (goal_sl_pt_.l <=
         -goal_r_bound +
             static_cast<double>(ego_vehicle_.param().width()) / 2.0 + 0.1)
            ? (-goal_r_bound +
               static_cast<double>(ego_vehicle_.param().width()) / 2.0 + 0.1)
            : goal_sl_pt_.l;
  } else if (goal_sl_pt_.l > sl_pt_cur.l) {
    is_right_forbid = true;
    goal_sl_pt_.l =
        (goal_sl_pt_.l >=
         goal_l_bound -
             static_cast<double>(ego_vehicle_.param().width()) / 2.0 - 0.1)
            ? (goal_l_bound -
               static_cast<double>(ego_vehicle_.param().width()) / 2.0 - 0.1)
            : goal_sl_pt_.l;
  }

  pre_deleted_seq_ids_.clear();
  std::vector<std::vector<DcpAction>> action_scripts = action_script();

  int n_sequence = action_scripts.size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = action_scripts[i];
    int num_actions = action_seq.size();
    if ((is_left_forbid &&
         action_seq[0].lat == DcpLatAction::kLaneChangeLeft) ||
        (is_right_forbid &&
         action_seq[0].lat == DcpLatAction::kLaneChangeRight)) {
      pre_deleted_seq_ids_.insert(i);
    }
    for (int j = 1; j < num_actions; j++) {
      if ((is_left_forbid &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft) ||
          (is_right_forbid &&
           action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == DcpLatAction::kLaneChangeLeft &&
           action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)) {
        pre_deleted_seq_ids_.insert(i);
        break;
      }
    }
  }
  if (!runEudm(action_scripts)) {
    LOG_ERROR("run SimLateralTrackPlanner failed");
    return false;
  }

  std::ostringstream line_info;
  line_info << "[SimLateralTrackPlanner]SUCCESS id:" << winner_id_ << " [";
  for (const auto& a : action_scripts[winner_id_]) {
    line_info << DcpTree::retLonActionName(a.lon);
  }
  line_info << "|";
  for (const auto& a : action_scripts[winner_id_]) {
    line_info << DcpTree::retLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3)
            << winner_score_;
  LOG_INFO("{}", line_info.str());

  return true;
}

bool SimLateralTrackPlanner::prepareMultiThreadContainers(
    const int n_sequence) {
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

bool SimLateralTrackPlanner::runEudm(
    const std::vector<std::vector<DcpAction>>& action_seq) {
  int n_sequence = action_seq.size();
  prepareMultiThreadContainers(n_sequence);
  for (int i = 0; i < n_sequence; ++i) {
    std::cout << "seq_id:" << i << std::endl;
    simulateActionSequence(ego_vehicle_, action_seq[i], i);
  }
  LOG_INFO(
      "[SimLateralTrackPlanner][Process]loop forward simulation finished!");
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
    line_info << "[SimLateralTrackPlanner][Result]" << i << " [";
    for (const auto& a : action_seq[i]) {
      line_info << DcpTree::retLonActionName(a.lon);
    }
    line_info << "|";
    for (const auto& a : action_seq[i]) {
      line_info << DcpTree::retLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << final_cost_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    if (sim_res_[i]) {
      line_info << "[SimLateralTrackPlanner][Result][e;s;n;w:";
      for (const auto& c : progress_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << " Ego_to_desired_vel:" << c.efficiency_.ego_to_desired_vel
                  << ", "
                  << " Goal_cost:" << c.goal_cost.goal_dis_cost << ", "
                  << " Smooth_cost:" << c.smooth_cost.accumulate_k_cost << ", "
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

  if (!evaluateMultiThreadSimResults(action_seq, &winner_id_, &winner_score_)) {
    LOG_ERROR("fail to evaluate multi-thread sim results. Exit!");
    return false;
  }

  return true;
}

bool SimLateralTrackPlanner::evaluateMultiThreadSimResults(
    const std::vector<std::vector<DcpAction>>& action_seq, int* winner_id,
    double* winner_cost) {
  double min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();
  for (int i = 0; i < num_sequences; ++i) {
    if (sim_res_[i] == 0) {
      continue;
    }
    double cost = 0.0;
    SimLateralTrackPlanner::evaluateSinglePolicyTrajs(progress_cost_[i], &cost);
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

bool SimLateralTrackPlanner::evaluateSinglePolicyTrajs(
    const std::vector<CostStructureRP>& progress_cost, double* score) {
  double score_tmp = 0.0;
  for (const auto& c : progress_cost) {
    score_tmp += c.ave();
  }
  *score = score_tmp;
  return true;
}

bool SimLateralTrackPlanner::simulateActionSequence(
    const Vehicle& ego_vehicle, const std::vector<DcpAction>& action_seq,
    const int seq_id) {
  if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = std::string("(Pre-deleted)");
    return false;
  }
  // Forward simulation result container
  std::vector<int> sub_sim_res(1);
  std::vector<int> sub_risky_res(1);
  std::vector<std::string> sub_sim_info(1);
  std::vector<std::vector<CostStructureRP>> sub_progress_cost(1);
  std::vector<std::vector<Vehicle>> sub_forward_trajs(1);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(1);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(1);
  simulateScenario(ego_vehicle, action_seq, seq_id, 0, &sub_sim_res,
                   &sub_risky_res, &sub_sim_info, &sub_progress_cost,
                   &sub_forward_trajs, &sub_forward_lat_behaviors,
                   &sub_forward_lon_behaviors);

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

bool SimLateralTrackPlanner::simulateScenario(
    const Vehicle& ego_vehicle, const std::vector<DcpAction>& action_seq,
    const int seq_id, const int sub_seq_id, std::vector<int>* sub_sim_res,
    std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
    std::vector<std::vector<CostStructureRP>>* sub_progress_cost,
    std::vector<std::vector<Vehicle>>* sub_forward_trajs,
    std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
    std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors) {
  std::vector<Vehicle> ego_traj_multilayers{ego_vehicle};

  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructureRP> cost_multilayers;

  ForwardSimEgoAgent ego_fsagent_this_layer;
  ego_fsagent_this_layer.vehicle = ego_vehicle;
  updateSimSetupForScenario(action_seq, &ego_fsagent_this_layer);

  std::vector<DcpAction> action_seq_sim = action_seq;

  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    auto action_this_layer = action_seq_sim[i];
    if (!updateSimSetupForLayer(action_this_layer, &ego_fsagent_this_layer)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      LOG_ERROR("update sim setup for layer failed");
      return false;
    }
    std::vector<Vehicle> ego_traj_multisteps;
    if (!simulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                              &ego_traj_multisteps)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      LOG_ERROR("sim single action failed");
      return false;
    }

    // if (ego_traj_multisteps.empty()) {
    //   ego_traj_multisteps.emplace_back(ego_traj_multilayers.back());
    // }

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
    }
    int current_lane_id;
    if (int road_id; !SimMap::Instance()->getRoadLaneId(
            ego_fsagent_this_layer.vehicle.state().vec_position, &road_id,
            &current_lane_id)) {
      LOG_ERROR("get road lane Id failed!");
    }
    // * trace
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.emplace_back(
        ego_fsagent_this_layer.lat_behavior);
    ego_lon_behavior_multilayers.emplace_back(
        ego_fsagent_this_layer.lon_behavior);

    CostStructureRP cost;
    bool verbose = false;
    if (!costFunction(action_this_layer, ego_fsagent_this_layer, verbose,
                      ego_traj_multisteps, &cost)) {
      LOG_ERROR("cost cal failed!");
      return false;
    }
    cost.weight = cost.weight * pow(cfg_.cost_.discount_factor, i);
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

bool SimLateralTrackPlanner::checkOverStepBound(const State& state,
                                                const double threshold,
                                                bool* res) {
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

  *res =
      (sm_pt.l > l_bound -
                     static_cast<double>(ego_vehicle_.param().width()) / 2.0 +
                     threshold ||
       sm_pt.l < -r_bound +
                     static_cast<double>(ego_vehicle_.param().width()) / 2.0 -
                     threshold)
          ? true
          : false;

  return true;
}

bool SimLateralTrackPlanner::updateSimSetupForLayer(
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

  SimMapPoint sm_pt_this_layer;
  if (!SimMap::Instance()->getNearestPoint(state.vec_position,
                                           &sm_pt_this_layer)) {
    LOG_ERROR("get nearest point failed!");
    return false;
  }
  int lane_current = 1e+8;

  // if (sm_pt_this_layer.l >= potential_center_id_ * kIntervalL) {
  //   lane_current =
  //       (sm_pt_this_layer.l >=
  //        0.5 * (potential_center_id_ + potential_left_id_) * kIntervalL)
  //           ? potential_left_id_
  //           : potential_center_id_;
  // } else {
  //   lane_current =
  //       (sm_pt_this_layer.l >=
  //        0.5 * (potential_center_id_ + potential_right_id_) * kIntervalL)
  //           ? potential_center_id_
  //           : potential_right_id_;
  // }

  lane_current = sm_pt_this_layer.l / kIntervalL;

  ego_fsagent->current_lane = lane_current;

  if (ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping) {
    neodrive::planning::Boundary front_boundary, rear_boundary;
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

bool SimLateralTrackPlanner::simulateSingleAction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
    std::vector<Vehicle>* ego_traj) {
  ego_traj->clear();

  std::vector<double> dt_steps;
  getSimTimeSteps(action, &dt_steps);

  ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;

  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    double sim_time_step = dt_steps[i];
    State ego_state_cache_this_step;
    //
    State state_output;

    SimMapPoint sl_pt;
    if (!SimMap::Instance()->getNearestPoint(
            ego_fsagent_this_step.vehicle.state().vec_position, &sl_pt)) {
      LOG_ERROR("get nearest point failed");
      return false;
    }

    if (!SimLateralTrackPlanner::egoAgentForwardSim(
            ego_fsagent_this_step, sim_time_step, &state_output)) {
      LOG_ERROR("ego forward sim failed");
      return false;
    }

    Vehicle v_tmp = ego_fsagent_this_step.vehicle;

    v_tmp.setState(state_output);
    ego_traj->emplace_back(v_tmp);
    ego_state_cache_this_step = state_output;

    //
    ego_fsagent_this_step.vehicle.setState(ego_state_cache_this_step);
    // ego_fsagent_this_step.target_lane = goal_sl_pt_.l;
  }

  return true;
}

bool SimLateralTrackPlanner::egoAgentForwardSim(ForwardSimEgoAgent& ego_fsagent,
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
    LOG_ERROR("search front obs failed");
    // return false;
  }
  if (ego_fsagent.lat_behavior == LateralBehavior::kLaneKeeping) {
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
    if (!getLookAheadDistOnLC(ego_fsagent, leading_boundary, 0.5, 5.0,
                              &look_ahead_dist)) {
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
            goal_sl_pt_, ego_fsagent.vehicle, ego_fsagent.obs_boundary,
            leading_boundary, lat_track_offset, look_ahead_dist, sim_time_step,
            sim_param, &state_output)) {
      LOG_ERROR("LC failed");
      return false;
    }
  }

  *state_out = state_output;
  return true;
}

bool SimLateralTrackPlanner::costFunction(const DcpAction& action,
                                          const ForwardSimEgoAgent& ego_fsagent,
                                          const bool verbose,
                                          const std::vector<Vehicle>& traj_set,
                                          CostStructureRP* cost) {
  double duration = action.t;

  double ego_velocity = ego_fsagent.vehicle.state().velocity;
  CostStructureRP cost_tmp;
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_) {
    cost_tmp.efficiency_.ego_to_desired_vel =
        cfg_.cost_.effciency_.ego_lack_speed_to_desired_unit_cost *
        fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_);
  } else {
    if (ego_fsagent.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost_.effciency_.ego_desired_speed_tolerate_gap) {
      cost_tmp.efficiency_.ego_to_desired_vel =
          cfg_.cost_.effciency_.ego_over_speed_to_desired_unit_cost *
          fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost_.effciency_.ego_desired_speed_tolerate_gap);
    }
  }

  cost_tmp.goal_cost.goal_dis_cost = 0;
  for (const auto& goal_pt : goal_sl_pts_) {
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& entry : traj_set) {
      min_distance =
          std::min(sqrt(pow(goal_pt.x - entry.state().vec_position.x(), 2) +
                        pow(goal_pt.y - entry.state().vec_position.y(), 2)),
                   min_distance);
    }
    cost_tmp.goal_cost.goal_dis_cost += min_distance * 0.3;
  }
  cost_tmp.smooth_cost.accumulate_k_cost = 0.0;  // Corner change rate

  cost_tmp.weight = duration;
  *cost = cost_tmp;
  return true;
}

int SimLateralTrackPlanner::winner_id() const { return winner_id_; }

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
#include "motorway_speed_game_theory_decider.h"
namespace neodrive {
namespace planning {
MotorwaySpeedGameTheoryDecider::MotorwaySpeedGameTheoryDecider() {
  name_ = "MotorwaySpeedGameTheoryDecider";
}

MotorwaySpeedGameTheoryDecider::~MotorwaySpeedGameTheoryDecider() { Reset(); }

ErrorCode MotorwaySpeedGameTheoryDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (!Init(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedGameTheoryDecider::MotorwaySpeedGameTheoryDecider::Init(
    TaskInfo& task_info) {
  if (!GetSnapshot(task_info)) {
    LOG_ERROR("Get snap shot failed!");
    return false;
  }
  if (!CalcVehInfoFromPrediction(task_info)) {
    LOG_INFO("Calc vehicle info from prediction failed!");
    return false;
  }
  State ego_state(data_center_->vehicle_state_proxy().X(),
                  data_center_->vehicle_state_proxy().Y(),
                  task_info.current_frame()->inside_planner_data().vel_v,
                  task_info.current_frame()->inside_planner_data().vel_heading,
                  task_info.current_frame()->inside_planner_data().vel_a, 0.0);
  Param ego_param(VehicleParam::Instance()->width(),
                  VehicleParam::Instance()->length(),
                  VehicleParam::Instance()->length() -
                      VehicleParam::Instance()->back_edge_to_center(),
                  VehicleParam::Instance()->back_edge_to_center(), 1.5, 5.0);
  Believes ego_believes;
  // todo (lvyang): base prediction
  Vec2d goal_pt;
  double goal_s;
  std::vector<Vec2d> ref_pts;
  ego_.SetParam(0, ego_state, ego_param, ego_believes, goal_pt, goal_s,
                ref_pts);
  // node_.state.set_current_ego_state(ego_);
  return true;
}

bool MotorwaySpeedGameTheoryDecider::MotorwaySpeedGameTheoryDecider::Process(
    TaskInfo& task_info) {
  // todo (lyu yang): need pull in single mcts solve
  if (!CalcLevelK(task_info)) {
    LOG_WARN("Solve level k failed!");
  }
  return true;
}

bool MotorwaySpeedGameTheoryDecider::FilterAgents(
    TaskInfo& task_info, std::vector<Obstacle*>& filter_agents) {
  return true;
}

bool MotorwaySpeedGameTheoryDecider::CalcVehInfoFromPrediction(
    TaskInfo& task_info) {
  std::vector<Obstacle*> filter_agents{};
  if (!FilterAgents(task_info, filter_agents)) {
    LOG_ERROR("Filter agents failed!");
    return false;
  }
  for (const auto& iter : filter_agents) {
    Vehicle agent;
    State agent_state(iter->center().x(), iter->center().y(), iter->speed(),
                      iter->velocity_heading(), 0.0, 0.0);
    Param agent_param(iter->width(), iter->length(), iter->length() / 2.0,
                      iter->length() / 2.0, 1.5, 5.0);
    Believes agent_believes;
    Vec2d agent_goal_pt;
    double agent_goal_s;
    std::vector<Vec2d> ref_pts{};  // todo (lvyang)
    agent.SetParam(iter->id(), agent_state, agent_param, agent_believes,
                   agent_goal_pt, agent_goal_s, ref_pts);
    if (!CalOrinTrajectory(*iter, agent)) {
      return false;
    }
    CalcAgentGoal(agent);
    agents_.emplace_back(agent);
  }
  // node_.state.set_current_agents_state(agents_);
  return true;
}

bool MotorwaySpeedGameTheoryDecider::CalOrinTrajectory(const Obstacle& obs,
                                                       Vehicle& agent) {
  auto pred_traj = obs.prediction_trajectories();
  if (pred_traj.empty()) {
    LOG_INFO("Obs {} 's predict trajs is empty!"), obs.id();
    return false;
  }
  for (auto& traj : pred_traj) {
    OrinTrajectory otj;
    otj.probability = traj.probability();
    if (traj.trajectory_point_ptr(0) == nullptr) {
      continue;
    }
    TrajectoryPoint* last_pt = traj.trajectory_point_ptr(0);
    for (size_t i = 0; i < 50; ++i) {
      auto pt = traj.trajectory_point_ptr(i);
      if (pt == nullptr) {
        break;
      }
      TrajPoint fix_pt;
      fix_pt.t = i * time_step_;
      fix_pt.x = pt->path_point().coordinate().x();
      fix_pt.y = pt->path_point().coordinate().y();
      fix_pt.v = pt->velocity();
      fix_pt.a = pt->acceleration();
      fix_pt.theta = pt->steer();
      fix_pt.s = std::sqrt(std::pow(pt->path_point().coordinate().x() -
                                        last_pt->path_point().coordinate().x(),
                                    2) +
                           std::pow(pt->path_point().coordinate().y() -
                                        last_pt->path_point().coordinate().y(),
                                    2));
      otj.orin_traj.pts.emplace_back(fix_pt);
      last_pt = pt;
    }
    agent.orin_trajs.emplace_back(otj);
  }
  return true;
}

void MotorwaySpeedGameTheoryDecider::CalcAgentGoal(Vehicle& agent) {
  // todo (lvyang): temp
  double aim_distance = agent.state.v * 5.0;
  agent.goal_s = aim_distance;
  agent.goal_pt =
      Vec2d(agent.state.x + aim_distance * std::cos(agent.state.theta),
            agent.state.y + aim_distance * std::sin(agent.state.theta));
}

bool MotorwaySpeedGameTheoryDecider::CalcLevelK(TaskInfo& task_info) {
  LevelK level_k_object(ego_, agents_, last_agents_);
  if (!level_k_object.CalcGlobalOptimalTrajectory()) {
    LOG_ERROR("Calc global optimal trajectory failed!");
    return false;
  }
  return true;
}

bool MotorwaySpeedGameTheoryDecider::SaveSnapshot(TaskInfo& task_info) {
  auto frame_records = task_info.current_frame()
                           ->outside_planner_data()
                           .motorway_game_theory_decision;
  frame_records.Reset();
  frame_records.agents_record = agents_;
  return true;
}

bool MotorwaySpeedGameTheoryDecider::GetSnapshot(TaskInfo& task_info) {
  last_agents_ = task_info.current_frame()
                     ->outside_planner_data()
                     .motorway_game_theory_decision.agents_record;
  return true;
}

}  // namespace planning
}  // namespace neodrive
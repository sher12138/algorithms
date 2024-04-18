#include "speed_constrained_iter_lqr_decider.h"

#include <unordered_map>

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {
namespace {
void VisStDecisonpara(const std::vector<std::shared_ptr<SpeedCilqrObsProcess>>&
                          completion_obs_list) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (const auto& obs_decision : completion_obs_list) {
    auto e_iter_decisoin_st =
        vis::EventSender::Instance()->GetEvent("cilqr_decision");
    e_iter_decisoin_st->set_type(visualizer::Event::k2D);
    e_iter_decisoin_st->mutable_color()->set_r(0.8);
    e_iter_decisoin_st->mutable_color()->set_g(0.0);
    e_iter_decisoin_st->mutable_color()->set_b(0.0);
    e_iter_decisoin_st->mutable_color()->set_a(0.6);
    std::vector<ObsDecisionBound> low_point =
        obs_decision->InterpolatePointsForLow();
    auto iter_decision_st = e_iter_decisoin_st->add_polyline();
    for (int i = 0; i < low_point.size(); i++) {
      set_pt(iter_decision_st->add_point(), low_point[i].time,
             low_point[i].obs_s);
    }
  }

  for (const auto& obs_decision : completion_obs_list) {
    auto e_iter_decisoin_st_up =
        vis::EventSender::Instance()->GetEvent("cilqr_up_decision");
    e_iter_decisoin_st_up->set_type(visualizer::Event::k2D);
    e_iter_decisoin_st_up->mutable_color()->set_r(0.8);
    e_iter_decisoin_st_up->mutable_color()->set_g(0.0);
    e_iter_decisoin_st_up->mutable_color()->set_b(0.0);
    e_iter_decisoin_st_up->mutable_color()->set_a(0.6);
    std::vector<ObsDecisionBound> up_point =
        obs_decision->InterpolatePointsForUpper();
    auto iter_decisoin_st_up = e_iter_decisoin_st_up->add_polyline();
    for (int i = 0; i < up_point.size(); i++) {
      set_pt(iter_decisoin_st_up->add_point(), up_point[i].time,
             up_point[i].obs_s);
    }
  }
}

void VisVgoal(std::vector<double> v) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_iter_result_st = vis::EventSender::Instance()->GetEvent("ilqr_v_goal");
  e_iter_result_st->set_type(visualizer::Event::k2D);
  e_iter_result_st->mutable_color()->set_r(0.8);
  e_iter_result_st->mutable_color()->set_g(0.0);
  e_iter_result_st->mutable_color()->set_b(0.0);
  e_iter_result_st->mutable_color()->set_a(0.6);
  auto iter_result_st = e_iter_result_st->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (int i = 0; i < 50; i++) {
    set_pt(iter_result_st->add_point(), i * 0.1, v[i]);
  }
}

void VisSgoal(std::vector<double> v) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_iter_result_st = vis::EventSender::Instance()->GetEvent("ilqr_s_goal");
  e_iter_result_st->set_type(visualizer::Event::k2D);
  e_iter_result_st->mutable_color()->set_r(0.8);
  e_iter_result_st->mutable_color()->set_g(0.0);
  e_iter_result_st->mutable_color()->set_b(0.0);
  e_iter_result_st->mutable_color()->set_a(0.6);
  auto iter_result_st = e_iter_result_st->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (int i = 0; i < 50; i++) {
    set_pt(iter_result_st->add_point(), i * 0.1, v[i]);
  }
}

}  // namespace

SpeedConstrainedIterLqrDecider::SpeedConstrainedIterLqrDecider() {
  name_ = "SpeedConstrainedIterLqrDecider";
}

SpeedConstrainedIterLqrDecider::~SpeedConstrainedIterLqrDecider() {}

void SpeedConstrainedIterLqrDecider::SaveLOGResults() {}
void SpeedConstrainedIterLqrDecider::SaveTaskResults(TaskInfo& task_info){};
void SpeedConstrainedIterLqrDecider::Reset(){};

ErrorCode SpeedConstrainedIterLqrDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    LOG_INFO("path successed tasks is 0, skip rest tasks.");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  LOG_INFO(">>>> Speed Constrained Iter Lqr Decider work normal");
  if (!Init(task_info)) {
    LOG_ERROR("Speed Constrained Iter Lqr Decider Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Speed Iter Deduction Decider Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("Process FInished.");
  SaveLOGResults();

  return ErrorCode::PLANNING_OK;
}
bool SpeedConstrainedIterLqrDecider::Init(TaskInfo& task_info) {
  reset();
  delta_t_ = 0.1;
  n_ = static_cast<std::size_t>(5.0 / delta_t_);

  auto& goal_decision_data = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->speed_obstacle_context.iter_deduction;
  std::vector<double> goal_s_vec{}, goal_v_vec{}, goal_a_vec{}, goal_t_vec{};
  tk::spline goal_s_spline, goal_v_spline, goal_a_spline;

  for (size_t i = 0; i < goal_decision_data.deduction_ego_t_sequence.size();
       i++) {
    goal_s_vec.emplace_back(goal_decision_data.deduction_ego_p_sequence[i]);
    goal_v_vec.emplace_back(goal_decision_data.deduction_ego_v_sequence[i]);
    goal_a_vec.emplace_back(goal_decision_data.deduction_ego_a_sequence[i]);
    goal_t_vec.emplace_back(goal_decision_data.deduction_ego_t_sequence[i]);
  }

  VisVgoal(goal_v_vec);
  VisSgoal(goal_s_vec);

  goal_s_spline.set_points(goal_t_vec, goal_s_vec,
                           tk::spline::spline_type::linear);
  goal_v_spline.set_points(goal_t_vec, goal_v_vec,
                           tk::spline::spline_type::linear);
  goal_a_spline.set_points(goal_t_vec, goal_a_vec,
                           tk::spline::spline_type::linear);

  ref_speed_plan_.goal_s_.resize(n_ + 4);
  ref_speed_plan_.goal_v_.resize(n_ + 4);
  ref_speed_plan_.goal_a_.resize(n_ + 4);
  for (std::size_t i = 0; i <= n_ + 1; ++i) {
    double curr_t = i * delta_t_;
    ref_speed_plan_.goal_s_[i].goal_t_s =
        STPoint(goal_s_spline(curr_t), curr_t);
    ref_speed_plan_.goal_v_[i].goal_t_v =
        STPoint(goal_v_spline(curr_t), curr_t);
    ref_speed_plan_.goal_a_[i].goal_t_a =
        STPoint(goal_a_spline(curr_t), curr_t);
  }
  if (ProcessObs(task_info)) return true;
}

bool SpeedConstrainedIterLqrDecider::ProcessObs(TaskInfo& task_info) {
  using ObsType = Obstacle::ObstacleType;
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dynamic_obstacles_decision;
  const auto& static_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.static_obstacles_decision;
  const auto& virtual_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.virtual_obstacle_decision;
  std::vector<SpeedObstacleDecision> obs_decision_info{};

  obs_decision_info.insert(obs_decision_info.end(),
                           dynamic_obstacles_decision.begin(),
                           dynamic_obstacles_decision.end());
  obs_decision_info.insert(obs_decision_info.end(),
                           static_obstacles_decision.begin(),
                           static_obstacles_decision.end());
  obs_decision_info.insert(obs_decision_info.end(),
                           virtual_obstacles_decision.begin(),
                           virtual_obstacles_decision.end());

  const auto& dp_st_ignore_static_obs =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dp_st_map_ignore_static_obs_id;

  const auto& dp_st_ignore_dynamic_obs =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;

  for (auto& obs_decision : obs_decision_info) {
    if (dp_st_ignore_static_obs.find(obs_decision.obstacle.id()) !=
        dp_st_ignore_static_obs.end()) {
      LOG_INFO("Because static decider , ignore static id :[{}] ",
               obs_decision.obstacle.id());
      continue;
    }
    if (dp_st_ignore_dynamic_obs.find(obs_decision.obstacle.id()) !=
        dp_st_ignore_dynamic_obs.end()) {
      LOG_INFO("Because dynamic decider , ignore static id :[{}] ",
               obs_decision.obstacle.id());
      continue;
    }
    PathPoint closest_pt{};
    auto& obstacle = obs_decision.obstacle;
    double path_heading_near_obs =
        path.query_closest_point(obstacle.center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff =
        normalize_angle(obstacle.velocity_heading() - path_heading_near_obs);
    bool if_reverse{false};
    if (std::abs(heading_diff) > 1.57) {
      if_reverse = true;
    }
    double end_t = (obs_decision.lower_points.back().first.t() > 8)
                       ? 8
                       : obs_decision.lower_points.back().first.t();
    SpeedCilqrObsProcess obs_extend_decison{
        obstacle.id(),
        obs_decision.lower_points[0].first.t(),
        end_t,
        obs_decision.lower_points[0].first.s(),
        obs_decision.upper_points[0].first.s(),
        obs_decision.lower_points.back().first.s(),
        obs_decision.upper_points.back().first.s(),
        if_reverse,
        obs_decision.obstacle.is_virtual(),
        obs_decision.obstacle.type() == ObsType::VEHICLE};
    sorted_obs_list_.emplace_back(
        std::make_shared<SpeedCilqrObsProcess>(obs_extend_decison));
  };

  std::sort(sorted_obs_list_.begin(), sorted_obs_list_.end(),
            [](const std::shared_ptr<SpeedCilqrObsProcess>& obs1,
               const std::shared_ptr<SpeedCilqrObsProcess>& obs2) {
              if (obs1->start_t() < obs2->start_t()) {
                return true;
              } else if (obs1->start_t() > obs2->start_t()) {
                return false;
              }
              return obs1->start_s_l() < obs2->start_s_l();
            });
  VisStDecisonpara(sorted_obs_list_);
  return true;
}

void SpeedConstrainedIterLqrDecider::reset() {
  ref_speed_plan_.Reset();
  sorted_obs_list_.clear();
}
bool SpeedConstrainedIterLqrDecider::Process(TaskInfo& task_info) {
  CalInitStateAndControl(task_info.current_frame()->inside_planner_data());

  std::shared_ptr<RunStepCilqr> cilqr_problem{nullptr};

  cilqr_problem = std::make_shared<RunStepCilqr>(ego_state_, sorted_obs_list_,
                                                 ref_speed_plan_, n_);
  auto result = cilqr_problem->Optimize();
  auto if_illness = cilqr_problem->is_illness();
  auto& goal_decision_data = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->speed_obstacle_context.iter_deduction;
  if (!if_illness) {
    goal_decision_data.Reset();
    DeductionEgoStateSequence ego_deduction_;
    ego_deduction_.Reset();

    for (int i = 0; i < result.size(); i++) {
      ego_deduction_.deduction_ego_v_sequence.push_back(result[i].state_v);
      ego_deduction_.deduction_ego_a_sequence.push_back(result[i].state_a);
      ego_deduction_.deduction_ego_p_sequence.push_back(result[i].state_s);
      ego_deduction_.deduction_ego_t_sequence.push_back(0.1 * i);
    }

    goal_decision_data.deduction_ego_a_sequence =
        ego_deduction_.deduction_ego_a_sequence;
    goal_decision_data.deduction_ego_v_sequence =
        ego_deduction_.deduction_ego_v_sequence;
    goal_decision_data.deduction_ego_p_sequence =
        ego_deduction_.deduction_ego_p_sequence;
    goal_decision_data.deduction_ego_t_sequence =
        ego_deduction_.deduction_ego_t_sequence;
  }

  return true;
};

void SpeedConstrainedIterLqrDecider::CalInitStateAndControl(
    const InsidePlannerData& inside_data) {
  ego_state_.state_v = inside_data.init_point.velocity() < 0.0
                           ? 0.0
                           : inside_data.init_point.velocity();
  ego_state_.state_a = inside_data.init_point.acceleration();

  LOG_INFO("init_state: {:.3}, {:.3}, {:.3}", ego_state_.state_s,
           ego_state_.state_v, ego_state_.state_a);
}

}  // namespace planning
}  // namespace neodrive

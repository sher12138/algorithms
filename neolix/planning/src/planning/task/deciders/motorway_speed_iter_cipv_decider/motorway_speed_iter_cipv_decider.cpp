#include "motorway_speed_iter_cipv_decider.h"

#include <unordered_map>

#include "common/visualizer_event/visualizer_event.h"
#include "motorway_speed_iter_obs_decision_extend.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/util/speed_planner_common.h"
using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {
namespace {
constexpr double kReverseHeading = 2.355;
}

namespace {

void VisStDecisonpara(
    const std::vector<MotorwaySpeedObsExtend>& completion_obs_list) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (auto obs_decision : completion_obs_list) {
    auto e_iter_decisoin_st =
        vis::EventSender::Instance()->GetEvent("Iter_decision");
    e_iter_decisoin_st->set_type(visualizer::Event::k2D);
    e_iter_decisoin_st->mutable_color()->set_r(0.8);
    e_iter_decisoin_st->mutable_color()->set_g(0.0);
    e_iter_decisoin_st->mutable_color()->set_b(0.0);
    e_iter_decisoin_st->mutable_color()->set_a(0.6);
    auto iter_decisoin_st = e_iter_decisoin_st->add_polyline();
    std::vector<ObsDecisionBound> low_point =
        obs_decision.InterpolatePointsForLow();
    for (std::size_t i = 0; i < low_point.size(); ++i) {
      set_pt(iter_decisoin_st->add_point(), low_point[i].time,
             low_point[i].obs_s);
    }
  }

  for (auto obs_decision : completion_obs_list) {
    auto e_iter_decisoin_st_up =
        vis::EventSender::Instance()->GetEvent("Iter_up_decision");
    e_iter_decisoin_st_up->set_type(visualizer::Event::k2D);
    e_iter_decisoin_st_up->mutable_color()->set_r(0.8);
    e_iter_decisoin_st_up->mutable_color()->set_g(0.0);
    e_iter_decisoin_st_up->mutable_color()->set_b(0.0);
    e_iter_decisoin_st_up->mutable_color()->set_a(0.6);
    auto iter_decisoin_st_up = e_iter_decisoin_st_up->add_polyline();
    std::vector<ObsDecisionBound> up_point =
        obs_decision.InterpolatePointsForUpper();
    for (std::size_t i = 0; i < up_point.size(); ++i) {
      set_pt(iter_decisoin_st_up->add_point(), up_point[i].time,
             up_point[i].obs_s);
    }
  }
}

void VisOriginST(
    const std::vector<MotorwayMultiCipvSpeedObstacleDecision>& obs_list_,
    TaskInfo& task_info) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (auto obs_id : obs_list_) {
    auto e_iter_decisoin_st =
        vis::EventSender::Instance()->GetEvent("origin_Iter_decision_low");
    e_iter_decisoin_st->set_type(visualizer::Event::k2D);
    e_iter_decisoin_st->mutable_color()->set_r(0.8);
    e_iter_decisoin_st->mutable_color()->set_g(0.0);
    e_iter_decisoin_st->mutable_color()->set_b(0.0);
    e_iter_decisoin_st->mutable_color()->set_a(0.6);
    // std::vector<ObsDecisionBound> low_point =
    //     obs_decision.InterpolatePointsForLow();
    auto iter_decision_st = e_iter_decisoin_st->add_polyline();
    for (int i = 0; i < obs_id.lower_points.size(); i++) {
      set_pt(iter_decision_st->add_point(), obs_id.lower_points[i].first.t(),
             obs_id.lower_points[i].first.s());
    }
  }

  for (auto obs_id : obs_list_) {
    auto e_iter_decisoin_st_up =
        vis::EventSender::Instance()->GetEvent("origin_Iter_up_decision");
    e_iter_decisoin_st_up->set_type(visualizer::Event::k2D);
    e_iter_decisoin_st_up->mutable_color()->set_r(0.8);
    e_iter_decisoin_st_up->mutable_color()->set_g(0.0);
    e_iter_decisoin_st_up->mutable_color()->set_b(0.0);
    e_iter_decisoin_st_up->mutable_color()->set_a(0.6);

    auto iter_decisoin_st_up = e_iter_decisoin_st_up->add_polyline();
    for (int i = 0; i < obs_id.upper_points.size(); i++) {
      set_pt(iter_decisoin_st_up->add_point(), obs_id.upper_points[i].first.t(),
             obs_id.upper_points[i].first.s());
    }
  }
}

void VisDeductionResult(const DeductionEgoStateSequence& ego_deduction) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_iter_result_st =
      vis::EventSender::Instance()->GetEvent("Iter_result_st");
  e_iter_result_st->set_type(visualizer::Event::k2D);
  e_iter_result_st->mutable_color()->set_r(0.8);
  e_iter_result_st->mutable_color()->set_g(0.0);
  e_iter_result_st->mutable_color()->set_b(0.0);
  e_iter_result_st->mutable_color()->set_a(0.6);
  auto iter_result_st = e_iter_result_st->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (int i; i < ego_deduction.deduction_ego_t_sequence.size(); i++) {
    set_pt(iter_result_st->add_point(),
           ego_deduction.deduction_ego_t_sequence[i],
           ego_deduction.deduction_ego_p_sequence[i]);
  }
  auto e_iter_result_vt =
      vis::EventSender::Instance()->GetEvent("Iter_result_vt");
  e_iter_result_vt->set_type(visualizer::Event::k2D);
  e_iter_result_vt->mutable_color()->set_r(0.8);
  e_iter_result_vt->mutable_color()->set_g(0.0);
  e_iter_result_vt->mutable_color()->set_b(0.0);
  e_iter_result_vt->mutable_color()->set_a(0.6);
  auto iter_result_vt = e_iter_result_vt->add_polyline();

  for (int i; i < ego_deduction.deduction_ego_t_sequence.size(); i++) {
    set_pt(iter_result_vt->add_point(),
           ego_deduction.deduction_ego_t_sequence[i],
           ego_deduction.deduction_ego_v_sequence[i]);
  }
}
}  // namespace

MotorwaySpeedIterCipvDecider::MotorwaySpeedIterCipvDecider() {
  name_ = "MotorwaySpeedIterCipvDecider";
}

MotorwaySpeedIterCipvDecider::~MotorwaySpeedIterCipvDecider() {}

ErrorCode MotorwaySpeedIterCipvDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    LOG_INFO("path successed tasks is 0, skip rest tasks.");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  LOG_INFO(">>>>Motorway Speed Iter Deduction Decider work normal");
  if (!Init(task_info)) {
    LOG_ERROR("MOtorway Speed Iter Deduction Decider Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Speed Iter Deduction Decider Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("Process FInished.");
  SaveLOGResults();
  SetupIntervalSpeedLimit(task_info);
  VisDeductionResult(ego_deduction_);
  GenerateSBoundForObs(task_info);
  GenerateVBoundForEgo(task_info);
  GenerateFollowTakeDecision(task_info);
  auto& backup_cipv = task_info.current_frame()
                          ->mutable_outside_planner_data()
                          ->motorway_speed_obstacle_context.backup_cipv;
  LOG_INFO("send backup id : {} , has_cipv : {}", backup_cipv.cipv_id,
           backup_cipv.has_cipv);
  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedIterCipvDecider::GenerateFollowTakeDecision(
    TaskInfo& task_info) {
  auto& take_follow_decision_map =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context
          .motorway_iter_deduction_take_follow_decision_map;
  take_follow_decision_map.clear();
  std::sort(original_obs_list_.begin(), original_obs_list_.end(),
            [](const MotorwaySpeedObsExtend& obs1,
               const MotorwaySpeedObsExtend& obs2) {
              if (obs1.StartTime() < obs2.StartTime()) {
                return true;
              } else if (obs1.StartTime() > obs2.StartTime()) {
                return false;
              }
              return obs1.StartLowS() < obs2.StartLowS();
            });
  std::vector<std::pair<int, bool>> backup_map{};
  for (const auto iter : original_obs_list_) {
    int ego_index = int(iter.StartTime() * 10);
    auto ego_p = ego_deduction_.deduction_ego_p_sequence[ego_index];
    if (ego_p > iter.StartUpS()) {
      LOG_INFO("take this obs  ,id is [{}].", iter.Id());
      take_follow_decision_map.emplace(iter.Id(), true);
      backup_map.emplace_back(std::make_pair(iter.Id(), true));
    } else {
      LOG_INFO("follow this obs ,id is {}.", iter.Id());
      take_follow_decision_map.emplace(iter.Id(), false);
      backup_map.emplace_back(std::make_pair(iter.Id(), false));
    }
  }
  LOG_INFO("follow take map size is [{}].", take_follow_decision_map.size());
  if (backup_map.empty()) {
    SetUpBackupPara(task_info, 0, false);
  } else {
    for (auto& obs_info : backup_map) {
      if (obs_info.second == false) {
        SetUpBackupPara(task_info, obs_info.first, true);
        break;
      }
      SetUpBackupPara(task_info, 0, false);
    }
  }
}

void MotorwaySpeedIterCipvDecider::SaveTaskResults(TaskInfo& task_info) {
  // ignore
}

void MotorwaySpeedIterCipvDecider::Reset() {
  ego_current_state_.Reset();
  ego_state_sequence_.Reset();
  completion_obs_list_.clear();
  ego_deduction_.Reset();
  min_v_bound_ = data_center_->drive_strategy_max_speed();
  v_target_ = data_center_->drive_strategy_max_speed();
  obs_list_.clear();
}

bool MotorwaySpeedIterCipvDecider::Init(TaskInfo& task_info) {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  auto& if_change_obs_stop_dis =
      task_info.current_frame()->mutable_outside_planner_data()->if_has_bump;
  if_change_obs_stop_dis = false;
  Reset();

  ego_current_state_.ego_a = task_info.current_frame()
                                 ->inside_planner_data()
                                 .init_point.acceleration();
  ego_current_state_.ego_v =
      task_info.current_frame()->inside_planner_data().init_point.velocity() <
              0.0
          ? 0.0
          : task_info.current_frame()
                ->inside_planner_data()
                .init_point.velocity();

  ego_current_state_.ego_p = 0.;
  ego_current_state_.ego_t = 0.;

  virtual_obs_dis_ = speed_iter_deduction_config.virtual_bound_t * v_target_;
  LOG_INFO(
      ">>>> speed iter deduction decider init para -- ego_v:{:.3f}, "
      "ego_a:{:.3f},  ",
      ego_current_state_.ego_v, ego_current_state_.ego_a);

  CheckConcernedObsDecision(task_info);
  return true;
}

void MotorwaySpeedIterCipvDecider::CheckConcernedObsDecision(
    TaskInfo& task_info) {
  auto TransData = [](std::vector<std::pair<STPoint, double>> st_point_data) {
    std::vector<STPoint> st_point{};
    for (const auto& st_point_pair : st_point_data) {
      st_point.emplace_back(st_point_pair.first);
    }
    return st_point;
  };

  using ObsType = Obstacle::ObstacleType;
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  const auto& outside_data = task_info.current_frame()->outside_planner_data();
  const auto& path = outside_data.path_data->path();
  const auto& obs_context = outside_data.motorway_speed_obstacle_context;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& speed_bump_overlaps =
      task_info.reference_line()->speed_bump_overlaps();
  const auto& ego_base_f = task_info.adc_boundary().end_s();
  const auto& dynamic_obstacles_decision =
      obs_context.multi_cipv_dynamic_obstacles_decision;
  const auto& static_obstacles_decision =
      obs_context.multi_cipv_static_obstacles_decision;
  const auto& virtual_obstacles_decision =
      obs_context.multi_cipv_virtual_obstacle_decision;

  LOG_INFO("all_decison obs vector size is ",
           dynamic_obstacles_decision.size() +
               static_obstacles_decision.size() +
               virtual_obstacles_decision.size());
  std::vector<MotorwaySpeedObsExtend> filtered_decison{};

  auto GenerateObsExtend =
      [&ego_base_f, &task_info, &speed_bump_overlaps, &inside_data,
       &outside_data, &TransData](
          const std::vector<MotorwayMultiCipvSpeedObstacleDecision>&
              obs_decisions,
          std::vector<MotorwaySpeedObsExtend>& obs_extends,
          const auto& speed_iter_deduction_config,
          std::vector<MotorwayMultiCipvSpeedObstacleDecision>& obs_list_)
      -> void {
    const auto& ignore_static_obs =
        outside_data.motorway_speed_obstacle_context.ignore_static_obs_id;
    const auto& ignore_dynamic_obs =
        outside_data.motorway_speed_obstacle_context.ignore_dynamic_obs_id;
    const auto& path = outside_data.path_data->path();
    for (const auto& obs_decision : obs_decisions) {
      if (ignore_static_obs.find(obs_decision.obstacle.id()) !=
          ignore_static_obs.end()) {
        LOG_INFO("Ignore static id :[{}] ", obs_decision.obstacle.id());
        continue;
      }
      if (ignore_dynamic_obs.find(obs_decision.obstacle.id()) !=
          ignore_dynamic_obs.end()) {
        LOG_INFO("Ignore dynamic id :[{}]", obs_decision.obstacle.id());
        continue;
      }
      if (obs_decision.lower_points.empty() ||
          obs_decision.upper_points.empty()) {
        LOG_INFO("Ignore obs [{}] because of lacking data.",
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
      bool if_reverse = std::abs(heading_diff) > kReverseHeading ? true : false;
      double end_t = (obs_decision.lower_points.back().first.t() > 8.)
                         ? 8.
                         : obs_decision.lower_points.back().first.t();
      double pedestrian_prodect{0.0};
      if (obstacle.type() == Obstacle::ObstacleType::PEDESTRIAN) {
        pedestrian_prodect =
            speed_iter_deduction_config.predestrian_protection_buffer;
      }

      bool speed_bump_found = false;
      if (obstacle.speed() > 0.1) speed_bump_found = false;
      for (const auto& speed_bump_overlaps : speed_bump_overlaps) {
        if (speed_bump_overlaps.object_id != 0 &&
            (ego_base_f + obs_decision.lower_points[0].first.s() >
                 speed_bump_overlaps.start_s - 5.0 &&
             ego_base_f + obs_decision.lower_points[0].first.s() <
                 speed_bump_overlaps.end_s + 5)) {
          speed_bump_found = true;
          auto& if_change_obs_stop_dis = task_info.current_frame()
                                             ->mutable_outside_planner_data()
                                             ->if_has_bump;
          if_change_obs_stop_dis = speed_bump_found;
          LOG_INFO(
              "adc curr s: {:.4f}, speed_bump[{}] start s: {:.4f},  end s: "
              "{:.4f}, "
              "obs  s: {:.4f}",
              ego_base_f, speed_bump_overlaps.object_id,
              speed_bump_overlaps.start_s, speed_bump_overlaps.end_s,
              ego_base_f + obs_decision.lower_points[0].first.s());
          LOG_INFO("speed_bump_found , obs id : [{}]", obstacle.id());
          break;
        }
      }
      bool if_dynamic = !(obstacle.is_static() || obstacle.is_virtual());
      MotorwaySpeedObsExtend obs_extend_decison{
          obstacle.id(),
          obs_decision.lower_points[0].first.t(),
          end_t,
          obs_decision.lower_points[0].first.s(),
          obs_decision.upper_points[0].first.s() + pedestrian_prodect,
          obs_decision.lower_points.back().first.s(),
          obs_decision.upper_points.back().first.s() + pedestrian_prodect,
          if_reverse,
          obs_decision.obstacle.is_virtual(),
          obs_decision.obstacle.type() == ObsType::VEHICLE,
          speed_bump_found,
          TransData(obs_decision.lower_points),
          TransData(obs_decision.upper_points),
          if_dynamic};
      obs_extends.emplace_back(obs_extend_decison);
      obs_list_.push_back(obs_decision);
    }
  };

  GenerateObsExtend(dynamic_obstacles_decision, filtered_decison,
                    speed_iter_deduction_config, obs_list_);
  GenerateObsExtend(static_obstacles_decision, filtered_decison,
                    speed_iter_deduction_config, obs_list_);
  GenerateObsExtend(virtual_obstacles_decision, filtered_decison,
                    speed_iter_deduction_config, obs_list_);

  original_obs_list_.clear();
  original_obs_list_ = filtered_decison;
  for (int i = 0; i < original_obs_list_.size(); i++) {
    LOG_INFO(
        "After filter ,obs  id : {},start_t : {:.3f},end_t {:.3f}, start _l_s "
        ": "
        "{:.3f} "
        ",end_l_s : {:.3f} ,start_up_s : {:.3f},end_u_s : {:.3f}",
        original_obs_list_[i].Id(), original_obs_list_[i].StartTime(),
        original_obs_list_[i].EndTime(), original_obs_list_[i].StartLowS(),
        original_obs_list_[i].EndLowS(), original_obs_list_[i].StartUpS(),
        original_obs_list_[i].EndUpS());
  }
  LOG_INFO("After filter reverse, obs vector size is {}",
           filtered_decison.size());
  // VisStDecisonpara(filtered_decison);
  SupplementCuttedObs(filtered_decison);
  IntegrateObsDecision(filtered_decison, completion_obs_list_);
  VisStDecisonpara(completion_obs_list_);
  VisOriginST(obs_list_, task_info);
}

void MotorwaySpeedIterCipvDecider::SupplementCuttedObs(
    std::vector<MotorwaySpeedObsExtend>& filtered_decison) {
  std::vector<MotorwaySpeedObsExtend> filtered_decison_new{};
  filtered_decison_new = filtered_decison;

  for (int i = 0; i < filtered_decison.size(); i++) {
    MotorwaySpeedObsExtend tt = filtered_decison[i];
    for (int j = 0; j < filtered_decison.size(); j++) {
      if (j == i) {
        continue;
      }
      if (tt.StartTime() > filtered_decison[j].EndTime()) {
        continue;
      }
      if (tt.EndTime() < filtered_decison[j].StartTime()) {
        continue;
      }
      if (tt.StartTime() > filtered_decison[j].StartTime()) {
        double s_low_mid = 0;
        double s_up_mid = 0;
        s_low_mid =
            (filtered_decison[j].EndLowS() - filtered_decison[j].StartLowS()) *
                (tt.StartTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartLowS();
        s_up_mid =
            (filtered_decison[j].EndUpS() - filtered_decison[j].StartUpS()) *
                (tt.StartTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartUpS();
        MotorwaySpeedObsExtend c{filtered_decison[j].Id(),
                                 filtered_decison[j].StartTime(),
                                 tt.StartTime(),
                                 filtered_decison[j].StartLowS(),
                                 filtered_decison[j].StartUpS(),
                                 s_low_mid,
                                 s_up_mid,
                                 filtered_decison[j].IfReverse(),
                                 filtered_decison[j].IfVirtual(),
                                 filtered_decison[j].IfVehicle(),
                                 filtered_decison[j].IfBump(),
                                 filtered_decison[j].lower_points(),
                                 filtered_decison[j].upper_points(),
                                 filtered_decison[j].if_dynamic()};
        filtered_decison_new.push_back(c);
      }
      if (tt.EndTime() < filtered_decison[j].EndTime()) {
        double s_low_mid = 0;
        double s_up_mid = 0;
        s_low_mid =
            (filtered_decison[j].EndLowS() - filtered_decison[j].StartLowS()) *
                (tt.EndTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartLowS();
        s_up_mid =
            (filtered_decison[j].EndUpS() - filtered_decison[j].StartUpS()) *
                (tt.EndTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartUpS();
        MotorwaySpeedObsExtend c{filtered_decison[j].Id(),
                                 tt.EndTime(),
                                 filtered_decison[j].EndTime(),
                                 s_low_mid,
                                 s_up_mid,
                                 filtered_decison[j].EndLowS(),
                                 filtered_decison[j].EndUpS(),
                                 filtered_decison[j].IfReverse(),
                                 filtered_decison[j].IfVirtual(),
                                 filtered_decison[j].IfVehicle(),
                                 filtered_decison[j].IfBump(),
                                 filtered_decison[j].lower_points(),
                                 filtered_decison[j].upper_points(),
                                 filtered_decison[j].if_dynamic()};
        filtered_decison_new.push_back(c);
      }
      if (tt.StartTime() > filtered_decison[j].StartTime() &&
          tt.EndTime() < filtered_decison[j].EndTime()) {
        double s_low_mid = 0;
        double s_up_mid = 0;
        double s_low_mid_sec = 0;
        double s_up_mid_sec = 0;
        s_low_mid =
            (filtered_decison[j].EndLowS() - filtered_decison[j].StartLowS()) *
                (tt.StartTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartLowS();
        s_up_mid =
            (filtered_decison[j].EndUpS() - filtered_decison[j].StartUpS()) *
                (tt.StartTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartUpS();
        s_low_mid_sec =
            (filtered_decison[j].EndLowS() - filtered_decison[j].StartLowS()) *
                (tt.EndTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartLowS();
        s_up_mid_sec =
            (filtered_decison[j].EndUpS() - filtered_decison[j].StartUpS()) *
                (tt.EndTime() - filtered_decison[j].StartTime()) /
                (filtered_decison[j].EndTime() -
                 filtered_decison[j].StartTime()) +
            filtered_decison[j].StartUpS();
        MotorwaySpeedObsExtend c{filtered_decison[j].Id(),
                                 tt.StartTime(),
                                 tt.EndTime(),
                                 s_low_mid,
                                 s_up_mid,
                                 s_low_mid_sec,
                                 s_up_mid_sec,
                                 filtered_decison[j].IfReverse(),
                                 filtered_decison[j].IfVirtual(),
                                 filtered_decison[j].IfVehicle(),
                                 filtered_decison[j].IfBump(),
                                 filtered_decison[j].lower_points(),
                                 filtered_decison[j].upper_points(),
                                 filtered_decison[j].if_dynamic()};
        filtered_decison_new.push_back(c);
      }
    }
  }

  filtered_decison.clear();
  filtered_decison = filtered_decison_new;
}

void MotorwaySpeedIterCipvDecider::IntegrateObsDecision(
    std::vector<MotorwaySpeedObsExtend>& filtered_segmentation_decision,
    std::vector<MotorwaySpeedObsExtend>& complete_list) {
  complete_list.clear();
  for (int i = 0; i < filtered_segmentation_decision.size(); i++) {
    LOG_INFO(
        "After cut ,obs  id : {},start_t : {:.3f},end_t {:.3f}, start _l_s : "
        "{:.3f} "
        ",end_l_s : {:.3f} ,start_up_s : {:.3f},end_u_s : {:.3f}",
        filtered_segmentation_decision[i].Id(),
        filtered_segmentation_decision[i].StartTime(),
        filtered_segmentation_decision[i].EndTime(),
        filtered_segmentation_decision[i].StartLowS(),
        filtered_segmentation_decision[i].EndLowS(),
        filtered_segmentation_decision[i].StartUpS(),
        filtered_segmentation_decision[i].EndUpS());
  }

  std::sort(filtered_segmentation_decision.begin(),
            filtered_segmentation_decision.end(),
            [](const MotorwaySpeedObsExtend& obs1,
               const MotorwaySpeedObsExtend& obs2) {
              if (obs1.StartTime() < obs2.StartTime()) {
                return true;
              } else if (obs1.StartTime() > obs2.StartTime()) {
                return false;
              }
              return obs1.StartLowS() < obs2.StartLowS();
            });
  if (filtered_segmentation_decision.size() == 0) {
    MotorwaySpeedObsExtend start_obs_extend{999,
                                            0,
                                            8,
                                            virtual_obs_dis_,
                                            virtual_obs_dis_ + 1,
                                            virtual_obs_dis_,
                                            virtual_obs_dis_ + 1,
                                            false,
                                            false,
                                            false,
                                            false,
                                            {},
                                            {},
                                            false};
    start_obs_extend.SetPathClear();
    filtered_segmentation_decision.push_back(start_obs_extend);
  }
  AddIntervalVirtualObs(filtered_segmentation_decision, complete_list);

  std::sort(complete_list.begin(), complete_list.end(),
            [](const MotorwaySpeedObsExtend& obs1,
               const MotorwaySpeedObsExtend& obs2) {
              if (obs1.StartTime() < obs2.StartTime()) {
                return true;
              } else if (obs1.StartTime() > obs2.StartTime()) {
                return false;
              }
              return obs1.StartLowS() < obs2.StartLowS();
            });
  LOG_INFO(
      "Iter Deduction Sort and add virual Obj process is Finished . Process "
      "fix and sorted obj "
      "size is {}",
      complete_list.size());
  LOG_INFO("Iter Deduction Complete obs list:");
  for (int i = 0; i < complete_list.size(); i++) {
    LOG_INFO(
        "obs id : {},start_t : {:.3f},end_t {:.3f}, start _l_s : {:.3f} "
        ",end_l_s : {:.3f} ,start_up_s : {:.3f},end_u_s : {:.3f}",
        complete_list[i].Id(), complete_list[i].StartTime(),
        complete_list[i].EndTime(), complete_list[i].StartLowS(),
        complete_list[i].EndLowS(), complete_list[i].StartUpS(),
        complete_list[i].EndUpS());
  }
}

void MotorwaySpeedIterCipvDecider::AddIntervalVirtualObs(
    const std::vector<MotorwaySpeedObsExtend>& filtered_segmentation_decision,
    std::vector<MotorwaySpeedObsExtend>& complete_list) {
  if (filtered_segmentation_decision[0].StartTime() > 0) {
    MotorwaySpeedObsExtend start_obs_extend{
        998,
        0,
        filtered_segmentation_decision[0].StartTime(),
        virtual_obs_dis_,
        virtual_obs_dis_ + 1,
        virtual_obs_dis_,
        virtual_obs_dis_ + 1,
        false,
        false,
        false,
        false,
        {},
        {},
        false};
    start_obs_extend.SetPathClear();
    complete_list.emplace_back(start_obs_extend);
  }
  double max_t = 0;
  for (const auto& obj : filtered_segmentation_decision) {
    max_t = std::max(obj.EndTime(), max_t);
  }
  if (max_t < 8) {
    MotorwaySpeedObsExtend end_obs_extend{1000,
                                          max_t,
                                          8,
                                          virtual_obs_dis_,
                                          virtual_obs_dis_ + 1,
                                          virtual_obs_dis_,
                                          virtual_obs_dis_ + 1,
                                          false,
                                          false,
                                          false,
                                          false,
                                          {},
                                          {},
                                          false};
    end_obs_extend.SetPathClear();
    complete_list.emplace_back(end_obs_extend);
  }
  complete_list.insert(complete_list.end(),
                       filtered_segmentation_decision.begin(),
                       filtered_segmentation_decision.end());
  double current_start_time = filtered_segmentation_decision[0].StartTime();

  for (const auto& interval : filtered_segmentation_decision) {
    if (current_start_time < interval.StartTime()) {
      MotorwaySpeedObsExtend interval_obs_extend{888,
                                                 current_start_time,
                                                 interval.StartTime(),
                                                 virtual_obs_dis_,
                                                 virtual_obs_dis_ + 1,
                                                 virtual_obs_dis_,
                                                 virtual_obs_dis_ + 1,
                                                 false,
                                                 false,
                                                 false,
                                                 false,
                                                 {},
                                                 {},
                                                 false};
      interval_obs_extend.SetPathClear();
      complete_list.emplace_back(interval_obs_extend);
    }
    current_start_time = std::max(current_start_time, interval.EndTime());
  }
}

void MotorwaySpeedIterCipvDecider::CalUpperDecisionLimit(TaskInfo& task_info) {
  LOG_INFO("Read minimum speed limit !");
  min_v_bound_ = data_center_->drive_strategy_max_speed();
  v_target_ = data_center_->drive_strategy_max_speed();
  limit_a_ = 0.0;
  const auto& speed_limits =
      data_center_->behavior_speed_limits().speed_limits();
  const auto& speed_limits_enable =
      data_center_->behavior_speed_limits().speed_limits_enable();

  for (std::size_t i = 0; i < speed_limits_enable.size(); ++i) {
    if (!speed_limits_enable[i]) {
      continue;
    }
    const auto& speed_limit = speed_limits[i];

    if (speed_limit.acceleration() <= -0.01) {
      limit_a_ = std::min(limit_a_, speed_limit.acceleration());
      continue;
    }
    if (speed_limit.constraint_type() == SpeedLimitType::SOFT &&
        speed_limit.upper_bounds_size() == 1) {
      v_target_ = std::min(v_target_, speed_limit.upper_bounds().at(0));
    }

    if (speed_limit.constraint_type() == SpeedLimitType::HARD &&
        speed_limit.upper_bounds_size() == 1) {
      LOG_INFO("show infinite speed limit is [{}]",
               speed_limit.upper_bounds().at(0));
      min_v_bound_ = std::min(min_v_bound_, speed_limit.upper_bounds().at(0));
    }
  }

  LOG_INFO(
      "show Hard speed limit is [{}] , soft speed limit is [{}] , limit "
      "deceleration is [{}]",
      min_v_bound_, v_target_, limit_a_);
}

void MotorwaySpeedIterCipvDecider::ProcessFastestDeduction() {
  LOG_INFO("First m Step into Fastest Deduction.");
  MotorwayHybirdPid pid_fastest_deduction{
      ego_current_state_, 0, 8, v_target_, min_v_bound_, limit_a_};
  DeductionEgoStateSequence fastest_deductioni_result =
      pid_fastest_deduction.CalPidProcess();

  ego_deduction_ = fastest_deductioni_result;
}

DeductionCollisionResult MotorwaySpeedIterCipvDecider::CollisionCheck() {
  int fist_obs_overlap_index{10000};
  std::pair<bool, int> check_result;
  for (int i = 0; i < completion_obs_list_.size(); i++) {
    MotorwayCheckDeductionCollisionCheck single_check{ego_deduction_,
                                                      completion_obs_list_[i]};
    check_result = single_check.CheckAndCalCollisionPoint();
    if (check_result.first) {
      fist_obs_overlap_index = i;
      LOG_INFO("deduction has overlap with obs index : {} ,id is :{}", i,
               completion_obs_list_[i].Id());
      break;
    }
  }
  // bool int int : if collision ; t index ; obs index;
  return DeductionCollisionResult{check_result.first, check_result.second,
                                  fist_obs_overlap_index};
}

DeductionCollisionResult MotorwaySpeedIterCipvDecider::CollisionCheckForTake(
    const DeductionEgoStateSequence& deduction_data) {
  int fist_obs_overlap_index{10000};
  std::pair<bool, int> check_result;
  for (int i = 0; i < completion_obs_list_.size(); i++) {
    MotorwayCheckDeductionCollisionCheck single_check{deduction_data,
                                                      completion_obs_list_[i]};
    check_result = single_check.CheckAndCalCollisionPoint();
    if (check_result.first) {
      fist_obs_overlap_index = i;
      LOG_INFO("deduction has overlap with obs index : {} ,id is  : {} ", i,
               completion_obs_list_[i].Id());
      break;
    }
  }
  // bool int int : if collision ; t index ; obs index;
  return DeductionCollisionResult{check_result.first, check_result.second,
                                  fist_obs_overlap_index};
}

void MotorwaySpeedIterCipvDecider::SetUpBackupPara(TaskInfo& task_info,
                                                   const int& id,
                                                   const bool& if_real) {
  auto& backup_cipv = task_info.current_frame()
                          ->mutable_outside_planner_data()
                          ->motorway_speed_obstacle_context.backup_cipv;
  backup_cipv.Reset();
  backup_cipv.cipv_id = id;
  backup_cipv.has_cipv = if_real;
}

void MotorwaySpeedIterCipvDecider::IterObsToHybirdIdm(
    const std::pair<MotorwaySpeedObsExtend, int>& obs_extend,
    ObsBoundPolySeris obs_extend_polypoint) {
  std::pair<double, int> nearest_t_pair = FindClosestTimeAndIndex(
      ego_deduction_.deduction_ego_t_sequence, obs_extend.first.ExtendTime());
  LOG_INFO(
      "this iter Hybird Idm Deduction start at deduction time stamp is "
      ": "
      "{:.3f}",
      nearest_t_pair.first);
  // update ego start state

  UpdataEgoDeductionStartState(nearest_t_pair.second);
  // delete collision data
  LOG_INFO("need cut protect data , cut index is : {}", nearest_t_pair.second);
  DeleteCollisonData(nearest_t_pair.second);
  MotorwayHybirdIDM hybird_idm_deduction{
      ego_current_state_,
      obs_extend_polypoint.lower_obs_decision_poly_seris,
      obs_extend.first.IfVirtual(),
      obs_extend.first.IfVehicle(),
      obs_extend.first.IfBump(),
      min_v_bound_,
      v_target_,
      limit_a_};
  DeductionEgoStateSequence hybird_idm_deduction_result =
      hybird_idm_deduction.CalHybirdIDMProcess();
  AddDeductionData(hybird_idm_deduction_result);
}

void MotorwaySpeedIterCipvDecider::BidirectionalIter(
    std::pair<MotorwaySpeedObsExtend, int>& obs_extend) {
  auto CalLowerBound = [](ObsBoundPolySeris& obs_extend_polypoint) {
    for (size_t i = 0;
         i < obs_extend_polypoint.lower_obs_decision_poly_seris.size(); ++i) {
      obs_extend_polypoint.lower_obs_decision_poly_seris[i].obs_s =
          obs_extend_polypoint.lower_obs_decision_poly_seris[i].obs_s >= 0.5
              ? (obs_extend_polypoint.lower_obs_decision_poly_seris[i].obs_s -
                 0.5)
              : 0;
    }
  };
  LOG_INFO(
      ">>-----------------Start S direction buffer iter "
      "process--------------->> ");
  int max_s_direction_time = 3;
  ObsBoundPolySeris obs_extend_polypoint = obs_extend.first.IterExtendStart();
  obs_extend.first.ResetExtendOrder();
  int cal_iter_index = 1;
  while (cal_iter_index <= 3) {
    // S方向向下拉buffer
    CalLowerBound(obs_extend_polypoint);

    LOG_INFO("Show !!!!!!!!     : {}",
             obs_extend_polypoint.lower_obs_decision_poly_seris[0].obs_s);

    IterObsToHybirdIdm(obs_extend, obs_extend_polypoint);
    auto check_result = CollisionCheck();
    auto collision_obs_index = check_result.collision_obs_index;
    LOG_INFO("if S direction Iter has overlap  ? : [{}]",
             check_result.has_collision);
    if (check_result.has_collision) {
      LOG_INFO(">>S direction buffer iter not enough ");
      cal_iter_index++;
    } else {
      LOG_INFO("S direction buffer can prevent collision , index is {}",
               cal_iter_index);
      break;
    }
  }
}

DeductionEgoStateSequence MotorwaySpeedIterCipvDecider::PidDeductionForReal(
    const int& obs_index) {
  LOG_INFO("step into no iter Hybird idm process future obs");
  // if pid canovertake
  std::vector<ObsDecisionBound> low_bound =
      completion_obs_list_[obs_index].InterpolatePointsForUpper();
  std::vector<double> obs_low_time{};
  for (int i = 0; i < low_bound.size(); i++) {
    obs_low_time.push_back(low_bound[i].time);
  }
  std::pair<double, int> nearest_t_pair =
      FindClosestTimeAndIndex(obs_low_time, ego_current_state_.ego_t);

  MotorwayHybirdPid pid_deduction{ego_current_state_,
                                  nearest_t_pair.first,
                                  completion_obs_list_[obs_index].EndTime(),
                                  v_target_,
                                  min_v_bound_,
                                  limit_a_};
  LOG_INFO(
      "step into Hybird Pid ,  start time is {:.3f} ,obs end time "
      "is {:.3f}",
      nearest_t_pair.first, completion_obs_list_[obs_index].EndTime());
  DeductionEgoStateSequence pid_deductioni_result =
      pid_deduction.CalPidProcess();
  return pid_deductioni_result;
}

void MotorwaySpeedIterCipvDecider::NormalIdmDeduction(const int& obs_index) {
  std::vector<ObsDecisionBound> follow_obs_poly_point =
      completion_obs_list_[obs_index].InterpolatePointsForLow();

  std::vector<double> obs_low_time{};
  for (int i = 0; i < follow_obs_poly_point.size(); i++) {
    obs_low_time.push_back(follow_obs_poly_point[i].time);
  }

  std::pair<double, int> nearest_t_pair =
      FindClosestTimeAndIndex(obs_low_time, ego_current_state_.ego_t);
  follow_obs_poly_point.erase(
      follow_obs_poly_point.begin(),
      follow_obs_poly_point.begin() + nearest_t_pair.second);

  LOG_INFO("check cut obs bound ,single idm start time : {:.3f}",
           follow_obs_poly_point[0].time);
  MotorwayHybirdIDM hybird_idm_deduction{
      ego_current_state_,
      follow_obs_poly_point,
      completion_obs_list_[obs_index].IfVirtual(),
      completion_obs_list_[obs_index].IfVehicle(),
      completion_obs_list_[obs_index].IfBump(),
      min_v_bound_,
      v_target_,
      limit_a_};
  DeductionEgoStateSequence hybird_idm_deduction_result =
      hybird_idm_deduction.CalHybirdIDMProcess();

  AddDeductionData(hybird_idm_deduction_result);
}

void MotorwaySpeedIterCipvDecider::IterDeduction(
    TaskInfo& task_info, const int& fist_obs_overlap_index) {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  std::vector<std::pair<MotorwaySpeedObsExtend, int>> obs_need_iter_list;
  obs_need_iter_list.clear();
  obs_need_iter_list.emplace_back(std::make_pair(
      completion_obs_list_[fist_obs_overlap_index], fist_obs_overlap_index));

  int index_before_clear = 10000;
  int iter_index = 0;
  int iter_obs_extend = 0;

  while (iter_index <= 20) {
    iter_index = iter_index + 1;
    if (obs_need_iter_list.size() > 0) {
      // need iter obs
      std::pair<MotorwaySpeedObsExtend, int> obs_extend =
          obs_need_iter_list.back();

      ObsBoundPolySeris obs_extend_polypoint =
          obs_extend.first.IterExtendStart();
      iter_obs_extend++;
      LOG_INFO(
          "Step into iter IDM Process, general iter index is {} ,  "
          "obs real id is {} , single obs extend iter is {}",
          iter_index, obs_need_iter_list.back().first.Id(), iter_obs_extend);
      // reset extend order
      obs_extend.first.ResetExtendOrder();
      obs_need_iter_list.emplace_back(obs_extend);
      IterObsToHybirdIdm(obs_extend, obs_extend_polypoint);

      auto check_result = CollisionCheck();
      auto collision_obs_index = check_result.collision_obs_index;
      LOG_INFO("if has overlap  ? : [{}]", check_result.has_collision);

      if (check_result.has_collision) {
        if (collision_obs_index != obs_extend.second) {
          LOG_INFO("Deduction has overlap with other obs index");
          obs_need_iter_list.push_back(std::make_pair(
              completion_obs_list_[collision_obs_index], collision_obs_index));
        }
      } else {
        LOG_INFO("iter hybird idm is clean , reset iter_obs_extend");
        iter_obs_extend = 0;
        index_before_clear = obs_need_iter_list.back().second;
        obs_need_iter_list.clear();
        LOG_INFO("index_before_clear : {} ", index_before_clear);
        if ((index_before_clear >= (completion_obs_list_.size() - 1)) ||
            (ego_deduction_.deduction_ego_t_sequence.back() >= 7.5)) {
          LOG_INFO("ALL deduction fininshed at iter Hybird Idm !");
          break;
        }
      }

      if (iter_obs_extend >= speed_iter_deduction_config.max_backtrack_time &&
          check_result.has_collision) {
        LOG_INFO("over the most iter obs expend  ! : [{}]", iter_obs_extend);
        LOG_INFO("the size of need concern list is  : {}",
                 obs_need_iter_list.size());
        LOG_INFO("Deducion stop at origin obs size is : {}",
                 collision_obs_index);

        BidirectionalIter(obs_extend);
        LOG_INFO("Deducion collision at time index is : {}, cut data",
                 check_result.collision_t_index);

        break;
      }
    } else {
      // no iter process
      LOG_INFO("step into process future obs");
      UpdataEgoDeductionStartState(
          ego_deduction_.deduction_ego_t_sequence.size() - 1);
      // check pid or idm
      if (completion_obs_list_[index_before_clear + 1].ShowPathClear()) {
        PidDeductionForVirtual(completion_obs_list_[index_before_clear + 1]);

        index_before_clear++;
        LOG_INFO("PID process finished, index is : {}", index_before_clear);
        if ((index_before_clear >= (completion_obs_list_.size() - 1)) ||
            (ego_deduction_.deduction_ego_t_sequence.back() >= 7.5)) {
          LOG_INFO("ALL deduction fininshed at Pid !");
          break;
        }
      } else if (!completion_obs_list_[index_before_clear + 1]
                      .ShowPathClear()) {
        auto pid_deductioni_result =
            PidDeductionForReal(index_before_clear + 1);

        auto check_result = CollisionCheckForTake(pid_deductioni_result);
        if (!check_result.has_collision) {
          LOG_INFO(
              "Although is idm process ,pid can deal with it ,obs index is : "
              "{}",
              index_before_clear + 1);
          AddDeductionData(pid_deductioni_result);
          index_before_clear++;
          LOG_INFO("IDM process but use pid finished");
          if ((index_before_clear >= (completion_obs_list_.size() - 1)) ||
              (ego_deduction_.deduction_ego_t_sequence.back() >= 7.5)) {
            LOG_INFO("ALL deduction fininshed at Pid !");
            break;
          }
        } else {
          std::pair<double, int> nearest_t_pair = FindClosestTimeAndIndex(
              ego_deduction_.deduction_ego_t_sequence,
              completion_obs_list_[index_before_clear + 1].StartTime());
          DeleteCollisonData(nearest_t_pair.second);

          NormalIdmDeduction(index_before_clear + 1);

          LOG_INFO("process single idm ,obs index is : {}",
                   index_before_clear + 1);

          auto check_result = CollisionCheck();

          if (check_result.has_collision) {
            LOG_INFO(
                "idm deduction has overlap with origin obs index : "
                "{}",
                check_result.collision_obs_index);

            if (check_result.collision_obs_index != (index_before_clear + 1)) {
              LOG_INFO("Deduction has overlap with other obs index");
              obs_need_iter_list.push_back(std::make_pair(
                  completion_obs_list_[check_result.collision_obs_index],
                  check_result.collision_obs_index));
            } else {
              LOG_INFO("Deduction has overlap with  this right now obs index");
              obs_need_iter_list.push_back(
                  std::make_pair(completion_obs_list_[index_before_clear + 1],
                                 index_before_clear + 1));
            }
          } else {
            index_before_clear++;
            obs_need_iter_list.clear();
            if ((index_before_clear >= (completion_obs_list_.size() - 1)) ||
                (ego_deduction_.deduction_ego_t_sequence.back() >= 7.5)) {
              LOG_INFO("ALL deduction fininshed at idm!");
              break;
            }
          }
        }
      }
    }
    if (iter_index >= 20) {
      LOG_INFO("Process over the most process time , wait for pushlish data");
      LOG_INFO("check end time : {}",
               ego_deduction_.deduction_ego_t_sequence.back());
      break;
    }
  }
}

void MotorwaySpeedIterCipvDecider::PidDeductionForVirtual(
    const MotorwaySpeedObsExtend& obs_extend) {
  LOG_INFO("step into Hybird Pid process future obs");
  LOG_INFO(
      "step into Hybird Pid , obs start time is {:.3f} ,obs end time "
      "is {:.3f}",
      obs_extend.StartTime(), obs_extend.EndTime());
  MotorwayHybirdPid pid_deduction{ego_current_state_,   obs_extend.StartTime(),
                                  obs_extend.EndTime(), v_target_,
                                  min_v_bound_,         limit_a_};
  DeductionEgoStateSequence pid_deductioni_result =
      pid_deduction.CalPidProcess();

  std::pair<double, int> nearest_t_pair = FindClosestTimeAndIndex(
      ego_deduction_.deduction_ego_t_sequence, obs_extend.StartTime());
  LOG_INFO(
      "this Hybird pid Deduction start at deduction time stamp is : "
      "{:.3f}",
      nearest_t_pair.first);
  // delete collision data
  LOG_INFO("need cut protect data for hybird pid, cut index is : {}",
           nearest_t_pair.second);
  DeleteCollisonData(nearest_t_pair.second);

  AddDeductionData(pid_deductioni_result);
}

bool MotorwaySpeedIterCipvDecider::Process(TaskInfo& task_info) {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;

  CalUpperDecisionLimit(task_info);

  LOG_INFO("Step into Process Function !");

  ProcessFastestDeduction();

  auto check_result = CollisionCheck();

  if (!check_result.has_collision) {
    LOG_INFO(
        "fastest deduction donot overlap with any obs ,deduction finished !");
    LOG_INFO("fastest deduction show finished !");

    return true;
  } else {
    LOG_INFO("fastest deduction has overlap! step into iter idm !");

    IterDeduction(task_info, check_result.collision_obs_index);
  }
  LOG_INFO("Process Function has finished , wait for pushlish data");

  return true;
}

void MotorwaySpeedIterCipvDecider::AddDeductionData(
    const DeductionEgoStateSequence& deduction_data) {
  int filter_size = 0;
  ego_deduction_.deduction_ego_a_sequence.insert(
      ego_deduction_.deduction_ego_a_sequence.end(),
      deduction_data.deduction_ego_a_sequence.begin() + filter_size,
      deduction_data.deduction_ego_a_sequence.end());
  ego_deduction_.deduction_ego_v_sequence.insert(
      ego_deduction_.deduction_ego_v_sequence.end(),
      deduction_data.deduction_ego_v_sequence.begin() + filter_size,
      deduction_data.deduction_ego_v_sequence.end());
  ego_deduction_.deduction_ego_p_sequence.insert(
      ego_deduction_.deduction_ego_p_sequence.end(),
      deduction_data.deduction_ego_p_sequence.begin() + filter_size,
      deduction_data.deduction_ego_p_sequence.end());
  ego_deduction_.deduction_ego_t_sequence.insert(
      ego_deduction_.deduction_ego_t_sequence.end(),
      deduction_data.deduction_ego_t_sequence.begin() + filter_size,
      deduction_data.deduction_ego_t_sequence.end());
}

void MotorwaySpeedIterCipvDecider::UpdataEgoDeductionStartState(
    int update_index) {
  ego_current_state_.ego_a =
      ego_deduction_.deduction_ego_a_sequence[update_index];
  ego_current_state_.ego_t =
      ego_deduction_.deduction_ego_t_sequence[update_index];
  ego_current_state_.ego_v =
      ego_deduction_.deduction_ego_v_sequence[update_index];
  ego_current_state_.ego_p =
      ego_deduction_.deduction_ego_p_sequence[update_index];
}

void MotorwaySpeedIterCipvDecider::DeleteCollisonData(int cut_index) {
  ego_deduction_.deduction_ego_a_sequence.erase(
      ego_deduction_.deduction_ego_a_sequence.begin() + cut_index,
      ego_deduction_.deduction_ego_a_sequence.end());
  ego_deduction_.deduction_ego_p_sequence.erase(
      ego_deduction_.deduction_ego_p_sequence.begin() + cut_index,
      ego_deduction_.deduction_ego_p_sequence.end());
  ego_deduction_.deduction_ego_v_sequence.erase(
      ego_deduction_.deduction_ego_v_sequence.begin() + cut_index,
      ego_deduction_.deduction_ego_v_sequence.end());
  ego_deduction_.deduction_ego_t_sequence.erase(
      ego_deduction_.deduction_ego_t_sequence.begin() + cut_index,
      ego_deduction_.deduction_ego_t_sequence.end());
}

std::pair<double, int> MotorwaySpeedIterCipvDecider::FindClosestTimeAndIndex(
    const std::vector<double>& list, double target) {
  int left = 0;
  int right = list.size() - 1;
  double closest = list[0];
  int closest_index = 0;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    double mid_value = list[mid];

    if (mid_value == target) {
      return std::make_pair(mid_value, mid);
    }

    if (std::abs(mid_value - target) < std::abs(closest - target)) {
      closest = mid_value;
      closest_index = mid;
    }

    if (mid_value < target) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }
  return std::make_pair(closest, closest_index);
}
void MotorwaySpeedIterCipvDecider::SetupIntervalSpeedLimit(
    TaskInfo& task_info) {
  MotorwaySequenceDecisionData goal_decision{
      .source = MotorwaySequenceDecisionType::MULTI_CIPV};
  for (std::size_t i = 0; i < ego_deduction_.deduction_ego_t_sequence.size();
       ++i) {
    STPoint st_point{}, vt_point{}, at_point{};
    st_point.set_s(ego_deduction_.deduction_ego_p_sequence[i]);
    st_point.set_t(ego_deduction_.deduction_ego_t_sequence[i]);
    vt_point.set_s(ego_deduction_.deduction_ego_v_sequence[i]);
    vt_point.set_t(ego_deduction_.deduction_ego_t_sequence[i]);
    at_point.set_s(ego_deduction_.deduction_ego_a_sequence[i]);
    at_point.set_t(ego_deduction_.deduction_ego_t_sequence[i]);
    goal_decision.goal_s.emplace_back(std::move(st_point));
    goal_decision.goal_v.emplace_back(std::move(vt_point));
    goal_decision.goal_a.emplace_back(std::move(at_point));
  }
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->motorway_speed_context.goal_decision.emplace_back(
          std::move(goal_decision));
  LOG_INFO("set iter deduction follow goal finished");
}

void MotorwaySpeedIterCipvDecider::AbnormalDataEliminate() {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  if (ego_deduction_.deduction_ego_t_sequence.size() < 51) {
    LOG_INFO("before add data , t size : {}",
             ego_deduction_.deduction_ego_t_sequence.size());
    LOG_INFO("Iter deduction result is to little ,add data");

    double cur_t = ego_deduction_.deduction_ego_t_sequence.back();
    size_t old_t_size{ego_deduction_.deduction_ego_t_sequence.size()};
    for (int i = 0; i <= (51 - old_t_size); i++) {
      ego_deduction_.deduction_ego_t_sequence.push_back(cur_t + (i + 1) * 0.1);

      ego_deduction_.deduction_ego_a_sequence.push_back(
          speed_iter_deduction_config.stop_a);
      double v_brake = ego_deduction_.deduction_ego_v_sequence.back() +
                       0.1 * speed_iter_deduction_config.stop_a;
      v_brake = v_brake < 0 ? 0 : v_brake;
      ego_deduction_.deduction_ego_v_sequence.push_back(v_brake);
      ego_deduction_.deduction_ego_p_sequence.push_back(
          ego_deduction_.deduction_ego_p_sequence.back() +
          0.1 * ego_deduction_.deduction_ego_v_sequence.back());
    }
  }
  // clear unused data
  std::vector<double> clean_deduction_ego_v_sequence{};
  std::vector<double> clean_deduction_ego_p_sequence{};
  std::vector<double> clean_deduction_ego_a_sequence{};
  std::vector<double> clean_deduction_ego_t_sequence{};

  DeductionEgoStateSequence clean_deduciton_data{};
  clean_deduciton_data.Reset();

  for (int i = 0; i < ego_deduction_.deduction_ego_t_sequence.size(); i++) {
    if (i == 0 ||
        int(ego_deduction_.deduction_ego_t_sequence[i] * 10) >
            int(ego_deduction_.deduction_ego_t_sequence[i - 1] * 10)) {
      clean_deduciton_data.deduction_ego_t_sequence.push_back(
          ego_deduction_.deduction_ego_t_sequence[i]);
      clean_deduciton_data.deduction_ego_v_sequence.push_back(
          ego_deduction_.deduction_ego_v_sequence[i]);
      clean_deduciton_data.deduction_ego_a_sequence.push_back(
          ego_deduction_.deduction_ego_a_sequence[i]);
      clean_deduciton_data.deduction_ego_p_sequence.push_back(
          ego_deduction_.deduction_ego_p_sequence[i]);
    } else {
      LOG_INFO("time stamp is abnormal : : {}",
               ego_deduction_.deduction_ego_t_sequence[i]);
    }
  }
  ego_deduction_.Reset();
  ego_deduction_ = clean_deduciton_data;
}

void MotorwaySpeedIterCipvDecider::SaveLOGResults() {
  AbnormalDataEliminate();

  for (std::size_t i = 0; i < ego_deduction_.deduction_ego_t_sequence.size();
       ++i) {
    if ((i + 1) % 3 == 0 || i == 0) {
      LOG_INFO(
          "Iter deduction result -- t : {:.6f} , v : {:.3f}, a : {:.3f}, s : "
          "{:.6f}",
          ego_deduction_.deduction_ego_t_sequence[i],
          ego_deduction_.deduction_ego_v_sequence[i],
          ego_deduction_.deduction_ego_a_sequence[i],
          ego_deduction_.deduction_ego_p_sequence[i]);
    }
  }
}

void MotorwaySpeedIterCipvDecider::GenerateSBoundForObs(TaskInfo& task_info) {
  auto& tunnel_decision = task_info.current_frame()
                              ->mutable_outside_planner_data()
                              ->motorway_speed_context.tunnel_decision;
  tunnel_decision.Reset();
  for (std::size_t index = 0;
       index < ego_deduction_.deduction_ego_t_sequence.size(); ++index) {
    double s_lower_bound = 0.0, s_upper_bound = virtual_obs_dis_;
    double ego_t = ego_deduction_.deduction_ego_t_sequence[index];
    double ego_s = ego_deduction_.deduction_ego_p_sequence[index];

    for (auto& obs : completion_obs_list_) {
      if (obs.StartTime() < ego_t && obs.EndTime() > ego_t) {
        const auto& obs_l_seq = obs.InterpolatePointsForLow();
        const auto& obs_u_seq = obs.InterpolatePointsForUpper();
        const auto& it_left =
            std::lower_bound(obs_l_seq.begin(), obs_l_seq.end(), ego_t,
                             [](const ObsDecisionBound& bound, double t) {
                               return bound.time < t;
                             });
        const auto& it_right = std::next(it_left);
        if (it_right == obs_l_seq.end()) {
          if (it_left->obs_s > ego_s)
            s_upper_bound =
                std::min(s_upper_bound, std::max(it_left->obs_s, ego_s));
        } else {
          if (it_left->obs_s > ego_s || it_right->obs_s > ego_s) {
            s_upper_bound = std::min(
                s_upper_bound, std::min(std::max(it_left->obs_s, ego_s),
                                        std::max(it_right->obs_s, ego_s)));
          }
        }
        const auto& it_left_u =
            std::lower_bound(obs_u_seq.begin(), obs_u_seq.end(), ego_t,
                             [](const ObsDecisionBound& bound, double t) {
                               return bound.time < t;
                             });
        const auto& it_right_u = std::next(it_left_u);
        if (it_right_u == obs_u_seq.end()) {
          if (it_left_u->obs_s < ego_s)
            s_lower_bound =
                std::max(s_lower_bound, std::min(it_left_u->obs_s, ego_s));
        } else {
          if (it_left_u->obs_s < ego_s || it_right_u->obs_s < ego_s) {
            s_lower_bound =
                std::max(std::max(std::min(it_left_u->obs_s, ego_s),
                                  std::min(it_right_u->obs_s, ego_s)),
                         s_lower_bound);
          }
        }
        if (it_left->obs_s - 0.01 <= ego_s &&
            it_left_u->obs_s + 0.01 >= ego_s) {
          s_upper_bound = ego_s;
          s_lower_bound = 0.0;
          break;
        }
      }
    }
    if ((index + 1) % 3 == 0 || index == 0) {
      LOG_INFO("t index : {} --->>>> upper bound : {}", index,
               s_upper_bound + .3);
      LOG_INFO("t index : {} --->>>> lower bound : {} ", index,
               s_lower_bound - .1);
    }
    tunnel_decision.upper_s_tunnel.push_back(
        STPoint(s_upper_bound + .3, ego_t));
    tunnel_decision.lower_s_tunnel.push_back(
        STPoint(s_lower_bound - .1, ego_t));
  }
}

void MotorwaySpeedIterCipvDecider::GenerateVBoundForEgo(TaskInfo& task_info) {
  std::vector<IntervelSpeedLimit> speed_limits_orin{};
  std::vector<IntervelSpeedLimit> speed_limits_sorted{};
  auto& tunnel_decision = task_info.current_frame()
                              ->mutable_outside_planner_data()
                              ->motorway_speed_context.tunnel_decision;
  const double max_limit_speed = data_center_->drive_strategy_max_speed();
  for (std::size_t ind = 0;
       ind < ego_deduction_.deduction_ego_t_sequence.size(); ++ind) {
    STGoalVInfo u_v_bound{
        .goal_t_v = STPoint(max_limit_speed,
                            ego_deduction_.deduction_ego_t_sequence[ind]),
        .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
    // upper_v_bounds.emplace_back(u_v_bound);

    STGoalVInfo l_v_bound{
        .goal_t_v = STPoint(.0, ego_deduction_.deduction_ego_t_sequence[ind]),
        .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
    // lower_v_bounds.emplace_back(l_v_bound);

    tunnel_decision.upper_v_tunnel.push_back(
        STPoint(max_limit_speed, ego_deduction_.deduction_ego_t_sequence[ind]));
    tunnel_decision.lower_v_tunnel.push_back(
        STPoint(.0, ego_deduction_.deduction_ego_t_sequence[ind]));
  }
}

}  // namespace planning
}  // namespace neodrive
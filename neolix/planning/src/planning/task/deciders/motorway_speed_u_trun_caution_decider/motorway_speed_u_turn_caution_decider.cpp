#include "motorway_speed_u_turn_caution_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "planning_map/planning_map.h"
#include "reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {

MotorwaySpeedUTurnCautionDecider::MotorwaySpeedUTurnCautionDecider() {
  name_ = "MotorwaySpeedUTurnDecider";
}

void MotorwaySpeedUTurnCautionDecider::SaveTaskResults(TaskInfo& task_info) {}

void MotorwaySpeedUTurnCautionDecider::Reset() {}
void MotorwaySpeedUTurnCautionDecider::ResetPara() {
  have_merging_ = false;
  have_meeting_ = false;
  set_merge_area_obs_ = false;
  ego_to_virtual_dis_ = 1000;
}
void MotorwaySpeedUTurnCautionDecider::SaveLOGResults() {}

ErrorCode MotorwaySpeedUTurnCautionDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    LOG_INFO("path successed tasks is 0, skip rest tasks.");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  LOG_INFO(">>>> Motorway Speed U turn work normal");

  if (!Init(task_info)) {
    LOG_INFO("Motorway Speed U turn Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Checkfeasibility(task_info)) {
    return ErrorCode::PLANNING_OK;
  }

  if (!Process(task_info)) {
    LOG_INFO("Motorway Speed U turn Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("Process FInished.");
  SaveLOGResults();

  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedUTurnCautionDecider::Init(TaskInfo& task_info) {
  ResetPara();
  adc_current_s_ =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  adc_current_l_ =
      task_info.current_frame()->inside_planner_data().init_sl_point.l();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;

  return true;
}

bool MotorwaySpeedUTurnCautionDecider::Process(TaskInfo& task_info) {
  CheckCautionObs(task_info);
  if (set_merge_area_obs_) {
    EstablishVirtualObs(task_info);
  }
  return true;
}

bool MotorwaySpeedUTurnCautionDecider::Checkfeasibility(TaskInfo& task_info) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;

  for (const auto& mapPtr : traffic_conflict_zone_context.meeting_geoinfo) {
    if (!mapPtr->empty()) {
      have_meeting_ = true;
    }
  }

  if (have_meeting_) {
    LOG_INFO(" is going to meeting !!!!!!!!!!!!!!!!!!!!!!");
  }
  if (traffic_conflict_zone_context.type ==
      TrafficConflictZoneContext::connectionType::Merging) {
    LOG_INFO(" is going to Merge !!!!!!!!!!!!!!!!!!!!!!");
    have_merging_ = true;
  }

  if (!(have_merging_ || have_meeting_)) {
    LOG_INFO("Now is not in U turn Scenario, continue");
    return false;
  }
  VisUturnPara(task_info);
  return true;
}

void MotorwaySpeedUTurnCautionDecider::CheckCautionObs(TaskInfo& task_info) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;

  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();

  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();

  ConflictInfoForUTurn conflic_data{};
  const auto& conflict_data = conflic_data.ComputeConflictInfo(task_info);
  conflict_obs_.clear();
  if (conflict_data.empty()) {
    LOG_INFO(" Merging or Meeting Obs is empty , donot care ");
  } else {
    LOG_INFO(" Merging or Meeting area has Obs , need check ");
    // double obs_dis_to_conflict{100};
    for (auto iter : conflict_data) {
      ConflictObs conflic_obs{iter.agent.id,
                              iter.conflict_area_bound[0],
                              iter.agent.sl_velocity.x(),
                              iter.agent.heading_diff,
                              iter.conflict_area_bound[3],
                              iter.agent.obs_offset};
      LOG_INFO(
          " checkout conflict obs info , id :{}, heading diff {} , obs to area "
          "dis {} , ego to area dis {} ,obs speed : {}, obs offset to road {}",
          iter.agent.id, iter.agent.heading_diff, iter.conflict_area_bound[0],
          iter.conflict_area_bound[3], iter.agent.sl_velocity.x(),
          iter.agent.obs_offset);
      conflict_obs_.emplace_back(conflic_obs);
    }
  }

  int log_obs_id{};
  for (auto const iter : conflict_obs_) {
    double concern_max_dis = std::max(25.0, iter.obs_v * 4);
    if (iter.obs_dis > concern_max_dis) {
      LOG_INFO(" Obs id {} dis with conflict area is huge  , need ignore",
               iter.obs_id);
      continue;
    }
    if (iter.obs_dis < 0.0) {
      LOG_INFO(" Obs id {} dis with conflict area is negative  , need ignore",
               iter.obs_id);
      continue;
    }
    // if (iter.ego_dis_to_area < 0.0) {
    //   LOG_INFO(
    //       " Obs id is {} ,ego has already step into conflict area  , need "
    //       "ignore",
    //       iter.obs_id);
    //   continue;
    // }
    if (std::abs(iter.obs_heading_diff_to_road) > M_PI / 3.0) {
      LOG_INFO(
          " Obs id {} heading diff with road is larger than 60  , need ignore",
          iter.obs_id);
      continue;
    }
    if (std::abs(iter.obs_offset) > 3) {
      LOG_INFO(" Obs id {} offset to road is larger than 60  , need ignore",
               iter.obs_id);
      continue;
    }
    if (std::abs(iter.obs_dis / (std::cos(iter.obs_heading_diff_to_road) *
                                 iter.obs_v)) > 5.5) {
      LOG_INFO(" Obs id {} get to conflict area time is too long , need ignore",
               iter.obs_id);
      continue;
    }
    if (iter.obs_dis < 0.0) {
      LOG_INFO(" Obs id {}  has already get out of conflict area , need ignore",
               iter.obs_id);
      continue;
    }

    if (std::abs(iter.obs_v) < 0.1) {
      LOG_INFO(" Obs speed is too little  , need ignore", iter.obs_id);
      continue;
    }
    set_merge_area_obs_ = true;
    ego_to_virtual_dis_ = std::min(ego_to_virtual_dis_, iter.ego_dis_to_area);

    if (ego_to_virtual_dis_ <= iter.ego_dis_to_area) {
      log_obs_id = iter.obs_id;
    }
  }
  if (set_merge_area_obs_) {
    LOG_INFO(" checkout caution obs id : {} ,ego to virtual dis {}", log_obs_id,
             ego_to_virtual_dis_);
  }
}

void MotorwaySpeedUTurnCautionDecider::EstablishVirtualObs(
    TaskInfo& task_info) {
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  const double init_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();

  auto create_virtual_obstacle_mr = [&decision_data, &init_s,
                                     &path](const auto& start_s) {
    decision_data->create_virtual_obstacle(path, init_s, start_s,
                                           VirtualObstacle::U_TURN_MERGE);
    DataCenter::Instance()->mutable_event_report_proxy()->ExistVirtualObstacle(
        "U_Turn");
    LOG_INFO("Created virtual obstacle for U_Turn.");
  };
  auto create_virtual_obstacle_meeting = [&decision_data, &init_s,
                                          &path](const auto& start_s) {
    decision_data->create_virtual_obstacle(path, init_s, start_s,
                                           VirtualObstacle::U_TURN_MEETING);
    DataCenter::Instance()->mutable_event_report_proxy()->ExistVirtualObstacle(
        "U_Turn");
    LOG_INFO("Created virtual obstacle for U_Turn.");
  };

  if (set_merge_area_obs_ && ego_to_virtual_dis_ >= 0.1) {
    LOG_INFO("Created virtual obstacle for U_Turn in merge scenario");
    create_virtual_obstacle_mr(adc_current_s_ + ego_to_virtual_dis_ +
                               FLAGS_planning_virtual_obstacle_length / 2.0 +
                               FLAGS_planning_speed_plan_enlarge_self_buffer);
  }
}

void MotorwaySpeedUTurnCautionDecider::VisUturnPara(TaskInfo& task_info) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto e_U_turn_mr = vis::EventSender::Instance()->GetEvent("U_turn_mr");
  e_U_turn_mr->set_type(visualizer::Event::k2D);
  e_U_turn_mr->mutable_color()->set_r(0.8);
  e_U_turn_mr->mutable_color()->set_g(0.0);
  e_U_turn_mr->mutable_color()->set_b(0.0);
  e_U_turn_mr->mutable_color()->set_a(0.6);
  auto U_turn_mr = e_U_turn_mr->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (size_t i = 0; i < 80; ++i) {
    set_pt(U_turn_mr->add_point(), i * 0.1, have_merging_);
  }
  auto e_U_turn_meeting =
      vis::EventSender::Instance()->GetEvent("U_turn_meetint");
  e_U_turn_meeting->set_type(visualizer::Event::k2D);
  e_U_turn_meeting->mutable_color()->set_r(0.8);
  e_U_turn_meeting->mutable_color()->set_g(0.0);
  e_U_turn_meeting->mutable_color()->set_b(0.0);
  e_U_turn_meeting->mutable_color()->set_a(0.6);
  auto U_turn_meeting = e_U_turn_meeting->add_polyline();

  for (size_t i = 0; i < 80; ++i) {
    set_pt(U_turn_meeting->add_point(), i * 0.1, have_meeting_);
  }
}

}  // namespace planning
}  // namespace neodrive

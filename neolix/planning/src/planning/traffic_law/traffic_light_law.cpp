#include "traffic_light_law.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

using neodrive::global::perception::traffic_light::TrafficLight;
using neodrive::global::perception::traffic_light::TrafficLight_Color;

namespace {

double KPassingTimeMax = 12.0;                  // s
double KPassingSpeed = 3.0;                     // m/s
double KPassingTimeLeftTime = 5.0;              // s
double KJunctionExportFinishedThreshold = 5.0;  // m

bool NonMotorizedGreenBlink(const uint64_t traffic_light_id) {
  const auto& traffic_light_proxy =
      DataCenter::Instance()->environment().traffic_light_proxy();
  const auto non_motorized_info =
      traffic_light_proxy.GetLightsNonMotorizedById(traffic_light_id);
  if (non_motorized_info.empty()) return false;

  for (const auto& info : non_motorized_info) {
    if (info.color() == TrafficLight_Color::TrafficLight_Color_GREEN &&
        info.blink()) {
      return true;
    }
  }

  return false;
}

TrafficLight_Color NonMotorizedColor(const uint64_t traffic_light_id) {
  const auto& traffic_light_proxy =
      DataCenter::Instance()->environment().traffic_light_proxy();
  const auto non_motorized_info =
      traffic_light_proxy.GetLightsNonMotorizedById(traffic_light_id);
  if (non_motorized_info.empty())
    return TrafficLight_Color::TrafficLight_Color_UNKNOWN;

  // if has green, we will should be sure this color
  for (const auto& info : non_motorized_info) {
    if (info.color() == TrafficLight_Color::TrafficLight_Color_GREEN) {
      return TrafficLight_Color::TrafficLight_Color_GREEN;
    }
  }

  return TrafficLight_Color::TrafficLight_Color_UNKNOWN;
}

double PassingTime(const Boundary& adc_boundary,
                   const ReferenceLinePtr& ref_line,
                   const RouteSignal& route_signal,
                   double& distance_to_selected_end_s) {
  /// compute pass current stop_line to next_stop_line(in same
  /// junction)/export(this junction)
  double adc_start_s = adc_boundary.start_s();
  double selected_end_s;
  double route_start_s = route_signal.start_route_s;
  double closest_junction_start_s{-1.0};
  double closest_junction_end_s{-1.0};
  for (const auto& junction_overlap : ref_line->junction_overlaps()) {
    if (junction_overlap.end_s < adc_start_s) continue;
    closest_junction_start_s = junction_overlap.start_s;
    closest_junction_end_s = junction_overlap.end_s;
    break;
  }
  LOG_DEBUG("Before crosswalk: closest_junction start_s, end_s: {:.3f}, {:.3f}",
            closest_junction_start_s, closest_junction_end_s);

  double junction_export_crosswalk_start_s{-1.0};
  if (closest_junction_end_s > 0.) {
    double distance = std::numeric_limits<double>::max();
    double tmp_dis;
    for (const auto& crosswalk_overlap : ref_line->crosswalk_overlaps()) {
      tmp_dis = std::abs(closest_junction_end_s - crosswalk_overlap.start_s);
      if (tmp_dis > 20.0) continue;
      if (distance > tmp_dis) {
        junction_export_crosswalk_start_s = crosswalk_overlap.start_s;
        distance = tmp_dis;
      }
    }
  }
  LOG_DEBUG("junction_export_crosswalk_start_s: {:.3f}",
            junction_export_crosswalk_start_s);

  closest_junction_end_s =
      (junction_export_crosswalk_start_s > 0.)
          ? std::max(closest_junction_start_s,
                     junction_export_crosswalk_start_s -
                         KJunctionExportFinishedThreshold)
          : (closest_junction_end_s - KJunctionExportFinishedThreshold);
  LOG_INFO("After crosswalk: closest_junction start_s = {:.3f}, end_s = {:.3f}",
           closest_junction_start_s, closest_junction_end_s);

  /// signal_overlaps has been sorted
  double closest_stop_line_s{route_start_s};
  double closest_next_stop_line_s{closest_stop_line_s};
  for (const auto& signal_overlap : ref_line->signal_overlaps()) {
    if (closest_stop_line_s >= signal_overlap.start_s) continue;
    closest_next_stop_line_s = signal_overlap.start_s;
    break;
  }
  LOG_INFO("front stop_line: {:.3f}, {:.3f}", closest_stop_line_s,
           closest_next_stop_line_s);
  if (adc_start_s > route_start_s) {
    // Ensure that passing_time is less than 0.0 when calculating it.
    selected_end_s = route_start_s;
    LOG_INFO("already over line");
  } else if (closest_next_stop_line_s <= closest_stop_line_s + kMathEpsilon) {
    selected_end_s = closest_junction_end_s;
    LOG_INFO("not over line and no next_stop_line");
  } else if (closest_next_stop_line_s > closest_junction_end_s) {
    selected_end_s = closest_junction_end_s;
    LOG_INFO("not over line and exist next_stop_line but far away");
  } else {
    selected_end_s = closest_next_stop_line_s;
    LOG_INFO("not over line, exist next_stop_line and in this junction");
  }
  LOG_INFO(
      "adc_start_s, route_start_s, closest_stop_s, closest_next_stop_s, "
      "closest_junction_end_s, "
      "selected_end_s: {:.3f}, {:.3f} {:.3f}, {:.3f}, {:.3f}, {:.3f}",
      adc_start_s, route_signal.start_route_s, closest_stop_line_s,
      closest_next_stop_line_s, closest_junction_end_s, selected_end_s);

  double passing_time = (selected_end_s - adc_start_s) / KPassingSpeed;
  distance_to_selected_end_s = selected_end_s - adc_start_s;
  LOG_INFO(
      "current signal should along distance: {:.3f}, passing_time: {:.3f}, "
      "passing_time_max: {:.3f}",
      selected_end_s - adc_start_s, passing_time, KPassingTimeMax);

  return std::min(passing_time, KPassingTimeMax);
}

bool RemainTimePassingDecision(const Boundary& adc_boundary,
                               const ReferenceLinePtr& ref_line,
                               const RouteSignal& route_signal,
                               double& distance_to_selected_end_s) {
  double passing_time = PassingTime(adc_boundary, ref_line, route_signal,
                                    distance_to_selected_end_s);
  if (!route_signal.enable_left_time) {
    LOG_ERROR("no left time");
    return false;
    // return (passing_time < KPassingTime);
  }
  return (route_signal.left_time > passing_time - KPassingTimeLeftTime);
}

}  // namespace

TrafficLightLaw::TrafficLightLaw() : TrafficLaw("TrafficLightLaw") {}

ErrorCode TrafficLightLaw::Apply(TaskInfo& task_info,
                                 const InsidePlannerData& inside_data,
                                 const OutsidePlannerData& outside_data,
                                 const Boundary& adc_boundary,
                                 DecisionData* const decision_data,
                                 TrafficLawContext* const traffic_law_context) {
  LOG_INFO("TrafficLightLaw started");
  if (decision_data == nullptr || traffic_law_context == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  KPassingSpeed = traffic_law_config_.passing_speed_ratio *
                  DataCenter::Instance()->drive_strategy_max_speed();
  KPassingTimeMax = traffic_law_config_.passing_time_max_threshold;
  KPassingTimeLeftTime = traffic_law_config_.passing_time_left_time_threshold;
  KJunctionExportFinishedThreshold =
      traffic_law_config_.junction_export_finished_threshold;
  const auto& curr_scenario =
      DataCenter::Instance()->master_info().curr_scenario();
  if (curr_scenario == ScenarioState::INIT ||
      curr_scenario == ScenarioState::CRUISE ||
      curr_scenario == ScenarioState::INDOOR_CRUISE ||
      curr_scenario == ScenarioState::NARROW_RAOD ||
      curr_scenario == ScenarioState::DETOUR ||
      curr_scenario == ScenarioState::INTERSECTION ||
      curr_scenario == ScenarioState::SIDE_WAY_INTERSECTION ||
      curr_scenario == ScenarioState::BACK_OUT ||
      curr_scenario == ScenarioState::BARRIER_GATE ||
      curr_scenario == ScenarioState::PARKING) {
    is_motorway_ = false;
  } else if (curr_scenario == ScenarioState::MOTORWAY_CRUISE ||
             curr_scenario == ScenarioState::MOTORWAY_LANE_CHANGE ||
             curr_scenario == ScenarioState::MOTORWAY_INTERSECTION ||
             curr_scenario == ScenarioState::MOTORWAY_DETOUR) {
    is_motorway_ = true;
  } else {
    LOG_WARN("unknown scenario");
    is_motorway_ = false;
  }
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->traffic_light_stop_line = 1000.0;
  /// compute signal info from hdmap
  auto map_id_and_2nd_stop_line =
      traffic_law_context->MutableTrafficLightlawcontext()
          ->mutable_map_id_and_2nd_stop_line();
  auto behind_active_protect_time_stamp =
      traffic_law_context->MutableTrafficLightlawcontext()
          ->mutable_behind_active_protect_time_stamp();
  auto forward_active_protect_time_stamp =
      traffic_law_context->MutableTrafficLightlawcontext()
          ->mutable_forward_active_protect_time_stamp();
  TrafficLightTable traffic_light_table;
  ComputeSignalOverlapsInfo(task_info, adc_boundary, &traffic_light_table,
                            traffic_law_context);
  if (traffic_light_table.empty()) {
    map_id_and_2nd_stop_line->clear();
    behind_active_protect_time_stamp->Reset();
    forward_active_protect_time_stamp->Reset();
    DataCenter::Instance()->mutable_event_report_proxy()->EventReset(
        EventType::UNKNOWN_TRAFFIC_LIGHT);
    DataCenter::Instance()->mutable_event_report_proxy()->EventReset(
        EventType::STOP_OVER_LINE);
    traffic_law_context->MutableTrafficLightlawcontext()
        ->mutable_traffic_light_infos()
        ->clear();
    LOG_INFO("hdmap no singnal overlap info, skip.");
    return ErrorCode::PLANNING_SKIP;
  }
  LOG_INFO(
      "hd_map rough traffic_light_table and map_id_and_2nd_stop_line size: {}, "
      "{}",
      traffic_light_table.size(), map_id_and_2nd_stop_line->size());

  /// combine perception traffic light
  CombinePerceptionTrafficLight(task_info, traffic_light_table);

  /// traffic light decider
  TrafficLightDecider(task_info, adc_boundary, traffic_light_table,
                      traffic_law_context);

  /// create virtual obstacle
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  const double init_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  auto create_virtual_obstacle = [&decision_data, &init_s,
                                  &path](const auto& start_s) {
    decision_data->create_virtual_obstacle(path, init_s, start_s,
                                           VirtualObstacle::TRAFFIC_LIGHT);
    DataCenter::Instance()->mutable_event_report_proxy()->ExistVirtualObstacle(
        "TRAFFIC_LIGHT");
    LOG_INFO("Created virtual obstacle for traffic light.");
  };
  bool enable_active_protect = config::PlanningConfig::Instance()
                                   ->planning_research_config()
                                   .traffic_light_law_config.active_protect;
  double active_protect_dis = config::PlanningConfig::Instance()
                                  ->planning_research_config()
                                  .traffic_light_law_config.active_protect_dis;
  double forward_active_protect_min_time =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .traffic_light_law_config.forward_active_protect_min_time;
  double behind_active_protect_min_time =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .traffic_light_law_config.behind_active_protect_min_time;
  double enable_second_stop_line_protect =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .traffic_light_law_config.enable_second_stop_line_protect;
  for (const auto& iter : traffic_light_table) {
    LOG_INFO("traffic_light_id: {}, need_stop: {}, active_protect: {}",
             PlanningMap::Instance()->GetHashIdString(iter.second.id),
             iter.second.need_stop, iter.second.active_protect);

    if (!iter.second.need_stop) {
      continue;
    }
    double stop_line_s = iter.second.stop_line_s;

    // used for unknown
    bool behind_active_protect =
        enable_active_protect && iter.second.active_protect &&
        (stop_line_s - adc_boundary.end_s()) < active_protect_dis &&
        (stop_line_s - adc_boundary.start_s()) >= 0.0;

    // judge forward active protect
    // Maybe we won't use it anymore
    bool forward_active_protect =
        (stop_line_s < adc_boundary.start_s() + kMathEpsilon &&
         stop_line_s + traffic_law_config_.length_of_passing_stop_line_buffer >=
             adc_boundary.end_s());

    // create virtual obstacle
    LOG_INFO("final stop_line_s = {:.2f}", stop_line_s);
    create_virtual_obstacle(stop_line_s +
                            FLAGS_planning_virtual_obstacle_length / 2.0 +
                            FLAGS_planning_speed_plan_enlarge_self_buffer +
                            traffic_law_config_.stop_compensation_distance);
    if (outside_data.traffic_light_stop_line >
        stop_line_s - adc_boundary.end_s()) {
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->traffic_light_stop_line = stop_line_s - adc_boundary.end_s();
    }
    if (behind_active_protect) {
      if (behind_active_protect_time_stamp->start_time < kMathEpsilon) {
        behind_active_protect_time_stamp->start_time =
            DataCenter::Instance()->init_frame_time();
        continue;
      }
      behind_active_protect_time_stamp->curr_time =
          DataCenter::Instance()->init_frame_time();
      LOG_INFO("behind_active_protect_time: [{:.3f}, {:.3f}]",
               behind_active_protect_time_stamp->start_time,
               behind_active_protect_time_stamp->curr_time);
      // use both new and old takeover interface
      if (behind_active_protect_time_stamp->curr_time -
              behind_active_protect_time_stamp->start_time >
          behind_active_protect_min_time) {
        DataCenter::Instance()->mutable_master_info()->set_is_ask_for_takeover(
            true);
        DataCenter::Instance()
            ->mutable_master_info()
            ->set_stop_due_to_junction_left_turn(true);
      }
      DataCenter::Instance()->mutable_event_report_proxy()->SetEvent(
          EventType::UNKNOWN_TRAFFIC_LIGHT);
    }
    if (forward_active_protect) {
      if (forward_active_protect_time_stamp->start_time < kMathEpsilon) {
        forward_active_protect_time_stamp->start_time =
            DataCenter::Instance()->init_frame_time();
        continue;
      }
      forward_active_protect_time_stamp->curr_time =
          DataCenter::Instance()->init_frame_time();
      LOG_INFO("forward_active_protect_time: [{:.3f}, {:.3f}]",
               forward_active_protect_time_stamp->start_time,
               forward_active_protect_time_stamp->curr_time);

      if (forward_active_protect_time_stamp->curr_time -
              forward_active_protect_time_stamp->start_time >
          forward_active_protect_min_time) {
        DataCenter::Instance()->mutable_master_info()->set_is_ask_for_takeover(
            true);
        DataCenter::Instance()
            ->mutable_master_info()
            ->set_stop_due_to_junction_left_turn(true);
      }
      DataCenter::Instance()->mutable_event_report_proxy()->SetEvent(
          EventType::STOP_OVER_LINE);
      if (iter.second.active_protect) {
        DataCenter::Instance()->mutable_event_report_proxy()->SetEvent(
            EventType::UNKNOWN_TRAFFIC_LIGHT);
      }
    }
  }
  return ErrorCode::PLANNING_OK;
}

void TrafficLightLaw::ComputeSignalOverlapsInfo(
    const TaskInfo& task_info, const Boundary& adc_boundary,
    TrafficLightTable* const front_traffic_lights,
    TrafficLawContext* const traffic_law_context) {
  auto refer_ptr = task_info.reference_line();
  auto map_id_and_2nd_stop_line =
      traffic_law_context->MutableTrafficLightlawcontext()
          ->mutable_map_id_and_2nd_stop_line();
  front_traffic_lights->clear();

  std::set<uint64_t> set_signal_id;
  for (const auto& signal_overlap : refer_ptr->signal_overlaps()) {
    if (signal_overlap.object_id == 0) {
      LOG_WARN("signal object id is null");
      continue;
    }
    if (signal_overlap.start_s <=
        adc_boundary.end_s() -
            traffic_law_config_.length_of_passing_stop_line_buffer) {
      LOG_INFO(
          "traffic light {} is behind adc, skip",
          PlanningMap::Instance()->GetHashIdString(signal_overlap.object_id));
      continue;
    }
    ReferencePoint tmp_point;
    if (!refer_ptr->GetNearestRefPoint(signal_overlap.end_s, &tmp_point)) {
      LOG_ERROR(
          "traffic light {} get nearest ref_pt failed skip, start_s[{:.3f}]",
          PlanningMap::Instance()->GetHashIdString(signal_overlap.object_id),
          signal_overlap.start_s);
      continue;
    }
    LOG_DEBUG("signal ref pt lane id:{}, s,x,y:{:.4f},{:.4f},{:.4f}",
              cyberverse::HDMap::Instance()->GetIdHashString(
                  tmp_point.hd_map_lane_id()),
              signal_overlap.end_s, tmp_point.x(), tmp_point.y());
    ReferencePoint signal_start_point;
    if (refer_ptr->GetNearestRefPoint(signal_overlap.start_s,
                                      &signal_start_point)) {
      LOG_INFO("signal start s, x, y: {}, {}, {}", signal_start_point.s(),
               signal_start_point.x(), signal_start_point.y());
    }

    if (is_motorway_) {
      const double traffic_yaw = DataCenter::Instance()
                                     ->environment()
                                     .traffic_light_proxy()
                                     .GetTrafficYaw();
      LOG_INFO("traffic yaw = {:.2f}", traffic_yaw);
      if (task_info.curr_sl().s() > signal_overlap.start_s &&
          traffic_yaw > traffic_law_config_.back_mask_heading) {
        // back signal maybe out of sight, mask back signal
        LOG_INFO("mask back signal(yaw)");
        continue;
      }
      ReferencePoint near_signal_point;
      if (!refer_ptr->GetNearestRefPoint(
              std::max(signal_overlap.start_s - 1.0, 0.1),
              &near_signal_point)) {
        LOG_ERROR(
            "traffic light {} get nearest ref_pt failed skip, start_s[{:.3f}]",
            PlanningMap::Instance()->GetHashIdString(signal_overlap.object_id),
            signal_overlap.start_s - 1.0);
      } else {
        LOG_INFO("heading : signal = {:.2f}, adc = {:.2f}",
                 near_signal_point.heading() / M_PI * 180.0,
                 task_info.frame()->inside_planner_data().vel_heading / M_PI *
                     180.0);
        if (task_info.curr_sl().s() > signal_overlap.start_s) {
          double heading_error =
              near_signal_point.heading() -
              task_info.frame()->inside_planner_data().vel_heading;
          if (std::abs(heading_error) > M_PI)
            heading_error = 2.0 * M_PI - std::abs(heading_error);
          if (std::abs(heading_error) >
              traffic_law_config_.back_mask_heading / 180.0 * M_PI) {
            // back signal maybe out of sight, mask back signal
            LOG_INFO("mask back signal(heading)");
            continue;
          }
        }
      }
    }

    RouteSignal route_signal{.id = signal_overlap.object_id,
                             .start_route_s = signal_overlap.start_s,
                             .end_route_s = signal_overlap.end_s,
                             .turn_type = tmp_point.signal_type()};
    front_traffic_lights->insert(
        TrafficLightTable::value_type(signal_overlap.object_id, route_signal));
    LOG_INFO(
        "from hd_map get signal info: object_id, signal_type: {}, {}, start_s: "
        "{:.3f}",
        PlanningMap::Instance()->GetHashIdString(signal_overlap.object_id),
        (int)(tmp_point.signal_type()), route_signal.start_route_s);

    if (set_signal_id.count(signal_overlap.object_id)) {
      LOG_INFO(
          "insert value to map_id_and_2nd_stop_line:id[{}], "
          "value[{:.2f}]",
          PlanningMap::Instance()->GetHashIdString(signal_overlap.object_id),
          route_signal.start_route_s);
      double tmp = 0.0;
      if (map_id_and_2nd_stop_line->count(signal_overlap.object_id)) {
        traffic_law_context->MutableTrafficLightlawcontext()->get_map_value(
            signal_overlap.object_id, tmp);
      }
      traffic_law_context->MutableTrafficLightlawcontext()->set_map_value(
          signal_overlap.object_id, std::max(tmp, signal_overlap.start_s));
    } else {
      set_signal_id.insert(signal_overlap.object_id);
    }
  }
}

bool TrafficLightLaw::CombinePerceptionTrafficLight(
    const TaskInfo& task_info, TrafficLightTable& traffic_light_table) {
  const auto& traffic_light_proxy =
      DataCenter::Instance()->environment().traffic_light_proxy();
  for (auto& iter : traffic_light_table) {
    auto lights = traffic_light_proxy.GetLightsById(iter.first);
    if (lights.empty()) {
      LOG_ERROR(
          "signal id {} has not traffic-light-info received from perception, "
          "should active-protection",
          PlanningMap::Instance()->GetHashIdString(iter.first));
      iter.second.active_protect = true;
      iter.second.need_stop = true;
      continue;
    }
    // Note: only for 4 turn_type light with hdmap
    if (lights.size() != 4) {
      LOG_ERROR("light {} size[{}] != 4, should active-protection",
                PlanningMap::Instance()->GetHashIdString(iter.first),
                lights.size());
      iter.second.active_protect = true;
      iter.second.need_stop = true;
      continue;
    }

    auto curr_turn_type = iter.second.turn_type;
    TrafficLight matched_light;
    auto match_light_func = [](const auto& lights, const auto type) {
      TrafficLight light;
      for (const auto& tmp : lights) {
        if (tmp.type() == type) {
          light = tmp;
          break;
        }
      }
      return light;
    };
    switch (curr_turn_type) {
      case TurnSignalType::TST_STRAIGHT:
        matched_light = match_light_func(lights, TrafficLight::STRAIGHT);
        break;
      case TurnSignalType::TST_LEFT:
        matched_light = match_light_func(lights, TrafficLight::LEFT);
        break;
      case TurnSignalType::TST_RIGHT:
        matched_light = match_light_func(lights, TrafficLight::RIGHT);
        break;
      case TurnSignalType::TST_UTURN:
        matched_light = match_light_func(lights, TrafficLight::U_TURN);
        break;
      default:
        break;
    }
    iter.second.signal_color = matched_light.color();
    iter.second.blink = matched_light.blink();
    iter.second.light_id = matched_light.id();

    iter.second.enable_left_time =
        matched_light.remaining_time() < -900. ? false : true;
    iter.second.left_time = std::max(0.0, matched_light.remaining_time());
    iter.second.tracking_time = matched_light.tracking_time();
    LOG_INFO(
        "match with perception id:{}, start_route_s: {:.2f}, end_route_s: "
        "{:.2f}, turn_type:{}, color:{}, enable_left_time:{}, "
        "left_time:{:.3f}, tracking_time: {:.3f}",
        matched_light.id(), iter.second.start_route_s, iter.second.end_route_s,
        matched_light.type(), matched_light.color(),
        iter.second.enable_left_time, iter.second.left_time,
        iter.second.tracking_time);
  }

  return true;
}

// documentation:
// https://r3c0qt6yjw.feishu.cn/wiki/wikcnXrFtnavqL6gHAXpbhD5G7b?appStyle=UI4&domain=www.feishu.cn&locale=zh-CN&refresh=1&tabName=space&userId=6953822583687053340#
void TrafficLightLaw::TrafficLightDecider(
    const TaskInfo& task_info, const Boundary& adc_boundary,
    TrafficLightTable& traffic_light_table,
    TrafficLawContext* const traffic_law_context) {
  DataCenter::Instance()
      ->mutable_master_info()
      ->set_stop_due_to_junction_left_turn(false);
  double speed = DataCenter::Instance()->vehicle_state_proxy().LinearVelocity();
  double brake_distance =
      std::pow(speed, 2) / traffic_law_config_.max_deceleration / 2.0;
  LOG_INFO("adc_start_s = {:.4f}, adc_end_s = {:.4f}, brake_distance = {:.4f}",
           adc_boundary.start_s(), adc_boundary.end_s(), brake_distance);
  auto traffic_light_infos =
      traffic_law_context->MutableTrafficLightlawcontext()
          ->mutable_traffic_light_infos();
  DataCenter::Instance()->set_is_waiting_traffic_light(false);
  for (auto& iter : traffic_light_table) {
    auto color = iter.second.signal_color;
    bool adc_back_over_stop_line_too_much =
        (adc_boundary.start_s() >
         2.0 * traffic_law_config_.max_over_line_distance +
             iter.second.start_route_s);
    bool adc_back_not_over_stop_line =
        (adc_boundary.start_s() < iter.second.start_route_s);
    bool adc_front_not_over_stop_line =
        (adc_boundary.end_s() < iter.second.start_route_s);
    bool stop_over_line = (adc_boundary.end_s() + brake_distance >
                           iter.second.start_route_s +
                               traffic_law_config_.max_over_line_distance);
    bool near_line = adc_boundary.end_s() +
                         traffic_law_config_.green_light_pass_max_distance >
                     iter.second.start_route_s;

    uint64_t stop_line_id = iter.second.id;
    LOG_DEBUG("stop_line_id = {}, id = {}", stop_line_id,
             PlanningMap::Instance()->GetHashIdString(iter.second.id));
    auto light_info = traffic_light_infos->find(stop_line_id);
    if (light_info == traffic_light_infos->end()) {
      LOG_INFO("new light");
    }

    // set stop_line_s
    if (adc_boundary.end_s() + brake_distance > iter.second.start_route_s) {
      if (adc_boundary.end_s() + brake_distance >
          iter.second.start_route_s +
              traffic_law_config_.max_brake_over_line_distance) {
        // max stop distance
        iter.second.stop_line_s =
            std::max(iter.second.start_route_s +
                         traffic_law_config_.max_brake_over_line_distance,
                     adc_boundary.end_s());
      } else {
        iter.second.stop_line_s = adc_boundary.end_s() + brake_distance;
      }
    } else {
      iter.second.stop_line_s = iter.second.start_route_s;
    }
    if (speed < traffic_law_config_.stop_line_speed &&
        iter.second.start_route_s - adc_boundary.end_s() <
            traffic_law_config_.stop_line_distance) {
      iter.second.stop_line_s = adc_boundary.end_s() - 0.5;
      LOG_DEBUG(
          "adc is waiting for green light, let virtual obstacle near adc.");
    }

    if (color == TrafficLight_Color::TrafficLight_Color_BLACK) {
      if (light_info == traffic_light_infos->end()) {
        (*traffic_light_infos)[stop_line_id].last_color =
            TrafficLight_Color::TrafficLight_Color_BLACK;
      }
      if (iter.second.tracking_time <
          traffic_law_config_.color_confirm_time_threshold) {
        color = (*traffic_light_infos)[stop_line_id].last_color;
        LOG_INFO("black light. keep last color = {}", color);
      }
    }

    if (color == TrafficLight_Color::TrafficLight_Color_UNKNOWN) {
      auto non_motorized_greenblink = NonMotorizedGreenBlink(iter.second.id);
      auto non_motorized_color = NonMotorizedColor(iter.second.id);
      do {
        if (DataCenter::Instance()
                ->environment()
                .traffic_light_proxy()
                .GetUnknownPass()) {
          iter.second.need_stop = false;
          LOG_INFO("light id:{} color is unknown, get unknown-pass, pass",
                   PlanningMap::Instance()->GetHashIdString(iter.second.id));
          break;
        }
        if (non_motorized_color ==
                TrafficLight_Color::TrafficLight_Color_GREEN &&
            !non_motorized_greenblink) {
          iter.second.need_stop = false;
          LOG_INFO("light id:{} color is unknown, non_motorized is green, pass",
                   PlanningMap::Instance()->GetHashIdString(iter.second.id));
          break;
        }
        if (non_motorized_color ==
                TrafficLight_Color::TrafficLight_Color_GREEN &&
            non_motorized_greenblink) {
          LOG_INFO("non_motorized is green and blink");
        } else {
          if (light_info != traffic_light_infos->end() &&
              light_info->second.last_need_stop) {
            iter.second.need_stop = light_info->second.last_need_stop;
            LOG_INFO("light id:{} color is unknown, keep stop",
                     PlanningMap::Instance()->GetHashIdString(iter.second.id));
            break;
          }
        }
        if (!adc_back_not_over_stop_line) {
          iter.second.need_stop = false;
          LOG_INFO(
              "light id:{} color is unknown, adc back over stop line, pass",
              PlanningMap::Instance()->GetHashIdString(iter.second.id));
          break;
        }
        iter.second.need_stop =
            !(stop_over_line ||
              (non_motorized_color ==
                   TrafficLight_Color::TrafficLight_Color_GREEN &&
               near_line));
        LOG_INFO(
            "light id:{} color is unknown, stop_over_line = {}, non motorrized "
            "green = {}, near_line = {}, need_stop = {}",
            PlanningMap::Instance()->GetHashIdString(iter.second.id),
            stop_over_line,
            non_motorized_color == TrafficLight_Color::TrafficLight_Color_GREEN,
            near_line, iter.second.need_stop);
      } while (false);

      (*traffic_light_infos)[stop_line_id].yellow_blink_time = -1.0;
      iter.second.active_protect = iter.second.need_stop;
      (*traffic_light_infos)[stop_line_id].last_color = color;
      (*traffic_light_infos)[stop_line_id].last_need_stop =
          iter.second.need_stop;
      continue;
    }
    DataCenter::Instance()->mutable_event_report_proxy()->EventReset(
        EventType::UNKNOWN_TRAFFIC_LIGHT);
    if (color == TrafficLight_Color::TrafficLight_Color_GREEN ||
        color == TrafficLight_Color::TrafficLight_Color_BLACK) {
      auto non_motorized_greenblink = NonMotorizedGreenBlink(iter.second.id);
      auto non_motorized_color = NonMotorizedColor(iter.second.id);
      // auto passing_traffic_light = PassingTrafficLight(iter.second);
      (*traffic_light_infos)[stop_line_id].last_color = color;
      if (iter.second.enable_left_time) {
        bool remain_time_pass =
            RemainTimePassingDecision(adc_boundary, task_info.reference_line(),
                                      iter.second, distance_to_selected_end_s_);
        if (!is_motorway_) {
          if (remain_time_pass) {
            iter.second.need_stop = false;
            LOG_INFO(
                "light id:{} color is green/black, non-motorway, remain time "
                "pass",
                PlanningMap::Instance()->GetHashIdString(iter.second.id));
          } else {
            iter.second.need_stop = !stop_over_line;
            LOG_INFO(
                "light id:{} color is green/black, non-motorway, no "
                "remain_time, stop_over_line = {}, need_stop = {}",
                PlanningMap::Instance()->GetHashIdString(iter.second.id),
                stop_over_line, iter.second.need_stop);
          }
        } else {
          if (speed < traffic_law_config_.passing_speed_threshold) {
            speed = traffic_law_config_.passing_speed_threshold;
            LOG_INFO("set min passing speed = {:.1f}", speed);
          }
          if (adc_boundary.end_s() +
                  speed * std::max(iter.second.left_time, 1.0) <
              iter.second.start_route_s +
                  traffic_law_config_.max_over_line_distance) {
            iter.second.need_stop = true;
            LOG_INFO(
                "light id:{} color is green/black, v = {:.2f}, can't pass "
                "line in left time, stop",
                PlanningMap::Instance()->GetHashIdString(iter.second.id),
                speed);
          } else {
            iter.second.need_stop = false;
            LOG_INFO(
                "light id:{} color is green/black, v = {:.2f}, pass line "
                "in left time",
                PlanningMap::Instance()->GetHashIdString(iter.second.id),
                speed);
          }
        }
      } else {
        if (iter.second.blink) {
          iter.second.need_stop = !(stop_over_line || near_line);
          LOG_INFO(
              "light id:{} color is green/black and blink, stop_over_line = "
              "{}, near_line = {}, need_stop = {}",
              PlanningMap::Instance()->GetHashIdString(iter.second.id),
              stop_over_line, near_line, iter.second.need_stop);
        } else {
          if (!is_motorway_) {
            LOG_INFO("distance_to_selected_end_s = {:.2f}",
                     distance_to_selected_end_s_);
            if (distance_to_selected_end_s_ >
                    traffic_law_config_.junction_length_threshold &&
                non_motorized_greenblink) {
              iter.second.need_stop = !stop_over_line;
              LOG_INFO(
                  "light id:{} color is green/black, big intersection and "
                  "non_motorized_greenblink, stop_over_line = {}, need_stop = "
                  "{}",
                  PlanningMap::Instance()->GetHashIdString(iter.second.id),
                  stop_over_line, iter.second.need_stop);
            } else {
              iter.second.need_stop = false;
              LOG_INFO(
                  "light id:{} color is green/black, small intersection or "
                  "non_motorized is not greenblink, pass",
                  PlanningMap::Instance()->GetHashIdString(iter.second.id));
            }
          } else {
            iter.second.need_stop = false;
            LOG_INFO(
                "light id:{} color is green/black, motorway, no blink, pass",
                PlanningMap::Instance()->GetHashIdString(iter.second.id));
          }
        }
      }

      (*traffic_light_infos)[stop_line_id].yellow_blink_time = -1.0;
      (*traffic_light_infos)[stop_line_id].last_need_stop =
          iter.second.need_stop;
      continue;
    }
    if (color == TrafficLight_Color::TrafficLight_Color_YELLOW) {
      do {
        if (adc_back_over_stop_line_too_much) {
          iter.second.need_stop = false;
          LOG_INFO(
              "traffic light id: {} color is yellow, "
              "adc_back_over_stop_line_too_much, pass",
              PlanningMap::Instance()->GetHashIdString(iter.second.id));
          break;
        }
        // If there is a decision before, keep decision
        if (light_info != traffic_light_infos->end() &&
            (light_info->second.last_need_stop ||
             (light_info->second.yellow_red_stop &&
              DataCenter::Instance()->init_frame_time() -
                      light_info->second.yellow_red_stop_timestamp <
                  traffic_law_config_.color_confirm_time_threshold))) {
          iter.second.need_stop = true;
          LOG_INFO("traffic light id: {} color is yellow, keep stop",
                   PlanningMap::Instance()->GetHashIdString(iter.second.id));
          break;
        }
        iter.second.need_stop = !stop_over_line;
        LOG_INFO(
            "traffic light id: {} color is yellow, stop_over_line = {}, "
            "need_stop = {}",
            PlanningMap::Instance()->GetHashIdString(iter.second.id),
            stop_over_line, iter.second.need_stop);
      } while (false);

      if (light_info == traffic_light_infos->end() ||
          (*traffic_light_infos)[stop_line_id].last_color != color) {
        (*traffic_light_infos)[stop_line_id].last_color = color;
      }

      if (iter.second.blink) {
        if ((*traffic_light_infos)[stop_line_id].yellow_blink_time < 0.0)
          (*traffic_light_infos)[stop_line_id].yellow_blink_time =
              DataCenter::Instance()->init_frame_time();
      } else {
        (*traffic_light_infos)[stop_line_id].yellow_blink_time = -1.0;
      }

      if (iter.second.need_stop &&
          (*traffic_light_infos)[stop_line_id].yellow_blink_time > 0.0 &&
          DataCenter::Instance()->init_frame_time() -
                  (*traffic_light_infos)[stop_line_id].yellow_blink_time >
              traffic_law_config_.color_confirm_time_threshold) {
        // When the yellow light flashes, the tracking_time of the yellow light
        // will refresh
        LOG_INFO("yellow time more than {:.1f} second, pass",
                 traffic_law_config_.color_confirm_time_threshold);
        iter.second.need_stop = false;
        // Blink yellow light may be wrong. If the light changes to red
        // or no-blink(yellow), adc should stop at once.
        (*traffic_light_infos)[stop_line_id].yellow_red_stop =
            (*traffic_light_infos)[stop_line_id].last_need_stop = true;
      } else {
        (*traffic_light_infos)[stop_line_id].yellow_red_stop =
            (*traffic_light_infos)[stop_line_id].last_need_stop =
                iter.second.need_stop;
      }
      (*traffic_light_infos)[stop_line_id].yellow_red_stop_timestamp =
          DataCenter::Instance()->init_frame_time();
      if (iter.second.need_stop)
        DataCenter::Instance()->set_is_waiting_traffic_light(true);
      continue;
    }
    if (color == TrafficLight_Color::TrafficLight_Color_RED) {
      do {
        if (adc_back_over_stop_line_too_much) {
          iter.second.need_stop = false;
          LOG_INFO(
              "traffic light id: {} color is red, "
              "adc_back_over_stop_line_too_much, pass",
              PlanningMap::Instance()->GetHashIdString(iter.second.id));
          break;
        }
        if (light_info != traffic_light_infos->end() &&
            (light_info->second.last_need_stop ||
             (light_info->second.yellow_red_stop &&
              DataCenter::Instance()->init_frame_time() -
                      light_info->second.yellow_red_stop_timestamp <
                  traffic_law_config_.color_confirm_time_threshold))) {
          iter.second.need_stop = true;
          LOG_INFO("traffic light id: {} color is red, keep stop",
                   PlanningMap::Instance()->GetHashIdString(iter.second.id));
          break;
        }
        iter.second.need_stop = adc_front_not_over_stop_line;
        LOG_INFO(
            "traffic light id: {} color is red, adc_front_not_over_stop_line = "
            "{}, need_stop = {}",
            PlanningMap::Instance()->GetHashIdString(iter.second.id),
            adc_front_not_over_stop_line, iter.second.need_stop);
      } while (false);
      if (iter.second.need_stop)
        DataCenter::Instance()->set_is_waiting_traffic_light(true);
      (*traffic_light_infos)[stop_line_id].yellow_blink_time = -1.0;
      (*traffic_light_infos)[stop_line_id].last_color = color;
      (*traffic_light_infos)[stop_line_id].yellow_red_stop =
          (*traffic_light_infos)[stop_line_id].last_need_stop =
              iter.second.need_stop;
      (*traffic_light_infos)[stop_line_id].yellow_red_stop_timestamp =
          DataCenter::Instance()->init_frame_time();
      continue;
    }
  }
}
// UNKNOWN = 0, RED = 1, YELLOW = 2, GREEN = 3, BLACK = 4
// STRAIGHT = 0; LEFT = 1; RIGHT = 2; U_TURN = 3; PEDESTRAIN = 4; CYCLIST = 5;
}  // namespace planning
}  // namespace neodrive

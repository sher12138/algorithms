#include "planning_publisher.h"

#include "src/planning/reference_line/reference_line_util.h"
#include "src/planning/util/util.h"

namespace neodrive {
namespace planning {

using JunctionType = autobot::cyberverse::Junction::JunctionType;
namespace {

int GetJunctionTypeOfAdc(const ReferenceLinePtr ref_line, double adc_currnt_s) {
  if (!ref_line) {
    return static_cast<int>(JunctionType::UNKNOWN);
  }

  std::vector<std::pair<double, int>> junction_end_s_and_type{};

  for (const auto &[junction_ptr, overlap] : ref_line->junctions()) {
    if (!junction_ptr) {
      continue;
    }
    junction_end_s_and_type.emplace_back(
        overlap.end_s, static_cast<int>(junction_ptr->Type()));
  }

  std::sort(junction_end_s_and_type.begin(), junction_end_s_and_type.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  for (auto &[s, type] : junction_end_s_and_type) {
    if (s > adc_currnt_s) {
      return type;
    }
  }

  return static_cast<int>(JunctionType::UNKNOWN);
}

std::string GetStrByJunctionType(Junction::JunctionType type) {
  switch (type) {
    case Junction::JunctionType::IN_ROAD:
      /* code */
      return std::string("IN_ROAD");
    case Junction::JunctionType::T_CROSS_ROAD:
      return std::string("T_CROSS_ROAD");
    case Junction::JunctionType::Y_CROSS_ROAD:
      return std::string("Y_CROSS_ROAD");
    case Junction::JunctionType::CROSS_ROAD:
      return std::string("CROSS_ROAD");
    default:
      return std::string("UNKNOWN");
  }
}

}  // namespace

using neodrive::global::planning::ADCTrajectoryPoint;
using neodrive::global::planning::DecisionResult;
using neodrive::global::planning::VirtualObstacle;
PlanningPublisher::PlanningPublisher() {}

// Save timestamp information in record message, which is used for calculate
// time latency.
void FillTimeCollector(::neodrive::global::common::header::Header *header,
                       const DataCenter *data_center, double loop_start_time) {
#define SET_TIMECOLLECTOR(var, name, in_msg)          \
  {                                                   \
    auto timestamp = in_msg.header().timestamp_sec(); \
    var->add_##name(timestamp);                       \
  }

  auto &env = data_center->environment();
  // chassis   /planning/proxy/DuDriveChassis
  auto &chassis_msg = env.vehicle_state_proxy().chassis();
  SET_TIMECOLLECTOR(header->mutable_time_collector(), chassis, chassis_msg);
  // odometry  /world_model/pose/base_link_in_odometry
  auto &odom_pose_msg = env.vehicle_state_odometry_proxy().pose();
  SET_TIMECOLLECTOR(header->mutable_time_collector(), wm_pose_odom,
                    odom_pose_msg);
  // twist  /world_model/twist/base_link
  auto &twist_msg = env.vehicle_state_odometry_proxy().twist();
  SET_TIMECOLLECTOR(header->mutable_time_collector(), wm_twist, twist_msg);
  // prediction /pnc/prediction
  auto &prediction_msg = env.prediction_proxy().Prediction();
  SET_TIMECOLLECTOR(header->mutable_time_collector(), pnc_prediction,
                    prediction_msg);
  // perception_obstacle /perception/obstacles
  auto &perc_obstacles_msg = env.perception_proxy().Perception();
  SET_TIMECOLLECTOR(header->mutable_time_collector(), perc_obstacles,
                    perc_obstacles_msg);
  // /perception/lanes
  auto &perc_lanes_msg = env.perception_lanes_proxy().perception_lanes();
  SET_TIMECOLLECTOR(header->mutable_time_collector(), perc_lanes,
                    perc_lanes_msg);

  header->mutable_time_collector()->add_pnc_planning_runtime(
      cyber::Time::Now().ToSecond() - loop_start_time);
}

bool PlanningPublisher::Init(
    const std::shared_ptr<neodrive::cyber::Node> &node) {
  if (initialized_) {
    return true;
  }
  node_ = node;
  auto &topics_config = neodrive::common::config::CommonConfig::Instance()
                            ->topics_config()
                            .topics;
  traffic_light_pub_ = node_->CreateWriter<TrafficLightDetection>(
      topics_config.at("pnc_traffic_light").topic);
  planning_pub_ = node_->CreateWriter<ADCTrajectory>(
      topics_config.at("pnc_planning").topic);
  decision_pub_ = node_->CreateWriter<DecisionResult>(
      topics_config.at("pnc_decision").topic);
  monitor_log_pub_ = node_->CreateWriter<MonitorString>(
      topics_config.at("pnc_monitor_log").topic);
  record_event_pub_ = node_->CreateWriter<EventOfInterest>(
      topics_config.at("event_of_interest").topic);
  global_state_pub_ = node_->CreateWriter<GlobalState>(
      topics_config.at("pnc_global_state").topic);
  event_report_pub_ = node_->CreateWriter<EventReport>(
      topics_config.at("pnc_event_report").topic);
  aeb_switch_pub_ =
      node_->CreateWriter<ForbidAeb>(topics_config.at("pnc_forbid_aeb").topic);

  if (FLAGS_planning_test_speed_publish) {
    pub_speed_test_ =
        node_->CreateWriter<neodrive::planning::test::SpeedPlotMsg>(
            topics_config.at("pnc_planning_speed_test").topic);
    CHECK_NOTNULL(pub_speed_test_);
  }

  if (FLAGS_planning_enable_vis_event) {
    event_sender_ = vis::EventSender::Instance();
    event_sender_->Init(node, topics_config.at("pnc_vis_event").topic, "plan_");
  }

  CHECK_NOTNULL(traffic_light_pub_);
  CHECK_NOTNULL(planning_pub_);
  CHECK_NOTNULL(decision_pub_);
  CHECK_NOTNULL(monitor_log_pub_);
  CHECK_NOTNULL(record_event_pub_);
  CHECK_NOTNULL(global_state_pub_);
  CHECK_NOTNULL(event_report_pub_);
  CHECK_NOTNULL(aeb_switch_pub_);

  initialized_ = true;

  return true;
}

void PlanningPublisher::Reset() {
  planning_trajectory_published_ = false;
  loop_start_time_ = cyber::Time::Now().ToSecond();
}

void PlanningPublisher::AddTrafficLight(
    const uint64_t id, const double s,
    const std::vector<ReferencePoint> &ref_pts,
    TrafficLightDetection *const traffic_light) {
  auto sig_ptr = planning_map_->GetSignalById(id);
  if (!sig_ptr) return;
  /// set id and type
  auto light = traffic_light->add_traffic_light();
  light->set_id(planning_map_->GetHashIdString(id));
  if (!sig_ptr->GetSplitTime()) traffic_light->set_unknow_check(false);
  auto &rp = ref_pts[ref_line_util::BinarySearchIndex(ref_pts, s)];
  Lane::TurningType turn_type{};
  planning_map_->GetLaneTurnType(rp.hd_map_lane_id(), &turn_type);
  using Traffic = global::perception::traffic_light::TrafficLight;
  light->set_type(static_cast<Traffic::Type>(static_cast<int>(turn_type) - 1));

  auto set_pt = [&](auto t, auto p) {
    t->set_x(p.x), t->set_y(p.y), t->set_z(p.z);
  };
  /// set signal
  auto signal = light->mutable_signal();
  for (uint32_t i = 0; i < sig_ptr->Boundary().size(); ++i) {
    set_pt(signal->mutable_boundary()->add_point(), sig_ptr->Boundary()[i]);
  }
  using SubSig = neodrive::global::hdmap::Subsignal;
  for (uint32_t i = 0; i < sig_ptr->SignalLights().size(); ++i) {
    auto &sig = sig_ptr->SignalLights()[i];
    auto sub = signal->add_subsignal();
    sub->set_id(planning_map_->GetHashIdString(sig.id));
    sub->set_type(static_cast<SubSig::Type>(sig.type));
    set_pt(sub->mutable_location(), sig.location);
  }

  const auto &curr_scenario =
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
    signal->set_on_motorway(false);
  } else if (curr_scenario == ScenarioState::MOTORWAY_CRUISE ||
             curr_scenario == ScenarioState::MOTORWAY_LANE_CHANGE ||
             curr_scenario == ScenarioState::MOTORWAY_INTERSECTION ||
             curr_scenario == ScenarioState::MOTORWAY_DETOUR) {
    signal->set_on_motorway(true);
  } else {
    LOG_WARN("unknown scenario");
    signal->set_on_motorway(false);
  }
  signal->set_main_type(
      static_cast<Traffic::Signal::MainType>(sig_ptr->MainType()));
}

bool PlanningPublisher::PublishTrafficLightData(const TaskInfo &task_info) {
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  // pub traffic light
  auto traffic_light = traffic_light_msg_pool_.GetSharedPtr();
  traffic_light->mutable_header()->set_timestamp_sec(
      data_center_->init_frame_time());
  traffic_light->mutable_header()->set_module_name(
      plan_config.planning_node_name);
  traffic_light->mutable_header()->set_sequence_num(
      data_center_->current_frame()->sequence_num());

  double front_attention_dis =
      data_center_->traffic_light_detection_msg.is_updated
          ? (data_center_->environment()
                 .traffic_light_proxy()
                 .GetTrafficLight()
                 .distance() *
             1.0)
          : plan_config.traffic_light_for_perception.front_attention_dis;
  const double curr_s = task_info.curr_sl().s();
  const double s_begin =
      curr_s - plan_config.traffic_light_for_perception.back_attention_dis;
  const double s_end = curr_s + front_attention_dis;
  auto &ref_pts = task_info.reference_line()->ref_points();

  traffic_light->clear_traffic_light();
  traffic_light->set_unknow_check(true);
  double front_signal_s = -1.0;
  for (auto [id, s, e] : task_info.reference_line()->signal_overlaps()) {
    if (s <= curr_s) continue;
    if (s > s_end) break;
    front_signal_s = s;
    LOG_INFO("find front traffic_light s ={:.2f}", s);
    break;
  }
  std::tuple<uint64_t, double, double> back_sig{0, -1, -1};
  for (auto [id, s, e] : task_info.reference_line()->signal_overlaps()) {
    if (e < s_begin) continue;
    if (s > curr_s) break;
    if (std::get<0>(back_sig) == 0 ||
        std::abs(std::get<1>(back_sig) - s) < 0.01) {
      LOG_INFO("find back traffic_light hash id = {}, id = {}, {:.2f}, {:.2f}",
               id, planning_map_->GetHashIdString(id), s, e);
      if (front_signal_s > 0.0 &&
          2.0 * (curr_s + VehicleParam::Instance()->front_edge_to_center()) >
              (s + front_signal_s)) {
        // front signal is closer than back signal, mask back signal
        LOG_INFO("mask back signal");
        continue;
      }
      back_sig = {id, s, e};
      AddTrafficLight(id, e, ref_pts, traffic_light.get());
      continue;
    }
  }
  if (front_signal_s > 0.0) {
    if (std::get<1>(back_sig) > 0.0 &&
        2.0 * (curr_s + VehicleParam::Instance()->front_edge_to_center()) <
            (std::get<1>(back_sig) + front_signal_s)) {
      LOG_INFO("mask front signal");
    } else {
      std::tuple<uint64_t, double, double> front_sig{0, -1, -1};
      for (auto [id, s, e] : task_info.reference_line()->signal_overlaps()) {
        if (s <= curr_s) continue;
        if (s > s_end) break;
        if (std::get<0>(front_sig) == 0 ||
            std::abs(std::get<1>(front_sig) - s) < 0.01) {
          front_sig = {id, s, e};
          LOG_INFO(
              "find front traffic_light hash id = {}, id = {}, {:.2f}, {:.2f}",
              id, planning_map_->GetHashIdString(id), s, e);
          AddTrafficLight(id, e, ref_pts, traffic_light.get());
          continue;
        }
        break;
      }
    }
  }
  const double junc_s =
      curr_s + plan_config.traffic_light_for_perception.forward_junction_dis;
  for (auto [id, s, e] : task_info.reference_line()->junction_overlaps()) {
    if (e < curr_s) continue;
    if (s > junc_s) break;
    cyberverse::JunctionInfoConstPtr junc_ptr;
    if (!planning_map_->GetJunctionById(id, junc_ptr)) continue;

    using JuncType = neodrive::global::hdmap::Junction::Type;
    traffic_light->mutable_forward_junction()->set_id(
        planning_map_->GetHashIdString(id));
    traffic_light->mutable_forward_junction()->set_type(
        static_cast<JuncType>(junc_ptr->Type()));
    break;
  }
  return traffic_light_pub_->Write(traffic_light);
}

bool PlanningPublisher::PublishPlanningResult(
    const std::vector<TrajectoryPoint> &stitch_trajectory, Frame *const frame,
    const bool has_closer_obs) {
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  // pub decision
  auto master_info = data_center_->mutable_master_info();
  double dis_to_end = master_info->distance_to_end();
  auto &outside_planner_data = frame->outside_planner_data();
  // pub planning trajectory
  auto computed_trajectory =
      frame->mutable_planning_data()->mutable_computed_trajectory();
  computed_trajectory->insert_into_front_without_last_point(stitch_trajectory);
  computed_trajectory->set_header_time(data_center_->init_frame_time());
  LOG_DEBUG("trajectory header time: {:.3f}", data_center_->init_frame_time());

  auto adc_trajectory_ptr = plan_traj_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(adc_trajectory_ptr);
  // set module information
  adc_trajectory_ptr->mutable_header()->set_module_name(
      plan_config.planning_node_name);
  adc_trajectory_ptr->mutable_header()->set_sequence_num(frame->sequence_num());
  adc_trajectory_ptr->mutable_header()->set_timestamp_sec(
      data_center_->init_frame_time());

  if (frame->inside_planner_data().trajectory_mode ==
      CoordinateType::ODOMETRY) {
    coordinate_type_str_ = "ODOM";
  } else {
    coordinate_type_str_ = "UTM";
  }
  LOG_DEBUG("coordinate_type: {}", coordinate_type_str_);

  adc_trajectory_ptr->mutable_header()->set_frame_id(coordinate_type_str_);

  frame->planning_data().computed_trajectory().to_trajectory_protobuf(
      frame->planning_data().reference_line(),
      outside_planner_data.path_data->path().path_points(), stitch_trajectory,
      adc_trajectory_ptr);
  if (adc_trajectory_ptr->adc_trajectory_point_size() > 0) {
    data_center_->mutable_environment()
        ->mutable_vehicle_state_proxy()
        ->SetTrajPitch(adc_trajectory_ptr->adc_trajectory_point(0).pitch());
  }

  SetGearState(adc_trajectory_ptr);
  adc_trajectory_ptr->mutable_signals()->clear_signal();
  if (!master_info->clean_adc_signal()) {
    adc_trajectory_ptr->mutable_signals()->mutable_signal()->Add(
        master_info->adc_signal());
  }

  if (master_info->behavior_stop_vehicle() ||
      outside_planner_data.speed_slow_down) {
    adc_trajectory_ptr->mutable_estop()->set_is_estop(true);
  } else {
    adc_trajectory_ptr->mutable_estop()->set_is_estop(false);
  }

  adc_trajectory_ptr->set_total_path_length(
      computed_trajectory->spatial_length());
  adc_trajectory_ptr->set_total_path_time(computed_trajectory->time_length());
  bool speed_bump_found = false;
  auto &task_info = data_center_->task_info_list().front();
  for (const auto &speed_bump_overlaps :
       task_info.reference_line()->speed_bump_overlaps()) {
    LOG_DEBUG(
        "adc curr s: {:.4f}, speed_bump[{}] start s: {:.4f},  end s: {:.4f}",
        task_info.curr_sl().s(), speed_bump_overlaps.object_id,
        speed_bump_overlaps.start_s, speed_bump_overlaps.end_s);
    if (speed_bump_overlaps.object_id != 0 &&
        (task_info.curr_sl().s() > speed_bump_overlaps.start_s - 3.0 &&
         task_info.curr_sl().s() < speed_bump_overlaps.end_s + 1.0)) {
      speed_bump_found = true;
      LOG_INFO("speed_bump_found");
      break;
    }
    LOG_DEBUG("speed_bump not found");
  }
  adc_trajectory_ptr->set_is_replan(
      trajectory_stitcher_->IsUsePositionStitch() && !speed_bump_found);
  LOG_INFO("is_replan: {}", adc_trajectory_ptr->is_replan());
  if (outside_planner_data.trajectory_replan) {
    adc_trajectory_ptr->set_is_replan(true);
  }
  adc_trajectory_ptr->set_use_center_pose(data_center_->use_center_pose());

  // set some function use special low speed control
  // need to find a more resaonable place to set it
  LOG_INFO(
      "path_succeed: {} path_fail: {} speed_succeed: {} speed_fail: {} "
      "is_lane_borrow: {}",
      outside_planner_data.path_succeed_tasks,
      outside_planner_data.path_fail_tasks,
      outside_planner_data.speed_succeed_tasks,
      outside_planner_data.speed_fail_tasks,
      frame->inside_planner_data().is_lane_borrowing);
  adc_trajectory_ptr->set_low_speed_control(false);

  const auto &dp_st_map_ignore_static_obs_id =
      outside_planner_data.speed_obstacle_context
          .dp_st_map_ignore_static_obs_id;
  const auto &ignore_static_obs_id =
      outside_planner_data.motorway_speed_obstacle_context.ignore_static_obs_id;
  double cloest_obs_dis{10000.0};
  int cloest_obs_id{0};
  neodrive::global::planning::ObstacleType cloest_obs_type =
      neodrive::global::planning::ObstacleType::NONE;
  for (const auto &obs :
       outside_planner_data.speed_obstacle_context.static_obstacles_decision) {
    if (dp_st_map_ignore_static_obs_id.find(obs.obstacle.id()) !=
        dp_st_map_ignore_static_obs_id.end()) {
      continue;
    }
    if (!obs.lower_points.empty() &&
        cloest_obs_dis > obs.lower_points.front().first.s()) {
      cloest_obs_dis = obs.lower_points.front().first.s();
      cloest_obs_type = neodrive::global::planning::ObstacleType::PHYSICAL;
      cloest_obs_id = obs.obstacle.id();
    }
  }
  for (const auto &obs : outside_planner_data.motorway_speed_obstacle_context
                             .multi_cipv_static_obstacles_decision) {
    if (ignore_static_obs_id.find(obs.obstacle.id()) !=
        ignore_static_obs_id.end()) {
      continue;
    }
    if (!obs.lower_points.empty() &&
        cloest_obs_dis > obs.lower_points.front().first.s()) {
      cloest_obs_dis = obs.lower_points.front().first.s();
      cloest_obs_type = neodrive::global::planning::ObstacleType::PHYSICAL;
      cloest_obs_id = obs.obstacle.id();
    }
  }
  for (const auto &obs :
       outside_planner_data.speed_obstacle_context.virtual_obstacle_decision) {
    if (!obs.lower_points.empty() &&
        cloest_obs_dis > obs.lower_points.front().first.s()) {
      cloest_obs_dis = obs.lower_points.front().first.s();
      cloest_obs_type = neodrive::global::planning::ObstacleType::VIRTUAL;
      cloest_obs_id = 0;
    }
  }
  for (const auto &obs : outside_planner_data.motorway_speed_obstacle_context
                             .multi_cipv_virtual_obstacle_decision) {
    if (!obs.lower_points.empty() &&
        cloest_obs_dis > obs.lower_points.front().first.s()) {
      cloest_obs_dis = obs.lower_points.front().first.s();
      cloest_obs_type = neodrive::global::planning::ObstacleType::VIRTUAL;
      cloest_obs_id = 0;
    }
  }
  adc_trajectory_ptr->set_closest_obstacle_type(cloest_obs_type);
  adc_trajectory_ptr->set_closest_obstacle_accumulated_s(cloest_obs_dis);

  double physical_obs_hold_on_min_dis = 4.0;

  auto &if_change_obs_stop_dis =
      frame->mutable_outside_planner_data()->if_has_bump;
  if (if_change_obs_stop_dis) {
    physical_obs_hold_on_min_dis = 1.5;
  }
  LOG_INFO("check physical_obs_hold_on_min_dis:{:.3f} ",
           physical_obs_hold_on_min_dis);
  bool hold_on{false};
  LOG_DEBUG("XXXX distance_to_end {}", cloest_obs_dis);
  if (cloest_obs_type == neodrive::global::planning::ObstacleType::PHYSICAL &&
      cloest_obs_dis < physical_obs_hold_on_min_dis) {
    hold_on = true;
  }
  if (cloest_obs_type == neodrive::global::planning::ObstacleType::VIRTUAL &&
      cloest_obs_dis < 0.2) {
    hold_on = true;
  }
  if (outside_planner_data.hold_on) {
    hold_on = true;
  }
  adc_trajectory_ptr->set_hold_on(hold_on);

  if (data_center_->master_info().curr_scenario() == ScenarioState::PARKING) {
    adc_trajectory_ptr->set_is_parking(true);
    data_center_->set_is_parking(true);
  } else {
    adc_trajectory_ptr->set_is_parking(false);
    data_center_->set_is_parking(false);
  }

  // speed limit vector
  adc_trajectory_ptr->clear_speed_limit_vec();
  for (double t = 0.; t < 2.0; t += 0.2) {
    adc_trajectory_ptr->add_speed_limit_vec(
        DataCenter::Instance()->drive_strategy_max_speed());
  }

  if (adc_trajectory_ptr->adc_trajectory_point_size() == 0 &&
      adc_trajectory_ptr->gear() != neodrive::global::status::GEAR_PARKING) {
    LOG_ERROR("Trajectory size is empty.");
  }

  if (adc_trajectory_ptr->adc_trajectory_point_size() == 0 &&
      adc_trajectory_ptr->gear() == neodrive::global::status::GEAR_PARKING) {
    // fake current pos
    ADCTrajectoryPoint fake_pt;
    auto &vehicle_state_proxy = data_center_->vehicle_state_proxy();
    fake_pt.set_x(vehicle_state_proxy.X());
    fake_pt.set_y(vehicle_state_proxy.Y());
    fake_pt.set_theta(vehicle_state_proxy.Heading());
    fake_pt.set_curvature(0.0);
    fake_pt.set_accumulated_s(0.0);
    fake_pt.set_relative_time(0.0);
    fake_pt.set_speed(0.0);
    fake_pt.set_acceleration_s(0.0);
    for (size_t i = 0; i < 4; ++i) {
      auto add_pt = adc_trajectory_ptr->add_adc_trajectory_point();
      add_pt->CopyFrom(fake_pt);
    }
  }
  bool adc_target_speed_setted = false;
  for (int i = 0; i < adc_trajectory_ptr->adc_trajectory_point_size(); ++i) {
    auto adc_pt = adc_trajectory_ptr->mutable_adc_trajectory_point(i);
    if (!adc_target_speed_setted && adc_pt->accumulated_s() > 0) {
      master_info->set_adc_target_speed(adc_pt->speed());
      master_info->set_adc_target_accel(adc_pt->acceleration_s());
      adc_target_speed_setted = true;
    }
  }
  data_center_->SetMonitorString(BackSpeedMonitorInfo(outside_planner_data),
                                 MonitorItemSource::BACK_SPEED_PLANNING);

  data_center_->SetMonitorString(DecisionSpeedMonitorInfo(outside_planner_data),
                                 MonitorItemSource::ITER_DEDUCTION_DECISION);

  data_center_->SetMonitorString(
      TrajectorySpeedMonitorInfo(*adc_trajectory_ptr),
      MonitorItemSource::SPEED_PLANNING);
  FillTimeCollector(adc_trajectory_ptr->mutable_header(), data_center_,
                    loop_start_time_);
  bool pub_flag = planning_pub_->Write(adc_trajectory_ptr);
  planning_trajectory_published_ = true;
  if (adc_trajectory_ptr->adc_trajectory_point_size() > 1) {
    LOG_INFO("final send planning length: {:.4f}",
             adc_trajectory_ptr
                 ->adc_trajectory_point(
                     adc_trajectory_ptr->adc_trajectory_point_size() - 1)
                 .accumulated_s());
  }

  if (outside_planner_data.speed_context.is_emergency) {
    data_center_->SetMonitorString(
        "HPIPM cannot solve, send kinematic command.",
        MonitorItemSource::SPEED_PLANNING);
  }
  double adc_current_s =
      frame->outside_planner_data().frenet_veh_real_point.s();
  std::string map_first_junction_str = "[EgoForwardFirstJunction]:";
  map_first_junction_str +=
      GetStrByJunctionType(static_cast<Junction::JunctionType>(
          GetJunctionTypeOfAdc(task_info.reference_line(), adc_current_s)));
  data_center_->SetMonitorString(map_first_junction_str,
                                 MonitorItemSource::JUNCTION);

  // pub decision.
  auto decision_result = decision_result_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(decision_result);
  decision_result->mutable_header()->set_module_name(
      plan_config.planning_node_name);
  decision_result->mutable_header()->set_sequence_num(frame->sequence_num());
  decision_result->mutable_header()->set_timestamp_sec(
      data_center_->init_frame_time());
  decision_result->set_remain_distance(data_center_->GetNavigationSwapContext()
                                           .dist_to_routing_destination.Get());
  decision_result->set_intention(master_info->curr_intention());
  decision_result->set_closer_obs_voice_prompts(has_closer_obs);
  decision_result->set_speed_limit_res(
      data_center_->behavior_speed_limits().GetSpeedLimitResStr());

  auto process_vir_obs = [](const std::vector<Obstacle *> &obstacles,
                            auto decision_result) {
    if (!obstacles.empty()) {
      for (Obstacle *obs : obstacles) {
        // put virtual obstacle into cyber_monitor
        VirtualObstacle virtual_obs;
        double t_x{0.}, t_y{0.}, t_h{0.}, x1{0.}, y1{0.}, h1{0.};
        auto &utm_pos =
            DataCenter::Instance()->environment().vehicle_state_proxy();
        auto &odom_pos = DataCenter::Instance()
                             ->environment()
                             .vehicle_state_odometry_proxy();
        earth2vehicle(odom_pos.X(), odom_pos.Y(), odom_pos.Heading(),
                      obs->center().x(), obs->center().y(), obs->heading(), t_x,
                      t_y, t_h);
        vehicle2earth(utm_pos.X(), utm_pos.Y(), utm_pos.Heading(), t_x, t_y,
                      normalize_angle(t_h), x1, y1, h1);

        virtual_obs.set_virtual_type(obs->virtual_type());
        virtual_obs.mutable_perception_obstacle()->set_timestamp(
            obs->get_time_stamp());
        virtual_obs.mutable_perception_obstacle()->set_id(obs->id());
        virtual_obs.mutable_perception_obstacle()->set_length(obs->length());
        virtual_obs.mutable_perception_obstacle()->set_width(obs->width());
        virtual_obs.mutable_perception_obstacle()->set_height(obs->height());
        virtual_obs.mutable_perception_obstacle()->set_theta(h1);
        virtual_obs.mutable_perception_obstacle()->set_type(
            PerceptionObstacle::UNKNOWN);
        // Future may have virtual obstacles with speed
        virtual_obs.mutable_perception_obstacle()->mutable_velocity()->set_x(
            obs->speed() * std::tan(h1));
        virtual_obs.mutable_perception_obstacle()->mutable_velocity()->set_y(
            obs->speed() / std::tan(h1));
        virtual_obs.mutable_perception_obstacle()->mutable_position()->set_x(
            x1);
        virtual_obs.mutable_perception_obstacle()->mutable_position()->set_y(
            y1);
        LOG_INFO("adc odom_pos x:{:3f},y:{:3f}", odom_pos.X(), odom_pos.Y());
        LOG_INFO("adc utm_pos x:{:3f},y:{:3f}", utm_pos.X(), utm_pos.Y());
        LOG_INFO("vir obs height:", obs->height());
        for (auto &pt : obs->polygon().points()) {
          double dx{0.}, dy{0.}, dh{0.}, pt_x{0.}, pt_y{0.}, h1{0.};
          earth2vehicle(odom_pos.X(), odom_pos.Y(), odom_pos.Heading(), pt.x(),
                        pt.y(), obs->heading(), dx, dy, dh);
          vehicle2earth(utm_pos.X(), utm_pos.Y(), utm_pos.Heading(), dx, dy,
                        normalize_angle(dh), pt_x, pt_y, h1);
          LOG_INFO("virtual obs polygon point in utm: x:{:3f},y:{:3f}", pt_x,
                   pt_y);
          auto new_point = virtual_obs.mutable_perception_obstacle()
                               ->mutable_polygon_point()
                               ->Add();
          new_point->set_x(pt_x);
          new_point->set_y(pt_y);
        }
        auto add_virtual = decision_result->add_virtual_obstacle();
        add_virtual->CopyFrom(virtual_obs);
      }
    } else {
      LOG_INFO("no virtual obstacle");
    }
  };
  process_vir_obs(data_center_->current_frame()
                      ->planning_data()
                      .decision_data()
                      .virtual_obstacle(),
                  decision_result);
  LOG_INFO("process virtual obstacles end");
  process_vir_obs(data_center_->current_frame()
                      ->planning_data()
                      .decision_data()
                      .lateral_virtual_obstacle(),
                  decision_result);
  LOG_INFO("process lateral virtual obstacles end");

  if (frame->outside_planner_data()
          .speed_obstacle_context.cipv_decision.has_cipv) {
    decision_result->set_cipv_id(
        frame->outside_planner_data()
            .speed_obstacle_context.cipv_decision.cipv_id);
  }
  decision_pub_->Write(decision_result);
  if (FLAGS_planning_test_speed_publish &&
      !SendPlanningSpeedTestDataOut(frame)) {
    LOG_ERROR("Send");
  }

  return pub_flag;
}

std::string PlanningPublisher::BackSpeedMonitorInfo(
    const OutsidePlannerData &outside_planner_data) {
  const auto &speed_pts = outside_planner_data.motorway_speed_context;
  std::string str_buffer = std::string("[BACK_SPEED]");
  if (!outside_planner_data.motorway_speed_context.trigger_backcipv) {
    return str_buffer;
  }
  int max_size = 5;
  int add_pt_num = 0;
  for (size_t i = 1; i < speed_pts.final_goal_v.size(); i += 5) {
    str_buffer += "[" + DoubleFormat(speed_pts.final_goal_s.at(i).s(), 1) +
                  "," + DoubleFormat(speed_pts.final_goal_v.at(i).s(), 1) +
                  "," + DoubleFormat(speed_pts.final_goal_a.at(i).s(), 1) + "]";
    ++add_pt_num;
    if (add_pt_num > max_size) {
      break;
    }
  }
  return str_buffer;
}

std::string PlanningPublisher::DecisionSpeedMonitorInfo(
    const OutsidePlannerData &outside_planner_data) {
  // const auto &obs_decision = outside_planner_data.speed_obstacle_context
  //                                .iter_deduction_take_follow_decision_map;
  auto obs_decision =
      !outside_planner_data.speed_obstacle_context
              .iter_deduction_take_follow_decision_map.empty()
          ? outside_planner_data.speed_obstacle_context
                .iter_deduction_take_follow_decision_map
          : outside_planner_data.motorway_speed_obstacle_context
                .motorway_iter_deduction_take_follow_decision_map;

  std::string str_buffer = std::string("[OBS_TAKE_FOLLOW_DECISION]");
  if (obs_decision.empty()) {
    return str_buffer;
  }
  int max_size = 5;
  int add_pt_num = 0;
  for (const auto &[key, value] : obs_decision) {
    auto value_str = value == 0 ? "F" : "T";
    str_buffer += "[" + std::to_string(key) + ":" + value_str + "]";
    ++add_pt_num;
    if (add_pt_num > max_size) {
      break;
    }
  }
  return str_buffer;
}

bool PlanningPublisher::PublishResetTrajectory() {
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  auto response = reset_trajectory_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(response);
  response->mutable_header()->set_module_name(plan_config.planning_node_name);
  response->mutable_header()->set_sequence_num(0);
  response->mutable_header()->set_timestamp_sec(
      data_center_->init_frame_time());
  std::string frame_id = "ODOM";
  response->mutable_header()->set_frame_id(frame_id);

  ADCTrajectoryPoint fake_pt;
  auto &vehicle_state_proxy = data_center_->vehicle_state_proxy();

  fake_pt.set_x(vehicle_state_proxy.X());
  fake_pt.set_y(vehicle_state_proxy.Y());
  fake_pt.set_theta(vehicle_state_proxy.Heading());
  fake_pt.set_curvature(0.0);
  fake_pt.set_accumulated_s(0.0);
  fake_pt.set_relative_time(0.0);
  fake_pt.set_speed(0.0);
  fake_pt.set_acceleration_s(0.0);
  response->clear_adc_trajectory_point();
  response->clear_adc_path_point();
  response->mutable_adc_trajectory_point()->Reserve(10);
  for (size_t i = 0; i < 10; ++i) {
    auto add_pt = response->add_adc_trajectory_point();
    add_pt->CopyFrom(fake_pt);
  }
  data_center_->mutable_master_info()->set_adc_target_speed(0.);
  data_center_->mutable_master_info()->set_adc_target_accel(0.);
  response->set_gear(neodrive::global::status::GEAR_PARKING);
  FillTimeCollector(response->mutable_header(), data_center_, loop_start_time_);
  planning_pub_->Write(response);
  planning_trajectory_published_ = true;
  return true;
}

bool PlanningPublisher::PublishClearTrajectory() {
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  auto response = reset_trajectory_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(response);
  response->mutable_header()->set_module_name(plan_config.planning_node_name);
  response->mutable_header()->set_sequence_num(0);
  response->mutable_header()->set_timestamp_sec(
      data_center_->init_frame_time());
  std::string frame_id = "ODOM";
  response->mutable_header()->set_frame_id(frame_id);

  ADCTrajectoryPoint fake_pt;
  auto &vehicle_state_proxy = data_center_->vehicle_state_proxy();
  fake_pt.set_x(vehicle_state_proxy.X());
  fake_pt.set_y(vehicle_state_proxy.Y());
  fake_pt.set_theta(vehicle_state_proxy.Heading());
  fake_pt.set_curvature(0.0);
  fake_pt.set_accumulated_s(0.0);
  fake_pt.set_relative_time(0.0);
  fake_pt.set_speed(0.0);
  fake_pt.set_acceleration_s(0.0);
  response->clear_adc_trajectory_point();
  response->clear_adc_path_point();
  response->mutable_adc_trajectory_point()->Reserve(10);
  for (size_t i = 0; i < 10; ++i) {
    auto add_pt = response->add_adc_trajectory_point();
    add_pt->CopyFrom(fake_pt);
  }
  data_center_->mutable_master_info()->set_adc_target_speed(0.);
  data_center_->mutable_master_info()->set_adc_target_accel(0.);
  SetGearState(response);
  FillTimeCollector(response->mutable_header(), data_center_, loop_start_time_);
  planning_pub_->Write(response);
  planning_trajectory_published_ = true;
  return true;
}

void PlanningPublisher::PublishFakeTrajectory(const uint32_t sequence_num) {
  auto planning_config = config::PlanningConfig::Instance();
  auto fake_trajectory = fake_trajectory_msg_pool_.GetSharedPtr();
  fake_trajectory->mutable_header()->set_module_name(
      planning_config->plan_config().planning_node_name);
  fake_trajectory->mutable_header()->set_frame_id("ODOM");
  fake_trajectory->mutable_estop()->set_is_estop(true);
  fake_trajectory->set_gear(neodrive::global::status::GEAR_PARKING);
  fake_trajectory->mutable_header()->set_sequence_num(sequence_num);
  fake_trajectory->mutable_header()->set_timestamp_sec(
      data_center_->init_frame_time());
  data_center_->mutable_master_info()->set_adc_target_speed(0.);
  data_center_->mutable_master_info()->set_adc_target_accel(0.);
  FillTimeCollector(fake_trajectory->mutable_header(), data_center_,
                    loop_start_time_);
  planning_pub_->Write(fake_trajectory);
}

void PlanningPublisher::PublishMessages() {
  data_center_->AggregateMonitorString();
  LOG_INFO("monitor_message: {}",
           data_center_->monitor_message().DebugString());
  auto monitor_msg = monitor_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(monitor_msg);
  monitor_msg->clear_msg();
  monitor_msg->CopyFrom(data_center_->monitor_message());
  monitor_log_pub_->Write(monitor_msg);
  auto planning_config = config::PlanningConfig::Instance();
  if (planning_config->plan_config().common.add_reinforcement_learning_event) {
    AddReinforcementLearningEvent();
  }
  if (data_center_->is_renavigation_fail())
    data_center_->mutable_event_report_proxy()->SetEvent(
        EventType::MIXED_LANE_CHANGE_FAIL, "");

  for (auto &msg : data_center_->record_event_list()) {
    record_event_pub_->Write(msg);
  }
  if (FLAGS_planning_enable_vis_event) {
    event_sender_->Send();
  }
  auto global_state_msg = global_state_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(global_state_msg);
  auto global_state = data_center_->mutable_global_state_proxy();
  global_state->SetHaveTask(
      data_center_->have_task() &&
      data_center_->routing_result_msg.ptr->error_code().error_id() ==
          RoutingResult_ErrorCode::SUCCESS);
  global_state->SyncReachStation();
  global_state_msg->CopyFrom(global_state->global_state());
  global_state_pub_->Write(global_state_msg);
  auto event_report_msg = event_report_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(event_report_msg);
  event_report_msg->clear_event_infos();
  event_report_msg->CopyFrom(data_center_->event_report_proxy().event_report());
  event_report_pub_->Write(event_report_msg);
  if (data_center_->master_info().curr_scenario() == ScenarioState::PARKING &&
      nullptr != data_center_->parking_ptr()) {
    auto aeb_switch_msg = aeb_switch_msg_pool_.GetSharedPtr();
    CHECK_NOTNULL(aeb_switch_msg);
    aeb_switch_msg->Clear();
    aeb_switch_msg->set_value(true);
    aeb_switch_pub_->Write(aeb_switch_msg);
  } else {
    auto aeb_switch_msg = aeb_switch_msg_pool_.GetSharedPtr();
    CHECK_NOTNULL(aeb_switch_msg);
    aeb_switch_msg->Clear();
    aeb_switch_msg->set_value(false);
    aeb_switch_pub_->Write(aeb_switch_msg);
  }
}

bool PlanningPublisher::PublishEstopTrajectory() {
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  auto response = reset_trajectory_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(response);
  response->mutable_header()->set_module_name(plan_config.planning_node_name);
  response->mutable_header()->set_sequence_num(0);
  response->mutable_header()->set_timestamp_sec(
      data_center_->init_frame_time());
  std::string frame_id = "ODOM";
  response->mutable_header()->set_frame_id(frame_id);

  ADCTrajectoryPoint fake_pt;
  auto &vehicle_state_proxy = data_center_->vehicle_state_proxy();

  fake_pt.set_theta(vehicle_state_proxy.Heading());
  fake_pt.set_curvature(0.0);
  fake_pt.set_accumulated_s(0.0);
  fake_pt.set_relative_time(0.0);
  fake_pt.set_speed(0.0);
  fake_pt.set_acceleration_s(0.0);
  response->clear_adc_trajectory_point();
  response->clear_adc_path_point();
  response->mutable_adc_trajectory_point()->Reserve(10);
  for (size_t i = 0; i < 10; ++i) {
    auto add_pt = response->add_adc_trajectory_point();
    add_pt->CopyFrom(fake_pt);
  }
  data_center_->mutable_master_info()->set_adc_target_speed(0.);
  data_center_->mutable_master_info()->set_adc_target_accel(0.);
  response->mutable_estop()->set_is_estop(true);
  response->mutable_signals()->mutable_signal()->Add(
      neodrive::global::planning::ADCSignals::EMERGENCY_LIGHT);
  SetGearState(response);
  FillTimeCollector(response->mutable_header(), data_center_, loop_start_time_);
  planning_pub_->Write(response);
  planning_trajectory_published_ = true;
  return true;
}

std::string PlanningPublisher::TrajectorySpeedMonitorInfo(
    const ADCTrajectory &trajectory) const {
  std::string str_buffer = std::string("[SPEED]");
  int max_size = 10;
  double min_delta_s = 0.5;
  double accumulated_s = -1.;
  int add_pt_num = 0;
  for (int i = 1; i < trajectory.adc_trajectory_point_size(); ++i) {
    auto &pt = trajectory.adc_trajectory_point(i);
    if (pt.accumulated_s() - accumulated_s > min_delta_s) {
      str_buffer += "[" + DoubleFormat(pt.accumulated_s(), 1) + "," +
                    DoubleFormat(pt.speed() * 3.6, 1) + "," +
                    DoubleFormat(pt.acceleration_s(), 1) + "]";
      accumulated_s = pt.accumulated_s();
      ++add_pt_num;
      if (add_pt_num > max_size) {
        break;
      }
    }
  }
  return str_buffer;
}

bool PlanningPublisher::SendPlanningSpeedTestDataOut(Frame *const frame) {
  if (pub_speed_test_ == nullptr || frame == nullptr) {
    LOG_ERROR("frame is nullptr, err");
    return false;
  }

  const auto speed_test =
      frame->outside_planner_data().speed_context.dp_st_data;
  auto publish_data = speed_test_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(publish_data);
  publish_data->set_dp_st_count(speed_test.dp_st_profile.size());
  publish_data->set_smooth_st_count(speed_test.smoothed_speed.size());
  publish_data->set_obstacles_count(
      speed_test.st_graph_data.obs_boundary().size());
  publish_data->clear_dp_st_points();
  publish_data->mutable_dp_st_points()->Reserve(
      speed_test.dp_st_profile.size());
  for (const auto &point : speed_test.dp_st_profile) {
    auto add_point = publish_data->add_dp_st_points();
    add_point->set_s(point.s());
    add_point->set_t(point.t());
  }
  publish_data->clear_smooth_st_points();
  publish_data->mutable_smooth_st_points()->Reserve(
      speed_test.smoothed_speed.size());
  for (const auto &point : speed_test.smoothed_speed) {
    auto add_point = publish_data->add_smooth_st_points();
    add_point->mutable_point()->set_s(point.s());
    add_point->mutable_point()->set_t(point.t());
    add_point->set_a(point.a());
    add_point->set_v(point.v());
    add_point->set_j(point.j());
  }
  publish_data->clear_stboundaries();
  publish_data->mutable_stboundaries()->Reserve(
      speed_test.st_graph_data.obs_boundary().size());
  for (const auto &boundary : speed_test.st_graph_data.obs_boundary()) {
    auto add_obstacle = publish_data->add_stboundaries();
    int lable_value = static_cast<int>(boundary.boundary_type());
    if (lable_value < 1 || lable_value > 13) return false;
    add_obstacle->set_lable(static_cast<neodrive::planning::test::BoundaryType>(
        boundary.boundary_type()));
    add_obstacle->mutable_ploygon()->Reserve(boundary.points().size());
    for (const auto &point : boundary.points()) {
      auto add_point = add_obstacle->add_ploygon();
      add_point->set_s(point.y());
      add_point->set_t(point.x());
    }
  }
  publish_data->clear_s_lower();
  publish_data->mutable_s_lower()->Reserve(speed_test.lower_tunnel.size());
  for (const auto &point : speed_test.lower_tunnel) {
    auto add_point = publish_data->add_s_lower();
    add_point->set_s(point.st_point.s());
    add_point->set_t(point.st_point.t());
  }
  publish_data->clear_s_upper();
  publish_data->mutable_s_upper()->Reserve(speed_test.upper_tunnel.size());
  for (const auto &point : speed_test.upper_tunnel) {
    auto add_point = publish_data->add_s_upper();
    add_point->set_s(point.st_point.s());
    add_point->set_t(point.st_point.t());
  }
  bool flag = pub_speed_test_->Write(publish_data);

  return flag;
}

void PlanningPublisher::AddReinforcementLearningEvent() {
  static constexpr double kEventDeltaTime = 5.;
  static double prev_event_time = 0.;
  auto curr_time = cyber::Time::Now().ToSecond();
  bool add_event = false;
  if (std::fabs(curr_time - data_center_->vehicle_state_utm().Timestamp()) >
          1. ||
      std::fabs(curr_time -
                data_center_->environment().perception_proxy().Timestamp()) >
          1.) {
    // utm pose or perception obstacles is not valid, current record is not
    // usefull for reinforcement learning
    return;
  }
  if (!data_center_->vehicle_state_odometry().IsStopped()) {
    // ego car is moving, current record is usefull for reinforcement
    // learning
    add_event = true;
  } else {
    if (curr_time - prev_event_time < kEventDeltaTime) {
      add_event = true;
    }
  }
  if (add_event && curr_time - prev_event_time >= kEventDeltaTime) {
    data_center_->AddRecordEvent(EventOfInterest::REINFORCEMENT_LEARNING, -10,
                                 10);
    prev_event_time = curr_time;
  }
}

void PlanningPublisher::SetGearState(std::shared_ptr<ADCTrajectory> &output) {
  if (DataCenter::Instance()->global_state_proxy().is_finish()) {
    output->set_gear(neodrive::global::status::GEAR_PARKING);
    LOG_INFO("set gear is parking");
  } else if (DataCenter::Instance()->master_info().drive_direction() ==
             MasterInfo::DriveDirection::DRIVE_BACKWARD) {
    output->set_gear(neodrive::global::status::GEAR_REVERSE);
    LOG_INFO("set gear is reverse");
  } else {
    output->set_gear(neodrive::global::status::GEAR_DRIVE);
    LOG_INFO("set gear is drive");
  }
}

}  // namespace planning
}  // namespace neodrive

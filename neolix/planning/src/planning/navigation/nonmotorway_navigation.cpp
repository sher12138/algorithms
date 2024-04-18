#include "nonmotorway_navigation.h"

#include "common/navigation_context.h"
#include "common/navigation_types.h"
#include "common/util/time_util.h"
#include "config/navigation_config.h"
#include "scenario_manager_msgs.pb.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {
using cyberverse::LaneInfoConstPtr;
using cyberverse::RoadInfoConstPtr;
using global::hdmap::Lane;
using JunctionType = autobot::cyberverse::Junction::JunctionType;
typedef global::hdmap::Lane::LaneTurn TurnType;
namespace {
using AD3 = std::array<double, 3>;

TrajectoryPoint GetLaneChangeTrajectoryPoint(
    cyberverse::LaneInfoConstPtr lane,
    const VehicleStateProxy &vehicle_state_proxy) {
  TrajectoryPoint init_pt;

  double min_dis = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < lane->Points().size(); ++i) {
    auto &pt = lane->Points()[i];
    if (auto dis = std::hypot(pt.x() - vehicle_state_proxy.X(),
                              pt.y() - vehicle_state_proxy.Y());
        dis < min_dis) {
      min_dis = dis;
      init_pt.set_x(pt.x());
      init_pt.set_y(pt.y());
      init_pt.set_theta(vehicle_state_proxy.Heading());
      init_pt.set_velocity(vehicle_state_proxy.LinearVelocity());
    }
  }
  LOG_INFO("ego pose:({:.4f}, {:.4f}), pt:({:.4f}, {:.4f}), min dist:{}",
           vehicle_state_proxy.X(), vehicle_state_proxy.Y(), init_pt.x(),
           init_pt.y(), min_dis);

  return init_pt;
}

static void VisRefPoints(const std::vector<ReferencePoint> &points,
                         const std::string &name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kUtm);

  auto set_pts = [](auto event, auto &pts) {
    for (auto &pt : pts) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->mutable_center()->set_x(pt.x());
      sphere->mutable_center()->set_y(pt.y());
      sphere->mutable_center()->set_z(0);
      sphere->set_radius(0.1);
    }
  };

  set_pts(event, points);
}

void VisPolyline(const std::vector<AD3> &polyline, const std::string &name,
                 const std::array<double, 4> &col) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("start VisPolyline: {}", name);
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  event->mutable_color()->set_r(col[0]);
  event->mutable_color()->set_g(col[1]);
  event->mutable_color()->set_b(col[2]);
  event->mutable_color()->set_a(col[3]);
  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p[0]), ans->set_y(p[1]), ans->set_z(0);
  };

  auto pl = event->add_polyline();
  for (auto &p : polyline) {
    set_pt(pl->add_point(), p);
  }
}

void VisPaths(const std::vector<ReferencePoint> &paths,
              const std::string &name) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("start VisPolyline");

  std::vector<AD3> pts{};
  for (auto &path : paths) {
    // LOG_INFO("point size {}", path.xs.size());
    // for (size_t i = 0; i < path.xs.size(); ++i) {
    pts.push_back({path.x(), path.y(), 0});
    // }
  }

  VisPolyline(pts, name, {1, 0, 0, 1});
}

void SetOdometrySpeedLimit() {
  double config_max_speed = DataCenter::Instance()->drive_strategy_max_speed();
  auto data_center = DataCenter::Instance();
  auto &localization_config =
      config::PlanningConfig::Instance()->plan_config().localization;
  auto status_msg = data_center->localization_status_msg.ptr;

  if (!FLAGS_playing_record && data_center->is_odom_sensor_delay()) {
    double odom_status_speed_limit =
        std::min(config_max_speed,
                 data_center->vehicle_state_odometry().LinearVelocity());
    data_center->set_odometry_speed_limit(std::max(
        0., odom_status_speed_limit -
                localization_config.odom_sensor_bad_deceleration *
                    localization_config.odom_sensor_bad_system_delay_time));
  } else {
    data_center->set_odometry_speed_limit(config_max_speed);
  }
  LOG_INFO("odometry_speed_limit: {:.3f}", data_center->odometry_speed_limit());
}
}  // namespace

bool NonMotorwayNavigationProcess::Init() {
  if (is_inited_) {
    return true;
  }
  InitBase();
  is_inited_ = true;
  return true;
}

void NonMotorwayNavigationProcess::Process(
    const std::shared_ptr<RoutingRequest> &routing_request,
    std::shared_ptr<RoutingResult> &routing_respons) {
  if (!UpdateEgoPose() && routing_request->waypoint_size() == 0) return;
  LOG_INFO("receive routing request:{}", routing_request->DebugString());
  // TODO: @yangqingpeng
  routing_respons->mutable_routing_request()->CopyFrom(*routing_request);
  const auto &lane_topo_map = lane_topo_->GetFromToMap();
  auto &lane_seq = ctx_->lane_seq;
  auto &lane_set = ctx_->lane_set;
  lane_seq.clear();
  lane_set.clear();
  Vec3d route_start{routing_request->waypoint(0).pose().x(),
                    routing_request->waypoint(0).pose().y(),
                    routing_request->waypoint(0).pose().z()};
  Vec3d route_end{
      routing_request->waypoint(routing_request->waypoint().size() - 1)
          .pose()
          .x(),
      routing_request->waypoint(routing_request->waypoint().size() - 1)
          .pose()
          .y(),
      routing_request->waypoint(routing_request->waypoint().size() - 1)
          .pose()
          .z()};

  for (int i = 0; i + 1 < routing_request->waypoint().size(); ++i) {
    auto start = routing_request->waypoint(i);
    auto end = routing_request->waypoint(i + 1);
    // cloud navigation waypoint position may be incorrect
    start.set_type(GetPointMotorwayType(start.pose().x(), start.pose().y(),
                                        start.pose().z()));
    end.set_type(
        GetPointMotorwayType(end.pose().x(), end.pose().y(), end.pose().z()));
    if (ctx_->is_cloud_navigation) {
      // CorrectWaypoint(start);
      // CorrectWaypoint(end);
    }
    ctx_->start_type = start.type();
    ctx_->end_type = end.type();
    constexpr double kRadius = 0.3;
    std::vector<LaneInfoConstPtr> start_lanes;
    for (int j = 0; j < 20; ++j) {
      hdmap_->GetLanes({start.pose().x(), start.pose().y()},
                       kRadius + j * kRadius, &start_lanes, start.pose().z());
      if (start_lanes.size() > 0) {
        break;
      }
    }
    std::vector<LaneInfoConstPtr> end_lanes;
    for (int j = 0; j < 20; ++j) {
      hdmap_->GetLanes({end.pose().x(), end.pose().y()}, kRadius + j * kRadius,
                       &end_lanes, end.pose().z());
      if (end_lanes.size() > 0) {
        break;
      }
    }
    if (start_lanes.empty() || end_lanes.empty()) {
      LOG_ERROR("failed to locate start or end lane!");
      lane_seq.clear();
      lane_set.clear();
      break;
    }

    ctx_->routing_start_pt = {start.pose().x(), start.pose().y(),
                              start.pose().z()};
    ctx_->routing_end_pt = {end.pose().x(), end.pose().y(), end.pose().z()};

    std::vector<std::vector<uint64_t>> lane_seqs;
    for (const auto start_lane : start_lanes) {
      // if (!start_lane->IsOnLane({start.pose().x(), start.pose().y()})) {
      //   LOG_INFO("pose x, y: {}, {} is not on start_lane {}, skip",
      //            start.pose().x(), start.pose().y(),
      //            hdmap_->GetIdHashString(start_lane->Id()));
      //   continue;
      // }
      if (!lane_seq.empty()) {
        auto last_lane_succ = lane_topo_map.find(lane_seq.back()->Id())->second;
        if (start_lane->Id() != lane_seq.back()->Id() &&
            last_lane_succ.count(start_lane->Id()) == 0) {
          LOG_INFO("start_lane {} is not successor of last route lane {}, skip",
                   hdmap_->GetIdHashString(start_lane->Id()),
                   hdmap_->GetIdHashString(lane_seq.back()->Id()));
          continue;
        }
      }
      for (const auto end_lane : end_lanes) {
        // if (!end_lane->IsOnLane({end.pose().x(), end.pose().y()})) {
        //   LOG_INFO("pose x, y: {}, {} is not on end_lane {}, skip",
        //            end.pose().x(), end.pose().y(),
        //            hdmap_->GetIdHashString(end_lane->Id()));
        //   continue;
        // }
        ctx_->routing_start_lane_id = start_lane->Id();
        ctx_->routing_end_lane_id = end_lane->Id();
        LOG_INFO("start lane:{}, end lane:{}",
                 hdmap_->GetIdHashString(start_lane->Id()),
                 hdmap_->GetIdHashString(end_lane->Id()));
        std::vector<uint64_t> each_lane_seq{};
        GetRoutingFromBikingLane(routing_respons, each_lane_seq);
        if (each_lane_seq.size() == 1) {
          if (CheckPtOrderOnLane(each_lane_seq[0], ctx_->routing_start_pt,
                                 ctx_->routing_end_pt))
            lane_seqs.push_back(each_lane_seq);
          continue;
        }
        if (!each_lane_seq.empty()) lane_seqs.push_back(each_lane_seq);
      }
    }
    if (lane_seqs.empty()) {
      LOG_ERROR("failed to find lane seq!");
      lane_seq.clear();
      lane_set.clear();
      break;
    }

    auto res = FindShortestLaneSeq(lane_seqs, ctx_->routing_start_pt,
                                   ctx_->routing_end_pt);
    for (int j = 0; j < res.size(); ++j) {
      if (j == 0 && !lane_seq.empty() && res[j] == lane_seq.back()->Id())
        continue;
      auto lane = hdmap_->GetLaneById(res[j]);
      if (lane == nullptr) {
        LOG_ERROR("find nullptr lane");
        return;
      }
      LOG_INFO("find lane id:{}. string:{}", res[j],
               hdmap_->GetIdHashString(res[j]));
      lane_seq.push_back(lane);
      lane_set.insert(res[j]);
    }
    data_center_->set_routing_destination({end.pose().x(), end.pose().y()});
    Vec3d route_end{end.pose().x(), end.pose().y(), end.pose().z()};
    current_lane_idx_ = -1;
  }

  for (int j = 0; j < lane_seq.size(); ++j) {
    AddRoute(routing_respons, lane_seq[j], lane_seq[j]->RoadId(),
             (j == 0 ? &route_start : nullptr),
             (j == lane_seq.size() - 1 ? &route_end : nullptr));
  }
  UpdateAccumulatedS();
  CalculateMileage();
  GeneratePathPoints(routing_respons);
  LOG_DEBUG("result:{}", routing_respons->DebugString());
  routing_respons->mutable_header()->set_sequence_num(last_request_seq_++);
  routing_respons->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  if (ctx_->lane_seq.empty()) {
    routing_respons->mutable_error_code()->set_error_id(
        RoutingResult_ErrorCode::ERROR_RESPONSE_FAILED);
    routing_respons->mutable_error_code()->set_error_string("routing_failed");
    LOG_ERROR("failed to search lane route with navigator.");
  } else {
    bool is_on_route = CheckIsOnRoute();
    auto &current_parking_space = ctx_->current_parking_space;
    bool is_in_park = current_parking_space &&
                      (current_parking_space->Type() ==
                           global::hdmap::ParkingSpace_ParkingType_VERTICAL ||
                       current_parking_space->Type() ==
                           global::hdmap::ParkingSpace_ParkingType_HORIZONTAL);
    if (!is_on_route && !is_in_park && !ctx_->is_cloud_navigation) {
      auto timestamp_sec = routing_respons->header().timestamp_sec();
      auto sequence_num = routing_request->header().sequence_num();
      routing_respons->Clear();
      lane_seq.clear();
      lane_set.clear();
      routing_respons->mutable_header()->set_sequence_num(sequence_num);
      routing_respons->mutable_header()->set_timestamp_sec(timestamp_sec);
      routing_respons->mutable_error_code()->set_error_id(
          RoutingResult_ErrorCode::ERROR_RESPONSE_FAILED);
      routing_respons->mutable_error_code()->set_error_string(
          "routing success,but ego car not on route");
      LOG_ERROR("routing success,but ego car not on route");
    } else {
      data_center_->GetNavigationSwapContext().routing_destion.Set(route_end);
      routing_respons->mutable_error_code()->set_error_id(
          RoutingResult_ErrorCode::SUCCESS);
      routing_respons->mutable_error_code()->set_error_string("success");
    }
  }
}

void NonMotorwayNavigationProcess::RunOnce() {
  auto start_time = cyber::Time::Now().ToSecond();
  if (!data_center_->vehicle_state_utm().IsValid()) return;
  auto context = ctx_;
  if (context->lane_seq.empty()) return;
  UpdateLaneSequenceIndex(context->lane_seq);
  if (current_lane_idx_ < 0) return;
  // TODO:@yangqingpeng
  CheckTurnHornLights();
  // a_star_lane_search_.VisLaneNode("lane_node_dist");
  ctx_->is_on_route = CheckIsOnRoute();
  LOG_INFO("is_on_route:{}", ctx_->is_on_route);
  auto end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("NonMotorwayNavigationProcess::RunOnce use time: {:.4f}",
           end_time - start_time);
}

std::vector<uint64_t> NonMotorwayNavigationProcess::GetNextLanes(
    uint64_t lane_id, uint64_t road_id) {
  std::vector<uint64_t> res;
  std::unordered_set<uint64_t> lane_set;
  const auto &lane_topo_map = lane_topo_->GetFromToMap();
  auto lane_from_iter = lane_topo_map.find(lane_id);
  if (lane_from_iter == lane_topo_map.end()) return res;
  for (auto iter = lane_from_iter->second.begin();
       iter != lane_from_iter->second.end(); ++iter) {
    auto next_lane = hdmap_->GetLaneById(*iter);
    LOG_INFO("lane id:{}, lr id:{}, road:{}", hdmap_->GetIdHashString(*iter),
             hdmap_->GetIdHashString(next_lane->RoadId()),
             hdmap_->GetIdHashString(road_id));
    if (next_lane->RoadId() == road_id) lane_set.insert(next_lane->Id());
  }
  if (lane_set.empty()) return res;
  std::unordered_set<uint64_t>::iterator it = lane_set.begin();
  auto lane = hdmap_->GetLaneById(*it);
  auto road = hdmap_->GetRoadById(lane->RoadId());
  for (int i = 0; i < road->Sections()[0]->LaneIds().size(); ++i) {
    lane = hdmap_->GetLaneById(road->Sections()[0]->LaneIds()[i]);
    bool is_bus_bay_lane = false;
    if (lane_set.find(lane->Id()) != lane_set.end()) {
      for (int i = 0; i < lane->LaneMultipleType().size(); ++i) {
        if (lane->LaneMultipleType()[i] ==
            static_cast<uint32_t>(global::hdmap::Lane::BUS_BAY_LANE)) {
          is_bus_bay_lane = true;
          LOG_INFO("find bus bay id:{}, string:{}", lane->Id(),
                   hdmap_->GetIdHashString(lane->Id()));
          break;
        }
      }
      if (!is_bus_bay_lane || lane_set.size() == 1)
        res.push_back(road->Sections()[0]->LaneIds()[i]);
    }
  }

  std::reverse(res.begin(), res.end());
  return res;
}

std::vector<uint64_t> NonMotorwayNavigationProcess::GetTurnNextLanes(
    uint64_t lane_id, std::vector<size_t> road_seq, const TurnType &turn, int i,
    bool main_loop = false) {
  std::vector<uint64_t> res;
  std::unordered_set<uint64_t> turn_set;
  const auto &lane_topo_map = lane_topo_->GetFromToMap();
  auto lane_from_iter = lane_topo_map.find(lane_id);
  if (lane_from_iter != lane_topo_map.end()) {
    for (auto iter = lane_from_iter->second.begin();
         iter != lane_from_iter->second.end(); ++iter) {
      auto next_lane = hdmap_->GetLaneById(*iter);
      if (next_lane->TurnType() == turn) {
        turn_set.insert(next_lane->Id());
      }
    }
  }
  // LOG_INFO("last junc size:{}, idx:{}", lane_cnt.first, lane_cnt.second);
  // if (turn_set.empty() && (!main_loop || lane_cnt.first != lane_cnt.second))
  //   return res;

  if (turn_set.empty()) {
    if (!main_loop) return res;
    double search_road_len = road_topo_->GetRoadCost(road_seq[i - 1]);
    double max_lane_change_len = config::NavigationConfig::Instance()
                                     ->navigation_config()
                                     .max_lane_change_len;
    for (int j = i - 2; j >= 0; --j) {
      LOG_INFO("j:{}, lane cnt size:{}, idx:{}, len:{}", j, lane_cnt_[j].first,
               lane_cnt_[j].second, search_road_len);
      if (search_road_len > max_lane_change_len) break;
      search_road_len += road_topo_->GetRoadCost(road_seq[j]);
      if (lane_cnt_[j].first != lane_cnt_[j].second) return res;
    }
  }

  if (!turn_set.empty()) {
    if (turn == Lane::NO_TURN) {
      std::unordered_set<uint64_t>::iterator it = turn_set.begin();
      auto lane = hdmap_->GetLaneById(*it);
      auto road = hdmap_->GetRoadById(lane->RoadId());
      for (int i = 0; i < road->Sections()[0]->LaneIds().size(); ++i) {
        lane = hdmap_->GetLaneById(road->Sections()[0]->LaneIds()[i]);
        if (turn_set.find(lane->Id()) != turn_set.end()) {
          LOG_INFO("matched turn lane, string:{}, id:{}",
                   hdmap_->GetIdHashString(lane->Id()), lane->Id());
          res.push_back(lane->Id());
        }
      }
      std::reverse(res.begin(), res.end());
    } else {
      std::unordered_set<uint64_t>::iterator iter = turn_set.begin();
      for (iter; iter != turn_set.end(); ++iter) res.push_back(*iter);
    }
    return res;
  }
  LOG_INFO("search in other lane");
  uint64_t road_id = road_seq[i];
  auto from_road_id = hdmap_->GetLaneById(lane_id)->RoadId();
  auto from_road = hdmap_->GetRoadById(from_road_id);
  auto &lanes = from_road->Sections()[0]->LaneIds();
  for (int j = lanes.size() - 1; j >= 0; --j) {
    res = GetTurnNextLanes(lanes[j], road_seq, turn, i);
    if (!res.empty()) return res;
  }
  return res;
}

bool NonMotorwayNavigationProcess::FindLanePath(
    int i, uint64_t last_lane_id, uint64_t goal_lane_id,
    std::vector<size_t> road_seq, std::vector<std::pair<int, uint64_t>> &res) {
  if (i == road_seq.size()) {
    if (lane_topo_->IsConnectLane(last_lane_id, goal_lane_id))
      return true;
    else
      return false;
  }

  TurnType turn;
  if (!road_topo_->GetLinkType(road_seq[i - 1], road_seq[i], turn)) {
    LOG_INFO("lane i:{}, last lane:{}, string:{}", i, last_lane_id,
             hdmap_->GetIdHashString(last_lane_id));
    auto lane_ret = GetNextLanes(last_lane_id, road_seq[i]);
    if (lane_ret.empty()) return false;
    LOG_INFO("ret size:{}", lane_ret.size());
    lane_cnt_[i].first = lane_ret.size();
    lane_cnt_[i].second = 0;
    for (auto lane_id : lane_ret) {
      ++lane_cnt_[i].second;
      res.push_back(std::make_pair(i, lane_id));
      if (FindLanePath(i + 1, lane_id, goal_lane_id, road_seq, res))
        return true;
      res.pop_back();
    }
    return false;
  } else {
    LOG_INFO("turn i:{}, last lane:{}, string:{}", i, last_lane_id,
             hdmap_->GetIdHashString(last_lane_id));
    auto turn_ret = GetTurnNextLanes(last_lane_id, road_seq, turn, i, true);
    if (turn_ret.empty()) return false;
    for (auto turn_id : turn_ret) {
      res.push_back(std::make_pair(i, turn_id));
      auto lane_ret = GetNextLanes(turn_id, road_seq[i]);
      if (lane_ret.empty()) continue;
      lane_cnt_[i].first = lane_ret.size();
      lane_cnt_[i].second = 0;
      for (auto lane_id : lane_ret) {
        ++lane_cnt_[i].second;
        res.push_back(std::make_pair(i, lane_id));
        if (FindLanePath(i + 1, lane_id, goal_lane_id, road_seq, res))
          return true;
        res.pop_back();
      }
      res.pop_back();
    }
    return false;
  }
}

bool NonMotorwayNavigationProcess::GetRoutingFromRoadSeq(
    std::shared_ptr<RoutingResult> &routing_respons) {
  LOG_ERROR("not support road search in nonmotorway map");
  return false;
}

bool NonMotorwayNavigationProcess::GetRoutingFromBikingLane(
    std::shared_ptr<RoutingResult> &routing_respons,
    std::vector<uint64_t> &res) {
  auto &lane_seq = ctx_->lane_seq;
  auto &lane_set = ctx_->lane_set;
  auto &routing_start_lane_id = ctx_->routing_start_lane_id;
  auto &routing_end_lane_id = ctx_->routing_end_lane_id;

  if (!a_star_lane_search_.BikingSearch(routing_end_lane_id,
                                        routing_start_lane_id)) {
    return false;
  }
  res = a_star_lane_search_.GetRoutingLanes(routing_start_lane_id);
  return true;
}

bool NonMotorwayNavigationProcess::ComputeReferenceLine(
    std::shared_ptr<RefeLineGenerator> &current_ref_generator,
    std::shared_ptr<RefeLineGenerator> &target_ref_generator) {
  auto &navigation_swap_context = data_center_->GetNavigationSwapContext();
  auto context = ctx_;
  // Generate target reference line
  LOG_INFO("clear tar ref generator");
  context->target_utm_ref = nullptr;

  // Generate current reference line
  // for (auto lane : ctx_->lane_seq)
  //   LOG_INFO("lane_seq id:{}, string:{}", lane->Id(),
  //            hdmap_->GetIdHashString(lane->Id()));
  ctx_->is_lane_change_ref = false;
  if (!current_ref_generator->Generate(
          ctx_->lane_seq, NavigationProcessBase::GetTrajectoryPoint(
                              data_center_->vehicle_state_utm()))) {
    LOG_ERROR("Generate cur reference line failed!");
    current_ref_generator.reset(new RefeLineGenerator(ctx_));
    return false;
  }

  context->current_utm_ref = current_ref_generator->GetUtmRef();
  navigation_swap_context.dest_point.Set(
      current_ref_generator->GetDestinationPoint());
  navigation_swap_context.dist_to_end.Set(
      current_ref_generator->GetDistanceToDestination());
  context->dist_to_lane_change_end = 0.0;
  navigation_swap_context.dist_to_routing_destination.Set(
      routing_destination_s_ - context->current_s);
  navigation_swap_context.driving_direction.Set(
      MasterInfo::DriveDirection::DRIVE_FORWARD);

  LOG_INFO("is_swap_ref_line:{}", context->is_swap_ref_line);

  SetOdometrySpeedLimit();

  return true;
}

}  // namespace planning
}  // namespace neodrive
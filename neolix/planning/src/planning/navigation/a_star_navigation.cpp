#include "a_star_navigation.h"

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

bool AStarNavigationProcess::Init() {
  if (is_inited_) {
    return true;
  }
  InitBase();
  is_inited_ = true;
  return true;
}

void AStarNavigationProcess::Process(
    const std::shared_ptr<RoutingRequest> &routing_request,
    std::shared_ptr<RoutingResult> &routing_respons) {
  if (!UpdateEgoPose() || routing_request->waypoint_size() == 0) return;
  LOG_ERROR(routing_request->DebugString());
  auto &road_seq = ctx_->road_seq;
  auto &last_road_seq = ctx_->last_road_seq;
  auto &lane_seq = ctx_->lane_seq;
  auto &lane_set = ctx_->lane_set;
  last_road_seq.clear();
  lane_seq.clear();
  lane_set.clear();
  if (!ctx_->renavigation_process) lane_keypoint_map_.clear();

  for (int i = 0; i + 1 < routing_request->waypoint().size(); ++i) {
    SearchRoutingRoadSequence(routing_request, i, i + 1);
    last_road_seq = road_seq;
    LOG_INFO("generate routing_request");
    if (!GenerateLaneSequenceFromRoadSequence(routing_request, i, i + 1,
                                              routing_respons)) {
      LOG_ERROR("GenerateLaneSequenceFromRoadSequence fails");
      lane_seq.clear();
      lane_set.clear();
      break;
    }
  }
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
  for (int j = 0; j < lane_seq.size(); ++j) {
    LOG_DEBUG("total lane seq id: {}",
              hdmap_->GetIdHashString(lane_seq[j]->Id()));
    AddRoute(routing_respons, lane_seq[j], lane_seq[j]->RoadId(),
             (j == 0 ? &route_start : nullptr),
             (j == lane_seq.size() - 1 ? &route_end : nullptr));
  }

  UpdateAccumulatedS();
  GeneratePathPoints(routing_respons);
  routing_respons->mutable_header()->set_sequence_num(last_request_seq_++);
  routing_respons->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  if (ctx_->lane_seq.empty()) {
    routing_respons->mutable_error_code()->set_error_id(
        RoutingResult_ErrorCode::ERROR_RESPONSE_FAILED);
    routing_respons->mutable_error_code()->set_error_string("routing_failed");
    LOG_ERROR("failed to search lane route with navigator.");
  } else {
    data_center_->GetNavigationSwapContext().routing_destion.Set(route_end);
    routing_respons->mutable_error_code()->set_error_id(
        RoutingResult_ErrorCode::SUCCESS);
    routing_respons->mutable_error_code()->set_error_string("success");
  }

  LOG_DEBUG("routing result:{}", routing_respons->DebugString());
  CalculateMileage();
}

void AStarNavigationProcess::RunOnce() {
  auto start_time = cyber::Time::Now().ToSecond();
  if (!data_center_->vehicle_state_utm().IsValid()) return;
  auto context = ctx_;
  // if (context->lane_seq.empty()) return;
  UpdateLaneSequenceIndex(context->lane_seq);
  if (current_lane_idx_ < 0) {
    LOG_ERROR("current_lane_idx_ < 0, skip runonce");
    return;
  }
  // if (!Renavigation()) return;
  Renavigation();

  CheckLaneChange();
  LOG_INFO("current s:{}", context->current_s);
  UpdateLaneChangeLaneSeq();
  PostProcessLaneChangeInfo();
  CheckTurnHornLights();
  // a_star_lane_search_.VisLaneNode("lane_node_dist");

  auto end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("AStarNavigationProcess::RunOnce use time: {:.4f}",
           end_time - start_time);
}

std::vector<uint64_t> AStarNavigationProcess::GetNextLanes(uint64_t lane_id,
                                                           uint64_t road_id) {
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

std::vector<uint64_t> AStarNavigationProcess::GetTurnNextLanes(
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

bool AStarNavigationProcess::FindLanePath(
    int i, uint64_t last_lane_id, uint64_t goal_lane_id,
    std::vector<size_t> road_seq, std::vector<std::pair<int, uint64_t>> &res) {
  if (i == road_seq.size()) {
    if (lane_topo_->IsConnectLane(last_lane_id, goal_lane_id) ||
        last_lane_id == goal_lane_id)
      return true;
    else
      return false;
  }

  // forbid lane change on solid lane divider
  auto cur_lane = hdmap_->GetLaneById(last_lane_id);
  if (ctx_->lane_change_direct != Lane::NO_TURN) {
    auto lane_boundary_type = ctx_->lane_change_direct == Lane::LEFT_TURN
                                  ? cur_lane->right_divider()[0].type
                                  : cur_lane->left_divider()[0].type;
    LOG_INFO("lane_boundary_type:{}", (int)lane_boundary_type);
    if (lane_boundary_type != global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
        lane_boundary_type != global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
        lane_boundary_type != global::hdmap::LaneBoundaryType::UNKNOWN) {
      LOG_INFO("SOLID_WHITE lane boundary, ban lane change!");
      return false;
    }
    if (!cur_lane->Signals().empty()) {
      LOG_INFO("cur lane has signals, ban lane change");
      return false;
    }
  }

  TurnType turn;
  if (!road_topo_->GetLinkType(road_seq[i - 1], road_seq[i], turn)) {
    LOG_INFO("lane i:{}, last lane:{}, string:{}", i, last_lane_id,
             hdmap_->GetIdHashString(last_lane_id));
    auto lane_ret = GetNextLanes(last_lane_id, road_seq[i]);
    if (lane_ret.empty()) return false;
    LOG_INFO("ret size:{}", lane_ret.size());

    // forbid lane change before cross road
    auto lane = hdmap_->GetLaneById(lane_ret[0]);
    auto road = hdmap_->GetRoadById(lane->RoadId());
    auto junction = hdmap_->GetJunctionById(road->JunctionId());
    if (junction != nullptr &&
        junction->Type() == static_cast<uint32_t>(JunctionType::CROSS_ROAD)) {
      return false;
    }

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

    // forbid lane change before cross road
    auto lane = hdmap_->GetLaneById(turn_ret[0]);
    auto road = hdmap_->GetRoadById(lane->RoadId());
    auto junction = hdmap_->GetJunctionById(road->JunctionId());
    if (junction != nullptr &&
        junction->Type() == static_cast<uint32_t>(JunctionType::CROSS_ROAD)) {
      return false;
    }

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

std::vector<uint64_t> AStarNavigationProcess::SearchLaneChangeLaneSeq(
    uint64_t start, uint64_t end, const std::vector<uint64_t> &road_seq) {
  ctx_->routing_start_pt = {data_center_->vehicle_state_utm().X(),
                            data_center_->vehicle_state_utm().Y(),
                            data_center_->vehicle_state_utm().Z()};

  // search for lane seq in road seq
  std::vector<uint64_t> res{};
  if (road_seq.size() == 1 && start != end) {
    // start and end point are in one road but different lanes
    res.push_back(start);
    res.push_back(end);
  } else {
    // start and end point are in different roads
    if (!a_star_lane_search_.Init(end, road_seq)) {
      return res;
    }

    if (!a_star_lane_search_.Search(end, start)) {
      return res;
    }
    res = a_star_lane_search_.GetRoutingLanes(start);
  }
  return res;
}

bool AStarNavigationProcess::GetRoutingFromRoadSeq(
    std::shared_ptr<RoutingResult> &routing_respons) {
  const auto &road_topo_map = road_topo_->GetFromToMap();
  const auto context = ctx_;
  auto &lane_seq = context->lane_seq;
  auto &lane_set = context->lane_set;
  auto &road_seq = context->road_seq;
  auto &routing_start_pt = context->routing_start_pt;
  auto &routing_end_pt = context->routing_end_pt;
  auto &routing_start_lane_id = context->routing_start_lane_id;
  auto &routing_end_lane_id = context->routing_end_lane_id;
  // search for lane seq in road seq
  std::vector<uint64_t> res{};
  if (road_seq.size() == 1 && routing_start_lane_id != routing_end_lane_id) {
    // check bound type and adjacency
    auto start_idx = GetRoadLaneIdx(routing_start_lane_id);
    auto end_idx = GetRoadLaneIdx(routing_end_lane_id);
    if (start_idx == -1 || end_idx == -1 || std::abs(start_idx - end_idx) > 1) {
      LOG_ERROR("{} and {} are not adjacent, fail to search for lanes",
                hdmap_->GetIdHashString(routing_start_lane_id),
                hdmap_->GetIdHashString(routing_end_lane_id));
      return false;
    }

    auto start_lane = hdmap_->GetLaneById(routing_start_lane_id);
    auto end_lane = hdmap_->GetLaneById(routing_end_lane_id);
    auto GetLeftDivider = [](const cyberverse::LaneInfoConstPtr &lane) {
      if (lane->left_divider().empty())
        return static_cast<uint32_t>(global::hdmap::LaneBoundaryType::UNKNOWN);
      else
        return lane->left_divider()[0].type;
    };
    if (end_idx < start_idx) {
      // left lane change
      auto lane_boundary_type = GetLeftDivider(start_lane);
      LOG_INFO("lane_boundary_type:{}", (int)lane_boundary_type);
      if (lane_boundary_type != global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
          lane_boundary_type !=
              global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
          lane_boundary_type != global::hdmap::LaneBoundaryType::UNKNOWN) {
        LOG_INFO("SOLID_WHITE lane boundary, ban lane change!");
        return false;
      }
    } else {
      // right lane change
      auto lane_boundary_type = GetLeftDivider(end_lane);
      LOG_INFO("lane_boundary_type:{}", (int)lane_boundary_type);
      if (lane_boundary_type != global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
          lane_boundary_type !=
              global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
          lane_boundary_type != global::hdmap::LaneBoundaryType::UNKNOWN) {
        LOG_INFO("SOLID_WHITE lane boundary, ban lane change!");
        return false;
      }
    }

    // start and end point are in one road but different lanes
    res.push_back(routing_start_lane_id);
    res.push_back(routing_end_lane_id);

    // calculate lane change s for NEM visualization
    auto lane = hdmap_->GetLaneById(routing_start_lane_id);
    double start_s = 0.0, end_s = 0.0, l = 0.0;
    lane->GetProjection({routing_start_pt.x(), routing_start_pt.y()}, &start_s,
                        &l);
    lane->GetProjection({routing_end_pt.x(), routing_end_pt.y()}, &end_s, &l);
    context->one_road_lane_change_s_map[routing_start_lane_id] =
        std::make_pair(true, (start_s + end_s) / 2.0);
    context->one_road_lane_change_s_map[routing_end_lane_id] =
        std::make_pair(false, (start_s + end_s) / 2.0);
  } else {
    // start and end point are in different roads
    if (!a_star_lane_search_.Init(routing_end_lane_id, road_seq)) {
      return false;
    }
    if (!a_star_lane_search_.Search(routing_end_lane_id,
                                    routing_start_lane_id)) {
      return false;
    }
    res = a_star_lane_search_.GetRoutingLanes(routing_start_lane_id);
    // if (res.empty() && !ctx_->is_cloud_navigation) {
    //   auto end_road = hdmap_->GetRoadById(
    //       ctx_->routing_end_lane->RoadId());
    //   auto end_lane_ids = end_road->Sections()[0]->LaneIds();
    //   for (int i = end_lane_ids.size() - 1; i >= 0; --i) {
    //     a_star_lane_search_.Init(end_lane_ids[i], road_seq);
    //     a_star_lane_search_.Search(end_lane_ids[i], routing_start_lane_id);
    //     res = a_star_lane_search_.GetRoutingLanes(routing_start_lane_id);
    //     if (!res.empty()) break;
    //   }
    // }
  }
  if (res.empty()) return false;

  for (int i = 0; i < res.size(); ++i) {
    if (i == 0 && ((!lane_seq.empty() && res[i] == lane_seq.back()->Id()) ||
                   (lane_seq.size() > 2 &&
                    res[i] == lane_seq[lane_seq.size() - 3]->Id())))
      continue;
    auto lane = hdmap_->GetLaneById(res[i]);
    if (lane == nullptr) {
      LOG_ERROR("find nullptr lane");
      return false;
    }
    LOG_INFO("find lane id:{}. string:{}", res[i],
             hdmap_->GetIdHashString(res[i]));
    lane_seq.push_back(lane);
    lane_set.insert(res[i]);
  }
  current_lane_idx_ = -1;
  return true;
}

bool CheckIsMixedLaneChange(cyberverse::LaneInfoConstPtr from,
                            cyberverse::LaneInfoConstPtr to,
                            cyberverse::HDMap *hdmap) {
  uint32_t from_type = from->LaneType();
  uint32_t to_type = to->LaneType();
  const uint32_t motorway_type =
      static_cast<uint32_t>(global::hdmap::Lane::CITY_DRIVING);
  const uint32_t nonmotorway_type =
      static_cast<uint32_t>(global::hdmap::Lane::BIKING);
  return from_type == motorway_type && to_type == nonmotorway_type ||
         from_type == nonmotorway_type && to_type == motorway_type;
}

bool AStarNavigationProcess::Renavigation() {
  auto context = ctx_;
  const auto &config =
      config::NavigationConfig::Instance()->navigation_config();
  static int cnt = 0;
  if (cnt < 3 && data_center_->is_renavigation_fail()) {
    ++cnt;
  } else {
    data_center_->set_is_renavigation_fail(false);
    cnt = 0;
  }

  if (!config.enable_renavigation) {
    LOG_INFO("skip Renavigation, enable_renavigation = false");
    return true;
  }
  if (context->routing_request == nullptr) {
    LOG_INFO(
        "skip Renavigation, task has been cleared, routing_request is null");
    return true;
  }
  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  double z = data_center_->vehicle_state_utm().Z();
  double vel = data_center_->vehicle_state_utm().LinearVelocity();
  double heading = data_center_->vehicle_state_utm().Heading();
  // renavigation when take over && have left route && have lane seq && is not
  // in road junction && at right heading lane
  const auto &chassis = data_center_->vehicle_state_proxy().chassis();
  bool is_current_on_route = CheckIsOnRouteWithBaseLink();
  bool manual_left_route =
      !is_current_on_route && !data_center_->is_auto_driving();

  std::vector<cyberverse::LaneInfoConstPtr> lanes{};
  hdmap_->GetLanes({x, y}, kMaxGetLaneRadius, &lanes);
  if (lanes.empty()) {
    LOG_INFO("skip Renavigation, empty lanes");
    return true;
  }
  auto road = hdmap_->GetRoadById(lanes[0]->RoadId());
  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  bool is_in_junction_turn =
      junction != nullptr &&
      lanes[0]->TurnType() !=
          static_cast<uint32_t>(global::hdmap::Lane::NO_TURN);
  bool is_in_signal = !lanes[0]->Signals().empty();
  bool is_cross_road = is_in_junction_turn || is_in_signal;

  bool is_valid_heading_lane = CheckLaneHeadingValidity(lanes[0]);

  static std::vector<cyberverse::LaneInfoConstPtr> origin_lane_seq{};
  if (!context->lane_seq.empty()) origin_lane_seq = context->lane_seq;

  context->manual_trigger_renavigation =
      !data_center_->is_auto_driving() && !origin_lane_seq.empty() &&
      manual_left_route && !is_cross_road && is_valid_heading_lane;
  LOG_INFO(
      "manual_trigger_renavigation:{}, is take over:{}, is empty origin "
      "lane seq:{}, have left route:{}, is cross road:{}, "
      "is_valid_heading_lane:{}",
      context->manual_trigger_renavigation, !data_center_->is_auto_driving(),
      origin_lane_seq.empty(), manual_left_route, is_cross_road,
      is_valid_heading_lane);
  if (!data_center_->is_auto_driving() && !origin_lane_seq.empty() &&
      is_cross_road && !is_current_on_route) {
    LOG_ERROR("take over and out of route, reset lane seq and ref line");
    context->lane_seq.clear();
    context->lane_change_seq.clear();
    context->target_utm_ref = nullptr;
    context->current_utm_ref = nullptr;
    context->is_on_route = false;
  }

  // renavigation when lane change failed
  double dist = std::hypot(x - context->lane_change_end_point.x(),
                           y - context->lane_change_end_point.y());
  LOG_INFO("ego x,y,s:{}, {}, lane_change_end_point x,y: {}, {}, dist:{}", x, y,
           context->lane_change_end_point.x(),
           context->lane_change_end_point.y(), dist);
  LOG_INFO("ego s:{}, lane_change_end_point s: {}, dist:{}", context->current_s,
           context->lane_change_end_point_s,
           context->current_s - context->lane_change_end_point_s);
  bool trigger_lane_change_fail_navigation =
      config.enable_lane_change_renavigation &&
      context->lane_change_end_point_s > 0.0 &&
      context->current_s - context->lane_change_end_point_s >
          config.lane_change_renavi_dist &&
      vel < config.lane_change_renavi_vel && !is_cross_road &&
      data_center_->master_info().curr_scenario() !=
          ScenarioState::MOTORWAY_LANE_CHANGE;

  if (!context->manual_trigger_renavigation &&
      !trigger_lane_change_fail_navigation) {
    LOG_INFO(
        "skip re-navigation, dist:{}, curr s:{}, lane_change_point_s:{}, "
        "vel:{}, trigger_lane_change_fail_navigation:{}, "
        "trigger_lane_change_fail_navigation:{}, origin_lane_seq size:{}",
        dist, context->current_s, context->lane_change_point_s, vel,
        context->manual_trigger_renavigation,
        trigger_lane_change_fail_navigation, origin_lane_seq.size());
    return true;
  }
  LOG_INFO(
      "Renavigation, curr s:{}, lane_change_point_s:{}, "
      "vel:{}",
      context->current_s, context->lane_change_point_s, vel);

  // get current lane idx
  const auto &request = ctx_->routing_request;
  int cur_idx = GetNearestLaneWithHeading(
      origin_lane_seq, data_center_->vehicle_state_utm().X(),
      data_center_->vehicle_state_utm().Y());
  if (cur_idx < 0) {
    LOG_ERROR("cur idx < 0, fail to find nearest lane idx");
    return false;
  }
  double cur_lane_dist = origin_lane_seq[cur_idx]->DistanceTo(
      {data_center_->vehicle_state_utm().X(),
       data_center_->vehicle_state_utm().Y()});
  if (cur_lane_dist >= 10.0) {
    LOG_ERROR("cur lane id:{}, cur_lane_dist:{} is too far, skip re-navigation",
              hdmap_->GetIdHashString(origin_lane_seq[cur_idx]->Id()),
              cur_lane_dist);
    return true;
  }

  // find next keypoint
  int next_keypoint_idx = -1;
  for (int i = cur_idx + 1; i < origin_lane_seq.size(); ++i) {
    if (lane_keypoint_map_.find(origin_lane_seq[i]->Id()) !=
        lane_keypoint_map_.end()) {
      next_keypoint_idx = lane_keypoint_map_[origin_lane_seq[i]->Id()];
      break;
    }
  }
  if (next_keypoint_idx < 0 || next_keypoint_idx >= request->waypoint_size()) {
    LOG_ERROR("find no keypoint behind current pose, invalid idx:{}",
              next_keypoint_idx);
    return false;
  }
  LOG_INFO("next keypoint idx:{}, waypoint size:{}", next_keypoint_idx,
           request->waypoint_size());
  LOG_INFO("Renavigation request:{}", request->DebugString());

  // re-navigation from current pose
  while (next_keypoint_idx < request->waypoint_size()) {
    auto new_request = std::make_shared<RoutingRequest>();
    auto new_result = std::make_shared<RoutingResult>();
    GenerateNewRoutingRequest({x, y, z}, next_keypoint_idx, new_request);
    LOG_INFO("next keypoint idx:{}, waypoint size:{}", next_keypoint_idx,
             new_request->waypoint_size());
    context->Reset();
    double start_to_next_key_dist =
        CalcDistBetweenPoints(next_keypoint_idx, cur_idx, origin_lane_seq);
    context->renavigation_process = true;
    Process(new_request, new_result);
    context->renavigation_process = false;

    if (!context->lane_seq.empty()) {
      CalcLaneSequenceIndex(context->lane_seq);
      double total_len = CalcDistBetweenPoints(
          next_keypoint_idx, current_lane_idx_, context->lane_seq);
      LOG_INFO("curr_lane_idx:{}, total_len - start_to_next_key_dist: {}",
               current_lane_idx_, total_len - start_to_next_key_dist);
      if (std::abs(total_len - start_to_next_key_dist) < 200.0) {
        for (auto lane : context->lane_seq)
          LOG_INFO("find new lane id: {}", hdmap_->GetIdHashString(lane->Id()));

        context->current_lane_idx = current_lane_idx_;
        context->need_reset_ref_generator = true;
        // context->routing_request = new_request;
        context->is_on_route = true;
        return true;
      }
    }
    ++next_keypoint_idx;
  }

  LOG_ERROR("Renavigation failed! find no route");
  context->lane_seq = origin_lane_seq;
  CalcLaneSequenceIndex(context->lane_seq);
  data_center_->set_is_renavigation_fail(true);
  if (context->manual_trigger_renavigation) {
    LOG_ERROR("Renavigation failed, reset lane seq and ref line");
    context->lane_seq.clear();
    context->lane_change_seq.clear();
    context->target_utm_ref = nullptr;
    context->current_utm_ref = nullptr;
    context->is_on_route = false;
  }
  return false;
}

bool AStarNavigationProcess::FindNextWayPointIdx(
    const std::vector<cyberverse::LaneInfoConstPtr> &origin_lane_seq,
    int &cur_idx, int &next_keypoint_idx) {
  // get current lane idx
  cur_idx = GetNearestLaneWithHeading(origin_lane_seq,
                                      data_center_->vehicle_state_utm().X(),
                                      data_center_->vehicle_state_utm().Y());
  if (cur_idx < 0) {
    LOG_ERROR("cur idx < 0, fail to find nearest lane idx");
    return false;
  }
  double cur_lane_dist = origin_lane_seq[cur_idx]->DistanceTo(
      {data_center_->vehicle_state_utm().X(),
       data_center_->vehicle_state_utm().Y()});
  if (cur_lane_dist >= config::NavigationConfig::Instance()
                           ->navigation_config()
                           .lane_locate_range) {
    LOG_ERROR("cur lane id:{}, cur_lane_dist:{} is too far, skip re-navigation",
              hdmap_->GetIdHashString(origin_lane_seq[cur_idx]->Id()),
              cur_lane_dist);
    return false;
  }

  // find next keypoint
  for (int i = cur_idx + 1; i < origin_lane_seq.size(); ++i) {
    if (lane_keypoint_map_.find(origin_lane_seq[i]->Id()) !=
        lane_keypoint_map_.end()) {
      next_keypoint_idx = lane_keypoint_map_[origin_lane_seq[i]->Id()];
      break;
    }
  }
  const auto &request = ctx_->routing_request;
  return next_keypoint_idx >= 0 && next_keypoint_idx < request->waypoint_size();
}

uint64_t AStarNavigationProcess::FindLaneIdByWaypointIdx(int idx) {
  for (const auto &pair : lane_keypoint_map_) {
    if (pair.second == idx) return pair.first;
  }
  return 0;
}

void AStarNavigationProcess::ExtractRoadSeq(int start_lane_idx,
                                            uint64_t target_lane_id,
                                            std::vector<uint64_t> &road_seq) {
  auto target_lane = hdmap_->GetLaneById(target_lane_id);
  if (start_lane_idx < 0 || start_lane_idx >= ctx_->lane_seq.size() ||
      target_lane == nullptr) {
    LOG_ERROR("invalid start_lane_idx:{}, target_lane_id:{}", start_lane_idx,
              hdmap_->GetIdHashString(target_lane_id));
    road_seq.clear();
    return;
  }
  for (auto i = start_lane_idx; i < ctx_->lane_seq.size(); ++i) {
    auto id = ctx_->lane_seq[i]->RoadId();
    road_seq.push_back(id);
    LOG_INFO("road id:{}, string:{}", id, hdmap_->GetIdHashString(id));
    if (id == target_lane->RoadId()) {
      return;
    }
  }
  LOG_ERROR("not find target! start_lane_idx:{}, target_lane_id:{}",
            start_lane_idx, hdmap_->GetIdHashString(target_lane_id));
  road_seq.clear();
  return;
}

void AStarNavigationProcess::GenerateNewRoutingRequest(
    const AD3 ego_pose, int idx, std::shared_ptr<RoutingRequest> &new_request) {
  LOG_INFO("enter KeypointRequestProcess");
  auto &request = ctx_->routing_request;
  new_request->mutable_header()->CopyFrom(request->header());
  if (request->has_parking_id()) {
    new_request->set_parking_id(request->parking_id());
  }
  // add ego pose as first waypoint
  auto start = new_request->add_waypoint();
  start->mutable_pose()->set_x(ego_pose[0]);
  start->mutable_pose()->set_y(ego_pose[1]);
  start->mutable_pose()->set_z(ego_pose[2]);
  CheckMapPointZLegality(*start);

  // add following keypoints
  for (int i = idx; i < request->waypoint_size(); ++i) {
    auto pt = new_request->add_waypoint();
    pt->CopyFrom(request->waypoint()[i]);
    CheckMapPointZLegality(*pt);
  }
}

void AStarNavigationProcess::CheckLaneChange() {
  auto context = ctx_;
  if (last_manual_lane_change_order_ != config::NavigationConfig::Instance()
                                            ->navigation_config()
                                            .manual_lane_change_order) {
    context->is_manual_lane_change_updated = false;
    last_manual_lane_change_order_ = config::NavigationConfig::Instance()
                                         ->navigation_config()
                                         .manual_lane_change_order;
  }
  // trigger lane change manually
  if (config::NavigationConfig::Instance()
              ->navigation_config()
              .manual_lane_change_order == 1 ||
      config::NavigationConfig::Instance()
              ->navigation_config()
              .manual_lane_change_order == 2) {
    CreateManualLaneChangeInfo();
    return;
  }

  // trigger lane change by planning
  if (CreatePlanningLaneChangeInfo()) return;

  // trigger lane change by navigation
  if (context->is_lane_change_point_updated &&
      context->current_s < context->min_lane_change_s)
    return;
  if (!context->is_lane_change_point_updated) CreateLaneChangeInfo();
}

void AStarNavigationProcess::CreateLaneChangeInfo() {
  LOG_INFO("enter lane change info");
  auto context = ctx_;
  auto &lane_seq = context->lane_seq;
  context->lane_change_point_s = 0.0;
  context->is_mixed_change = false;
  int lane_idx_before_change_point = -1;
  double max_lane_change_len = config::NavigationConfig::Instance()
                                   ->navigation_config()
                                   .max_lane_change_len;
  context->ban_lane_set.clear();
  for (int i = current_lane_idx_; i + 1 < lane_seq.size(); ++i) {
    context->lane_change_point_s =
        accumulated_s_[i] + lane_seq[i]->TotalLength();
    if (!lane_topo_->IsConnectLane(lane_seq[i]->Id(), lane_seq[i + 1]->Id())) {
      lane_idx_before_change_point = i;
      if (lane_seq[i]->RoadId() == lane_seq[i + 1]->RoadId()) {
        LOG_INFO("need lane change in one road");
        max_lane_change_len = lane_seq[i]->TotalLength();
      }
      LOG_INFO("id:{}, string:{}, len:{}", lane_seq[i]->Id(),
               hdmap_->GetIdHashString(lane_seq[i]->Id()),
               context->lane_change_point_s);
      break;
    }
    context->ban_lane_set.insert(lane_seq[i]->Id());
  }
  auto road = hdmap_->GetRoadById(lane_seq[current_lane_idx_]->RoadId());
  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  bool is_in_junction_turn =
      junction != nullptr && lane_seq[current_lane_idx_]->TurnType() !=
                                 static_cast<uint32_t>(Lane::NO_TURN);
  bool is_in_signal = !lane_seq[current_lane_idx_]->Signals().empty();
  if (lane_idx_before_change_point < 0 || is_in_junction_turn || is_in_signal) {
    LOG_INFO("no need to change lane, cur lane id:{}",
             hdmap_->GetIdHashString(lane_seq[current_lane_idx_]->Id()));
    context->is_lane_change_point_updated = false;
    context->need_lanechange = false;
    return;
  }
  context->is_mixed_change = CheckIsMixedLaneChange(
      lane_seq[lane_idx_before_change_point],
      lane_seq[lane_idx_before_change_point + 1], hdmap_);

  lane_seq_after_lane_change_.assign(
      lane_seq.begin() + lane_idx_before_change_point + 1, lane_seq.end());
  // if (lane_seq[lane_idx_before_change_point + 1]->TurnType() ==
  // Lane::LEFT_TURN)
  //   context->lane_change_direct = static_cast<TurnType>(
  //       lane_seq[lane_idx_before_change_point + 1]->TurnType());
  auto &laneids =
      hdmap_->GetRoadById(lane_seq[lane_idx_before_change_point]->RoadId())
          ->Sections()[0]
          ->LaneIds();
  int change_count = 100;
  std::vector<int> connect_idxs;
  int next_idx = lane_seq[lane_idx_before_change_point]->LaneNo();
  for (uint32_t i = 0; i < laneids.size(); ++i) {
    if (lane_topo_->IsConnectLane(
            laneids[i], lane_seq[lane_idx_before_change_point + 1]->Id()) ||
        laneids[i] == lane_seq[lane_idx_before_change_point + 1]->Id()) {
      auto connect_lane = hdmap_->GetLaneById(laneids[i]);
      bool need_skip = false;
      for (int j = 0; j < connect_lane->LaneMultipleType().size(); ++j) {
        LOG_DEBUG("lane id:{}, X type:{}",
                  hdmap_->GetIdHashString(connect_lane->Id()),
                  connect_lane->LaneMultipleType()[j]);
        if (connect_lane->LaneMultipleType()[j] ==
                static_cast<uint32_t>(
                    global::hdmap::Lane::NONMOTORWAY_TO_MOTORWAY) ||
            connect_lane->LaneMultipleType()[j] ==
                static_cast<uint32_t>(
                    global::hdmap::Lane::MOTORWAY_TO_NONMOTORWAY)) {
          need_skip = true;
          break;
        }
      }
      if (need_skip) continue;
      connect_idxs.push_back(connect_lane->LaneNo());
      LOG_DEBUG("connect_idx:{}, connect id:{}, next_idx:{}, next id:{}",
                connect_lane->LaneNo(),
                hdmap_->GetIdHashString(connect_lane->Id()), next_idx,
                hdmap_->GetIdHashString(
                    lane_seq[lane_idx_before_change_point]->Id()));
      break;
    }
  }
  if (!connect_idxs.empty()) {
    ctx_->lane_change_direct =
        connect_idxs[0] > next_idx ? Lane::LEFT_TURN : Lane::RIGHT_TURN;
    for (auto idx : connect_idxs)
      change_count = std::min(change_count, std::abs(next_idx - idx));
  }
  context->is_lane_change_point_updated = true;
  context->need_lanechange = true;
  context->min_lane_change_s =
      context->lane_change_point_s - change_count * max_lane_change_len;
  LOG_INFO("min lane change s:{}", context->min_lane_change_s);
}

void AStarNavigationProcess::CreateManualLaneChangeInfo() {
  auto context = ctx_;
  if (context->is_manual_lane_change_updated) return;
  LOG_INFO("Create manual lane change info");
  auto &lane_seq = context->lane_seq;
  context->lane_change_point_s = 0.0;
  int lane_idx_before_change_point = -1;
  for (int i = current_lane_idx_; i + 1 < lane_seq.size(); ++i) {
    context->lane_change_point_s =
        accumulated_s_[i] + lane_seq[i]->TotalLength();
    if (!lane_topo_->IsConnectLane(lane_seq[i]->Id(), lane_seq[i + 1]->Id()) ||
        context->lane_change_point_s - context->current_s > 400.0) {
      lane_idx_before_change_point = std::min(
          (int)lane_seq.size() - 1, std::max(i, current_lane_idx_ + 1));
      LOG_INFO("id:{}, string:{}, len:{}", lane_seq[i]->Id(),
               hdmap_->GetIdHashString(lane_seq[i]->Id()),
               context->lane_change_point_s);
      break;
    }
  }
  if (lane_idx_before_change_point < 0)
    lane_idx_before_change_point = lane_seq.size() - 1;

  // Calc lane change direct
  switch (config::NavigationConfig::Instance()
              ->navigation_config()
              .manual_lane_change_order) {
    case 1:
      context->lane_change_direct = Lane::LEFT_TURN;
      break;

    case 2:
      context->lane_change_direct = Lane::RIGHT_TURN;
      break;

    default:
      context->lane_change_direct = Lane::NO_TURN;
      break;
  }

  auto neighbour = lane_topo_->GetNeighbour(
      context->lane_seq[lane_idx_before_change_point]->Id(),
      context->lane_change_direct);
  auto neighbour_lane = hdmap_->GetLaneById(neighbour);
  if (neighbour_lane == nullptr) {
    LOG_INFO("Find no neighbour lane of {}",
             hdmap_->GetIdHashString(
                 context->lane_seq[lane_idx_before_change_point]->Id()));
    context->is_lane_change_point_updated = false;
    context->need_lanechange = false;
    return;
  }
  lane_seq_after_lane_change_.clear();
  lane_seq_after_lane_change_.push_back(neighbour_lane);

  context->is_manual_lane_change_updated = true;
  context->is_lane_change_point_updated = true;
  context->need_lanechange = true;
  context->min_lane_change_s = 0.0;
  LOG_INFO("min lane change s:{}", context->min_lane_change_s);
}

bool AStarNavigationProcess::CreatePlanningLaneChangeInfo() {
  const auto &request = ctx_->routing_request;
  static uint64_t start_lane_id = 0;
  uint64_t input = data_center_->lane_change_target_id();
  if (input == 0 || input == start_lane_id) return false;
  start_lane_id = input;
  // locate ego car idx and next waypoint idx
  int cur_idx = -1, next_keypoint_idx = -1;
  if (!FindNextWayPointIdx(ctx_->lane_seq, cur_idx, next_keypoint_idx)) {
    LOG_ERROR(
        "find no waypoint behind current pose, cur_idx:{}, next_keypoint_idx",
        cur_idx, next_keypoint_idx);
    return false;
  }

  while (next_keypoint_idx < request->waypoint_size()) {
    uint64_t target_lane_id = FindLaneIdByWaypointIdx(next_keypoint_idx);
    // get new road seq from ego pose to target lane
    std::vector<uint64_t> road_seq;
    ExtractRoadSeq(current_lane_idx_, target_lane_id, road_seq);
    if (road_seq.empty()) {
      LOG_ERROR("empty lane change road seq by next_keypoint_idx:{}",
                next_keypoint_idx);
      next_keypoint_idx++;
      continue;
    }

    // ban original lane seq in front of crossroad
    ctx_->planning_ban_lane_set.clear();
    for (int i = current_lane_idx_; i < ctx_->lane_seq.size(); ++i) {
      LOG_INFO("ban lane id:{}",
               hdmap_->GetIdHashString(ctx_->lane_seq[i]->Id()));
      ctx_->planning_ban_lane_set.insert(ctx_->lane_seq[i]->Id());
      // break at next junction
      auto road = hdmap_->GetRoadById(ctx_->lane_seq[i]->RoadId());
      auto junction = hdmap_->GetJunctionById(road->JunctionId());
      bool is_in_junction_turn =
          junction != nullptr &&
          ctx_->lane_seq[i]->TurnType() != static_cast<uint32_t>(Lane::NO_TURN);
      bool is_in_signal = !ctx_->lane_seq[i]->Signals().empty();
      if (is_in_junction_turn || is_in_signal) break;
    }

    // search for lane change seq
    ctx_->is_planning_lane_change = true;
    std::vector<uint64_t> res =
        SearchLaneChangeLaneSeq(start_lane_id, target_lane_id, road_seq);
    ctx_->is_planning_lane_change = false;
    if (!res.empty()) {
      ctx_->ResetLaneChangeContext();
      // save lane change seq until target lane
      for (auto id : res) {
        auto index_lane = hdmap_->GetLaneById(id);
        if (index_lane == nullptr) {
          ctx_->lane_change_seq.clear();
          break;
        }
        ctx_->lane_change_seq.push_back(index_lane);
      }
      if (ctx_->lane_change_seq.empty()) continue;
      // stitch lane seq after target lane
      auto target_lane_idx = GetSeqLaneIdx(target_lane_id);
      if (target_lane_idx == -1) {
        LOG_ERROR("invalid target_lane_idx");
        ctx_->lane_change_seq.clear();
      } else if (target_lane_idx + 1 < ctx_->lane_seq.size())
        ctx_->lane_change_seq.insert(
            ctx_->lane_change_seq.end(),
            ctx_->lane_seq.begin() + target_lane_idx + 1, ctx_->lane_seq.end());
      LOG_INFO("lane seq size:{}, lane change seq size:{}",
               ctx_->lane_seq.size(), ctx_->lane_change_seq.size());
      break;
    } else {
      LOG_ERROR("find no lane change seq by next_keypoint_idx:{}",
                next_keypoint_idx);
      next_keypoint_idx++;
    }
  }
  if (ctx_->lane_change_seq.empty()) {
    LOG_ERROR("fail to find lane change seq by planning");
    return false;
  }

  // // generate lane change info
  auto start_idx = GetRoadLaneIdx(start_lane_id);
  auto curr_idx = GetRoadLaneIdx(ctx_->lane_seq[current_lane_idx_]->Id());
  if (start_idx == -1 || curr_idx == -1) {
    ctx_->lane_change_seq.clear();
    LOG_ERROR("fail to find road lane idx. start_idx:{}, curr_idx:{}",
              start_idx, curr_idx);
    return false;
  }
  ctx_->navigator_request =
      start_idx <= curr_idx
          ? planning::NavigatorLaneChangeRequest::REQUEST_LEFT_LANE_CHANGE
          : planning::NavigatorLaneChangeRequest::REQUEST_RIGHT_LANE_CHANGE;
  ctx_->preview_navigator_request =
      start_idx <= curr_idx
          ? planning::NavigatorLaneChangeRequest::REQUEST_LEFT_LANE_CHANGE
          : planning::NavigatorLaneChangeRequest::REQUEST_RIGHT_LANE_CHANGE;
  LOG_INFO("lane change direct:{}, request:{}, lane change stage:{}",
           ctx_->lane_change_direct, ctx_->navigator_request,
           data_center_->master_info().motorway_lane_change_context().stage);
  return true;
}

void AStarNavigationProcess::UpdateLaneChangeLaneSeq() {
  auto context = ctx_;
  const auto &config =
      config::NavigationConfig::Instance()->navigation_config();
  if (!context->need_lanechange) return;
  double generated_length = 0.0;
  LOG_INFO("cur s:{:.4f}, min change s:{:.4f}, lane change seq size:{}",
           context->current_s, context->min_lane_change_s,
           context->lane_change_seq.size());
  auto &lane_seq = context->lane_seq;
  auto road = hdmap_->GetRoadById(lane_seq[current_lane_idx_]->RoadId());
  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  bool is_in_junction_turn =
      junction != nullptr && lane_seq[current_lane_idx_]->TurnType() !=
                                 static_cast<uint32_t>(Lane::NO_TURN);
  bool is_in_signal = !lane_seq[current_lane_idx_]->Signals().empty();
  bool junc_wait = is_in_junction_turn || is_in_signal;
  bool need_wait = context->current_s - context->last_lane_change_finish_s <
                       config.lane_change_wait_dist &&
                   context->lane_change_point_s - context->current_s >
                       config.base_change_length * 2;
  LOG_INFO(
      "need wait:{}, last_lane_change_finish_s:{}, wait dist:{}, dist to "
      "lane_change_point_s:{}, junc_wait:{}",
      need_wait, context->last_lane_change_finish_s,
      context->current_s - context->last_lane_change_finish_s,
      context->lane_change_point_s - context->current_s, junc_wait);
  if (context->current_s >= context->min_lane_change_s) {
    context->preview_navigator_request =
        context->lane_change_direct == Lane::LEFT_TURN
            ? planning::NavigatorLaneChangeRequest::REQUEST_LEFT_LANE_CHANGE
            : planning::NavigatorLaneChangeRequest::REQUEST_RIGHT_LANE_CHANGE;
    LOG_INFO("into lane change range, preview_navigator_request:{}",
             context->preview_navigator_request);
  }
  if (context->current_s >= context->min_lane_change_s &&
      context->lane_change_seq.empty() && !need_wait && !junc_wait) {
    LOG_INFO("cur lane idx:{}, lane seq size:{}", current_lane_idx_,
             context->lane_seq.size());
    // get neighbor lane as first lane of lane change seq
    auto neighbour =
        lane_topo_->GetNeighbour(context->lane_seq[current_lane_idx_]->Id(),
                                 context->lane_change_direct);
    if (neighbour != 0) {
      // context->lane_change_seq.push_back(hdmap_->GetLaneById(neighbour));
      LOG_INFO(
          "cur lane:{} find neighb:{}",
          hdmap_->GetIdHashString(context->lane_seq[current_lane_idx_]->Id()),
          hdmap_->GetIdHashString(neighbour));
    } else {
      LOG_ERROR("fail to find neighbour of current lane");
      return;
    }
    // backtracking search for lane change seq before junction
    // check backtracking road range
    std::vector<uint64_t> road_seq;
    ExtractRoadSeq(current_lane_idx_, lane_seq_after_lane_change_[0]->Id(),
                   road_seq);
    if (road_seq.empty()) {
      LOG_ERROR("empty lane change road seq");
      return;
    }
    // backtracking mainbody
    ctx_->is_lane_change_ref = true;
    std::vector<uint64_t> res = SearchLaneChangeLaneSeq(
        neighbour, lane_seq_after_lane_change_[0]->Id(), road_seq);
    ctx_->is_lane_change_ref = false;
    if (!res.empty()) {
      // last lane of res is lane_seq_after_lane_change_[0], skip check and push
      for (int j = 0; j + 1 < res.size(); ++j) {
        LOG_INFO("find lane id:{}. string:{}", res[j],
                 hdmap_->GetIdHashString(res[j]));
        auto index_lane = hdmap_->GetLaneById(res[j]);

        // check lane change valid
        auto GetLeftDivider = [index_lane]() {
          if (index_lane->left_divider().empty())
            return static_cast<uint32_t>(
                global::hdmap::LaneBoundaryType::UNKNOWN);
          else
            return index_lane->left_divider()[0].type;
        };
        auto GetRightDivider = [index_lane]() {
          if (index_lane->right_divider().empty())
            return static_cast<uint32_t>(
                global::hdmap::LaneBoundaryType::UNKNOWN);
          else
            return index_lane->right_divider()[0].type;
        };
        bool is_valid = true;
        auto lane_boundary_type = ctx_->lane_change_direct == Lane::LEFT_TURN
                                      ? GetRightDivider()
                                      : GetLeftDivider();
        LOG_INFO("lane_boundary_type:{}", (int)lane_boundary_type);
        if (lane_boundary_type !=
                global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
            lane_boundary_type !=
                global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
            lane_boundary_type != global::hdmap::LaneBoundaryType::UNKNOWN) {
          LOG_INFO("SOLID_WHITE lane boundary, ban lane change!");
          is_valid = false;
        }
        auto lc_road = hdmap_->GetRoadById(index_lane->RoadId());
        auto lc_junction = hdmap_->GetJunctionById(lc_road->JunctionId());
        bool lc_is_in_junction_turn =
            lc_junction != nullptr &&
            index_lane->TurnType() != static_cast<uint32_t>(Lane::NO_TURN);
        bool lc_is_in_signal = !index_lane->Signals().empty();
        if (lc_is_in_signal || lc_is_in_junction_turn) {
          LOG_INFO("cur lane has signals or junction turn, ban lane change");
          is_valid = false;
        }
        if (!is_valid) {
          context->lane_change_seq.clear();
          LOG_ERROR("fail to find complete lane seq before turn");
          return;
        }

        context->lane_change_seq.push_back(index_lane);
      }
    } else {
      context->lane_change_seq.clear();
      LOG_ERROR("fail to find complete lane seq before turn");
      return;
    }

    // stitch lane_seq_after_lane_change
    LOG_INFO("lane_change_seq size:{}, lane_seq_after_lane_change_ size:{}",
             context->lane_change_seq.size(),
             lane_seq_after_lane_change_.size());
    context->lane_change_seq.insert(context->lane_change_seq.end(),
                                    lane_seq_after_lane_change_.begin(),
                                    lane_seq_after_lane_change_.end());
    LOG_INFO("update change lane update:{}", context->lane_change_seq.size());

    context->navigator_request =
        context->lane_change_direct == Lane::LEFT_TURN
            ? planning::NavigatorLaneChangeRequest::REQUEST_LEFT_LANE_CHANGE
            : planning::NavigatorLaneChangeRequest::REQUEST_RIGHT_LANE_CHANGE;

    LOG_INFO("lane change direct:{}, request:{}, lane change stage:{}",
             context->lane_change_direct, context->navigator_request,
             data_center_->master_info().motorway_lane_change_context().stage);
  }
}

void AStarNavigationProcess::PostProcessLaneChangeInfo() {
  auto context = ctx_;
  auto &config = config::NavigationConfig::Instance()->navigation_config();
  static bool is_first_finish = true;
  if (data_center_->master_info().curr_scenario() ==
      ScenarioState::MOTORWAY_LANE_CHANGE) {
    // swap lane_seq and lane_change_seq when lane change finished
    if (data_center_->master_info().motorway_lane_change_context().stage ==
            global::planning::MotorwayLaneChangeStageState::FINISH &&
        !context->lane_change_seq.empty() && is_first_finish) {
      LOG_INFO(
          "enter LANE_CHANGE FINISH stage, swap lane seq and lane change seq");
      context->lane_seq.clear();
      context->lane_seq.insert(context->lane_seq.begin(),
                               context->lane_change_seq.begin(),
                               context->lane_change_seq.end());
      context->lane_change_seq.clear();
      ctx_->is_lane_change_point_updated = false;
      ctx_->need_lanechange = false;
      context->navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
      context->preview_navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
      is_first_finish = false;
    }
    // finish lane change when ego car enter lane change route by SD
    if (!data_center_->is_auto_driving() && CheckIsOnLaneChangeRoute() &&
        data_center_->master_info().motorway_lane_change_context().stage !=
            global::planning::MotorwayLaneChangeStageState::FINISH &&
        !context->lane_change_seq.empty()) {
      LOG_INFO(
          "enter lane change route by SD, finish lane change by navigation");
      context->lane_seq.clear();
      context->lane_seq.insert(context->lane_seq.begin(),
                               context->lane_change_seq.begin(),
                               context->lane_change_seq.end());
      context->lane_change_seq.clear();
      ctx_->is_lane_change_point_updated = false;
      ctx_->need_lanechange = false;
      context->navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
      context->preview_navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
      is_first_finish = false;
    }

    if (data_center_->master_info().motorway_lane_change_context().stage !=
        global::planning::MotorwayLaneChangeStageState::FINISH) {
      is_first_finish = true;
      if (!context->lane_change_seq.empty() && context->is_mixed_change &&
          context->lane_change_point_s - context->current_s <=
              config.mixed_lanechange_fail_dist) {
        data_center_->mutable_event_report_proxy()->SetEvent(
            EventType::MIXED_LANE_CHANGE_FAIL);
      }
    }
    // clear lane_change_seq when lane change canceled
    if (data_center_->master_info().motorway_lane_change_context().stage ==
        global::planning::MotorwayLaneChangeStageState::CANCEL) {
      LOG_INFO("enter LANE_CHANGE CANCEL stage, reset");
      context->lane_change_seq.clear();
      // ctx_->is_lane_change_point_updated = false;
      ctx_->need_lanechange = false;
      context->navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
      context->preview_navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
    }
  } else {
    is_first_finish = true;
  }

  // set lane id info of lane change
  if (context->navigator_request !=
      planning::NavigatorLaneChangeRequest::NO_REQUEST) {
    auto neighbour =
        lane_topo_->GetNeighbour(context->lane_seq[current_lane_idx_]->Id(),
                                 context->lane_change_direct);
    data_center_->set_current_lane_idx(
        GetRoadLaneIdx(context->lane_seq[current_lane_idx_]->Id()));
    data_center_->set_target_lane_idx(GetRoadLaneIdx(neighbour));
  } else {
    if (context->lane_seq[current_lane_idx_]->IsOnLane(
            {data_center_->vehicle_state_utm().X(),
             data_center_->vehicle_state_utm().Y()})) {
      data_center_->set_current_lane_idx(
          GetRoadLaneIdx(context->lane_seq[current_lane_idx_]->Id()));
      data_center_->set_target_lane_idx(
          GetRoadLaneIdx(context->lane_seq[current_lane_idx_]->Id()));
    } else {
      data_center_->set_current_lane_idx(-1);
      data_center_->set_target_lane_idx(-1);
    }
  }
}

bool AStarNavigationProcess::ComputeReferenceLine(
    std::shared_ptr<RefeLineGenerator> &current_ref_generator,
    std::shared_ptr<RefeLineGenerator> &target_ref_generator) {
  auto &navigation_swap_context = data_center_->GetNavigationSwapContext();
  auto navigation_context = ctx_;
  navigation_context->first_finish_lane_change = false;
  if (data_center_->master_info().motorway_lane_change_context().stage ==
          global::planning::MotorwayLaneChangeStageState::FINISH &&
      navigation_context->lane_change_seq.empty() &&
      !navigation_context->need_reset_ref_generator) {
    // first finish lane change, swap ref-line generator
    LOG_INFO("first finish lane change, swap ref-line generator");
    // for (auto lane : navigation_context->lane_change_seq)
    //   LOG_INFO("lane_change_seq id:{}", lane->Id());
    // for (auto lane : navigation_context->lane_seq)
    //   LOG_INFO("lane_seq id:{}", lane->Id());

    if (lane_change_finish_cnt_ == 0) {
      ++lane_change_finish_cnt_;
      current_lane_idx_ = -1;
      std::swap(current_ref_generator, target_ref_generator);
      navigation_context->first_finish_lane_change = true;
      // current_ref_generator->ResetUtmRef();
    }
  } else if (data_center_->master_info().motorway_lane_change_context().stage !=
             global::planning::MotorwayLaneChangeStageState::FINISH) {
    lane_change_finish_cnt_ = 0;
  }
  LOG_INFO("cur addr:{}, tar addr:{}", (void *)&current_ref_generator,
           (void *)&target_ref_generator);

  // Generate target reference line
  if (!navigation_context->lane_change_seq.empty()) {
    LOG_INFO("get lane change seq:{}",
             navigation_context->lane_change_seq.size());
    for (auto lane : navigation_context->lane_change_seq)
      LOG_INFO("lane_change_seq id:{}, string:{}", lane->Id(),
               hdmap_->GetIdHashString(lane->Id()));
    navigation_context->is_lane_change_ref = true;
    if (!target_ref_generator->Generate(
            navigation_context->lane_change_seq,
            NavigationProcessBase::GetTrajectoryPoint(
                data_center_->vehicle_state_utm()))) {
      LOG_ERROR("Generate tar reference line failed!");
      target_ref_generator.reset(new RefeLineGenerator(ctx_));

      return false;
    }
    navigation_context->target_utm_ref = target_ref_generator->GetUtmRef();

    // if (data_center_->target_odom_ref() != nullptr &&
    //     !data_center_->target_odom_ref()->ref_points().empty())
    //   VisPaths(data_center_->target_odom_ref()->ref_points(),
    //            "lane_change_ref");
  } else {
    LOG_INFO("clear tar ref generator");
    target_ref_generator.reset(new RefeLineGenerator(ctx_));
    navigation_context->target_utm_ref = nullptr;
  }

  // Generate current reference line
  for (auto lane : navigation_context->lane_seq)
    LOG_DEBUG("lane_seq id:{}, string:{}", lane->Id(),
              hdmap_->GetIdHashString(lane->Id()));
  navigation_context->is_lane_change_ref = false;
  if (!current_ref_generator->Generate(
          navigation_context->lane_seq,
          NavigationProcessBase::GetTrajectoryPoint(
              data_center_->vehicle_state_utm()))) {
    LOG_ERROR("Generate cur reference line failed!");
    current_ref_generator.reset(new RefeLineGenerator(ctx_));
    return false;
  }
  navigation_context->current_utm_ref = current_ref_generator->GetUtmRef();
  navigation_context->lane_change_end_point = {
      current_ref_generator->GetLaneChangeEndPoint().x(),
      current_ref_generator->GetLaneChangeEndPoint().y(), 0.0};
  if (navigation_context->lane_change_end_point.x() > 0.0 &&
      navigation_context->lane_change_end_point.y() > 0.0) {
    ReferencePoint ref_pt{};
    navigation_context->current_utm_ref->GetNearestRefPoint(
        {navigation_context->lane_change_end_point.x(),
         navigation_context->lane_change_end_point.y()},
        &ref_pt);
    if (std::hypot(navigation_context->lane_change_end_point.x() - ref_pt.x(),
                   navigation_context->lane_change_end_point.y() - ref_pt.y()) <
        1.0) {
      LOG_INFO("find nearest ref pt x,y,s: {}, {}, {}", ref_pt.x(), ref_pt.y(),
               ref_pt.s());
      navigation_context->lane_change_end_point_s = ref_pt.s();
    } else {
      LOG_INFO("ref pt is far away from lane_change_end_point");
      navigation_context->lane_change_end_point_s = 0.0;
    }
  } else {
    LOG_INFO("find no lane_change_end_point");
    navigation_context->lane_change_end_point_s = 0.0;
  }
  LOG_INFO("lane_change_end_point x, y ,s: {}, {}, {}",
           current_ref_generator->GetLaneChangeEndPoint().x(),
           current_ref_generator->GetLaneChangeEndPoint().y(),
           navigation_context->lane_change_end_point_s);
  navigation_swap_context.dest_point.Set(
      current_ref_generator->GetDestinationPoint());
  navigation_swap_context.dist_to_end.Set(
      current_ref_generator->GetDistanceToDestination());
  navigation_context->dist_to_lane_change_end =
      navigation_context->lane_change_point_s - navigation_context->current_s;
  // navigation_swap_context.lane_change_end_point.Set(
  //     current_ref_generator->GetLaneChangeEndPoint());
  LOG_INFO(
      "routing_destination_s_:{}, current_s:{}, dist_to_routing_destination:{}",
      routing_destination_s_, navigation_context->current_s,
      routing_destination_s_ - navigation_context->current_s);
  navigation_swap_context.dist_to_routing_destination.Set(
      routing_destination_s_ - navigation_context->current_s);
  navigation_swap_context.driving_direction.Set(
      MasterInfo::DriveDirection::DRIVE_FORWARD);
  static int cnt = 0;
  if (ctx_->first_finish_lane_change ||
      (navigation_context->is_swap_ref_line == true && cnt < 3)) {
    navigation_context->is_swap_ref_line = true;
    ++cnt;
  } else {
    navigation_context->is_swap_ref_line = false;
    cnt = 0;
  }

  if (ctx_->first_finish_lane_change) {
    ctx_->last_lane_change_finish_s = current_ref_generator->GetCurrentS();
    UpdateAccumulatedS();
    CalculateMileage();
  }

  LOG_INFO("is_swap_ref_line:{}", navigation_context->is_swap_ref_line);

  SetOdometrySpeedLimit();

  return true;
}

double AStarNavigationProcess::CalcDistBetweenPoints(
    int next_keypoint_idx, int idx,
    const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq) {
  double len = 0.0;
  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  const auto &request = ctx_->routing_request;

  LOG_INFO("idx:{}, lane seq size:{}", idx, lane_seq.size());
  for (int i = idx; i < lane_seq.size(); ++i) {
    double next_key_x = request->waypoint(next_keypoint_idx).pose().x();
    double next_key_y = request->waypoint(next_keypoint_idx).pose().y();
    if (lane_seq[i]->IsOnLane({next_key_x, next_key_y})) {
      double s, l;
      lane_seq[i]->GetProjection({next_key_x, next_key_y}, &s, &l);
      len += s;
      break;
    }
    len += lane_seq[i]->TotalLength();
  }
  double s, l;
  lane_seq[idx]->GetProjection({x, y}, &s, &l);
  len -= s;

  LOG_INFO("CalcStartToNextKeypointDist: {}", len);
  return len;
}

}  // namespace planning
}  // namespace neodrive
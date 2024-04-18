#include "breadth_first_navigation.h"

#include "common/navigation_context.h"
#include "common/navigation_types.h"
#include "common/util/time_util.h"
#include "config/navigation_config.h"
#include "proto/map_lane.pb.h"
#include "scenario_manager_msgs.pb.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {
using cyberverse::LaneInfoConstPtr;
using cyberverse::RoadInfoConstPtr;
using global::hdmap::Lane;
using JunctionType = autobot::cyberverse::Junction::JunctionType;
using neodrive::global::hdmap::Lane_LaneTurn_Name;

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

bool BreadthFirstNavigationProcess::Init() {
  if (is_inited_) {
    return true;
  }
  InitBase();
  is_inited_ = true;
  return true;
}

void BreadthFirstNavigationProcess::Process(
    const std::shared_ptr<RoutingRequest> &routing_request,
    std::shared_ptr<RoutingResult> &routing_respons) {
  if (!UpdateEgoPose()) return;
  LOG_INFO("receive routing request:{}", routing_request->DebugString());
  SearchRoutingRoadSequence(routing_request, 0,
                            routing_request->waypoint_size() - 1);
  LOG_ERROR("generate routing_request");
  GenerateLaneSequenceFromRoadSequence(routing_request, 0,
                                       routing_request->waypoint_size() - 1,
                                       routing_respons);
  LOG_INFO("routing result:{}", routing_respons->DebugString());
  CalculateMileage();
}

void BreadthFirstNavigationProcess::RunOnce() {
  auto start_time = cyber::Time::Now().ToSecond();
  do {
    if (!data_center_->vehicle_state_utm().IsValid()) break;
    if (routing_lane_seq_.empty()) break;
    UpdateLaneSequenceIndex(routing_lane_seq_);
    if (current_lane_idx_ < 0) break;
    if (prev_lane_idx_ != current_lane_idx_ || !is_changing_lane_) {
      // if arrived the next new lane, need to recalculate lane change info
      prev_lane_idx_ = current_lane_idx_;
      if (!CalcLaneChangeInfo()) break;
    } else {
      if (!UpdateLaneChangeInfo()) break;
    }
    // CheckLaneChange();
    auto context = ctx_;
    LOG_INFO("current s:{}", context->current_s);
    UpdateLaneChangeLaneSeq();
    if (data_center_->master_info().motorway_lane_change_context().stage ==
            global::planning::MotorwayLaneChangeStageState::FINISH &&
        !context->lane_change_seq.empty()) {
      context->lane_seq.clear();
      context->lane_seq.insert(context->lane_seq.begin(),
                               context->lane_change_seq.begin(),
                               context->lane_change_seq.end());
      context->lane_change_seq.clear();
      ctx_->is_lane_change_point_updated = false;
      ctx_->need_lanechange = false;
      context->navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
    }
    CheckTurnHornLights();
  } while (false);

  auto end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("BreadthFirstNavigationProcess::RunOnce use time: {:.4f}",
           end_time - start_time);
}

std::vector<uint64_t> BreadthFirstNavigationProcess::GetNextLanes(
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

std::vector<uint64_t> BreadthFirstNavigationProcess::GetTurnNextLanes(
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
  // if (turn_set.empty() && (!main_loop || lane_cnt.first !=
  // lane_cnt.second))
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

bool BreadthFirstNavigationProcess::FindLanePath(
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

bool BreadthFirstNavigationProcess::GetRoutingFromRoadSeq(
    std::shared_ptr<RoutingResult> &routing_respons) {
  const auto &road_topo_map = road_topo_->GetFromToMap();
  auto &lane_seq = ctx_->lane_seq;
  auto &road_seq = ctx_->road_seq;
  auto &routing_start_pt = ctx_->routing_start_pt;
  auto &routing_end_pt = ctx_->routing_end_pt;

  auto ans = breadth_first_search_.Search(road_seq);
  if (!ans.first) return false;
  auto &res = ans.second;
  lane_seq.clear();
  routing_lane_seq_.clear();
  for (int i = 0; i < res.size(); ++i) {
    auto lane = hdmap_->GetLaneById(res[i]);
    if (lane == nullptr) {
      LOG_ERROR("find nullptr lane");
      return false;
    }
    LOG_INFO("find lane id:{}. string:{}", res[i],
             hdmap_->GetIdHashString(res[i]));
    lane_seq.push_back(lane);
    AddRoute(routing_respons, lane, lane->RoadId(),
             (i == 0 ? &routing_start_pt : nullptr),
             (i == res.size() - 1 ? &routing_end_pt : nullptr));
  }
  current_lane_idx_ = -1;
  routing_lane_seq_ = lane_seq;
  return true;
}

void BreadthFirstNavigationProcess::CreateLaneChangeInfo() {
  LOG_INFO("enter lane change info");
  auto context = ctx_;
  auto &lane_seq = context->lane_seq;
  context->lane_change_point_s = 0.0;
  int lane_idx_before_change_point = -1;
  for (int i = 0; i < lane_seq.size() - 1; ++i) {
    context->lane_change_point_s += lane_seq[i]->TotalLength();
    if (!lane_topo_->IsConnectLane(lane_seq[i]->Id(), lane_seq[i + 1]->Id())) {
      lane_idx_before_change_point = i;
      LOG_INFO("id:{}, string:{}, len:{}", lane_seq[i]->Id(),
               hdmap_->GetIdHashString(lane_seq[i]->Id()),
               context->lane_change_point_s);
      break;
    }
  }
  auto road = hdmap_->GetRoadById(lane_seq[current_lane_idx_]->RoadId());
  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  bool is_cross_road =
      (junction != nullptr &&
       (junction->Type() == static_cast<uint32_t>(JunctionType::CROSS_ROAD) ||
        junction->Type() == static_cast<uint32_t>(JunctionType::T_CROSS_ROAD) ||
        junction->Type() == static_cast<uint32_t>(JunctionType::Y_CROSS_ROAD)));
  if (lane_idx_before_change_point < 0 || is_cross_road) {
    LOG_INFO("no need to change lane, cur lane id:{}",
             hdmap_->GetIdHashString(lane_seq[current_lane_idx_]->Id()));
    context->is_lane_change_point_updated = false;
    context->need_lanechange = false;
    return;
  }

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
  int change_count = -1;
  for (uint32_t i = 0; i < laneids.size(); ++i) {
    if (change_count >= 0) change_count += 1;
    if (laneids[i] == lane_seq[lane_idx_before_change_point]->Id()) {
      context->lane_change_direct =
          (change_count < 0) ? Lane::RIGHT_TURN : Lane::LEFT_TURN;
      if (change_count < 0)
        change_count = 0;
      else
        break;
    }
    if (lane_topo_->IsConnectLane(
            laneids[i], lane_seq[lane_idx_before_change_point + 1]->Id())) {
      if (change_count < 0)
        change_count = 0;
      else
        break;
    }
  }
  context->is_lane_change_point_updated = true;
  context->need_lanechange = true;
  context->min_lane_change_s =
      context->lane_change_point_s -
      change_count * config::NavigationConfig::Instance()
                         ->navigation_config()
                         .max_lane_change_len;
  LOG_INFO("min lane change s:{}", context->min_lane_change_s);
}

void BreadthFirstNavigationProcess::UpdateLaneChangeLaneSeq() {
  auto context = ctx_;
  if (!context->need_lanechange) return;
  double generated_length = 0.0;
  LOG_INFO("cur s:{:.4f}, min change s:{:.4f}, lane change seq size:{}",
           context->current_s, context->min_lane_change_s,
           context->lane_change_seq.size());
  if (context->current_s >= context->min_lane_change_s &&
      context->lane_change_seq.empty()) {
    LOG_INFO("cur lane idx:{}, lane seq size:{}", current_lane_idx_,
             context->lane_seq.size());
    // get neighbor lane as first lane of lane change seq
    auto neighbour =
        lane_topo_->GetNeighbour(context->lane_seq[current_lane_idx_]->Id(),
                                 context->lane_change_direct);
    if (neighbour != 0) {
      context->lane_change_seq.push_back(hdmap_->GetLaneById(neighbour));
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
    const auto &road_topo_map = road_topo_->GetFromToMap();
    for (auto i = current_lane_idx_; i < context->lane_seq.size(); ++i) {
      auto id = context->lane_seq[i]->RoadId();
      if (id == lane_seq_after_lane_change_[0]->RoadId()) break;
      if (road_topo_map.find(id) != road_topo_map.end()) {
        road_seq.push_back(id);
        LOG_INFO("road id:{}, string:{}", id, hdmap_->GetIdHashString(id));
      }
    }
    if (road_seq.empty()) {
      LOG_ERROR("empty lane change road seq");
      return;
    }
    // backtracking mainbody
    std::vector<std::pair<int, uint64_t>> res;
    lane_cnt_.clear();
    lane_cnt_.reserve(road_seq.size());
    lane_cnt_[0] = std::make_pair(0, 0);
    if (FindLanePath(1, neighbour, lane_seq_after_lane_change_[0]->Id(),
                     road_seq, res)) {
      for (auto p : res) {
        LOG_INFO("find lane i:{}, id:{}. string:{}", p.first, p.second,
                 hdmap_->GetIdHashString(p.second));
        auto index_lane = hdmap_->GetLaneById(p.second);
        context->lane_change_seq.push_back(index_lane);
      }
      std::unordered_set<uint64_t> lane_set;
      for (auto lane : lane_seq_after_lane_change_) lane_set.insert(lane->Id());
      LOG_INFO("lane_change_seq back id:{}",
               context->lane_change_seq.back()->Id());
      while (lane_set.find(context->lane_change_seq.back()->Id()) !=
             lane_set.end()) {
        context->lane_change_seq.pop_back();
        LOG_INFO("lane_change_seq back id:{}",
                 context->lane_change_seq.back()->Id());
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

bool BreadthFirstNavigationProcess::CalcLaneChangeInfo() {
  auto context = ctx_;
  context->need_lanechange = false;
  context->lane_change_direct = Lane::NO_TURN;
  context->navigator_request = planning::NavigatorLaneChangeRequest::NO_REQUEST;
  is_changing_lane_ = false;

  auto target_lane_info = routing_lane_seq_[current_lane_idx_];
  auto road = hdmap_->GetRoadById(target_lane_info->RoadId());
  if (nullptr == road) {
    LOG_ERROR("failed to find current road");
    return false;
  }

  auto &laneids = road->Sections()[0]->LaneIds();
  double min_dist = std::numeric_limits<double>::max();
  cyberverse::LaneInfoConstPtr min_dist_lane{nullptr};
  uint32_t target_lane_ids_idx_ = laneids.size();
  uint32_t current_lane_ids_idx_ = laneids.size();
  for (uint32_t i = 0; i < laneids.size(); ++i) {
    auto lane = hdmap_->GetLaneById(laneids[i]);
    if (nullptr == lane) {
      LOG_ERROR("failed to find lane");
      return false;
    }
    if (lane->Id() == target_lane_info->Id()) {
      target_lane_ids_idx_ = i;
    }
    auto dis = lane->DistanceTo({data_center_->vehicle_state_utm().X(),
                                 data_center_->vehicle_state_utm().Y()});
    if (dis < min_dist) {
      min_dist_lane = lane;
      min_dist = dis;
      current_lane_ids_idx_ = i;
    }
  }

  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  bool is_cross_road =
      (junction != nullptr &&
       (junction->Type() == static_cast<uint32_t>(JunctionType::CROSS_ROAD) ||
        junction->Type() == static_cast<uint32_t>(JunctionType::T_CROSS_ROAD) ||
        junction->Type() == static_cast<uint32_t>(JunctionType::Y_CROSS_ROAD)));
  if (is_cross_road) {
    LOG_INFO("no need to change lane in cross road junction, cur lane id:{}",
             hdmap_->GetIdHashString(target_lane_info->Id()));
    return true;
  }
  if (current_lane_ids_idx_ > target_lane_ids_idx_) {
    // current lane is on the right of target lane, change to left lane
    context->need_lanechange = true;
    context->lane_change_direct = Lane::LEFT_TURN;
    target_change_lane_ids_idx_ = current_lane_ids_idx_ - 1;
    is_changing_lane_ = true;
  } else if (current_lane_ids_idx_ < target_lane_ids_idx_) {
    // current lane is on the left of target lane, change to right lane
    context->need_lanechange = true;
    context->lane_change_direct = Lane::RIGHT_TURN;
    target_change_lane_ids_idx_ = current_lane_ids_idx_ + 1;
    is_changing_lane_ = true;
  } else {
    context->need_lanechange = false;
    context->lane_change_direct = Lane::NO_TURN;
  }

  if (Lane::LEFT_TURN == context->lane_change_direct) {
    context->navigator_request =
        planning::NavigatorLaneChangeRequest::REQUEST_LEFT_LANE_CHANGE;
  } else if (Lane::RIGHT_TURN == context->lane_change_direct) {
    context->navigator_request =
        planning::NavigatorLaneChangeRequest::REQUEST_RIGHT_LANE_CHANGE;
  } else {
    context->navigator_request =
        planning::NavigatorLaneChangeRequest::NO_REQUEST;
  }
}

bool BreadthFirstNavigationProcess::UpdateLaneChangeInfo() {
  auto target_lane_info = routing_lane_seq_[current_lane_idx_];
  auto road = hdmap_->GetRoadById(target_lane_info->RoadId());
  if (nullptr == road) {
    LOG_ERROR("failed to find current road");
    return false;
  }

  auto &laneids = road->Sections()[0]->LaneIds();
  double min_dist = std::numeric_limits<double>::max();
  cyberverse::LaneInfoConstPtr min_dist_lane{nullptr};
  uint32_t target_lane_ids_idx_ = laneids.size();
  uint32_t current_lane_ids_idx_ = laneids.size();
  for (uint32_t i = 0; i < laneids.size(); ++i) {
    auto lane = hdmap_->GetLaneById(laneids[i]);
    if (nullptr == lane) {
      LOG_ERROR("failed to find lane");
      return false;
    }
    if (lane->Id() == target_lane_info->Id()) {
      target_lane_ids_idx_ = i;
    }
    auto dis = lane->DistanceTo({data_center_->vehicle_state_utm().X(),
                                 data_center_->vehicle_state_utm().Y()});
    if (dis < min_dist) {
      min_dist_lane = lane;
      min_dist = dis;
      current_lane_ids_idx_ = i;
    }
  }

  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  bool is_cross_road =
      (junction != nullptr &&
       (junction->Type() == static_cast<uint32_t>(JunctionType::CROSS_ROAD) ||
        junction->Type() == static_cast<uint32_t>(JunctionType::T_CROSS_ROAD) ||
        junction->Type() == static_cast<uint32_t>(JunctionType::Y_CROSS_ROAD)));
  if (is_cross_road) {
    LOG_INFO("no need to change lane in cross road junction, cur lane id:{}",
             hdmap_->GetIdHashString(target_lane_info->Id()));
    return true;
  }
}

void BreadthFirstNavigationProcess::CheckLaneChange() {
  auto context = ctx_;
  if (context->is_lane_change_point_updated &&
      context->current_s < context->min_lane_change_s)
    return;
  if (!context->is_lane_change_point_updated) CreateLaneChangeInfo();
}

bool BreadthFirstNavigationProcess::ComputeReferenceLine(
    std::shared_ptr<RefeLineGenerator> &current_ref_generator,
    std::shared_ptr<RefeLineGenerator> &target_ref_generator) {
  auto &navigation_swap_context = data_center_->GetNavigationSwapContext();
  auto context = ctx_;
  context->first_finish_lane_change = false;
  if (data_center_->master_info().motorway_lane_change_context().stage ==
          global::planning::MotorwayLaneChangeStageState::FINISH &&
      context->lane_change_seq.empty()) {
    // first finish lane change, swap ref-line generator
    LOG_INFO("first finish lane change, swap ref-line generator");
    for (auto lane : context->lane_change_seq)
      LOG_INFO("lane_change_seq id:{}", lane->Id());
    for (auto lane : context->lane_seq) LOG_INFO("lane_seq id:{}", lane->Id());

    if (lane_change_finish_cnt_ == 0) {
      ++lane_change_finish_cnt_;
      current_lane_idx_ = -1;
      std::swap(current_ref_generator, target_ref_generator);
      context->first_finish_lane_change = true;
      // current_ref_generator->ResetUtmRef();
    }
  } else if (data_center_->master_info().motorway_lane_change_context().stage !=
             global::planning::MotorwayLaneChangeStageState::FINISH) {
    lane_change_finish_cnt_ = 0;
  }
  LOG_INFO("cur addr:{}, tar addr:{}", (void *)&current_ref_generator,
           (void *)&target_ref_generator);

  // Generate target reference line
  if (!context->lane_change_seq.empty()) {
    LOG_INFO("get lane change seq:{}", context->lane_change_seq.size());
    for (auto lane : context->lane_change_seq) {
      LOG_INFO("lane_change_seq id:{}, string:{}", lane->Id(),
               hdmap_->GetIdHashString(lane->Id()));
    }
    context->is_lane_change_ref = true;
    if (!target_ref_generator->Generate(
            context->lane_change_seq, NavigationProcessBase::GetTrajectoryPoint(
                                          data_center_->vehicle_state_utm()))) {
      LOG_ERROR("Generate tar reference line failed!");
      target_ref_generator.reset(new RefeLineGenerator(ctx_));

      return false;
    }
    context->target_utm_ref = target_ref_generator->GetUtmRef();

    // if (data_center_->target_odom_ref() != nullptr &&
    //     !data_center_->target_odom_ref()->ref_points().empty())
    //   VisPaths(data_center_->target_odom_ref()->ref_points(),
    //            "lane_change_ref");
  } else {
    LOG_INFO("clear tar ref generator");
    target_ref_generator.reset(new RefeLineGenerator(ctx_));
    context->target_utm_ref = nullptr;
  }

  // Generate current reference line
  for (auto lane : context->lane_seq) {
    LOG_INFO("lane_seq id:{}, string:{}", lane->Id(),
             hdmap_->GetIdHashString(lane->Id()));
  }
  context->is_lane_change_ref = false;
  if (!current_ref_generator->Generate(
          context->lane_seq, NavigationProcessBase::GetTrajectoryPoint(
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
  context->dist_to_lane_change_end =
      current_ref_generator->GetDistanceToRefEnd();
  navigation_swap_context.dist_to_routing_destination.Set(
      routing_destination_s_ - context->current_s);
  navigation_swap_context.driving_direction.Set(
      MasterInfo::DriveDirection::DRIVE_FORWARD);
  static int cnt = 0;
  if (context->first_finish_lane_change ||
      (context->is_swap_ref_line == true && cnt < 3)) {
    context->is_swap_ref_line = true;
    ++cnt;
  } else {
    context->is_swap_ref_line = false;
    cnt = 0;
  }

  if (context->first_finish_lane_change) UpdateAccumulatedS();

  LOG_INFO("is_swap_ref_line:{}", context->is_swap_ref_line);

  SetOdometrySpeedLimit();

  return true;
}
}  // namespace planning
}  // namespace neodrive
#include "a_star_lane_search.h"

#include <queue>

#include "src/common/util/vector_angle_util.h"
#include "src/planning/navigation/config/navigation_config.h"
namespace neodrive {
namespace planning {
using cyberverse::LaneInfoConstPtr;
using global::hdmap::Lane;
using JunctionType = autobot::cyberverse::Junction::JunctionType;

double GetCurLaneS(LaneInfoConstPtr lane, double x, double y) {
  double s, l;
  lane->GetProjection({x, y}, &s, &l);
  return s;
}

double GetTurnTypeCost(const uint32_t turn_type) {
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  double cost = 0.0;
  switch (turn_type) {
    case static_cast<uint32_t>(Lane::LEFT_TURN):
      cost = navigation_config.left_turn_penalty;
      break;
    case static_cast<uint32_t>(Lane::RIGHT_TURN):
      cost = navigation_config.right_turn_penalty;
      break;
    case static_cast<uint32_t>(Lane::NO_TURN):
      cost = 0.0;
      break;
    case static_cast<uint32_t>(Lane::U_TURN):
      cost = navigation_config.uturn_penalty;
      break;
    default:
      cost = 0.0;
  }
  LOG_DEBUG("turn cost:{}", cost);
  return cost;
}

bool AStarLaneSearch::Init(const uint64_t end,
                           const std::vector<uint64_t> &road_seq) {
  auto context = ctx_;
  Reset();

  // calculate road accumulated s
  if (!CalcAccumulatedS(context->routing_start_pt, road_seq)) return false;

  // build search map
  std::vector<uint64_t> curr_lanes{end};
  std::vector<uint64_t> next_lanes{};
  TurnType turn;
  for (int i = road_seq.size() - 1; i >= 1; --i) {
    LOG_DEBUG("road from {} to {}", hdmap_->GetIdHashString(road_seq[i]),
              hdmap_->GetIdHashString(road_seq[i - 1]));
    if (road_topo_->GetLinkType(road_seq[i - 1], road_seq[i], turn))
      curr_lanes = GetTurnLanes(curr_lanes, turn);

    auto next_road = hdmap_->GetRoadById(road_seq[i - 1]);
    next_lanes.clear();
    for (int j = 0; j < next_road->Sections()[0]->LaneIds().size(); ++j)
      next_lanes.push_back(next_road->Sections()[0]->LaneIds()[j]);
    for (auto curr_lane : curr_lanes) {
      for (auto next_lane : next_lanes) {
        if (context->is_lane_change_ref &&
            context->ban_lane_set.count(next_lane) != 0)
          continue;
        if (context->is_planning_lane_change &&
            context->planning_ban_lane_set.count(next_lane) != 0)
          continue;
        SetCost(curr_lane, next_lane, false);
      }
      lanes_map_[curr_lane] = next_lanes;
      LOG_DEBUG("lane id:{}, lanes_map size:{}",
                hdmap_->GetIdHashString(curr_lane),
                lanes_map_[curr_lane].size());
    }
    curr_lanes = next_lanes;
  }

  return true;
}

bool AStarLaneSearch::Search(const uint64_t start, const uint64_t end) {
  LOG_INFO("start back lane search from: {}, string:{}", start,
           hdmap_->GetIdHashString(start));
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  const auto &to_from_map = lane_topo_->GetToFromMap();
  LOG_INFO("to_from_map size:{}", to_from_map.size());
  std::priority_queue<LaneNode *, std::vector<LaneNode *>, Compair> open_set{};
  std::vector<LaneNode> all_nodes;
  all_nodes.reserve(navigation_config.max_lane_search_size);
  all_nodes.emplace_back(start, nullptr, 0.0, false, 0.0);
  LOG_INFO("all_nodes size:{}, top id:{}, string", all_nodes.size(),
           all_nodes.back().id, hdmap_->GetIdHashString(all_nodes.back().id));
  open_set.push(&all_nodes.back());
  SaveLanePath(&all_nodes.back());
  LOG_INFO("open set size:{}, top id:{}, string", open_set.size(),
           open_set.top()->id, hdmap_->GetIdHashString(open_set.top()->id));
  std::unordered_set<uint64_t> close_set{};
  score_map_[start] = 0.0;

  // limit lane type 0:no limit, 1:motorway, 2:nonmotorway
  limit_type_ = "0";
  auto context = ctx_;
  if (context->start_type == "1" && context->end_type == "1")
    limit_type_ = "1";
  else if (context->start_type == "2" && context->end_type == "2")
    limit_type_ = "2";
  LOG_INFO("AStarLaneSearch limit type: {}", limit_type_);

  while (!open_set.empty()) {
    auto top_node = open_set.top();
    open_set.pop();
    LOG_DEBUG("topnode id:{}, hash:{}", hdmap_->GetIdHashString(top_node->id),
              top_node->id);
    if (top_node->id == end) {
      LOG_INFO("find end, all_nodes size:{}", all_nodes.size());
      SaveLanePath(top_node);
      return true;
    }
    // if (close_set.count(top_node->id)) continue;

    auto lane_iter = lanes_map_.find(top_node->id);
    if (lane_iter == lanes_map_.end()) {
      LOG_ERROR("cannot find lane {} in lanes_map_",
                hdmap_->GetIdHashString(top_node->id));
      continue;
    }
    LOG_DEBUG("next lanes size:{}", lane_iter->second.size());
    for (auto next_lane : lane_iter->second) {
      bool is_change = false;
      double lane_change_dist_cost = 0.0;
      double lane_change_cnt_cost = 0.0;
      if (!CheckLaneChange(next_lane, top_node, is_change,
                           lane_change_dist_cost, lane_change_cnt_cost))
        continue;

      double merge_in_cost = CheckMergeIn(top_node->id, next_lane) && !is_change
                                 ? navigation_config.merge_in_cost
                                 : 0.0;
      double sort_cost =
          lane_change_dist_cost + lane_change_cnt_cost + merge_in_cost;

      LaneNode next(next_lane, top_node,
                    top_node->cost + GetCost(top_node->id, next_lane),
                    is_change, sort_cost);
      LOG_DEBUG("next node id:{}, cost:{}, cnt:{}",
                hdmap_->GetIdHashString(next.id), next.cost,
                score_map_.count(next.id));
      if ((next.cost < 10e5 && score_map_.count(next.id) == 0) ||
          (score_map_.count(next.id) != 0 &&
           next.cost <= score_map_[next.id])) {
        LOG_DEBUG("find smaller node id:{}, cost:{}",
                  hdmap_->GetIdHashString(next.id), next.cost);
        score_map_[next.id] = next.cost;
        all_nodes.push_back(next);
        auto &last_push = all_nodes.back();
        open_set.push(&last_push);
        close_set.insert(top_node->id);

        if (all_nodes.size() >= navigation_config.max_lane_search_size) {
          LOG_ERROR("reach all_nodes max capacity, stop searching");
          return false;
        }
      }
    }
  }
  LOG_INFO("all_nodes size:{}, lanes_map size:{}, routing_lanes_map size:{}",
           all_nodes.size(), lanes_map_.size(), routing_lanes_map_.size());
  return true;
}

bool AStarLaneSearch::BikingSearch(const uint64_t start, const uint64_t end) {
  LOG_INFO("start BIKING search from: {} to {}", hdmap_->GetIdHashString(start),
           hdmap_->GetIdHashString(end));
  Reset();
  auto const to_from_map = lane_topo_->GetToFromMap();
  std::priority_queue<LaneNode *, std::vector<LaneNode *>, Compair> open_set{};
  std::vector<LaneNode> all_nodes;
  all_nodes.reserve(lane_topo_->GetFromToMap().size());
  all_nodes.emplace_back(start, nullptr, 0.0, false, 0.0);
  LOG_INFO("all_nodes size:{}, top id:{}, string", all_nodes.size(),
           all_nodes.back().id, hdmap_->GetIdHashString(all_nodes.back().id));
  open_set.push(&all_nodes.back());
  SaveLanePath(&all_nodes.back());
  std::unordered_set<uint64_t> close_set{};
  score_map_[start] = 0.0;

  // limit lane type 0:no limit, 1:motorway, 2:nonmotorway
  std::string limit_type_ = "0";
  auto context = ctx_;
  if (context->start_type == "1" && context->end_type == "1")
    limit_type_ = "1";
  else if (context->start_type == "2" && context->end_type == "2")
    limit_type_ = "2";
  LOG_INFO("BikingSearch limit type: {}", limit_type_);

  while (!open_set.empty()) {
    auto top_node = open_set.top();
    open_set.pop();
    LOG_DEBUG("topnode id:{}, hash:{}", hdmap_->GetIdHashString(top_node->id),
              top_node->id);

    if (top_node->id == end) {
      LOG_INFO("find end, all_nodes size:{}", all_nodes.size());
      SaveLanePath(top_node);
      return true;
    }

    if (close_set.count(top_node->id)) continue;
    close_set.insert(top_node->id);

    auto map_iter = to_from_map.find(top_node->id);
    if (map_iter == to_from_map.end()) continue;

    for (auto iter_neighbour = map_iter->second.begin();
         iter_neighbour != map_iter->second.end(); ++iter_neighbour) {
      auto next_lane = hdmap_->GetLaneById(*iter_neighbour);
      auto next_type = GetNodeType(*iter_neighbour);
      LaneNode next(*iter_neighbour, top_node,
                    top_node->cost + GetTurnTypeCost(next_lane->TurnType()) +
                        next_lane->TotalLength() +
                        GetCloudNaviMotorwayCost(next_type),
                    false, 0.0);
      LOG_DEBUG("next node id:{}, cost:{}, cnt:{}",
                hdmap_->GetIdHashString(next.id), next.cost,
                score_map_.count(next.id));
      if (score_map_.count(next.id) == 0 || next.cost < score_map_[next.id]) {
        LOG_DEBUG("find smaller node id:{}, cost:{}",
                  hdmap_->GetIdHashString(next.id), next.cost);
        score_map_[next.id] = next.cost;
        all_nodes.push_back(next);
        auto &last_push = all_nodes.back();
        open_set.push(&last_push);
      }
    }
  }
  return false;
}

std::vector<uint64_t> AStarLaneSearch::GetRoutingLanes(uint64_t start) {
  if (routing_lanes_map_.find(start) == routing_lanes_map_.end()) {
    LOG_ERROR("cannot find lane {} in routing_lanes_map_",
              hdmap_->GetIdHashString(start));
    return {};
  } else {
    LOG_INFO("find lane routing lanes, size:{}",
             routing_lanes_map_[start].size());
    return routing_lanes_map_[start];
  }
}

void AStarLaneSearch::VisLaneNode(std::string name) {
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  for (std::unordered_map<uint64_t, double>::iterator iter = score_map_.begin();
       iter != score_map_.end(); ++iter) {
    auto lane = hdmap_->GetLaneById(iter->first);
    auto p = lane->Points().front();

    auto text = event->mutable_text()->Add();
    text->mutable_position()->set_x(p.x());
    text->mutable_position()->set_y(p.y());
    text->mutable_position()->set_z(0);
    text->set_text(std::to_string(int(iter->second)));
  }
}

void AStarLaneSearch::Reset() {
  accumulated_s_.clear();
  score_map_.clear();
  routing_lanes_map_.clear();
  lanes_map_.clear();
  cost_map_.clear();
}

bool AStarLaneSearch::CalcAccumulatedS(const Vec3d start,
                                       const std::vector<uint64_t> &road_seq) {
  LOG_INFO("get in CalcAccumulatedS");
  auto data_center = DataCenter::Instance();
  for (int i = 0; i < road_seq.size(); ++i) {
    if (i == 0) {
      auto cur_lane_id = ctx_->routing_start_lane_id;
      auto lane = hdmap_->GetLaneById(cur_lane_id);
      if (lane == nullptr) {
        LOG_INFO("find null lane, quit!");
        return false;
      }
      double s = GetCurLaneS(lane, start.x(), start.y());
      accumulated_s_[road_seq[i]] = lane->TotalLength() - s;
      LOG_INFO("id:{}, lane s:{}, total len:{}",
               hdmap_->GetIdHashString(cur_lane_id), s, lane->TotalLength());
    } else {
      accumulated_s_[road_seq[i]] = accumulated_s_[road_seq[i - 1]] +
                                    road_topo_->GetRoadCost(road_seq[i]);
    }
    LOG_DEBUG("i:{}, accu s:{}, id:{}", i, accumulated_s_[road_seq[i]],
              hdmap_->GetIdHashString(road_seq[i]));
  }
  LOG_INFO("accumulated_s_ size:{}", accumulated_s_.size());
  return true;
}

std::vector<uint64_t> AStarLaneSearch::GetTurnLanes(
    const std::vector<uint64_t> &curr_lanes, TurnType turn) {
  LOG_DEBUG("GetTurnLanes");
  const auto context = ctx_;
  std::vector<uint64_t> res;
  const auto &to_from_map = lane_topo_->GetToFromMap();
  for (auto curr_lane : curr_lanes) {
    std::vector<uint64_t> turn_lanes;
    auto to_from_iter = to_from_map.find(curr_lane);
    if (to_from_iter != to_from_map.end()) {
      for (auto iter = to_from_iter->second.begin();
           iter != to_from_iter->second.end(); ++iter) {
        auto next_lane = hdmap_->GetLaneById(*iter);
        LOG_DEBUG("cur id:{}, next id:{}", hdmap_->GetIdHashString(curr_lane),
                  hdmap_->GetIdHashString(next_lane->Id()));
        if (context->is_lane_change_ref &&
            context->ban_lane_set.count(next_lane->Id()) != 0)
          continue;
        if (context->is_planning_lane_change &&
            context->planning_ban_lane_set.count(next_lane->Id()) != 0)
          continue;
        if (next_lane->TurnType() == turn) {
          res.push_back(next_lane->Id());
          turn_lanes.push_back(next_lane->Id());
          link_lanes_.insert(next_lane->Id());
          SetCost(curr_lane, *iter, true);
        }
      }
    }
    lanes_map_[curr_lane] = turn_lanes;
  }
  return res;
}

int AStarLaneSearch::GetLaneChangeCnt(uint64_t cur, uint64_t next) {
  auto lane = hdmap_->GetLaneById(next);
  auto road = hdmap_->GetRoadById(lane->RoadId());
  auto lane_ids = road->Sections()[0]->LaneIds();
  std::vector<int> connect_idxs;
  int next_idx = lane->LaneNo(), lane_change_cnt = 100;
  for (int i = 0; i < lane_ids.size(); ++i) {
    if (lane_topo_->IsConnectLane(lane_ids[i], cur)) {
      auto connect_lane = hdmap_->GetLaneById(lane_ids[i]);
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
    }
  }
  for (auto idx : connect_idxs)
    lane_change_cnt = std::min(lane_change_cnt, std::abs(next_idx - idx));

  return lane_change_cnt;
}

int AStarLaneSearch::GetLaneChangeDirection(uint64_t cur, uint64_t next,
                                            TurnType &turn) {
  auto lane = hdmap_->GetLaneById(next);
  auto road = hdmap_->GetRoadById(lane->RoadId());
  auto lane_ids = road->Sections()[0]->LaneIds();
  int change_count = -1;
  int connect_idx = 0, next_idx = lane->LaneNo();
  for (uint32_t i = 0; i < lane_ids.size(); ++i) {
    if (lane_topo_->IsConnectLane(lane_ids[i], cur)) {
      auto connect_lane = hdmap_->GetLaneById(lane_ids[i]);
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
      connect_idx = connect_lane->LaneNo();
      break;
    }
  }
  if (connect_idx != 0) {
    turn = connect_idx > next_idx ? Lane::LEFT_TURN : Lane::RIGHT_TURN;
  }
  return std::abs(connect_idx - next_idx);
}

bool AStarLaneSearch::CheckLaneChange(uint64_t next_id, const LaneNode *node,
                                      bool &is_change,
                                      double &lane_change_dist_cost,
                                      double &lane_change_cnt_cost) {
  double length = 0.0;
  auto tmp = node;
  TurnType last_turn = Lane::NO_TURN;
  while (length < 500.0 && tmp != nullptr) {
    auto lane = hdmap_->GetLaneById(tmp->id);
    length += lane->TotalLength();
    if (tmp->is_change) {
      GetLaneChangeDirection(tmp->parent->id, tmp->id, last_turn);
      break;
    }
    LOG_DEBUG("id:{}, len:{}", hdmap_->GetIdHashString(tmp->id), length);
    tmp = tmp->parent;
  }

  auto lane = hdmap_->GetLaneById(next_id);
  auto road = hdmap_->GetRoadById(lane->RoadId());
  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  bool is_in_junction_turn =
      junction != nullptr &&
      lane->TurnType() != static_cast<uint32_t>(Lane::NO_TURN);
  bool is_in_signal = !lane->Signals().empty();

  double single_change_len = config::NavigationConfig::Instance()
                                 ->navigation_config()
                                 .base_change_length;
  double min_lane_change_length = single_change_len;

  auto CalcLaneChangeDistCost = [length]() {
    if (length > 200.0) {
      return 0.0;
    } else {
      return (200.0 - length) / 100.0;
    }
  };
  auto GetLeftDivider = [lane]() {
    if (lane->left_divider().empty())
      return static_cast<uint32_t>(global::hdmap::LaneBoundaryType::UNKNOWN);
    else
      return lane->left_divider()[0].type;
  };
  auto GetRightDivider = [lane]() {
    if (lane->right_divider().empty())
      return static_cast<uint32_t>(global::hdmap::LaneBoundaryType::UNKNOWN);
    else
      return lane->right_divider()[0].type;
  };

  int lane_change_cnt = GetLaneChangeCnt(node->id, next_id);
  if (tmp != nullptr && tmp->is_change && tmp->parent != nullptr) {
    int former_lane_change_cnt = GetLaneChangeCnt(tmp->parent->id, tmp->id);
    min_lane_change_length = single_change_len * former_lane_change_cnt;
  }
  if (length >= min_lane_change_length || tmp == nullptr) {
    is_change = !lane_topo_->IsConnectLane(next_id, node->id);
    if (is_change) {
      // forbid lane change in signal lane and junction turn
      if (is_in_signal || is_in_junction_turn) {
        LOG_DEBUG("forbid lane change in signal lane and junction turn:{}, {}",
                  is_in_signal, is_in_junction_turn);
        return false;
      }

      lane_change_dist_cost = CalcLaneChangeDistCost();
      lane_change_cnt_cost = lane_change_cnt < 2 ? 0 : lane_change_cnt;
      LOG_DEBUG("lane change dist: {}, cost: {}, lane change cnt:{}", length,
                lane_change_dist_cost, lane_change_cnt);

      // forbid lane change in merge-in junction
      // if (junction != nullptr) {
      //   auto neighbour = lane_topo->GetNeighbour(lane->Id(), turn);
      //   auto lane_iter = from_to_map.find(neighbour);
      //   if (lane_iter != from_to_map.end() &&
      //       lane_iter->second.find(node->id) != lane_iter->second.end())
      //     return false;
      // }

      // forbid lane change in solid-line lane
      TurnType turn;
      GetLaneChangeDirection(node->id, next_id, turn);
      if (ctx_->is_lane_change_ref && turn != ctx_->lane_change_direct &&
          turn != Lane::NO_TURN) {
        LOG_INFO("wrong lane change direction, ban lane change!");
        return false;
      }
      auto lane_boundary_type =
          turn == Lane::LEFT_TURN ? GetLeftDivider() : GetRightDivider();
      LOG_DEBUG("next lane_boundary_type id:{}, type:{}, left:{}, right:{}",
                hdmap_->GetIdHashString(next_id), lane_boundary_type,
                GetLeftDivider(), GetRightDivider());
      if (lane_boundary_type != global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
          lane_boundary_type !=
              global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
          lane_boundary_type != global::hdmap::LaneBoundaryType::UNKNOWN) {
        LOG_DEBUG("SOLID_WHITE lane boundary, ban lane change!");
        return false;
      }
    }
    return true;
  } else if (lane_topo_->IsConnectLane(next_id, node->id) && !is_in_signal &&
             !is_in_junction_turn) {
    LOG_DEBUG("cur id:{}, next id:{}, last turn:{}",
              hdmap_->GetIdHashString(node->id),
              hdmap_->GetIdHashString(next_id), (int)last_turn);
    is_change = false;
    if (last_turn == Lane::NO_TURN) {
      return true;
    } else {
      auto last_lane_boundary_type =
          last_turn == Lane::LEFT_TURN ? GetLeftDivider() : GetRightDivider();
      if (last_lane_boundary_type !=
              global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
          last_lane_boundary_type !=
              global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
          last_lane_boundary_type != global::hdmap::LaneBoundaryType::UNKNOWN)
        return false;
      return true;
      auto neighbour = lane_topo_->GetNeighbour(next_id, last_turn);
      LOG_DEBUG("neighb id:{}", hdmap_->GetIdHashString(neighbour));
      if (neighbour != 0) {
        const auto &from_to_map = lane_topo_->GetFromToMap();
        auto neighbour_to = from_to_map.find(neighbour);
        // LOG_INFO("neighb to id:{}",
        //          hdmap_->GetIdHashString(neighbour_to->first));
        if (neighbour_to != from_to_map.end() &&
            neighbour_to->second.find(node->id) == neighbour_to->second.end())
          return true;
      }
    }
  }

  LOG_DEBUG(
      "CheckLaneChange false, lane change len:{:.4f}, is connect:{}, is "
      "signal:{}, from {} to {}",
      length, lane_topo_->IsConnectLane(next_id, node->id), is_in_signal,
      hdmap_->GetIdHashString(node->id), hdmap_->GetIdHashString(next_id));
  return false;
}

void AStarLaneSearch::SetCost(uint64_t cur, uint64_t next,
                              bool is_link = false) {
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  const auto context = ctx_;
  // 1.1 lane type cost
  double type_cost = 0.0;
  auto cur_type = GetNodeType(cur);
  auto next_type = GetNodeType(next);
  LOG_DEBUG("cur type:{}, next type:{}", (int)cur_type, (int)next_type);
  // 1.1.1 bus bay
  if (next_type != NodeType::BUS_BAY && cur_type == NodeType::BUS_BAY) {
    const auto &from_to_map = lane_topo_->GetFromToMap();
    if (from_to_map.find(next) != from_to_map.end() &&
        from_to_map.find(next)->second.size() > 1) {
      type_cost += navigation_config.bus_bay_cost;
    }
  }
  if (next_type == NodeType::BUS_BAY && cur_type != NodeType::BUS_BAY) {
    const auto &to_from_map = lane_topo_->GetFromToMap();
    if (to_from_map.find(cur) != to_from_map.end() &&
        to_from_map.find(cur)->second.size() > 1) {
      type_cost += navigation_config.bus_bay_cost;
    }
  }
  // 1.1.2 lane type change
  if (cur_type != next_type && cur_type != NodeType::BUS_BAY &&
      next_type != NodeType::BUS_BAY)
    type_cost += navigation_config.lane_type_change_cost;
  // 1.1.3 cloud navigation motorway
  type_cost += GetCloudNaviMotorwayCost(next_type);

  // 1.2 Merge-in cost
  double merge_in_cost = 0.0;

  // 2.lane change cost
  double lane_change_cost = 0.0;
  double single_change_len = config::NavigationConfig::Instance()
                                 ->navigation_config()
                                 .base_change_length;
  double min_start_lane_change_len =
      context->is_lane_change_ref || context->lane_seq.empty()
          ? single_change_len
          : 0.0;
  auto lane = hdmap_->GetLaneById(next);
  auto road = hdmap_->GetRoadById(lane->RoadId());
  auto junction = hdmap_->GetJunctionById(road->JunctionId());
  if (!lane_topo_->IsConnectLane(next, cur)) {
    if (junction == nullptr ||
        junction->Type() != static_cast<uint32_t>(JunctionType::CROSS_ROAD)) {
      // 2.1 lane change count cost
      int lane_change_cnt = GetLaneChangeCnt(cur, next);
      LOG_DEBUG("id:{}, accu s:{}, cnt:{}", hdmap_->GetIdHashString(next),
                accumulated_s_[road->Id()], lane_change_cnt);
      if (accumulated_s_[road->Id()] / lane_change_cnt >
              navigation_config.base_change_length ||
          lane_change_cnt < 2 || ctx_->is_lane_change_ref)
        lane_change_cost = navigation_config.lane_change_cost * lane_change_cnt;
      else
        lane_change_cost = DBL_MAX;
    } else {
      // 2.2 crossroad lane change cost
      LOG_DEBUG("next {} is link", hdmap_->GetIdHashString(next));
      lane_change_cost = DBL_MAX;
    }
  }

  // save cost
  cost_map_[cur].emplace(next, type_cost + merge_in_cost + lane_change_cost);
  LOG_DEBUG(
      "Set cost cur:{}, next:{}, type_cost:{} ,merge_in_cost:{}, lane change "
      "cost:{}, total cost:{}",
      hdmap_->GetIdHashString(cur), hdmap_->GetIdHashString(next), type_cost,
      merge_in_cost, lane_change_cost, cost_map_[cur][next]);
}

double AStarLaneSearch::GetCost(uint64_t cur, uint64_t next) {
  auto cur_iter = cost_map_.find(cur);
  if (cur_iter == cost_map_.end()) {
    LOG_INFO("no cur id:{} in cost map", hdmap_->GetIdHashString(cur));
    return DBL_MAX;
  }
  auto next_iter = cur_iter->second.find(next);
  if (next_iter == cur_iter->second.end()) {
    LOG_INFO("no next id:{} in cost map", hdmap_->GetIdHashString(next));
    return DBL_MAX;
  }
  return cost_map_[cur][next];
}

double AStarLaneSearch::GetCloudNaviMotorwayCost(NodeType type) {
  if ((type == NodeType::MOTORWAY && limit_type_ == "2") &&
      (type == NodeType::BIKING && limit_type_ == "1"))
    return 10.0;
  else
    return 0.0;
}

NodeType AStarLaneSearch::GetNodeType(const uint64_t id) {
  bool is_motorway = false, is_biking = false, is_bus_bay = false;

  auto lane = hdmap_->GetLaneById(id);
  auto lane_multiple_type = lane->LaneMultipleType();
  for (int j = 0; j < lane_multiple_type.size(); ++j) {
    if (lane_multiple_type[j] == global::hdmap::Lane::CITY_DRIVING)
      is_motorway = true;
    if (lane_multiple_type[j] == global::hdmap::Lane::BIKING ||
        lane_multiple_type[j] == global::hdmap::Lane::INDOOR_LANE)
      is_biking = true;
    if (lane_multiple_type[j] == global::hdmap::Lane::BUS_BAY_LANE)
      is_bus_bay = true;
  }

  LOG_DEBUG("id:{}, motor:{}, biking:{}", hdmap_->GetIdHashString(id),
            is_motorway, is_biking);
  if (is_bus_bay) return NodeType::BUS_BAY;
  if (is_biking) return NodeType::BIKING;
  return NodeType::MOTORWAY;
}

bool AStarLaneSearch::CheckMergeIn(const uint64_t cur, const uint64_t next) {
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  const auto &to_from_map = lane_topo_->GetToFromMap();
  auto next_lane = hdmap_->GetLaneById(next);
  auto cur_lane = hdmap_->GetLaneById(cur);
  if (!next_lane->Signals().empty()) return false;

  bool left_merge = false, right_merge = false;
  uint64_t left_merge_id = 0, right_merge_id = 0;
  auto road = hdmap_->GetRoadById(cur_lane->RoadId());
  if (road == nullptr) return false;
  const auto &lane_ids = road->Sections()[0]->LaneIds();
  if (lane_topo_->IsConnectLane(next, cur) &&
      to_from_map.find(cur) != to_from_map.end() &&
      to_from_map.find(cur)->second.size() > 1) {
    const auto &from_lane_ids = to_from_map.find(cur)->second;
    double left_merge_angle = -std::numeric_limits<double>::max(),
           right_merge_angle = std::numeric_limits<double>::max();
    for (auto id : from_lane_ids) {
      if (id == next) continue;
      auto neigh_lane = hdmap_->GetLaneById(id);
      double angle = common::CalcAngle2NeighborVec(next_lane->Points(),
                                                   neigh_lane->Points());
      LOG_DEBUG("cur id:{}, neigh id:{}, angle diff:{}",
                hdmap_->GetIdHashString(next_lane->Id()),
                hdmap_->GetIdHashString(id), angle);
      if (angle > 0 && angle <= right_merge_angle) {
        // find closer right merge lane
        right_merge_id = id;
        right_merge_angle = angle;
        right_merge = true;
      }
      if (angle < 0 && angle >= left_merge_angle) {
        // find closer left merge lane
        left_merge_id = id;
        left_merge_angle = angle;
        left_merge = true;
      }
    }
  }

  LOG_DEBUG("cur id:{} ,left id:{}, left_merge:{}, right id:{}, right_merge:{}",
            hdmap_->GetIdHashString(next),
            hdmap_->GetIdHashString(left_merge_id), left_merge,
            hdmap_->GetIdHashString(right_merge_id), right_merge);
  if (left_merge) {
    auto left_neighbour_lane = hdmap_->GetLaneById(left_merge_id);
    double cur_angle =
        common::CalcAngle2NextVec(next_lane->Points(), cur_lane->Points());
    double left_merge_angle = common::CalcAngle2NextVec(
        left_neighbour_lane->Points(), cur_lane->Points());
    LOG_DEBUG("cur angle:{}, left angle:{}, difference:{}", cur_angle,
              left_merge_angle, cur_angle - left_merge_angle);
    if (cur_angle - left_merge_angle >
        navigation_config.min_merge_angle_diff * kDegree2Radian) {
      LOG_DEBUG("left merge from {} to {}", hdmap_->GetIdHashString(next),
                hdmap_->GetIdHashString(left_merge_id));
      return true;
    }
  }
  if (right_merge) {
    auto right_neighbour_lane = hdmap_->GetLaneById(right_merge_id);
    double cur_angle =
        common::CalcAngle2NextVec(next_lane->Points(), cur_lane->Points());
    double right_merge_angle = common::CalcAngle2NextVec(
        right_neighbour_lane->Points(), cur_lane->Points());
    LOG_DEBUG("cur angle:{}, right angle:{}, difference:{}", cur_angle,
              right_merge_angle, cur_angle - right_merge_angle);
    if (cur_angle - right_merge_angle >
        navigation_config.min_merge_angle_diff * kDegree2Radian) {
      LOG_DEBUG("right merge from {} to {}", hdmap_->GetIdHashString(next),
                hdmap_->GetIdHashString(right_merge_id));
      return true;
    }
  }
  return false;
}

void AStarLaneSearch::SaveLanePath(LaneNode *node) {
  LOG_DEBUG("find to end,generate ret");
  std::vector<uint64_t> res;
  res.push_back(node->id);
  auto tmp = node->parent;
  while (tmp) {
    res.push_back(tmp->id);
    tmp = tmp->parent;
  }
  LOG_DEBUG("finish push, size: {}", res.size());
  routing_lanes_map_[node->id] = res;
}
}  // namespace planning
}  // namespace neodrive
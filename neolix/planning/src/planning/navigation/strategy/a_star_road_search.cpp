#include "a_star_road_search.h"

#include <queue>

#include "src/planning/navigation/config/navigation_config.h"
namespace neodrive {
namespace planning {
using global::hdmap::Lane;

bool AStarRoadSearch::GetNodeType(const uint64_t id,
                                  const NodeType &parent_type,
                                  NodeType &node_type, bool &is_mix_type) {
  auto road = hdmap_->GetRoadById(id);
  if (road == nullptr) {
    LOG_ERROR("cannot find road in hdmap, id:{}, string:{}", id,
              hdmap_->GetIdHashString(id));
    return false;
  }
  auto lane_ids = road->Sections()[0]->LaneIds();
  if (lane_ids.empty()) {
    LOG_ERROR("cannot find lane_ids of road id:{}, string:{}", id,
              hdmap_->GetIdHashString(id));
    return false;
  }
  bool is_motorway = false, is_biking = false, is_bus_bay = false,
       is_solid_biking = false;
  int bus_bay_cnt = 0;
  for (int i = 0; i < lane_ids.size(); ++i) {
    auto lane = hdmap_->GetLaneById(lane_ids[i]);
    if (lane == nullptr) {
      LOG_ERROR("cannot find lane in hdmap, id:{}, string:{}", lane->Id(),
                hdmap_->GetIdHashString(lane->Id()));
      return false;
    }
    auto lane_multiple_type = lane->LaneMultipleType();
    for (int j = 0; j < lane_multiple_type.size(); ++j) {
      if (lane_multiple_type[j] == global::hdmap::Lane::CITY_DRIVING)
        is_motorway = true;
      if (lane_multiple_type[j] == global::hdmap::Lane::BIKING ||
          lane_multiple_type[j] == global::hdmap::Lane::INDOOR_LANE) {
        is_biking = true;
        if (!lane->left_divider().empty()) {
          auto type = lane->left_divider()[0].type;
          if (type != global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
              type != global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
              type != global::hdmap::LaneBoundaryType::UNKNOWN)
            is_solid_biking = true;
        }
      }
      if (lane_multiple_type[j] == global::hdmap::Lane::BUS_BAY_LANE)
        ++bus_bay_cnt;
    }
  }
  is_bus_bay = bus_bay_cnt == lane_ids.size();
  LOG_DEBUG("id:{}, parent type:{}, motor:{}, biking:{}",
            hdmap_->GetIdHashString(id), (int)parent_type, is_motorway,
            is_biking);
  is_mix_type = false;
  if (is_motorway && is_biking) {
    node_type = parent_type;
    is_mix_type = !is_solid_biking;
  }
  if (is_motorway && !is_biking) node_type = NodeType::MOTORWAY;
  if (!is_motorway && is_biking) node_type = NodeType::BIKING;
  return true;
}

double AStarRoadSearch::GetNodeCost(const uint64_t id,
                                    const Lane::LaneTurn turn_type) {
  auto &config = config::NavigationConfig::Instance()->navigation_config();
  double cost = 0.0;
  switch (turn_type) {
    case static_cast<uint32_t>(Lane::LEFT_TURN):
      cost = config.left_turn_penalty;
      break;
    case static_cast<uint32_t>(Lane::RIGHT_TURN):
      cost = config.right_turn_penalty;
      break;
    case static_cast<uint32_t>(Lane::NO_TURN):
      cost = 0.0;
      break;
    case static_cast<uint32_t>(Lane::U_TURN):
      cost = config.uturn_penalty;
      break;
    default:
      cost = 0.0;
  }
  LOG_DEBUG("id:{}, length cost:{}, node cost:{}", hdmap_->GetIdHashString(id),
            road_topo_->GetRoadCost(id), cost);
  return road_topo_->GetRoadCost(id) + cost;
}

double AStarRoadSearch::GetNodeTypeCost(const NodeType &parent,
                                        const NodeType &child,
                                        uint64_t child_id,
                                        std::string limit_type) {
  double cost = 0.0;
  auto context = ctx_;

  // keep same type with previous road
  if (!(parent == NodeType::MIX || child == NodeType::MIX || parent == child))
    cost += 50.0;
  // keep same type with type limit
  if ((limit_type == "1" && child == NodeType::BIKING) ||
      (limit_type == "2" && child == NodeType::MOTORWAY))
    cost += 10.0;
  // When motorway and non-motorway lanes are mixed, non-motorway lanes
  // are preferred
  if (context->start_type != context->end_type && child == NodeType::MOTORWAY)
    cost += 10.0;
  // single bus bay road cost
  auto child_road = hdmap_->GetRoadById(child_id);
  auto lane_ids = child_road->Sections()[0]->LaneIds();
  if (lane_ids.size() == 1) {
    auto lane = hdmap_->GetLaneById(lane_ids[0]);
    auto lane_multiple_type = lane->LaneMultipleType();
    for (int j = 0; j < lane_multiple_type.size(); ++j) {
      if (lane_multiple_type[j] == global::hdmap::Lane::BUS_BAY_LANE)
        cost += 50.0;
    }
  }

  return cost;
}

bool AStarRoadSearch::CheckMotorwaySwitch(const NaviNode *parent,
                                          NaviNode *child) {
  if (parent->type == child->type) return true;
  // get switch roads
  auto tmp = parent;
  const NaviNode *first_mix_road{nullptr};
  double mix_length = 0.0;
  std::vector<uint64_t> switch_roads{child->id};
  while (tmp != nullptr && tmp->is_mix_type) {
    switch_roads.push_back(tmp->id);
    first_mix_road = tmp;
    mix_length += road_topo_->GetRoadCost(tmp->id);
    tmp = tmp->parent;
  }
  if (first_mix_road == nullptr) {
    switch_roads.push_back(parent->id);
    tmp = parent;
  } else {
    if (tmp != nullptr)
      switch_roads.push_back(tmp->id);
    else
      tmp = first_mix_road;
  }
  std::reverse(switch_roads.begin(), switch_roads.end());

  // get lanes
  bool is_connected = false;
  std::vector<uint64_t> start_lane_ids{}, end_lane_ids{};
  auto start_road = hdmap_->GetRoadById(switch_roads.front());
  GetLaneIds(start_road, tmp->type, start_lane_ids);
  auto end_road = hdmap_->GetRoadById(switch_roads.back());
  GetLaneIds(end_road, child->type, end_lane_ids);
  for (auto start_lane_id : start_lane_ids) {
    for (auto end_lane_id : end_lane_ids) {
      std::unordered_map<uint64_t, std::vector<uint64_t>> lanes_map;
      std::vector<uint64_t> curr_lanes{end_lane_id};
      std::vector<uint64_t> next_lanes{};
      TurnType turn;

      for (int i = switch_roads.size() - 1; i >= 1; --i) {
        LOG_DEBUG("road from {} to {}",
                  hdmap_->GetIdHashString(switch_roads[i]),
                  hdmap_->GetIdHashString(switch_roads[i - 1]));
        if (road_topo_->GetLinkType(switch_roads[i - 1], switch_roads[i], turn))
          curr_lanes = GetTurnLanes(curr_lanes, turn, lanes_map);

        auto next_road = hdmap_->GetRoadById(switch_roads[i - 1]);
        if (next_road == nullptr) {
          LOG_ERROR("find no road from switch_roads");
          return false;
        }
        next_lanes.clear();
        for (int j = 0; j < next_road->Sections()[0]->LaneIds().size(); ++j)
          next_lanes.push_back(next_road->Sections()[0]->LaneIds()[j]);
        for (auto curr_lane : curr_lanes) {
          lanes_map[curr_lane] = next_lanes;
        }
        curr_lanes = next_lanes;
      }
      is_connected = IsConnected(end_lane_id, start_lane_id, lanes_map);
      LOG_DEBUG("start id:{}, end id:{}, mix len:{}, is connected:{}",
                hdmap_->GetIdHashString(start_lane_id),
                hdmap_->GetIdHashString(end_lane_id), mix_length, is_connected);
      if (is_connected) return true;
    }
  }

  if (mix_length < 100.0 && !is_connected) return false;
  return true;
}

std::vector<uint64_t> AStarRoadSearch::GetTurnLanes(
    const std::vector<uint64_t> &curr_lanes, TurnType turn,
    std::unordered_map<uint64_t, std::vector<uint64_t>> &lanes_map) {
  LOG_DEBUG("GetTurnLanes");
  std::vector<uint64_t> res;
  const auto &to_from_map = lane_topo_->GetToFromMap();
  for (auto curr_lane : curr_lanes) {
    std::vector<uint64_t> turn_lanes;
    auto to_from_iter = to_from_map.find(curr_lane);
    if (to_from_iter != to_from_map.end()) {
      for (auto iter = to_from_iter->second.begin();
           iter != to_from_iter->second.end(); ++iter) {
        auto next_lane = hdmap_->GetLaneById(*iter);
        if (next_lane->TurnType() == turn) {
          res.push_back(next_lane->Id());
          turn_lanes.push_back(next_lane->Id());
        }
      }
    }
    lanes_map[curr_lane] = turn_lanes;
  }
  return res;
}

bool AStarRoadSearch::IsConnected(
    const uint64_t start, const uint64_t end,
    const std::unordered_map<uint64_t, std::vector<uint64_t>> &lanes_map) {
  if (start == end) return true;
  if (lanes_map.find(start) == lanes_map.end()) return false;
  auto lane_ids = lanes_map.find(start)->second;
  for (int i = 0; i < lane_ids.size(); ++i) {
    if (lane_topo_->IsConnectLane(lane_ids[i], start) &&
        IsConnected(lane_ids[i], end, lanes_map))
      return true;
  }
  return false;
}

bool AStarRoadSearch::IsMotorwayLane(uint64_t lane_id) {
  auto lane = hdmap_->GetLaneById(lane_id);
  auto lane_multiple_type = lane->LaneMultipleType();
  for (int j = 0; j < lane_multiple_type.size(); ++j) {
    if (lane_multiple_type[j] == global::hdmap::Lane::BIKING ||
        lane_multiple_type[j] == global::hdmap::Lane::INDOOR_LANE) {
      return false;
    }
  }
  return true;
}

uint64_t AStarRoadSearch::GetLaneId(cyberverse::RoadInfoConstPtr road,
                                    NodeType type) {
  if (road->Sections().empty() || road->Sections()[0] == nullptr ||
      road->Sections()[0]->LaneIds().empty())
    return 0;
  auto lane_ids = road->Sections()[0]->LaneIds();
  for (int i = lane_ids.size() - 1; i >= 0; --i) {
    auto is_motorway_lane = IsMotorwayLane(lane_ids[i]);
    bool is_motorway_road = type != NodeType::BIKING;
    if (is_motorway_lane == is_motorway_road) return lane_ids[i];
  }
  return lane_ids[0];
}

bool AStarRoadSearch::GetLaneIds(cyberverse::RoadInfoConstPtr road,
                                 NodeType type, std::vector<uint64_t> &res) {
  if (road->Sections().empty() || road->Sections()[0] == nullptr ||
      road->Sections()[0]->LaneIds().empty())
    return false;
  auto lane_ids = road->Sections()[0]->LaneIds();
  for (int i = lane_ids.size() - 1; i >= 0; --i) {
    auto is_motorway_lane = IsMotorwayLane(lane_ids[i]);
    bool is_motorway_road = type != NodeType::BIKING;
    if (is_motorway_lane == is_motorway_road) res.push_back(lane_ids[i]);
  }
  return true;
}

bool AStarRoadSearch::Search(const uint64_t start, const uint64_t end,
                             std::vector<uint64_t> &res) {
  LOG_INFO("start search from: {} to {}", start, end);
  auto const from_to_map = road_topo_->GetFromToMap();
  auto &config = config::NavigationConfig::Instance()->navigation_config();
  std::priority_queue<NaviNode *, std::vector<NaviNode *>, Compair> open_set{};

  // limit road type 0:no limit, 1:motorway, 2:nonmotorway
  std::string limit_type = "0";
  auto context = ctx_;
  if (context->start_type == "1" && context->end_type == "1")
    limit_type = "1";
  else if (context->start_type == "2" && context->end_type == "2")
    limit_type = "2";
  LOG_INFO("AStarRoadSearch limit type: {}", limit_type);

  NodeType type = NodeType::MIX;
  bool is_mix_type = false;
  if (context->start_type == "0") {
    if (!GetNodeType(start, NodeType::MIX, type, is_mix_type)) return false;
  } else {
    type = context->start_type == "1" ? NodeType::MOTORWAY : NodeType::BIKING;
  }
  std::vector<NaviNode> all_nodes;
  all_nodes.reserve(config.enlarge_memory_time *
                    road_topo_->GetNodeCost().size());
  all_nodes.emplace_back(start, nullptr, road_topo_->GetRoadCost(start), type,
                         is_mix_type);
  open_set.push(&all_nodes.back());
  std::unordered_map<uint64_t, double> score_map;
  std::unordered_set<uint64_t> close_set{};
  score_map[start] = 0.0;
  while (!open_set.empty()) {
    auto top_node = open_set.top();
    open_set.pop();
    LOG_DEBUG("topnode id:{}", hdmap_->GetIdHashString(top_node->id));

    if (top_node->id == end) {
      LOG_ERROR("find to end,generate ret");
      res.push_back(top_node->id);
      auto tmp = top_node->parent;
      while (tmp) {
        res.push_back(tmp->id);
        tmp = tmp->parent;
      }
      LOG_DEBUG("finish push, size: {}", res.size());
      std::reverse(res.begin(), res.end());
      return true;
    }

    if (close_set.count(top_node->id)) continue;
    auto road = hdmap_->GetRoadById(top_node->id);
    if (road == nullptr) continue;
    auto junction = hdmap_->GetJunctionById(road->JunctionId());
    if (junction == nullptr) close_set.insert(top_node->id);

    auto map_iter = from_to_map.find(top_node->id);
    if (map_iter == from_to_map.end()) continue;

    for (auto iter_neighbour = map_iter->second.begin();
         iter_neighbour != map_iter->second.end(); ++iter_neighbour) {
      if (!GetNodeType(iter_neighbour->first, top_node->type, type,
                       is_mix_type))
        return false;
      NaviNode next(
          iter_neighbour->first, top_node,
          top_node->cost +
              GetNodeCost(iter_neighbour->first, iter_neighbour->second.turn) +
              GetNodeTypeCost(top_node->type, type, iter_neighbour->first,
                              limit_type),
          type, is_mix_type);
      LOG_DEBUG(
          "cur id:{}, cur type:{}, next id:{}, next type:{}, is mix:{}, "
          "cost:{}",
          hdmap_->GetIdHashString(top_node->id), (int)top_node->type,
          hdmap_->GetIdHashString(iter_neighbour->first), (int)type,
          is_mix_type, next.cost);
      if (!CheckMotorwaySwitch(top_node, &next)) continue;
      if (score_map.count(next.id) == 0 || next.cost < score_map[next.id] ||
          junction != nullptr) {
        score_map[next.id] = next.cost;
        all_nodes.push_back(next);
        auto &last_push = all_nodes.back();
        // LOG_INFO("find smaller node cost:{:.4f}, open set size:{}",
        // next.cost,
        //          open_set.size());
        open_set.push(&last_push);
        LOG_DEBUG("find smaller node cost:{:.4f}, open set size:{}",
                  last_push.cost, open_set.size());
      }
    }
    if (open_set.size() >
        config.enlarge_memory_time * road_topo_->GetNodeCost().size()) {
      LOG_ERROR("open_set over size, road search failed");
      return false;
    }
  }
  return false;
}
}  // namespace planning
}  // namespace neodrive
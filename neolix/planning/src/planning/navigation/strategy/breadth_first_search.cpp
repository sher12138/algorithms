#include "breadth_first_search.h"

#include <queue>

#include "src/planning/navigation/config/navigation_config.h"

namespace neodrive {
namespace planning {
using JunctionType = autobot::cyberverse::Junction::JunctionType;

std::pair<bool, std::vector<uint64_t>> BreadthFirstSearch::Search(
    const std::vector<uint64_t> &road_seq) {
  std::pair<bool, std::vector<uint64_t>> res;
  if (!Init(road_seq)) {
    res.first = false;
    return res;
  }
  if (!BackwardUpdateRoadLaneCost(road_seq)) {
    res.first = false;
    return res;
  }
  return ForwardSearch(road_seq);
}

bool BreadthFirstSearch::Init(const std::vector<uint64_t> &road_seq) {
  if (road_seq.empty()) {
    LOG_ERROR("road_seq is empty");
    return false;
  }
  start_lane_id_ = ctx_->routing_start_lane_id;
  end_lane_id_ = ctx_->routing_end_lane_id;
  start_lane_pt_ = ctx_->routing_start_pt;
  end_lane_pt_ = ctx_->routing_end_pt;
  LOG_INFO("BreadthFirstSearch::search from {} to {}",
           hdmap_->GetIdHashString(start_lane_id_),
           hdmap_->GetIdHashString(end_lane_id_));
  Reset();
  // calculate road accumulated s
  if (!CalcAccumulatedS(road_seq, start_lane_pt_, start_lane_id_, end_lane_pt_,
                        end_lane_id_)) {
    LOG_ERROR("CalcAccumulatedS failed");
    return false;
  }
  return true;
}

bool BreadthFirstSearch::BackwardUpdateRoadLaneCost(
    const std::vector<uint64_t> &road_seq) {
  // build search map
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();

  std::vector<uint64_t> curr_lanes{end_lane_id_};
  auto road_seq_idx = road_seq.size() - 1;
  auto lane_change_dis = length_map_[road_seq[road_seq_idx]];
  int lane_chang_cnt = 0;
  backward_lane_cost_[end_lane_id_] = length_map_[road_seq[road_seq_idx]];
  road_lanes_.push_back(curr_lanes);
  road_seq_idxs_.push_back(road_seq_idx);
  PrintRoadLanesCost(road_seq, road_lanes_.size() - 1);

  TurnType turn;
  std::vector<uint64_t> prev_lanes{};
  for (int i = road_seq.size() - 1; i >= 1; --i) {
    auto curr_road = hdmap_->GetRoadById(road_seq[i]);
    auto curr_road_length = length_map_[road_seq[i]];
    LOG_DEBUG("road from {} to {}", hdmap_->GetIdHashString(road_seq[i]),
              hdmap_->GetIdHashString(road_seq[i - 1]));
    if (road_topo_->GetLinkType(road_seq[i - 1], road_seq[i], turn)) {
      std::vector<uint64_t> need_change_lanes;
      std::vector<uint64_t> link_connected_lanes;
      curr_lanes = AddRoadTurnLinkLanes(curr_lanes, turn, need_change_lanes,
                                        link_connected_lanes);
      if (!need_change_lanes.empty()) {
        lane_chang_cnt = std::floor(lane_change_dis /
                                    navigation_config.lane_change_min_distance);
        LOG_INFO("lange change: dis: {}, cnt: {}", lane_change_dis,
                 lane_chang_cnt);
        auto &curr_lane_ids = curr_road->Sections()[0]->LaneIds();
        for (auto change_to_lane : need_change_lanes) {
          backward_lane_cost_[change_to_lane] = navigation_config.max_lane_cost;
          auto change_to_lane_idx = GetLaneIdx(curr_lane_ids, change_to_lane);
          for (auto change_from_lane : link_connected_lanes) {
            auto change_from_lane_idx =
                GetLaneIdx(curr_lane_ids, change_from_lane);
            int lane_change_num =
                abs(change_to_lane_idx - change_from_lane_idx);
            if (lane_change_num <= lane_chang_cnt) {
              UpdateBackwardLaneCost(
                  change_to_lane,
                  backward_lane_cost_[change_from_lane] +
                      lane_change_num *
                          navigation_config.lane_change_min_distance *
                          navigation_config.lane_change_cost_rate);
            }
          }
        }
      }
      // forbid to change lane before intersection
      lane_change_dis =
          -1 *
          navigation_config.forbid_lane_change_distance_before_intersection;
      road_lanes_.push_back(curr_lanes);
      road_seq_idxs_.push_back(-1);
      PrintRoadLanesCost(road_seq, road_lanes_.size() - 1);
    }
    int prev_road_seq_idx = i - 1;
    auto prev_road_length = length_map_[road_seq[prev_road_seq_idx]];
    auto prev_road = hdmap_->GetRoadById(road_seq[prev_road_seq_idx]);
    lane_chang_cnt = std::floor(lane_change_dis /
                                navigation_config.lane_change_min_distance);
    LOG_INFO("lange change: dis: {}, cnt: {}", lane_change_dis, lane_chang_cnt);
    prev_lanes.clear();
    auto &prev_lane_ids = prev_road->Sections()[0]->LaneIds();
    // initialize all prev lanes' cost
    for (int j = 0; j < prev_lane_ids.size(); ++j) {
      auto prev_lane_id = prev_lane_ids[j];
      backward_lane_cost_[prev_lane_id] = navigation_config.max_lane_cost;
    }
    // update lane connect cost
    std::vector<uint64_t> link_connected_lanes;
    for (auto curr_lane : curr_lanes) {
      for (int j = 0; j < prev_lane_ids.size(); ++j) {
        auto prev_lane_id = prev_lane_ids[j];
        double bus_bay_merge_cost = 0.;
        if (lane_topo_->IsConnectLane(prev_lane_id, curr_lane)) {
          auto curr_lane_type = GetNodeType(curr_lane);
          auto prev_lane_type = GetNodeType(prev_lane_id);
          if (prev_lane_type == NodeType::BUS_BAY &&
              curr_lane_type != NodeType::BUS_BAY) {
            auto &to_from_map = lane_topo_->GetToFromMap();
            auto lane_iter = to_from_map.find(curr_lane);
            if (lane_iter != to_from_map.end() &&
                lane_iter->second.size() > 1) {
              bus_bay_merge_cost = navigation_config.bus_bay_cost;
              LOG_DEBUG("add busbay cost cur type:{}, next type:{} {:.1f}",
                        (int)curr_lane_type, (int)prev_lane_type,
                        bus_bay_merge_cost);
            }
          }
          CheckAndPushToVector(prev_lane_id, link_connected_lanes);
          UpdateBackwardLaneCost(prev_lane_id, backward_lane_cost_[curr_lane] +
                                                   prev_road_length +
                                                   bus_bay_merge_cost);
          if (CheckAndPushToVector(prev_lane_id, prev_lanes)) {
            LOG_INFO("add connect lane: {} cost: {:.1f} -> {} cost: {:.1f}",
                     hdmap_->GetIdHashString(curr_lane),
                     backward_lane_cost_[curr_lane],
                     hdmap_->GetIdHashString(prev_lane_id),
                     backward_lane_cost_[prev_lane_id]);
          }
        }
      }
    }
    // update lane change cost
    for (auto from_lane_id : link_connected_lanes) {
      auto from_lane_idx = GetLaneIdx(prev_lane_ids, from_lane_id);
      if (from_lane_idx == -1) {
        LOG_ERROR("GetLaneIdx failed: {}",
                  hdmap_->GetIdHashString(from_lane_id));
        return false;
      }
      for (int k = 0; k < prev_lane_ids.size(); ++k) {
        auto to_lane_id = prev_lane_ids[k];
        int lange_change_num = abs(k - from_lane_idx);
        if (lange_change_num > 0 && lange_change_num <= lane_chang_cnt) {
          UpdateBackwardLaneCost(
              to_lane_id, backward_lane_cost_[from_lane_id] +
                              lange_change_num *
                                  navigation_config.lane_change_min_distance *
                                  navigation_config.lane_change_cost_rate);
          CheckAndPushToVector(to_lane_id, prev_lanes);
          LOG_INFO("add lane_change lane: {} cost: {:.1f} -> {} cost: {:.1f}",
                   hdmap_->GetIdHashString(from_lane_id),
                   backward_lane_cost_[from_lane_id],
                   hdmap_->GetIdHashString(to_lane_id),
                   backward_lane_cost_[to_lane_id]);
        }
      }
    }
    // update distance to rightest lane cost
    if (prev_road_seq_idx > 0) {
      for (int j = 0; j < prev_lane_ids.size(); ++j) {
        auto prev_lane_id = prev_lane_ids[j];
        auto rightest_lane_cost =
            prev_road_length *
            navigation_config.dis_to_rightest_lane_cost_rate *
            (prev_lane_ids.size() - 1 - j);
        backward_lane_cost_[prev_lane_id] += rightest_lane_cost;
        LOG_INFO("add rightest_lane_cost: lane:{} idx:{} cost+{:.1f}={:.1f}",
                 hdmap_->GetIdHashString(prev_lane_id), j, rightest_lane_cost,
                 backward_lane_cost_[prev_lane_id]);
      }
    }
    road_lanes_.push_back(prev_lanes);
    road_seq_idxs_.push_back(prev_road_seq_idx);
    PrintRoadLanesCost(road_seq, road_lanes_.size() - 1);
    curr_lanes = prev_lanes;
    if (lane_chang_cnt >= 1) {
      lane_change_dis = prev_road_length;
    } else {
      lane_change_dis += prev_road_length;
    }
  }

  return true;
}

std::pair<bool, std::vector<uint64_t>> BreadthFirstSearch::ForwardSearch(
    const std::vector<uint64_t> &road_seq) {
  std::pair<bool, std::vector<uint64_t>> res;
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  // search forward from start_lane_id
  auto prev_lane_id = start_lane_id_;
  for (int i = road_lanes_.size() - 1; i >= 0; --i) {
    std::vector<uint64_t> connected_lanes{};
    auto road_seq_idx = road_seq_idxs_[i];
    LOG_INFO("PrintForwardLaneCost: {} {}", i, road_seq_idx);
    auto &lanes = road_lanes_[i];
    if (road_seq_idx == -1) {
      if (i == road_lanes_.size() - 1) {
        // check start_lane_id
        connected_lanes.push_back(start_lane_id_);
        forward_lane_cost_[start_lane_id_] = 0.;
      } else {
        auto from_lane_id = res.second.back();
        for (int j = 0; j < lanes.size(); ++j) {
          auto to_lane_id = lanes[j];
          if (lane_topo_->IsConnectLane(from_lane_id, to_lane_id)) {
            CheckAndPushToVector(to_lane_id, connected_lanes);
            UpdateForwardLaneCost(to_lane_id, forward_lane_cost_[from_lane_id]);
          }
        }
      }
      if (connected_lanes.empty()) {
        res.first = false;
        return res;
      }
      PrintForwardLaneCost(connected_lanes);
      auto min_cost_lane = GetMinCostLaneId(connected_lanes);
      res.second.push_back(min_cost_lane);
      continue;
    }
    auto curr_road = hdmap_->GetRoadById(road_seq[road_seq_idx]);
    auto &curr_road_lane_ids = curr_road->Sections()[0]->LaneIds();
    auto road_length = length_map_[road_seq[road_seq_idx]];
    if (i == road_lanes_.size() - 1) {
      // check start_lane_id
      connected_lanes.push_back(start_lane_id_);
      forward_lane_cost_[start_lane_id_] = 0.;
    } else {
      auto from_lane_id = res.second.back();
      auto from_lane = hdmap_->GetLaneById(from_lane_id);
      if (from_lane == nullptr) {
        LOG_ERROR("failed to find lane: {}",
                  hdmap_->GetIdHashString(from_lane_id));
        res.first = false;
        return res;
      }
      for (int j = 0; j < curr_road_lane_ids.size(); ++j) {
        auto to_lane_id = curr_road_lane_ids[j];
        if (lane_topo_->IsConnectLane(from_lane_id, to_lane_id)) {
          CheckAndPushToVector(to_lane_id, connected_lanes);
          UpdateForwardLaneCost(to_lane_id, forward_lane_cost_[from_lane_id] +
                                                from_lane->TotalLength());
        }
      }
    }
    if (connected_lanes.empty()) {
      res.first = false;
      return res;
    }
    auto prev_lane_idx = GetLaneIdx(curr_road_lane_ids, prev_lane_id);
    double min_cost = navigation_config.max_lane_cost;
    uint64_t min_cost_lane_id = 0;
    auto target_lanes = connected_lanes;
    for (auto change_from_lane : connected_lanes) {
      auto change_from_lane_idx =
          GetLaneIdx(curr_road_lane_ids, change_from_lane);
      for (auto change_to_lane : lanes) {
        auto change_to_lane_idx =
            GetLaneIdx(curr_road_lane_ids, change_to_lane);
        int lane_change_num = abs(change_from_lane_idx - change_to_lane_idx);
        if (lane_change_num > 0) {
          double forward_cost = forward_lane_cost_[change_from_lane] +
                                lane_change_num *
                                    navigation_config.lane_change_min_distance *
                                    navigation_config.lane_change_cost_rate;
          UpdateForwardLaneCost(change_to_lane, forward_cost);
          CheckAndPushToVector(change_to_lane, target_lanes);
        }
      }
    }

    PrintForwardLaneCost(target_lanes);
    auto min_cost_lane = GetMinCostLaneId(target_lanes);
    res.second.push_back(min_cost_lane);
  }
  LOG_DEBUG("finish push, size: {}", res.second.size());
  res.first = true;
  PrintResultLanes(res.second);
  return res;
}

void BreadthFirstSearch::Reset() {
  accumulated_s_.clear();
  length_map_.clear();
  backward_lane_cost_.clear();
  forward_lane_cost_.clear();
  road_lanes_.clear();
  road_seq_idxs_.clear();
  routing_lanes_map_.clear();
  predecessor_lanes_map_.clear();
  cost_map_.clear();
}

double BreadthFirstSearch::GetCurLaneS(cyberverse::LaneInfoConstPtr lane,
                                       double x, double y) {
  double s, l;
  lane->GetProjection({x, y}, &s, &l);
  return s;
}

bool BreadthFirstSearch::CalcAccumulatedS(const std::vector<uint64_t> &road_seq,
                                          const Vec3d &start,
                                          uint64_t start_lane_id,
                                          const Vec3d &end,
                                          uint64_t end_lane_id) {
  LOG_INFO("get in CalcAccumulatedS");
  accumulated_s_.clear();
  length_map_.clear();
  auto data_center = DataCenter::Instance();
  for (size_t i = 0; i < road_seq.size(); ++i) {
    double length = 0.;
    if (i == 0) {
      auto lane = hdmap_->GetLaneById(start_lane_id);
      if (lane == nullptr) {
        LOG_INFO("find null start lane, quit!");
        return false;
      }
      length = lane->TotalLength() - GetCurLaneS(lane, start.x(), start.y());
      length_map_[road_seq[i]] = length;
      accumulated_s_[road_seq[i]] = length;
      LOG_INFO("id:{}, start lane s:[{}/{}], accu s:{}",
               hdmap_->GetIdHashString(start_lane_id), length,
               lane->TotalLength(), accumulated_s_[road_seq[i]]);
    } else if (i + 1 == road_seq.size()) {
      auto lane = hdmap_->GetLaneById(end_lane_id);
      if (lane == nullptr) {
        LOG_INFO("find null end lane, quit!");
        return false;
      }
      length = GetCurLaneS(lane, end.x(), end.y());
      length_map_[road_seq[i]] = length;
      accumulated_s_[road_seq[i]] = accumulated_s_[road_seq[i - 1]] + length;
      LOG_INFO("id:{}, end lane s:[{}/{}] accu s:{}",
               hdmap_->GetIdHashString(start_lane_id), length,
               lane->TotalLength(), accumulated_s_[road_seq[i]]);
    } else {
      length = road_topo_->GetRoadCost(road_seq[i]);
      length_map_[road_seq[i]] = length;
      accumulated_s_[road_seq[i]] = accumulated_s_[road_seq[i - 1]] + length;
      LOG_INFO("i:{}, len: {} accu s:{}", i, length,
               accumulated_s_[road_seq[i]]);
    }
  }
  LOG_INFO("accumulated_s_ size:{}", accumulated_s_.size());
  return true;
}

void BreadthFirstSearch::UpdateBackwardLaneCost(const uint64_t lane_id,
                                                const double cost) {
  auto lane_it = backward_lane_cost_.find(lane_id);
  if (lane_it == backward_lane_cost_.end()) {
    backward_lane_cost_[lane_id] = cost;
  } else {
    backward_lane_cost_[lane_id] = std::min(lane_it->second, cost);
  }
}

void BreadthFirstSearch::UpdateForwardLaneCost(const uint64_t lane_id,
                                               const double cost) {
  auto lane_it = forward_lane_cost_.find(lane_id);
  if (lane_it == forward_lane_cost_.end()) {
    forward_lane_cost_[lane_id] = cost;
  } else {
    forward_lane_cost_[lane_id] = std::min(lane_it->second, cost);
  }
}

void BreadthFirstSearch::PrintRoadLanesCost(
    const std::vector<uint64_t> &road_seq, const size_t idx) {
  if (idx < 0 || idx >= road_lanes_.size()) {
    return;
  }
  auto &lane_ids = road_lanes_[idx];
  auto road_seq_idx = road_seq_idxs_[idx];
  if (road_seq_idx != -1) {
    auto curr_road = hdmap_->GetRoadById(road_seq[road_seq_idx]);
    auto &curr_lane_ids = curr_road->Sections()[0]->LaneIds();
    for (auto lane_id : lane_ids) {
      auto lane_id_idx = GetLaneIdx(curr_lane_ids, lane_id);
      LOG_INFO("road_seq_idx: {} lane_id: {} idx: {}, cost: {:.1f}",
               road_seq_idx, hdmap_->GetIdHashString(lane_id), lane_id_idx,
               backward_lane_cost_[lane_id]);
    }
  } else {
    for (auto lane_id : lane_ids) {
      LOG_INFO("road_seq_idx: {} lane_id: {}, cost: {:.1f}", road_seq_idx,
               hdmap_->GetIdHashString(lane_id), backward_lane_cost_[lane_id]);
    }
  }
}

void BreadthFirstSearch::PrintResultLanes(
    const std::vector<uint64_t> &res_lanes) {
  LOG_INFO("PrintResultLanes: size: {}", res_lanes.size());
  for (auto lane_id : res_lanes) {
    auto lane_id_idx = GetLaneIdx(lane_id);
    LOG_INFO("lane_id: {} idx: {} cost: {:.1f} = {:.1f} + {:.1f}",
             hdmap_->GetIdHashString(lane_id), lane_id_idx,
             backward_lane_cost_[lane_id] + forward_lane_cost_[lane_id],
             backward_lane_cost_[lane_id], forward_lane_cost_[lane_id]);
  }
}

void BreadthFirstSearch::PrintBackwardLaneCost(
    const std::vector<uint64_t> &lane_ids) {
  for (auto lane_id : lane_ids) {
    auto lane_id_idx = GetLaneIdx(lane_id);
    LOG_INFO("lane_id: {} idx: {} cost: {:.1f}",
             hdmap_->GetIdHashString(lane_id), lane_id_idx,
             backward_lane_cost_[lane_id]);
  }
}

void BreadthFirstSearch::PrintForwardLaneCost(
    const std::vector<uint64_t> &lane_ids) {
  for (auto lane_id : lane_ids) {
    auto lane_id_idx = GetLaneIdx(lane_id);
    LOG_INFO("lane_id: {} idx: {} cost: {:.1f}",
             hdmap_->GetIdHashString(lane_id), lane_id_idx,
             forward_lane_cost_[lane_id]);
  }
}

std::vector<uint64_t> BreadthFirstSearch::AddRoadTurnLinkLanes(
    const std::vector<uint64_t> &curr_lanes, const TurnType turn,
    std::vector<uint64_t> &need_change_lanes,
    std::vector<uint64_t> &link_connected_lanes) {
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  LOG_INFO("AddRoadTurnLinkLanes");
  std::vector<uint64_t> all_prev_lanes;
  const auto &to_from_map = lane_topo_->GetToFromMap();
  for (auto curr_lane : curr_lanes) {
    std::vector<uint64_t> turn_lanes;
    auto to_from_iter = to_from_map.find(curr_lane);
    if (to_from_iter != to_from_map.end()) {
      for (auto iter = to_from_iter->second.begin();
           iter != to_from_iter->second.end(); ++iter) {
        auto prev_lane = hdmap_->GetLaneById(*iter);
        auto prev_lane_id = prev_lane->Id();
        if (prev_lane->TurnType() == turn) {
          CheckAndPushToVector(prev_lane_id, all_prev_lanes);
          CheckAndPushToVector(prev_lane_id, turn_lanes);
          UpdateBackwardLaneCost(prev_lane_id, backward_lane_cost_[curr_lane]);
          LOG_INFO("add link connect: {} -> {}",
                   hdmap_->GetIdHashString(curr_lane),
                   hdmap_->GetIdHashString(prev_lane_id));
        } else {
          backward_lane_cost_[prev_lane_id] = navigation_config.max_lane_cost;
        }
      }
    }
    if (turn_lanes.empty()) {
      LOG_INFO("lane {} not connect to turn link",
               hdmap_->GetIdHashString(curr_lane));
      CheckAndPushToVector(curr_lane, need_change_lanes);
    } else {
      CheckAndPushToVector(curr_lane, link_connected_lanes);
    }
    predecessor_lanes_map_[curr_lane] = turn_lanes;
  }
  return all_prev_lanes;
}

int BreadthFirstSearch::GetLaneIdx(common::math::ShmVector<uint64_t> lane_ids,
                                   const uint64_t lane_id) {
  for (int i = 0; i < lane_ids.size(); ++i) {
    if (lane_ids[i] == lane_id) {
      return i;
    }
  }
  return -1;
}

int BreadthFirstSearch::GetLaneIdx(const uint64_t lane_id) {
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) {
    return -1;
  }
  auto road = hdmap_->GetRoadById(lane->RoadId());
  if (road == nullptr) {
    return -1;
  }
  auto lane_ids = road->Sections()[0]->LaneIds();
  return GetLaneIdx(lane_ids, lane_id);
}

bool BreadthFirstSearch::CheckAndPushToVector(
    const uint64_t val, std::vector<uint64_t> &target_vec) {
  auto iter = std::find(target_vec.begin(), target_vec.end(), val);
  if (iter == target_vec.end()) {
    target_vec.push_back(val);
    return true;
  }
  return false;
}

BreadthFirstSearch::NodeType BreadthFirstSearch::GetNodeType(
    const uint64_t id) {
  bool is_motorway = false, is_biking = false, is_bus_bay = false;

  auto lane = hdmap_->GetLaneById(id);
  auto lane_multiple_type = lane->LaneMultipleType();
  for (int j = 0; j < lane_multiple_type.size(); ++j) {
    if (lane_multiple_type[j] == global::hdmap::Lane::CITY_DRIVING)
      is_motorway = true;
    if (lane_multiple_type[j] == global::hdmap::Lane::BIKING) is_biking = true;
    if (lane_multiple_type[j] == global::hdmap::Lane::BUS_BAY_LANE)
      is_bus_bay = true;
  }

  // LOG_DEBUG("id:{}, motor:{}, biking:{}", hdmap_->GetIdHashString(id),
  //           is_motorway, is_biking);
  if (is_bus_bay) return NodeType::BUS_BAY;
  if (is_biking) return NodeType::BIKING;
  return NodeType::MOTORWAY;
}

uint64_t BreadthFirstSearch::GetMinCostLaneId(std::vector<uint64_t> &lane_ids) {
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  int min_cost = navigation_config.max_lane_cost;
  uint64_t min_cost_lane_id = 0;
  for (auto lane_id : lane_ids) {
    auto lane_cost = backward_lane_cost_[lane_id] + forward_lane_cost_[lane_id];
    if (lane_cost < min_cost) {
      min_cost = lane_cost;
      min_cost_lane_id = lane_id;
    }
  }
  return min_cost_lane_id;
}

}  // namespace planning
}  // namespace neodrive
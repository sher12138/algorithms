#pragma once
#include <stdint.h>
#include <vector>
#include "common/navigation_context.h"
#include "hdmap/hdmap.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "hdmap/topo_map/road_topomap.h"

namespace neodrive {
namespace planning {
class AStarRoadSearch {
  typedef global::hdmap::Lane::LaneTurn TurnType;
  enum class NodeType { MIX = 0, MOTORWAY = 1, BIKING = 2, BUS_BAY = 3 };
  struct NaviNode {
    NaviNode(uint64_t id_in, NaviNode *parent_in, double cost_in,
             NodeType type_in, bool is_mix_type_in)
        : id(id_in),
          parent(parent_in),
          cost(cost_in),
          type(type_in),
          is_mix_type(is_mix_type_in) {}
    uint64_t id{0};
    NaviNode *parent{nullptr};
    double cost{0.0};
    NodeType type{NodeType::MIX};
    bool is_mix_type{true};
  };
  struct Compair {
    bool operator()(const NaviNode *left, const NaviNode *right) {
      return left->cost > right->cost;
    }
  };

 public:
  AStarRoadSearch(NavigationContext *ctx)
      : ctx_(ctx),
        hdmap_(ctx->hdmap),
        road_topo_(ctx->road_topo),
        lane_topo_(ctx->lane_topo) {}
  bool Search(const uint64_t start, const uint64_t end,
              std::vector<uint64_t> &res);

 private:
  bool GetNodeType(const uint64_t id, const NodeType &parent_type,
                   NodeType &node_type, bool &is_mix_type);
  double GetNodeCost(const uint64_t id,
                     const global::hdmap::Lane::LaneTurn turn_type);
  double GetNodeTypeCost(const NodeType &parent, const NodeType &child,
                         uint64_t child_id, std::string type_limit);
  bool CheckMotorwaySwitch(const NaviNode *parent, NaviNode *child);
  std::vector<uint64_t> GetTurnLanes(
      const std::vector<uint64_t> &curr_lanes, TurnType turn,
      std::unordered_map<uint64_t, std::vector<uint64_t>> &lanes_map);
  bool IsConnected(
      const uint64_t start, const uint64_t end,
      const std::unordered_map<uint64_t, std::vector<uint64_t>> &lanes_map);
  bool IsMotorwayLane(uint64_t lane_id);
  uint64_t GetLaneId(cyberverse::RoadInfoConstPtr road, NodeType type);
  bool GetLaneIds(cyberverse::RoadInfoConstPtr road, NodeType type,
                  std::vector<uint64_t> &res);

 private:
  cyberverse::HDMap *hdmap_{nullptr};
  cyberverse::RoadTopo *road_topo_{nullptr};
  cyberverse::LaneTopo *lane_topo_{nullptr};
  NavigationContext *ctx_{nullptr};
};
}  // namespace planning
}  // namespace neodrive
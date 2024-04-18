#pragma once

#include <stdint.h>

#include <unordered_map>
#include <vector>

#include "common/data_center/data_center.h"
#include "common/navigation_context.h"
#include "hdmap/hdmap.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "hdmap/topo_map/road_topomap.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {
enum class NodeType { MOTORWAY = 0, BIKING = 1, BUS_BAY = 2 };
struct LaneNode {
  LaneNode(uint64_t id_in, LaneNode *parent_in, double cost_in,
           bool is_change_in, double lane_change_dist_cost_in)
      : id(id_in),
        parent(parent_in),
        cost(cost_in),
        is_change(is_change_in),
        lane_change_dist_cost(lane_change_dist_cost_in) {}
  uint64_t id{0};
  LaneNode *parent{nullptr};
  double cost{0.0};
  bool is_change{false};
  double lane_change_dist_cost{0.};
};
struct Compair {
  bool operator()(const LaneNode *left, const LaneNode *right) {
    return left->cost + left->lane_change_dist_cost >
           right->cost + right->lane_change_dist_cost;
  }
};
class AStarLaneSearch {
  typedef global::hdmap::Lane::LaneTurn TurnType;

 public:
  AStarLaneSearch(NavigationContext *ctx)
      : ctx_(ctx),
        hdmap_(ctx->hdmap),
        road_topo_(ctx->road_topo),
        lane_topo_(ctx->lane_topo) {}
  std::pair<bool, std::vector<uint64_t>> Search(
      const std::vector<uint64_t> &road_seq) {}

  /// initialize lane map for Dijkstra
  /// @param end destination of routing (start of search)
  /// @param road_seq road routing
  /// @return if initialize successfully
  bool Init(const uint64_t end, const std::vector<uint64_t> &road_seq);

  /// search for optimal lane seq from destination to all lanes
  /// @param start start of search(routing destination lane)
  /// @param end end of search(routing start lane)
  /// @return if Dijkstra successfully
  bool Search(const uint64_t start, const uint64_t end);

  /// search for optimal lane seq of BIKING lanes
  /// @param start start of search(routing destination lane)
  /// @param end end of search(routing start lane)
  /// @return if Dijkstra successfully
  bool BikingSearch(const uint64_t start, const uint64_t end);

  /// get optimal lane seq from start to routing destination
  /// @param start start of routing
  /// @return lane seq from start to routing destination
  std::vector<uint64_t> GetRoutingLanes(uint64_t start);
  void VisLaneNode(std::string name);

  static constexpr double kDegree2Radian = M_PI / 180.0;

 private:
  void Reset();
  bool CalcAccumulatedS(const Vec3d start,
                        const std::vector<uint64_t> &road_seq);
  std::vector<uint64_t> GetTurnLanes(const std::vector<uint64_t> &curr_lanes,
                                     TurnType turn);
  int GetLaneChangeCnt(uint64_t cur, uint64_t next);
  int GetLaneChangeDirection(uint64_t cur, uint64_t next, TurnType &turn);
  bool CheckLaneChange(uint64_t next_id, const LaneNode *node, bool &is_change,
                       double &lane_change_dist_cost,
                       double &lane_change_cnt_cost);
  void SetCost(uint64_t cur, uint64_t next, bool is_link);
  double GetCloudNaviMotorwayCost(NodeType type);
  double GetCost(uint64_t cur, uint64_t next);
  NodeType GetNodeType(const uint64_t id);
  bool CheckMergeIn(const uint64_t cur, const uint64_t next);
  void SaveLanePath(LaneNode *node);

 private:
  std::unordered_map<uint64_t, double> accumulated_s_;
  std::unordered_map<uint64_t, double> score_map_;
  std::unordered_map<uint64_t, std::vector<uint64_t>> routing_lanes_map_;
  std::unordered_map<uint64_t, std::vector<uint64_t>> lanes_map_;
  std::unordered_map<uint64_t, std::unordered_map<uint64_t, double>> cost_map_;
  std::unordered_set<uint64_t> link_lanes_;
  cyberverse::HDMap *hdmap_{nullptr};
  cyberverse::RoadTopo *road_topo_{nullptr};
  cyberverse::LaneTopo *lane_topo_{nullptr};
  std::string limit_type_{"0"};
  NavigationContext *ctx_{nullptr};
};
}  // namespace planning
}  // namespace neodrive
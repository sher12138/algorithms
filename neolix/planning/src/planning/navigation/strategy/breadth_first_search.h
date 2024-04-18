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
class BreadthFirstSearch {
  typedef global::hdmap::Lane::LaneTurn TurnType;
  enum class NodeType { MOTORWAY = 0, BIKING = 1, BUS_BAY = 2 };

 public:
  BreadthFirstSearch(NavigationContext *ctx)
      : ctx_(ctx),
        hdmap_(ctx->hdmap),
        road_topo_(ctx->road_topo),
        lane_topo_(ctx->lane_topo) {}
  /// search for optimal lane seq from destination to all lanes
  /// @param start start of search(routing destination lane)
  /// @param end end of search(routing start lane)
  /// @return if Dijkstra successfully
  std::pair<bool, std::vector<uint64_t>> Search(
      const std::vector<uint64_t> &road_seq);

 private:
  /// initialize lane map for Dijkstra
  /// @param end destination of routing (start of search)
  /// @param road_seq road routing
  /// @return if initialize successfully
  bool Init(const std::vector<uint64_t> &road_seq);
  bool BackwardUpdateRoadLaneCost(const std::vector<uint64_t> &road_seq);
  std::pair<bool, std::vector<uint64_t>> ForwardSearch(
      const std::vector<uint64_t> &road_seq);
  void Reset();
  double GetCurLaneS(cyberverse::LaneInfoConstPtr lane, double x, double y);
  bool CalcAccumulatedS(const std::vector<uint64_t> &road_seq,
                        const Vec3d &start, uint64_t start_lane_id,
                        const Vec3d &end, uint64_t end_lane_id);
  void UpdateBackwardLaneCost(const uint64_t lane_id, const double cost);
  void UpdateForwardLaneCost(const uint64_t lane_id, const double cost);
  void PrintRoadLanesCost(const std::vector<uint64_t> &road_seq,
                          const size_t idx);
  void PrintResultLanes(const std::vector<uint64_t> &res_lanes);
  void PrintBackwardLaneCost(const std::vector<uint64_t> &lane_ids);
  void PrintForwardLaneCost(const std::vector<uint64_t> &lane_ids);
  std::vector<uint64_t> AddRoadTurnLinkLanes(
      const std::vector<uint64_t> &curr_lanes, const TurnType turn,
      std::vector<uint64_t> &need_change_lanes,
      std::vector<uint64_t> &link_connected_lanes);
  int GetLaneIdx(common::math::ShmVector<uint64_t> lane_ids,
                 const uint64_t lane_id);
  int GetLaneIdx(const uint64_t lane_id);
  bool CheckAndPushToVector(const uint64_t val,
                            std::vector<uint64_t> &target_vec);
  NodeType GetNodeType(const uint64_t id);
  uint64_t GetMinCostLaneId(

      std::vector<uint64_t> &lane_ids);

 private:
  uint64_t start_lane_id_, end_lane_id_;
  Vec3d start_lane_pt_{};
  Vec3d end_lane_pt_{};
  std::unordered_map<uint64_t, double> accumulated_s_;
  std::unordered_map<uint64_t, double> length_map_;
  std::unordered_map<uint64_t, double> backward_lane_cost_;
  std::unordered_map<uint64_t, double> forward_lane_cost_;
  std::vector<std::vector<uint64_t>> road_lanes_;
  std::vector<int> road_seq_idxs_;
  std::unordered_map<uint64_t, std::vector<uint64_t>> routing_lanes_map_;
  std::unordered_map<uint64_t, std::vector<uint64_t>> predecessor_lanes_map_;
  std::unordered_map<uint64_t, std::unordered_map<uint64_t, double>> cost_map_;
  cyberverse::HDMap *hdmap_{nullptr};
  cyberverse::RoadTopo *road_topo_{nullptr};
  cyberverse::LaneTopo *lane_topo_{nullptr};
  NavigationContext *ctx_{nullptr};
};
}  // namespace planning
}  // namespace neodrive
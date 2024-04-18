#pragma once
#include <string>

#include "common/data_center/data_center.h"
#include "common/navigation_types.h"
#include "cyber/common/memory_pool.h"
#include "hdmap/hdmap.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "hdmap/topo_map/road_topomap.h"
#include "navigation_base.h"
#include "reference_line/refe_generator.h"
#include "strategy/a_star_lane_search.h"

namespace neodrive {
namespace planning {

class AStarNavigationProcess : public NavigationProcessBase {
  using AD3 = std::array<double, 3>;

 public:
  AStarNavigationProcess(NavigationContext *ctx)
      : NavigationProcessBase(ctx), a_star_lane_search_(ctx) {}
  bool Init();
  void Process(const std::shared_ptr<RoutingRequest> &routing_request,
               std::shared_ptr<RoutingResult> &routing_respons);
  void RunOnce();
  bool ComputeReferenceLine(
      std::shared_ptr<RefeLineGenerator> &current_ref_generator,
      std::shared_ptr<RefeLineGenerator> &target_ref_generator);

 private:
  std::vector<uint64_t> GetNextLanes(uint64_t lane_id, uint64_t road_id);
  std::vector<uint64_t> GetTurnNextLanes(
      uint64_t lane_id, std::vector<size_t> road_seq,
      const global::hdmap::Lane::LaneTurn &turn, int i, bool main_loop);
  bool FindLanePath(int i, uint64_t last_lane_id, uint64_t goal_lane_id,
                    std::vector<size_t> road_seq,
                    std::vector<std::pair<int, uint64_t>> &res);
  std::vector<uint64_t> SearchLaneChangeLaneSeq(
      uint64_t start, uint64_t end, const std::vector<uint64_t> &road_seq);
  bool GetRoutingFromRoadSeq(std::shared_ptr<RoutingResult> &routing_respons);
  bool Renavigation();
  bool FindNextWayPointIdx(
      const std::vector<cyberverse::LaneInfoConstPtr> &origin_lane_seq,
      int &cur_idx, int &next_keypoint_idx);
  uint64_t FindLaneIdByWaypointIdx(int idx);
  void ExtractRoadSeq(int start_lane_idx, uint64_t target_lane_id,
                      std::vector<uint64_t> &road_seq);
  void GenerateNewRoutingRequest(const AD3 ego_pose, int idx,
                                 std::shared_ptr<RoutingRequest> &new_request);
  void CheckLaneChange();
  void CreateLaneChangeInfo();
  void CreateManualLaneChangeInfo();
  bool CreatePlanningLaneChangeInfo();
  void UpdateLaneChangeLaneSeq();
  void PostProcessLaneChangeInfo();
  double CalcDistBetweenPoints(
      int next_keypoint_idx, int idx,
      const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq);

 private:
  std::vector<cyberverse::LaneInfoConstPtr> lane_seq_after_lane_change_;
  AStarLaneSearch a_star_lane_search_;
  int lane_change_finish_cnt_{0};
  std::vector<std::pair<int, int>> lane_cnt_{};
};
}  // namespace planning
}  // namespace neodrive

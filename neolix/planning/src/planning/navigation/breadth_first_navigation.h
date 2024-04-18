#pragma once
#include <string>

#include "navigation_base.h"
#include "strategy/breadth_first_search.h"

namespace neodrive {
namespace planning {

class BreadthFirstNavigationProcess : public NavigationProcessBase {
 public:
  BreadthFirstNavigationProcess(NavigationContext *ctx)
      : NavigationProcessBase(ctx), breadth_first_search_(ctx) {}
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
  bool GetRoutingFromRoadSeq(std::shared_ptr<RoutingResult> &routing_respons);
  bool CalcLaneChangeInfo();
  bool UpdateLaneChangeInfo();
  void CheckLaneChange();
  void CreateLaneChangeInfo();
  void UpdateLaneChangeLaneSeq();

 private:
  std::vector<cyberverse::LaneInfoConstPtr> routing_lane_seq_;
  std::vector<cyberverse::LaneInfoConstPtr> lane_seq_after_lane_change_;
  int lane_change_finish_cnt_{0};
  std::vector<std::pair<int, int>> lane_cnt_{};
  BreadthFirstSearch breadth_first_search_;
  bool is_changing_lane_{false};
  uint32_t target_change_lane_ids_idx_{
      0};  // target lane ids index in current road
  global::hdmap::Lane::LaneTurn prev_lane_change_direct_{
      global::hdmap::Lane::LaneTurn::Lane_LaneTurn_NO_TURN};
};
}  // namespace planning
}  // namespace neodrive

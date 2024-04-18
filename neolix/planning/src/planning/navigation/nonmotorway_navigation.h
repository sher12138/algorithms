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

class NonMotorwayNavigationProcess : public NavigationProcessBase {
 public:
  NonMotorwayNavigationProcess(NavigationContext *ctx)
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
  bool GetRoutingFromRoadSeq(std::shared_ptr<RoutingResult> &routing_respons);
  bool GetRoutingFromBikingLane(std::shared_ptr<RoutingResult> &routing_respons,
                                std::vector<uint64_t> &res);

 private:
  AStarLaneSearch a_star_lane_search_;
  std::vector<std::pair<int, int>> lane_cnt_{};
};
}  // namespace planning
}  // namespace neodrive

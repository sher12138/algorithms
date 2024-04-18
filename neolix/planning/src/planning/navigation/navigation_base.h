#pragma once
#include <string>

#include "common/data_center/data_center.h"
#include "common/navigation_types.h"
#include "cyber/common/memory_pool.h"
#include "hdmap/hdmap.h"
#include "hdmap/hdmap_cloud.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "hdmap/topo_map/lane_topomap_cloud.h"
#include "hdmap/topo_map/road_topomap.h"
#include "hdmap/topo_map/road_topomap_cloud.h"
#include "reference_line/refe_generator.h"
#include "strategy/a_star_road_search.h"

namespace neodrive {
namespace planning {

enum class NavigationStrategy : int { ASTAR = 0, BREADTH_FIRST = 1 };

class NavigationProcessBase {
  using AD3 = std::array<double, 3>;

 public:
  NavigationProcessBase(NavigationContext *ctx)
      : ctx_(ctx),
        hdmap_(ctx->hdmap),
        road_topo_(ctx->road_topo),
        lane_topo_(ctx->lane_topo) {
    a_star_road_search_ = std::make_shared<AStarRoadSearch>(ctx);
  }
  virtual bool Init() = 0;
  void ProcessIntegratedNavigation();
  void ProcessRoadSeqRequest(std::shared_ptr<RoutingRequest> &routing_request);
  void ProcessKeyPointSeqRequest();
  virtual void Process(const std::shared_ptr<RoutingRequest> &routing_request,
                       std::shared_ptr<RoutingResult> &routing_respons) = 0;
  virtual void RunOnce() = 0;
  virtual bool ComputeReferenceLine(
      std::shared_ptr<RefeLineGenerator> &current_ref_generator,
      std::shared_ptr<RefeLineGenerator> &target_ref_generator) = 0;
  bool GenerateKeyWayPoints(
      const std::shared_ptr<RoutingRequest> &routing_request,
      std::shared_ptr<RoutingResult> &routing_respons);
  void CheckMapPointZLegality(LaneWaypoint &point);
  std::string GetPointMotorwayType(double x, double y, double z);

  static constexpr double kDefaultRadius = 3.0;
  static constexpr double kMaxZDiffThreshold = 2.0;
  static constexpr double kMaxGetLaneRadius = 6.0;
  static constexpr double kDegree2Radian = M_PI / 180.0;

 protected:
  virtual bool GetRoutingFromRoadSeq(
      std::shared_ptr<RoutingResult> &routing_respons) = 0;
  void InitBase();
  bool GenerateLaneSequenceFromRoadSequence(
      const std::shared_ptr<RoutingRequest> &routing_request, int start_idx,
      int end_idx, std::shared_ptr<RoutingResult> &routing_respons);
  void UpdateAccumulatedS();
  bool UpdateEgoPose();
  bool SearchRoutingRoadSequence(
      const std::shared_ptr<RoutingRequest> &routing_request, int start_idx,
      int end_idx);
  std::vector<uint64_t> GetRoad(const AD3 ego_pose, bool find_pre);
  bool GetRoadSeqFromKeyPoints(const RoutingRequest::KeyPoints &key_pts,
                               std::unordered_set<uint64_t> &road_seq);
  std::string GetRoadType(uint64_t road_id);
  void GeneratePathPoints(std::shared_ptr<RoutingResult> &routing_respons);
  cyberverse::LaneInfoConstPtr GetRoadLane(const uint64_t road_id,
                                           common::math::Vec2d pt);
  std::vector<cyberverse::LaneInfoConstPtr> GetRoadLanes(
      const uint64_t road_id, common::math::Vec2d pt);
  int GetNearestLaneWithHeading(
      const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq, double x,
      double y);
  bool CheckLaneHeadingValidity(const cyberverse::LaneInfoConstPtr &lane);
  void CalcLaneSequenceIndex(
      const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq);
  void CheckTakeOverReplan();
  void UpdateLaneSequenceIndex(
      const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq);
  double GetLaneS(const cyberverse::LaneInfoConstPtr &lane, double x, double y);

  int GetIterIndex(const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq,
                   const cyberverse::LaneInfoConstPtr &lane_info,
                   double &current_s);
  int GetRoadLaneIdx(const uint64_t lane_id);
  int GetSeqLaneIdx(const uint64_t lane_id);
  void DFSTraverse(const uint64_t &end_id, std::vector<uint64_t> &path,
                   std::unordered_set<uint64_t> visted_road, double cur_length,
                   double fit_length, double continuous_unfit_length,
                   double &res_length, double &max_fit_rate);
  std::vector<uint64_t> PostProcessRoadSeq(
      const std::vector<uint64_t> &road_ids);
  static TrajectoryPoint GetTrajectoryPoint(
      const VehicleStateProxy &vehicle_state_proxy);
  void CheckTurnHornLights();
  bool CheckCrossroadTurn(double turn_light_check_length,
                          HornLightsCmd::TurnLevel &light_turn);
  bool CheckFrontMergeIn(double turn_light_check_length,
                         HornLightsCmd::TurnLevel &light_turn);
  bool CheckMergeIn(HornLightsCmd::TurnLevel &light_turn, int lane_idx);
  std::vector<uint64_t> FindShortestLaneSeq(
      const std::vector<std::vector<uint64_t>> &lane_seqs, const Vec3d &start,
      const Vec3d &end);
  std::vector<uint64_t> FindShortestRoadSeq(
      const std::vector<std::vector<uint64_t>> &road_seqs, const Vec3d &start,
      const Vec3d &end);
  bool CheckIsOnRoute();
  bool CheckIsOnRouteWithBaseLink();
  bool CheckIsOnLaneChangeRoute();
  void CalculateMileage();
  void AddRoute(std::shared_ptr<RoutingResult> &routing_respons,
                const cyberverse::LaneInfoConstPtr &lane, uint64_t road_id,
                Vec3d *start = nullptr, Vec3d *end = nullptr);
  void CorrectWaypoint(LaneWaypoint &pt);
  bool CheckPtOrderOnLane(uint64_t id, const Vec3d &start, const Vec3d &end);

 protected:
  std::atomic<bool> is_inited_{false};
  static constexpr double kLaneEndDist = 0.3;
  planning::VehicleStateProxy ego_pose_{};
  int current_lane_idx_{-1};          // lane index in routing requence lanes
  int prev_lane_idx_{-1};             // lane index in routing requence lanes
  uint32_t current_lane_ids_idx_{0};  // lane ids index in current road
  uint32_t target_lane_ids_idx_{0};   // target lane ids index in current road
  uint32_t last_request_seq_{0};
  cyberverse::HDMap *hdmap_{nullptr};
  cyberverse::RoadTopo *road_topo_{nullptr};
  cyberverse::LaneTopo *lane_topo_{nullptr};
  std::vector<double> accumulated_s_;
  neodrive::planning::DataCenter *data_center_{nullptr};
  std::shared_ptr<AStarRoadSearch> a_star_road_search_{nullptr};
  int last_manual_lane_change_order_{0};
  double routing_destination_s_{0.};
  std::unordered_map<uint64_t, int> lane_keypoint_map_{};
  // cloud navigation
  std::vector<std::vector<uint64_t>> cloud_search_paths_{};
  std::unordered_set<uint64_t> cloud_road_set_{};
  std::unordered_set<uint64_t> fully_search_road_set_{};

  NavigationContext *ctx_{nullptr};
};
}  // namespace planning
}  // namespace neodrive

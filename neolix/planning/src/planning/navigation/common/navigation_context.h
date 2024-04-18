#pragma once

#include "common/message_util.h"
#include "hdmap/hdmap.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "hdmap/topo_map/road_topomap.h"
#include "navigation_swap_context.h"
#include "navigation_types.h"

namespace neodrive {
namespace planning {
class NavigationContext {
  // DECLARE_SINGLETON(NavigationContext)
 public:
  typedef global::hdmap::Lane::LaneTurn TurnType;
  typedef HornLightsCmd::TurnLevel TurnLevel;
  typedef HornLightsCmd::BeamLevel BeamLevel;

 public:
  NavigationContext() = default;
  ~NavigationContext() = default;

  void Reset();
  void ResetLaneChangeContext();

  std::vector<uint64_t> road_seq;
  std::vector<uint64_t> last_road_seq;
  std::vector<cyberverse::LaneInfoConstPtr> lane_seq;
  std::vector<cyberverse::LaneInfoConstPtr> lane_change_seq;
  std::unordered_set<uint64_t> lane_set;
  double route_length{0.0};
  double current_s{0.0};
  bool is_lane_change_point_updated{false};
  bool is_manual_lane_change_updated{false};
  bool need_lanechange{false};
  double lane_change_point_s{0.0};
  double min_lane_change_s{0.0};
  TurnType lane_change_direct;
  TurnLevel light_turn{HornLightsCmd::NO_TURN};
  bool first_finish_lane_change{false};
  bool is_lane_change_ref{false};
  bool is_planning_lane_change{false};
  cyberverse::LaneInfoConstPtr routing_start_lane{nullptr};
  cyberverse::LaneInfoConstPtr routing_end_lane{nullptr};
  Vec3d routing_start_pt{};
  Vec3d routing_end_pt{};
  uint64_t routing_start_lane_id{};
  uint64_t routing_end_lane_id{};
  Vec3d ego_pose{};
  std::string start_type{"0"};
  std::string end_type{"0"};
  bool is_on_route{true};
  bool is_sim{false};
  double last_lane_change_finish_s{0.0};
  bool need_reset_ref_generator{false};
  int current_lane_idx{0};
  bool manual_trigger_renavigation{false};
  std::shared_ptr<RoutingRequest> routing_request;
  Vec3d lane_change_end_point{};
  double dist_to_lane_change_end{0.0};
  double lane_change_end_point_s{0.0};
  std::unordered_map<uint64_t, std::pair<bool, double>>
      one_road_lane_change_s_map{};
  bool renavigation_process{false};

  ReferenceLinePtr current_utm_ref{nullptr};
  ReferenceLinePtr target_utm_ref{nullptr};
  bool is_swap_ref_line{false};
  NavigatorLaneChangeRequest navigator_request{
      NavigatorLaneChangeRequest::NO_REQUEST};
  NavigatorLaneChangeRequest preview_navigator_request{
      NavigatorLaneChangeRequest::NO_REQUEST};
  cyberverse::ParkingSpaceInfoConstPtr current_parking_space{nullptr};
  common::math::Vec2d park_out_start_pt{};
  std::unordered_set<uint64_t> ban_lane_set{};
  std::unordered_set<uint64_t> planning_ban_lane_set{};

  // context for cloud navigation
  std::vector<std::vector<common::math::Vec2d>> motorway_pts_vec{};
  std::vector<std::vector<uint64_t>> road_seqs{};
  std::unordered_set<uint64_t> road_set{};
  std::unordered_set<uint64_t> motorway_road_set{};
  bool is_cloud_navigation{false};
  int request_type{0};
  bool is_nonmotorway_map{false};
  bool start_in_junction{false};
  bool is_mixed_change{false};

  cyberverse::HDMap *hdmap{nullptr};
  cyberverse::RoadTopo *road_topo{nullptr};
  cyberverse::LaneTopo *lane_topo{nullptr};

 public:
  common::util::MessageUnit<Chassis> chassis_msg;
  common::util::MessageUnit<RoutingRequest> request_msg;
  common::util::MessageUnit<GlobalState> global_state_msg;
  common::util::MessageUnit<LocalizationEstimate> localization_msg;
  common::util::MessageUnit<PlanningInterface> planning_interface_msg;
  common::util::MessageUnit<PoseStamped> pose_base_link_in_utm_msg;
};
}  // namespace planning
}  // namespace neodrive

#pragma once

#include <list>
#include <memory>
#include <mutex>

#include "common/navigation_context.h"
#include "common/navigation_types.h"
#include "common_config/config/common_config.h"
#include "common_config/config/get_drive_strategy.h"
#include "config/navigation_config.h"
#include "cyber.h"
#include "cyber/common/memory_pool.h"
#include "navigation_base.h"
#include "neolix_log.h"
#include "planning/common/data_center/data_center.h"
#include "reference_line/refe_generator.h"
namespace neodrive {
namespace planning {

class NavigationComponent final : public neodrive::cyber::Component<> {
 public:
  NavigationComponent() = default;
  ~NavigationComponent();
  bool Init() override;
  void Proc();
  void Observe();

  NavigationContext *GetContext() { return &ctx_; }

 private:
  void Reset();
  void ProcessRoutingRequest();
  bool AddPathPoint();
  void SaveResult();
  void BroadcastResult();
  bool UpdateCurrentRef();
  void UpdateHornLightCmd();
  void UpdateIsOnMapStatus();
  bool HasTraffciLiaght(const uint64_t junction_id);
  void RequestProcess(const std::shared_ptr<RoutingRequest> &request);
  void KeypointRequestProcess(const std::shared_ptr<RoutingRequest> &request);
  bool IsChangeStartPoint(
      const std::shared_ptr<RoutingRequest> &request,
      global::routing::RoutingRequest_LaneWaypoint &new_start) const;
  bool IsChangeEndPoint(const std::shared_ptr<RoutingRequest> &request,
                        Vec2d &new_end) const;
  bool FinishProcess();
  void CheckParkingSpace(const std::shared_ptr<RoutingRequest> &new_request);
  void SetRoutingDestination(
      const std::shared_ptr<RoutingRequest> &new_request);
  std::string SortParkingSpaceByZDiff(
      std::vector<cyberverse::ParkingSpaceInfoConstPtr> &parks,
      const double current_z, cyberverse::HDMap *hdmap);
  bool IsOnMotorway(double x, double y,
                    const cyberverse::LaneInfoConstPtr &lane);

 private:
  std::shared_ptr<cyber::Writer<RoutingResult>> routing_pub_;
  std::shared_ptr<cyber::Writer<RoutingRequest>> routing_request_pub_;
  std::shared_ptr<cyber::Writer<IsOnMap>> is_on_map_pub_;
  std::shared_ptr<cyber::Writer<HornLightsCmd>> horn_light_cmd_pub_;
  std::shared_ptr<cyber::Writer<ADCTrajectory>> refer_line_pub_;
  std::shared_ptr<NavigationProcessBase> navigation_ptr_;
  std::shared_ptr<RefeLineGenerator> current_ref_generator_;
  std::shared_ptr<RefeLineGenerator> target_ref_generator_;
  std::shared_ptr<RoutingRequest> latest_request_{nullptr};
  std::shared_ptr<RoutingResult> latest_result_{nullptr};

  cyber::common::MemoryPool<planning::ADCTrajectory> ref_line_msg_pool_;
  cyber::common::MemoryPool<HornLightsCmd> horn_lights_msg_pool_;
  cyber::common::MemoryPool<IsOnMap> is_on_map_msg_pool_;

 private:
  neodrive::common::config::DriveStrategyType drive_stratedy_type_{};
  DataCenter *data_center_{nullptr};
  std::atomic<bool> initialized_{false};
  std::unique_ptr<std::thread> proc_thread_;
  uint32_t alive_notify_count_{0};
  uint32_t reach_station_count_{0};
  double last_send_t_{0.0};
  bool broadcast_last_result_{false};
  bool have_set_is_sim_{false};
  bool use_parking_out_{true};
  NavigationContext ctx_;
};

CYBER_REGISTER_COMPONENT(NavigationComponent);

}  // namespace planning
}  // namespace neodrive

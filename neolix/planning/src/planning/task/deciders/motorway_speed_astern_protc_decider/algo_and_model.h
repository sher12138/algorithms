#pragma once

#include "common/math/util.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {

struct EgoState {
  double x{};
  double y{};
  double s{};
  double l{};
  double width{};
  double length{};
  double speed{};
  double heading{};
  double speed_heading{};
  double acc{};
  double duration{0.2};
  double alpha_max{3.0};
  double beta_min{3.0};
  double beta_max{3.0};
  double beta_mean{1.0};
  double miu{0.2};
  double heading_diff{};
  double path_heading{};
};

class RssDistanceUtils {
 public:
  RssDistanceUtils() = default;
  RssDistanceUtils(const Obstacle& obs, const EgoState& ego_state);
  bool DataCheck();
  ~RssDistanceUtils();

  void CalRSSLateralDistance();
  void CalRSSLongtiDistance();

  double GetLateralMinDistance() const { return lateral_min_distance_; }
  double GetLongtiMinDistance() const { return longti_min_distance_; }
  const Obstacle* obs_ptr_{nullptr};
  const EgoState* ego_state_ptr_{nullptr};

 private:
  double lateral_min_distance_{};
  double longti_min_distance_{};
};

class ActionMap {
 public:
  ActionMap(RssDistanceUtils rss_info);
  ~ActionMap();

  void GenerateActions();

  bool GenerateSpeedLimit(const double safe_lateral_distance,
                          const double safe_longti_distance,
                          const double real_lateral_distance,
                          const double real_longti_distance);

  void GenerateSafeKeepDistance(double& safe_stop_dis);

  const std::pair<double, double>& GetSpeedLimit() const { return action_; }

 private:
  std::pair<double, double> action_{};
  RssDistanceUtils rss_info_;
};
}  // namespace planning
}  // namespace neodrive

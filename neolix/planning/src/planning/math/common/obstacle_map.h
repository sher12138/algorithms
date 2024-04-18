#pragma once

#include <deque>
#include <functional>
#include <list>
#include <map>
#include <queue>
#include <vector>

#include "common/math/polygon2d.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"
#include "common/obstacle/decision_data.h"
#include "common/vehicle_param.h"
#include "cyber/common/macros.h"
#include "cyber/time/time.h"
#include "obsmap.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/math/common/kinematics.h"
#include "task/task_info.h"

namespace neodrive {
namespace planning {
namespace obsmap {

struct State {
  int frame_id{-1};
  std::array<double, 6> x{0.0};
  uint64_t matched_lane_id{0};
  double matched_lane_heading_deviation{0.0};
  double dt{0.0};
  double w{0.0};
  math::Polygon *shape{nullptr};
  State() = default;
  State(const int f, const std::array<double, 6> &x0, double dT, double omega,
        math::Polygon *p, uint64_t lid = 0, double lheading = 0.0)
      : frame_id{f},
        x{x0},
        dt{dT},
        w{omega},
        matched_lane_id{lid},
        matched_lane_heading_deviation{lheading} {
    if (p != nullptr) {
      shape = p;
    }
  }
};

struct EgoState {
  localmap::ObsState ego_state{0.0};
  math::AD2 imu_accleration{0, 0};
  math::AD2 dr_velocity{0, 0};
  math::AD2 dr_accleration{0, 0};
  std::array<math::AD2, 2> dr_est_velocity{{{0, 0}, {0, 0}}};
  bool init{false};
};

class GlobalObsInfo {
 public:
  GlobalObsInfo() = default;

  GlobalObsInfo(const localmap::ObsState &states);

  GlobalObsInfo(ObsFrameContainer::ObsInfo *binfo, math::Polygon *bNode,
                const localmap::ObsState &states);

  void Update(ObsFrameContainer::ObsInfo *binfo, math::Polygon *bNode,
              const localmap::ObsState &states, bool do_prediect = true);
  void Predict(bool ego = false);

  bool Metabolism();

  ObsFrameContainer::ObsInfo *info{};
  std::deque<State> trajectory{};
  std::deque<std::unordered_map<std::string, Eigen::VectorXd>> prediction{{}};
  int id{0};
  int cycle{0};
  int birth{0};
  int last_time{0};
  friend class ObstacleMap;
};

struct ObsInfoComp {
  bool operator()(const std::shared_ptr<GlobalObsInfo> &a,
                  const std::shared_ptr<GlobalObsInfo> &b) const {
    if (a->last_time == b->last_time) {
      return a->info->obs_id < b->info->obs_id;
    } else {
      return a->last_time < b.get()->last_time;
    }
  }
};

struct ObsKinematicsInfo {
  std::weak_ptr<GlobalObsInfo> obs_info{};
  std::shared_ptr<math::KalmanFilter> cv_kf{};
  std::shared_ptr<math::KalmanFilter> ca_kf{};
  std::shared_ptr<math::KalmanFilter> ct_kf{};
};

class ObstacleMap : public ObsMap {
 private:
  ObstacleMap();
  ObstacleMap(const ObstacleMap &) = delete;
  ObstacleMap &operator=(const ObstacleMap &) = delete;

 public:
  static ObstacleMap *Instance() {
    static ObstacleMap Instance;
    return &Instance;
  }

 public:
  enum Region {
    Front = 0,
    Behind = 1,
    Vortex = 2,
    Merging = 3,
    ROISize,
  };

 public:
  void Update(ObsFrameContainer &currentFrame);

  void Update(const ObjectTable &object_table,
              const ObjectTable &object_table_utm,
              const std::vector<int> &obstacle_ids,
              const VehicleStateProxy &proxy);

  void VisObstacles(int frame = 0);

  std::weak_ptr<GlobalObsInfo> GetObs(int id);

  ObsKinematicsInfo &GetObsKinematics(int id);

  std::weak_ptr<GlobalObsInfo> ego();

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ObsKinematicsInfo, ego_kinematics);

 private:
  void InitROI(const VehicleStateProxy &proxy, double sensorRange = 30.0);
  void InitAttention(const VehicleStateProxy &proxy,
                     const math::AD2 &intersection);
  void CalculateROI(const VehicleStateProxy &proxy, int frame = 0);
  bool ObstacleParser(const ObjectTable &object_table,
                      const ObjectTable &object_table_utm,
                      const std::vector<int> &obstacle_ids);

 public:
  static constexpr int kLifeSpan{50};

 private:
  std::array<math::Polygon, 4> roi_{};
  std::set<std::shared_ptr<GlobalObsInfo>, ObsInfoComp> localmap_{};
  math::TrackingDiff td_1D_1e1_{0.1, 100, 0.1};
  std::shared_ptr<GlobalObsInfo> ego_{};
  EgoState ego_state_{};
  ObsKinematicsInfo ego_kinematics_{};
  std::unordered_map<int, ObsKinematicsInfo> idx_mapping_{};
};

}  // namespace obsmap
}  // namespace planning
}  // namespace neodrive

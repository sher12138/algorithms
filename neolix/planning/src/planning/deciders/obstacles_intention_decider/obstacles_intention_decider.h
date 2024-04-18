#pragma once
#include <algorithm>
#include <cmath>

#include "common/angles/angles.h"
#include "common/coordinate/coodrdinate_convertion.h"
#include "common/math/vec2d.h"
#include "common_geometry.pb.h"
#include "cyber/common/memory_pool.h"
#include "hdmap/hdmap.h"
#include "obstacles_intention.pb.h"
#include "obstacles_intention_base_decider.h"
#include "src/planning/common/math/math_utils.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/common/path/path_data.h"
#include "src/planning/planning_map/planning_map.h"

using ObstacleIntentionType = neodrive::global::planning::ObstacleIntentionType;
using ObstaclesIntention = neodrive::global::planning::ObstaclesIntention;
using neodrive::global::common::Point3D;
namespace neodrive {
namespace planning {

class ObstaclesIntentionDecider final : public ObstaclesIntentionBaseDecider {
  DECLARE_SINGLETON(ObstaclesIntentionDecider);

 public:
  virtual ~ObstaclesIntentionDecider() override;

 private:
  /// @brief
  /// @param task_info
  bool ObstaclesIntentionProcess(TaskInfo &task_info) override;
  /// @brief
  bool OtherSideObstaclesIntentionProcess(
      TaskInfo &task_info,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  /// @brief
  bool OtherSideObstacleIntentionProcess(
      TaskInfo &task_info, std::pair<const int, std::vector<Obstacle *>> &obs,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  /// @brief
  /// @param task_info
  bool CurrentLaneObstaclesIntentionProcess(
      TaskInfo &task_info,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  /// @brief
  void FilterObsIntentionsProcess(TaskInfo &task_info) override;
  /// @brief
  bool ObsRiskCheck(
      const std::pair<
          int, std::vector<neodrive::global::planning::ObstaclesIntention>>
          &obs_intention);
  /// @brief
  bool ObsGoFowardRiskCheck(const Vec3d &relative_obs);
  /// @brief
  bool ObsTurnLeftRiskCheck(const Vec3d &relative_obs);
  /// @brief
  bool ObsTurnRightRiskCheck(const Vec3d &relative_obs);
  /// @brief
  /// @return
  bool ObsCurrentLaneTurnIntentionProcess(
      const cyberverse::LaneInfoConstPtr lane, const Obstacle &obs,
      const double &obs_heading_diff,
      std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  /// @brief
  /// @return
  bool ObsSuccessorLanesTurnIntentionProcess(
      const cyberverse::LaneInfoConstPtr lane, const Obstacle &obs,
      const Obstacle &obs_utm, const double &obs_heading_diff,
      std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  /// @brief
  /// @return
  bool ObsCutinIntentionProcess(
      TaskInfo &task_info,
      std::pair<const int, std::vector<Obstacle *>> &obs_pair,
      std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);

  std::unordered_map<Obstacle::ObstacleType, std::string> obs_map_ = {
      {Obstacle::ObstacleType::UNKNOWN, "UNKNOWN"},
      {Obstacle::ObstacleType::UNKNOWN_MOVABLE, "UNKNOWN_MOVABLE"},
      {Obstacle::ObstacleType::UNKNOWN_UNMOVABLE, "UNKNOWN_UNMOVABLE"},
      {Obstacle::ObstacleType::PEDESTRIAN, "PEDESTRIAN"},
      {Obstacle::ObstacleType::BICYCLE, "BICYCLE"},
      {Obstacle::ObstacleType::VEHICLE, "VEHICLE"}};

 private:
  static constexpr double kTurnLDiffWeight = 0.5;
  static constexpr double kTurnHeadingDiffWeight = 6.37;
  static constexpr double kStaticUTurnThreshold = 2.8;
  static constexpr double kLaneLengthThreshold = 0.2;
  static constexpr double kReverseLaneThreshold = 1.57;
  static constexpr double kCutinTimeThreshold = 4.01;
  static constexpr double kCutinHeadingThreshold = 1.75;
  static constexpr double kBoundThreshold = 0.2;
};

REGISTER_SCENARIO_TASK(ObstaclesIntentionDecider);

}  // namespace planning
}  // namespace neodrive

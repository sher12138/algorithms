#pragma once
#include <algorithm>
#include <cmath>
#include <random>
#include <tuple>
#include <vector>

#include "common/angles/angles.h"
#include "common/coordinate/coodrdinate_convertion.h"
#include "common/macros.h"
#include "common_geometry.pb.h"
#include "cyber/common/memory_pool.h"
#include "hdmap/hdmap.h"
#include "obstacles_intention.pb.h"
#include "obstacles_intention_base_decider.h"
#include "src/planning/common/math/math_utils.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/planning_map/planning_map.h"

using ObstacleIntentionType = neodrive::global::planning::ObstacleIntentionType;
using ObstaclesIntention = neodrive::global::planning::ObstaclesIntention;
using neodrive::global::common::Point3D;
namespace neodrive {
namespace planning {

struct Particle {
  double x, y, heading, speed, weight;
  Particle(double x, double y, double heading, double speed, double weight)
      : x(x), y(y), heading(heading), speed(speed), weight(weight) {}
};

using ParticleSet = std::vector<Particle>;

class ParticleFilter {
 public:
  ParticleFilter();
  ParticleFilter(const int &numParticles, const Obstacle &obs_utm,
                 std::vector<cyberverse::LaneInfoConstPtr> &lane_sq,
                 const double &dt,
                 const ObstaclesIntentionBaseDecider::State &predict_state,
                 const double &current_obstacle_s,
                 const double &current_obstacle_l);
  ~ParticleFilter();
  void initParticles(ParticleSet &particles, const int &numParticles,
                     const double &x, const double &y, const double &heading,
                     const double &speed);
  void predict(ParticleSet &particles, double dt);
  void updateWeights(ParticleSet &particles, const double &observed_x,
                     const double &observed_y, const double &observed_speed,
                     const double &observed_heading);
  void resample(ParticleSet &particles);
  std::tuple<double, double, double, double> estimateState(
      const ParticleSet &particles);
  double weights_sum_;
  bool GetLaneSquencesIntention();
  bool GetLaneSquencesTargetPoint(global::common::PointENU &point);
  ObstacleIntentionType intention_;

 private:
  ParticleSet particles_;
  double time_step_predict_;
  double current_obstacle_s_, current_obstacle_l_;
  std::vector<cyberverse::LaneInfoConstPtr> lane_sq_;
  Obstacle obs_utm_;
  ObstaclesIntentionBaseDecider::State predict_state_;
  static constexpr double kMaxTurnRate = 0.4;
};

using ParticleFilterSet =
    std::unordered_map<ObstacleIntentionType, std::vector<ParticleFilter>>;

class ObsParticleFilter {
 public:
  ObsParticleFilter();
  ObsParticleFilter(const int &obs_id, const std::vector<Obstacle *> &obs_table,
                    const Obstacle &obs_utm,
                    const cyberverse::LaneInfoConstPtr &current_lane,
                    const ObstaclesIntentionBaseDecider::State &state);
  ~ObsParticleFilter();
  bool Process();
  bool PostProcess(std::unordered_map<int, std::vector<ObstaclesIntention>>
                       &cur_frame_obs_intention);
  std::unordered_map<ObstacleIntentionType, std::string> intention_map_ = {
      {ObstacleIntentionType::GO_FORWARD_INTENTION, "go_forward"},
      {ObstacleIntentionType::CUT_IN_INTENTION, "cut_in"},
      {ObstacleIntentionType::PASS_BY_INTENTION, "pass_by"},
      {ObstacleIntentionType::STOPING_INTENTION, "stopping"},
      {ObstacleIntentionType::TURN_LEFT_INTENTION, "turn_left"},
      {ObstacleIntentionType::TURN_RIGHT_INTENTION, "turn_right"},
      {ObstacleIntentionType::U_TURN_INTENTION, "U_TURN"}};

 private:
  bool LaneMatchingProcess(
      std::vector<std::vector<cyberverse::LaneInfoConstPtr>> &lane_squences_2d,
      double &current_obstacle_s, double &current_obstacle_l);
  int obs_id_;
  Obstacle obs_utm_;
  ParticleFilterSet obs_particle_filterset_;
  std::vector<Obstacle *> obs_table_;
  cyberverse::LaneInfoConstPtr current_lane_ = nullptr;
  int numParticles_ = 50;
  double time_step_predict_ = 0.5;
  ObstaclesIntentionBaseDecider::State predict_state_;
};

class ObstaclesIntentionParticleDecider final
    : public ObstaclesIntentionBaseDecider {
  DECLARE_SINGLETON(ObstaclesIntentionParticleDecider);

 public:
  virtual ~ObstaclesIntentionParticleDecider() override;

 private:
  /// @brief
  /// @param task_info
  bool ObstaclesIntentionProcess(TaskInfo &task_info) override;
  /// @brief
  void FilterObsIntentionsProcess(TaskInfo &task_info) override;
  /// @brief
  bool ObsRiskCheck(
      const std::pair<
          int, std::vector<neodrive::global::planning::ObstaclesIntention>>
          &obs_intention);
  /// @brief
  bool ObsGoFowardRiskCheck(const Point3D &relative_obs);
  /// @brief
  bool ObsTurnLeftRiskCheck(const Point3D &relative_obs);
  /// @brief
  bool ObsTurnRightRiskCheck(const Point3D &relative_obs);
  /// @brief
  bool ObstaclesIntentionParticleProcess(
      std::pair<const int, std::vector<Obstacle *>> &obs,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
};

REGISTER_SCENARIO_TASK(ObstaclesIntentionParticleDecider);

}  // namespace planning
}  // namespace neodrive

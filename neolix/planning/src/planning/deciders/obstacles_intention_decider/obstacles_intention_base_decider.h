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
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/math/math_utils.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

using ObstacleIntentionType = neodrive::global::planning::ObstacleIntentionType;
using ObstaclesIntention = neodrive::global::planning::ObstaclesIntention;
using neodrive::global::common::Point3D;
namespace neodrive {
namespace planning {

class ObstaclesIntentionBaseDecider : public ScenarioTaskInterface {
 public:
  std::unordered_map<ObstacleIntentionType, std::string> intention_map_ = {
      {ObstacleIntentionType::GO_FORWARD_INTENTION, "go_forward"},
      {ObstacleIntentionType::CUT_IN_INTENTION, "cut_in"},
      {ObstacleIntentionType::PASS_BY_INTENTION, "pass_by"},
      {ObstacleIntentionType::STOPING_INTENTION, "stopping"},
      {ObstacleIntentionType::TURN_LEFT_INTENTION, "turn_left"},
      {ObstacleIntentionType::TURN_RIGHT_INTENTION, "turn_right"},
      {ObstacleIntentionType::U_TURN_INTENTION, "U_TURN"}};

  struct ObsIntentionWeight {
    ObstacleIntentionType type;
    double weight;
    ObsIntentionWeight(neodrive::global::planning::ObstacleIntentionType t,
                       double w)
        : type(t), weight(w) {}
  };
  struct State {
    double x;
    double y;
    double v;
    double heading;
    double time;
    double a = 0;
    double turn_rate = 0;
  };

  virtual ~ObstaclesIntentionBaseDecider() override;

  ErrorCode Execute(TaskInfo &task_info) override;
  void SaveTaskResults(TaskInfo &task_info) override;
  void Reset() override{};
  bool GetObsRelativeCoordinate(const Obstacle *obs, Vec3d &relative_obs);
  bool GetNearestLaneWithHeadingForObs(const double &x, const double &y,
                                       const double &radius,
                                       const double &heading,
                                       const double &heading_diff,
                                       const double &obs_heading_diff,
                                       cyberverse::LaneInfoConstPtr &lane);
  static double GetLaneHeadingDiff(const double &x, const double &y,
                                   const cyberverse::LaneInfoConstPtr &lane);
  bool GetObsMaxMinSLInTrajectory(const Obstacle *obs, double &min_s,
                                  double &max_s, double &min_l, double &max_l,
                                  std::vector<PathPoint> &index_vc);

  bool FilterObs(const Obstacle *obs);
  static bool CompareByProbability(const ObstaclesIntention &a,
                                   const ObstaclesIntention &b);
  static constexpr double kStaticSpeedThreshold = 0.5;
  static constexpr double kTurnClassifyRadLimit = 0.7;
  static constexpr double kEndSThreshold = 20.0;
  static constexpr double kStartSThreshold = 20.0;
  static constexpr double kSearchRadius = 5.0;
  static constexpr double kSearchAngle = M_PI / 4.5;
  static constexpr double kBackFilterThreshold = 1.6;
  static constexpr double kForwardLeftFilterThreshold = 0.17;
  static constexpr double kForwardRightFilterThreshold = -0.17;
  static constexpr double kRightFilterDistanceThreshold = -1.5;
  static constexpr double kLeftFilterDistanceThreshold = 1.5;
  static constexpr double KLeftTurnRateThreshold = 0.2;
  static constexpr double KRightTurnRateThreshold = -0.2;
  static constexpr double KLeftTurnRateWeight = 1.0 / KLeftTurnRateThreshold;
  static constexpr double KRightTurnRateWeight = 1.0 / 0.2;
  static constexpr double KMaxNumber = 10000.0;
  static constexpr double KMinNumber = 0.0001;
  static constexpr double KLDiffThreshold = 0.01;

 protected:
  bool Init(TaskInfo &task_info);
  bool Process(TaskInfo &task_info);
  bool CheckObs(const std::vector<Obstacle *> &obs_table, const std::size_t &j);
  void ResetTable();
  void ResetFrame();
  void SaveObsIntentionContextStr(
      TaskInfo &task_info,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  /// @brief
  /// @param task_info
  virtual bool ObstaclesIntentionProcess(TaskInfo &task_info);
  /// @brief
  /// @param task_info
  bool DynamicPerceptionObstaclesPreProcess(TaskInfo &task_info);
  /// @brief
  /// @param task_info
  bool AddFrame(TaskInfo &task_info);
  /// @brief
  void PerceptionObstaclesTableProcess(TaskInfo &task_info);
  /// @brief
  virtual void FilterObsIntentionsProcess(TaskInfo &task_info);
  /// @brief
  /// @return
  bool ClassifyDynamicObstacles(TaskInfo &task_info);
  //   /// @brief
  //   /// @return
  //   bool GetIntentionsFromLane(TaskInfo& task_info);
  /// @brief
  /// @return
  bool EgoStatePreProcess(TaskInfo &task_info);

  /// @brief
  /// @return
  bool ObsIntentionWeightsProcess(
      const Obstacle &obs,
      std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  /// @brief
  /// @return
  bool ObsIntentionAddAndProcess(
      ObstacleIntentionType intention_type, double weight, const Obstacle &obs,
      std::vector<ObsIntentionWeight> &obs_intention_weight_vector,
      std::unordered_map<int, std::vector<ObstaclesIntention>>
          &cur_frame_obs_intention);
  bool EstimateAccelerationAndTurnRate(const std::vector<Obstacle *> &history,
                                       double &a, double &turn_rate);
  State UpdateState(const Obstacle *current_state, double a, double turn_rate,
                    double dt);
  State GetNextState(const Obstacle &obs_utm,
                     const std::vector<Obstacle *> &history);
  bool GetObsAvgSLSpeed(const std::vector<Obstacle *> &history, double &s_speed,
                        double &l_speed, bool &is_dynamic);
  bool GetObsAvgSLSpeedInTrajectory(const std::vector<Obstacle *> &history,
                                    double &s_speed, double &l_speed,
                                    bool &is_dynamic);
  cyberverse::LaneInfoConstPtr GetObsCurrentLane(
      const double &obs_heading_diff);
  cyberverse::LaneInfoConstPtr GetObsCurrentLane(
      Obstacle &obs_utm, const double &obs_heading_diff);
  bool ObsTurnCheck(const double &obs_heading_diff,
                    ObstacleIntentionType &intention_type);
  bool ObsGoForwardCheck(ObstacleIntentionType &intention_type,
                         const double &obs_heading_diff, double &weight);

 protected:
  std::unordered_map<int, std::vector<ObstaclesIntention>>
      last_frame_obs_intention_;
  std::deque<std::vector<Obstacle *>> perception_obstacles_list_;
  std::unordered_map<int, std::vector<Obstacle *>> obstacles_table_;
  std::unordered_map<int, std::vector<Obstacle *>> other_side_obstacles_table_;
  std::unordered_map<int, std::vector<Obstacle *>>
      other_side_cutin_all_obstacles_table_;
  std::unordered_map<int, std::vector<Obstacle *>> bicycle_obstacles_table_;
  std::unordered_map<int, std::vector<Obstacle *>>
      current_lane_obstacles_table_;
  std::unordered_map<int, ObstacleIntentionContext> cur_obs_intention_context_;
  std::deque<double> time_list_;
  std::deque<VehicleStateProxy> vehicle_state_list_;
  DataCenter *data_center_{DataCenter::Instance()};
  SLPoint ego_vehicle_sl_;
  bool is_right_lane_tmp_ = false;
  bool is_left_lane_tmp_ = false;
  State current_predicted_state_;
  State current_predicted_state_odom_;
  Boundary adc_boundary_;
  const int kPerObsListTimestampNums = 30;
  const int kPerObsListTimestampMinNums = 5;
  const double kOtherMaxLObsLimit = 15.0;
  const double kCurrentLaneObsSLimit = 0.0;
  const double kOtherLaneObsSMinLimit = -25.0;
  const double kOtherLaneObsSMaxLimit = 25.0;
  static constexpr double kWeightMax = 1.0;
  static constexpr double kWeightMin = 0.2;
  static constexpr double KCurrentLaneBoundBuffer = 0.5;
  static constexpr double KDt = 0.4;
  PathData *last_path_data_ = nullptr;
};

}  // namespace planning
}  // namespace neodrive

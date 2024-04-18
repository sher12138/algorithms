#pragma once

#include "planning/common/data_center/data_center.h"
#include "planning/planning_map/planning_map.h"
#include "planning/reference_line/reference_line.h"
#include "src/planning/common/planning_types.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {
namespace scenario_common {
using MotorwayDetourStageState =
    neodrive::global::planning::MotorwayDetourStageState;
using FRAME_OBS = std::vector<Obstacle *>;

constexpr double kMinDistance = 0.5;
constexpr int kFrameNums = 10;
constexpr int kMinHorizon = 3;
constexpr bool kMultiFrameLogPrint = false;
constexpr double kFrameTimeLowLim = 0.06;
constexpr double kFrameTimeHighLim = 0.15;

class FrameCheckInfoBase {
 public:
  bool frame_ans{true};
  double frame_time{0.};
  std::string frame_info_sign{""};
  bool frame_final_ans{true};
  virtual void PrintFrameCheckInfo(std::string sign){};
};

struct TargetLaneCheckInfo {
  int driving_direction{0};
  float preview_front_distance{0.0};
  float preview_back_distance{0.0};
  float near_front_distance{0.0};
  float near_back_distance{0.0};
  float preview_time{0.0};
};
struct DetourScenarioInfo {
  float crossroad_preview_distance{0.0};
  float queued_crossroad_preview_distance{0.0};
  float traffic_light_preview_distance{0.0};
  float queued_traffic_light_preview_distance{0.0};
  float road_bound_preview_distance{0.0};
  float lane_turn_preview_distance{0.0};
};

bool FrontMinLaneBound(const ReferencePointVec1d &ref_points,
                       double preview_distance, const SLPoint &curr_sl,
                       double &left_min_bound, double &right_min_bound);
bool IsFrontLaneTurning(const TaskInfo &task_info);
bool IsInJunction(const TaskInfo &task_info);
void ComputeRefL(TaskInfo &task_info, bool enable_bias_drive);
void ComputeTargetPoint(TaskInfo &task_info);
void RefLFilter(TaskInfo &task_info);
bool NearbyMinLaneBound(const ReferencePointVec1d &ref_points,
                        double preview_front_distance,
                        double preview_back_distance, const SLPoint &curr_sl,
                        double &left_min_bound, double &right_min_bound);
bool IsDynamicObsClear(const TaskInfo &task_info,
                       const TargetLaneCheckInfo &target_lane_check_info,
                       const double &left_bound, const double &right_bound,
                       std::vector<int> &dynamic_obs_ids,
                       std::vector<Obstacle *> &front_reverse_obs);
bool IsLeftDynamicObsClear(const TaskInfo &task_info,
                           const TargetLaneCheckInfo &target_lane_check_info,
                           const double &left_bound, const double &right_bound,
                           std::vector<int> &dynamic_obs_ids,
                           std::vector<Obstacle *> &front_reverse_obs,
                           bool use_multi_frame);
bool IsRightDynamicObsClear(const TaskInfo &task_info,
                            const TargetLaneCheckInfo &target_lane_check_info,
                            const double &left_bound, const double &right_bound,
                            std::vector<int> &dynamic_obs_ids,
                            std::vector<Obstacle *> &front_reverse_obs,
                            bool use_multi_frame);
bool CanCrossLane(const DividerFeature::DividerType &type,
                  const DividerFeature::DividerColor &color);
bool ComputePreviewRoadWidth(const TaskInfo &task_info,
                             const double preview_distance, double &width);
double ComputeObstacleSpeedDecision(double veh_v, bool motorway_driving);
bool IsCloseStaticDetour(const TaskInfo &task_info);
bool IsJunction(const TaskInfo &task_info, uint64_t &junction_id,
                const double preview_distance = 0.0);
bool IsCertainJunction(const TaskInfo &task_info, const uint64_t junction_id,
                       const double preview_distance = 0.0);
bool IsFrontHasTrafficLight(const TaskInfo &task_info, double preview_distance);
bool IsFrontHasTrafficLight(const TaskInfo &task_info, double preview_distance,
                            bool use_multi_frame);
bool IsRightTurnHasTrafficLight(const TaskInfo &task_info,
                                const double preview_distance,
                                bool road_queued);
bool IsNearTrafficJunction(const ReferenceLinePtr &reference_line,
                           const double preview_distance, const double curr_s);
bool IsTargetLineAdjacent();
bool IsAdcOnRefLine(const TaskInfo &task_info);
bool IsTCrossRoadJunctionScenario(const TaskInfo &task_info,
                                  double preview_distance);
bool IsFrontRoadQueued(const TaskInfo &task_info, double preview_distance,
                       double left_bound, double right_bound);
bool IsFrontHasCrossRoad(const TaskInfo &task_info, double preview_distance,
                         double ignore_distance = 0.0);
bool IsFrontHasCrossRoad(const TaskInfo &task_info, double preview_distance,
                         double ignore_distance, bool use_multi_frame);
bool IsFrontHasRoadBoundary(const TaskInfo &task_info, double preview_distance,
                            double ignore_distance = 0.0);
bool IsFrontHasRoadBoundary(const TaskInfo &task_info, double preview_distance,
                            double ignore_distance, bool use_multi_frame);
bool IsLeftFrontHasRoadBoundary(const TaskInfo &task_info,
                                double preview_distance,
                                double ignore_distance = 0.0);
bool IsLeftFrontHasRoadBoundary(const TaskInfo &task_info,
                                double preview_distance, double ignore_distance,
                                bool use_multi_frame);
bool IsRightFrontHasRoadBoundary(const TaskInfo &task_info,
                                 double preview_distance,
                                 double ignore_distance = 0.0);
bool IsRightFrontHasRoadBoundary(const TaskInfo &task_info,
                                 double preview_distance,
                                 double ignore_distance, bool use_multi_frame);
bool IsRightFirstLane(const TaskInfo &task_info, double preview_distance,
                      double ignore_distance = 0.0);
bool IsRightFirstLane(const TaskInfo &task_info, double preview_distance,
                      double ignore_distance, bool use_multi_frame);
bool IsFrontHasLaneTurn(const TaskInfo &task_info, double preview_distance,
                        double ignore_distance = 0.0,
                        bool ignore_right_turn = false);
bool IsFrontHasLaneTurn(const TaskInfo &task_info, double preview_distance,
                        double ignore_distance, bool ignore_right_turn,
                        bool use_multi_frame);
bool IsFrontHasUTurn(const TaskInfo &task_info, double preview_distance,
                     double ignore_distance = 0.0);
bool IsFrontStaticObsClear(const TaskInfo &task_info, const double left_bound,
                           const double right_bound, const double preview_s,
                           double &detour_obs_end_s, int &obs_id,
                           std::vector<Obstacle *> &detour_obs);
bool IsFrontBlockingObsClear(const TaskInfo &task_info, const double left_bound,
                             const double right_bound, const double preview_s,
                             std::vector<Obstacle *> &detour_obs);
bool IsMiddleFrontStaticObsClear(
    const TaskInfo &task_info, const double left_bound,
    const double right_bound, const double preview_s, double &detour_obs_end_s,
    int &obs_id, std::vector<Obstacle *> &detour_obs, bool use_multi_frame);
bool IsLeftFrontStaticObsClear(const TaskInfo &task_info,
                               const double left_bound,
                               const double right_bound, const double preview_s,
                               double &detour_obs_end_s, int &obs_id,
                               std::vector<Obstacle *> &detour_obs,
                               bool use_multi_frame);
bool IsRightFrontStaticObsClear(
    const TaskInfo &task_info, const double left_bound,
    const double right_bound, const double preview_s, double &detour_obs_end_s,
    int &obs_id, std::vector<Obstacle *> &detour_obs, bool use_multi_frame);
bool IsNonMovableObstacle(const TaskInfo &task_info, const Obstacle *obstacle,
                          double obstacles_distance);
bool IsAllowedDetourInReverseLane(const TaskInfo &task_info,
                                  double preview_distance,
                                  double ignore_distance);
bool IsAllowedDetourInReverseLane(const TaskInfo &task_info,
                                  double preview_distance,
                                  double ignore_distance, bool use_multi_frame);
bool IsRoadQueued(const TaskInfo &task_info,
                  std::vector<Obstacle *> &detour_obs);
void AdcPositionDiscussion(const TaskInfo &task_info);
void ComputeCrossRoadIgnoreDistance(const TaskInfo &task_info,
                                    double overlap_start_s,
                                    double overlap_end_s,
                                    double &ignore_distance);
bool IsLeftFrontReverseObsExist(
    const TaskInfo &task_info,
    const std::vector<Obstacle *> &left_front_reverse_obs,
    const std::vector<Obstacle *> &front_reverse_obs);

bool CheckMotorwayBackOut(const ReferenceLinePtr &reference_line,
                          const double curr_s, const VehicleStateProxy &pose);
bool BorrowDirection(const std::shared_ptr<DecisionData> &decision_data,
                     const ReferencePointVec1d &extend_lane_ref_points,
                     const int &obs_id, const double left_min_bound,
                     const double right_min_bound, bool *borrow_left,
                     bool *borrow_right);
void UpdateRecordEvents(ScenarioState::State curr_state);
void UpdateLaneType(const TaskInfo &task_info, std::string &lane_type);
void UpdateDetourInfo(DetourScenarioInfo &detour_conf,
                      bool motorway_drive = false);
std::string PrintDetourInfo(bool enable, bool check_pass, bool is_motorway);

}  // namespace scenario_common
}  // namespace planning
}  // namespace neodrive

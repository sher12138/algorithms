#pragma once
#include <cmath>

#include "motorway_speed_iter_obs_decision_extend.h"
#include "src/planning/common/data_center/outside_planner_data.h"
namespace neodrive {
namespace planning {

class MotorwayHybirdPid {
 public:
  ~MotorwayHybirdPid() = default;
  MotorwayHybirdPid() = default;
  MotorwayHybirdPid(const IterDeductionEgoState& ego_state,
                    double range_start_t, double range_end_t,
                    double target_speed, double speed_limit, double target_acc);
  DeductionEgoStateSequence CalPidProcess();

 private:
  DeductionEgoStateSequence hybird_pid_deduction_result_{};
  double ego_v_start_{};
  double ego_a_start_{};
  double ego_p_start_{};
  double range_start_t_{};
  double range_end_t_{};
  double step_{};
  double desire_max_a_{};
  double v_set_{};
  double speed_limit_{};
  double target_acc_{};
  double p_{};
};
class MotorwayHybirdIDM {
 public:
  MotorwayHybirdIDM() = default;
  ~MotorwayHybirdIDM() = default;
  MotorwayHybirdIDM(const IterDeductionEgoState& ego_state_idm,
                    const std::vector<ObsDecisionBound>& obs_low_points,
                    bool if_virtual, bool if_vehicle, bool if_bump,
                    double upper_limit, double target_speed, double target_acc);
  DeductionEgoStateSequence CalHybirdIDMProcess();

 private:
  DeductionEgoStateSequence hybird_idm_deduction_result_{};
  double ego_v_start_{};
  double ego_a_start_{};
  double ego_p_start_{};
  double start_theta_s_{};
  std::vector<ObsDecisionBound> obs_follow_info_;
  double upper_limit_{100.0};
  double target_speed_{100.0};
  double target_acc_{0.0};
  bool if_virtual_{false};
  bool if_vehicle_{false};
  double range_start_t_{};
  double range_end_t_{};
  double step_{};
  double desire_max_a_{};
  double v_set_{};
  double p_{};
  double headway_{};
  double default_space_{};
};
class MotorwayCheckDeductionCollisionCheck {
 public:
  ~MotorwayCheckDeductionCollisionCheck() = default;
  MotorwayCheckDeductionCollisionCheck() = default;
  MotorwayCheckDeductionCollisionCheck(
      const DeductionEgoStateSequence& ego_forward_deduction,
      const MotorwaySpeedObsExtend& obs_decision);
  std::pair<double, int> FindClosestPoint(const std::vector<double>& list,
                                          double target);
  std::pair<bool, int> CheckAndCalCollisionPoint();

 private:
  double step_{};
  double collision_buffer_{};
  double collision_buffer_l_{};
  double collision_buffer_u_{};
  bool obs_is_virtual_{false};
  bool obs_is_reverse_{false};
  MotorwaySpeedObsExtend obs_decision_;
  std::vector<double> ego_p_sequence_{};
  std::vector<double> ego_t_sequence_{};
  std::vector<double> ego_v_sequence_{};
};
}  // namespace planning
}  // namespace neodrive
#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/math/public_math/utility.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ConflictAreaRushOptModel {
 public:
  ConflictAreaRushOptModel(const VehicleInfo& ego,
                           const ConnectionConflictInfo& key_agent_info,
                           const std::vector<double>& conflict_area_bound)
      : ego_(ego),
        key_agent_info_(key_agent_info),
        conflict_area_bound_(conflict_area_bound) {
    other_agent_ = key_agent_info.agent;
  };
  ConflictAreaRushOptModel(const VehicleInfo& ego,
                           const VehicleInfo& other_agent,
                           const std::vector<double>& conflict_area_bound)
      : ego_(ego),
        other_agent_(other_agent),
        conflict_area_bound_(conflict_area_bound){};
  ConflictAreaRushOptModel(){};
  ~ConflictAreaRushOptModel(){};
  bool Init(const double rho, const double sf_a, const double rush_time,
            double* special_t);
  bool Planner(double upper_bound, double lower_bound = 0.0,
               const double step = 0.1);

  double get_special_t() { return special_t_; }

  void set_speed_limit(const std::vector<double>& speed_limit) {
    upstream_speed_limit_ = speed_limit;
  }

  void set_s_limit(const std::vector<double>& s_limit) {
    upstream_s_limit_ = s_limit;
  }

  void set_acc_limit(const std::vector<double>& acc_limit) {
    upstream_a_limit_ = acc_limit;
  }

  void set_min_distance(const double distance) { min_distance_ = distance; }

  std::vector<double> get_expected_s() { return expected_s_; }
  std::vector<double> get_expected_v() { return expected_v_; }
  std::vector<double> get_expected_a() { return expected_a_; }
  std::vector<double> get_expected_t() { return expected_t_; }

 private:
  bool CalSpecialTime(const double max_velocity);
  bool CalSpecialTime();
  bool ObjectiveFunction(const double x, double* value, double* verify_t);
  void CalInfluenceFromEgo(const double time, const double velocity,
                           const double s, const double comfort_brake,
                           double* max_acc);
  void CalVehicleSpeedPrediction();
  void CalVehSafetyDistance(const double max_velocity, double* safety_distance);
  bool ForwardSimulation(const double max_velocity, const double max_acc,
                         const double s0, const double v0,
                         const double forward_t, const double s_goal,
                         bool* sim_success, double* sim_t);
  void CalVelocityUpper(const double R, const double width, const double height,
                        const double speed_limit_upper, double* velocity_upper,
                        double g = 9.8, double mu = 0.9);
  void CalAccumulateJerk(const double v, const double acc, double* jerks);
  bool CaltToJunction(const double v, const double acc, double* t);
  bool CalVehStateAtEgoCross(const double v, const double acc, const double t,
                             std::vector<double>* veh_state);
  bool MakeUpperSpeedCons(const std::vector<double>& vel_reality,
                          double* vel_diff);
  bool MakeUpperSCons(const std::vector<double>& s_reality,
                      const std::vector<double>& vel_reality);
  bool CalOtherCrossTime(double* cross_time);
  bool VerifySpecialTime(const double v_f, double* verify_t);
  bool ViolateDistanceCons(const double v, const double current_s,
                           const double upper_s);
  void CalOptimalBoundaryValueByQuintic(const Eigen::VectorXd& sf,
                                        const double verify_t,
                                        Eigen::VectorXd* result,
                                        double* min_jerk);
  bool CompleteDecision(const int limit_size, std::vector<double>& s_reality,
                        std::vector<double>& v_reality,
                        std::vector<double>& a_reality);

 private:
  VehicleInfo ego_, other_agent_;
  ConnectionConflictInfo key_agent_info_;
  double time_step_ = 0.1;
  double rho_, sf_a_, rush_time_, min_distance_ = 0.0;
  Eigen::VectorXd past_vel_records_;
  Eigen::VectorXd quartic_coeff_;
  double special_t_ = 0.0;
  std::vector<double> conflict_area_bound_, speed_limit_, s_limit_, acc_limit_,
      upstream_speed_limit_, upstream_s_limit_, upstream_a_limit_;
  std::vector<double> expected_s_, expected_v_, expected_a_, expected_t_;
};
}  // namespace planning
}  // namespace neodrive
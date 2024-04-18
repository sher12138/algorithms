#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/math/public_math/utility.h"

namespace neodrive {
namespace planning {
enum class StateTransType { UNKNOWN = 0, CUBIC, QUARTIC, QUINTIC };

class ConflictAreaYieldOptModel {
 public:
  ConflictAreaYieldOptModel(const VehicleInfo& ego,
                            const VehicleInfo& other_agent,
                            const std::vector<double>& conflict_area_bound)
      : ego_(ego),
        other_agent_(other_agent),
        conflict_area_bound_(conflict_area_bound){};

  ConflictAreaYieldOptModel(){};

  ~ConflictAreaYieldOptModel(){};

  bool Init(const double rho, const double sf_a, const double yield_time,
            double* special_t);
  bool Planner(const double upper_bound, const double loss_threshold = 1e+2,
               const double lower_bound = 0.0, const double step = 0.1);

  double get_special_t() { return special_t_[0]; }

  void set_speed_limit(const std::vector<double>& speed_limit) {
    upstream_speed_limit_ = speed_limit;
  }

  void set_s_limit(const std::vector<double>& s_limit) {
    upstream_s_limit_ = s_limit;
  }

  void set_acc_limit(const std::vector<double>& acc_limit) {
    upstream_a_limit_ = acc_limit;
  }

  std::vector<double> get_expected_s() { return expected_s_; }
  std::vector<double> get_expected_v() { return expected_v_; }
  std::vector<double> get_expected_a() { return expected_a_; }
  std::vector<double> get_expected_t() { return expected_t_; }

 private:
  void CalVelocityUpper(const double R, const double width, const double height,
                        const double speed_limit_upper, double* velocity_upper,
                        double g = 9.8, double mu = 0.9);
  bool CalSpecialTime();
  bool ObjectiveFunctionBaseQuintic(const double x, double* value,
                                    double* verify_t, double* real_x,
                                    std::vector<double>* s_reality,
                                    std::vector<double>* t_reality,
                                    std::vector<double>* vel_reality,
                                    std::vector<double>* acc_reality);
  bool ObjectiveFunctionBaseQuartic(const double x, double* value,
                                    double* verify_t, double* real_x,
                                    std::vector<double>* s_reality,
                                    std::vector<double>* t_reality,
                                    std::vector<double>* vel_reality,
                                    std::vector<double>* acc_reality);
  bool ObjectiveFunctionBaseCubic(const double x, double* value,
                                  double* verify_t, double* real_x,
                                  std::vector<double>* s_reality,
                                  std::vector<double>* t_reality,
                                  std::vector<double>* vel_reality,
                                  std::vector<double>* acc_reality);

  void CalOptimalBoundaryValueByQuintic(const Eigen::VectorXd& sf,
                                        const double verify_t,
                                        Eigen::VectorXd* result,
                                        double* min_jerk);
  void CalOptimalBoundaryValueByQuartic(const Eigen::VectorXd& sf,
                                        const double verify_t,
                                        Eigen::VectorXd* result,
                                        double* min_jerk);
  void CalOptimalBoundaryValueByCubic(const Eigen::VectorXd& sf,
                                      const double verify_t,
                                      Eigen::VectorXd* result,
                                      double* min_jerk);

  void CalVehicleSpeedPrediction();
  bool CaltToJunction(const double v, const double acc, double* t);
  bool MakeUpperSpeedCons(const std::vector<double>& vel_reality,
                          double* vel_diff);
  bool MakeUpperSCons(const std::vector<double>& s_reality, double* s_diff);
  bool CalOtherCrossTime(double* cross_time);
  bool VerifySpecialTime(const double v_f, const double s, double* verify_t);
  bool CalMixOutput(const double acc_front, const double acc_rear,
                    const double cur_s, const double cur_speed, double* mix_acc,
                    double* mix_speed,
                    double* mix_s);  // simple mix
  bool CalFinalS(const double s_bound, const double goal_v, double* final_s);

 private:
  VehicleInfo ego_, other_agent_;
  double time_step_ = 0.1;
  double rho_, sf_a_, yield_time_;
  Eigen::VectorXd past_vel_records_;
  Eigen::VectorXd quartic_coeff_;
  std::vector<double> special_t_ = {0.0, 0.0};
  std::vector<double> conflict_area_bound_, speed_limit_, s_limit_, acc_limit_,
      upstream_speed_limit_, upstream_s_limit_, upstream_a_limit_;
  std::vector<double> expected_s_, expected_v_, expected_a_, expected_t_;
  bool veh_leave_ = false;
  StateTransType state_trans_func_{StateTransType::UNKNOWN};
};
}  // namespace planning
}  // namespace neodrive
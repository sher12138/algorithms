#include "conflict_area_yield_model.h"

#include <cmath>

namespace neodrive {
namespace planning {

void ConflictAreaYieldOptModel::CalVehicleSpeedPrediction() {
  Eigen::VectorXd xs(5);
  xs << -4.0, -3.0, -2.0, -1.0, 0.0;
  Utility::FittingQuartic(xs, past_vel_records_, &quartic_coeff_);
}

void ConflictAreaYieldOptModel::CalVelocityUpper(const double R,
                                                 const double width,
                                                 const double height,
                                                 const double speed_limit_upper,
                                                 double* velocity_upper,
                                                 double g, double mu) {
  double path_speed_limit =
      std::min(0.5 * g * width * R / height - 0.1, g * R * mu - 0.1);
  *velocity_upper = std::min(speed_limit_upper, path_speed_limit);
}

bool ConflictAreaYieldOptModel::CalSpecialTime() {
  bool sign_leave = true;
  // Considering the situation where other car's brakes are stuck without a
  // solution
  double time = time_step_;
  while (true) {
    double v = other_agent_.sl_velocity.x();
    if (v <= 0.0) {
      special_t_ = {1e+8, 1e+8};
      break;
    }
    double s = v * time;
    double s_ = s + 0.5 * std::pow(v, 2) / other_agent_.max_longitudinal_brake;

    if (s_ > conflict_area_bound_[0] - other_agent_.front_suspension &&
        sign_leave) {
      sign_leave = false;
      special_t_[0] = time + std::max(conflict_area_bound_[0] -
                                          other_agent_.front_suspension - s,
                                      0.0) /
                                 v;
    }
    if (s_ > conflict_area_bound_[1] + other_agent_.rear_suspension) {
      special_t_[1] = time + std::max(conflict_area_bound_[1] +
                                          other_agent_.rear_suspension - s,
                                      0.0) /
                                 v;
      if (special_t_[0] == special_t_[1]) {
        veh_leave_ = true;
      }
      break;
    }
    time += time_step_;
  }
  // Over 100 seconds, yielding failed
  if (special_t_[0] > 1e+6) {
    return false;
  }
  special_t_[0] += yield_time_;
  return true;
}

bool ConflictAreaYieldOptModel::VerifySpecialTime(const double v_f,
                                                  const double s,
                                                  double* verify_t) {
  double verify_t_ =
      2.0 * s / (ego_.sl_velocity.x() + v_f + 1e-2) + 1.0 / std::max(v_f, 1e-3);
  if (verify_t_ < special_t_[0] ||
      (verify_t_ - 1.0 / std::max(v_f, 1e-3) <= 1e-3)) {
    if (v_f < 0.1) {
      *verify_t = 2.0 * s / (ego_.sl_velocity.x() + v_f + 1e-2);
      return true;
    }
    // LOG_WARN("Verify t is too small!");
    return false;
  }
  *verify_t = 2.0 * s / (ego_.sl_velocity.x() + v_f + 1e-2);
  return true;
}

bool ConflictAreaYieldOptModel::CaltToJunction(const double v, const double acc,
                                               double* t) {
  double t_ = 0.0, v_ = std::max(v, 1e-3);
  double acc_ = (std::abs(acc - 0.0) < 1e-6) ? 1e-3 : acc;
  double safety_distance = v_ * ego_.react_time +
                           0.5 * std::pow(v_, 2) / ego_.max_longitudinal_brake +
                           2.0;
  if (acc_ < 0.0) {
    double d_ = -0.5 * std::pow(v_, 2) / acc_;
    if (d_ < safety_distance) {
      t_ = 1e+8;
    } else {
      double a = 0.5 * acc_;
      double b = v_;
      double c = -safety_distance;
      if (!Utility::SolveQuadraticEquation(a, b, c, &t_)) {
        LOG_WARN("Quadratic Equation No root!");
        return false;
      }
    }
  } else {
    double a = 0.5 * acc_;
    double b = v;
    double c = -safety_distance;
    if (!Utility::SolveQuadraticEquation(a, b, c, &t_)) {
      LOG_WARN("Quadratic Equation No root!");
      return false;
    }
  }
  *t = t_;
  return true;
}

bool ConflictAreaYieldOptModel::MakeUpperSpeedCons(
    const std::vector<double>& vel_reality, double* vel_diff) {
  double min_diff = 0.0;
  if (vel_reality.size() != speed_limit_.size()) {
    LOG_ERROR(
        "Container size misalignment, reality velocity's size is {}, limit "
        "size is {}!",
        vel_reality.size(), speed_limit_.size());
    return false;
  }
  for (int i = 0; i < vel_reality.size(); ++i) {
    double diff = speed_limit_[i] - vel_reality[i];
    if (diff < min_diff) {
      min_diff = diff;
    }
  }
  *vel_diff = min_diff;
  return true;
}

bool ConflictAreaYieldOptModel::MakeUpperSCons(
    const std::vector<double>& s_reality, double* s_diff) {
  double min_diff = 0.0;
  if (s_reality.size() != s_limit_.size()) {
    LOG_ERROR("Container size misalignment!");
    return false;
  }
  for (int i = 0; i < s_reality.size(); ++i) {
    double diff = s_limit_[i] - s_reality[i];
    if (diff < min_diff) {
      min_diff = diff;
    }
  }
  *s_diff = min_diff;
  return true;
}

bool ConflictAreaYieldOptModel::CalOtherCrossTime(double* cross_time) {
  double cross_time_ = 1e+8, time = 0.0;
  double s_ = conflict_area_bound_[1] + other_agent_.rear_suspension,
         s_rear = conflict_area_bound_[3] - ego_.front_suspension;
  double v = other_agent_.sl_velocity.x();
  // while (true) {
  //   double v = other_agent_.sl_velocity.x();
  //   if (v < 0.0) break;
  //   double s = v * time;
  //   if (s >= s_) {
  //     cross_time_ = time;
  //     break;
  //   }
  //   time += time_step_;
  // }
  cross_time_ = s_ / v;
  *cross_time = cross_time_;
  return true;
}

void ConflictAreaYieldOptModel::CalOptimalBoundaryValueByQuintic(
    const Eigen::VectorXd& sf, const double verify_t, Eigen::VectorXd* result,
    double* min_jerk) {
  Eigen::VectorXd s_(6);
  Eigen::VectorXd result_(6);
  s_ << 0.0, ego_.sl_velocity.x(), ego_.sl_accel.x(), sf[0], sf[1], sf[2];
  double T = verify_t;
  Eigen::MatrixXd coeff(6, 6);
  coeff << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      2.0, 0.0, 0.0, 0.0, 1.0, T, std::pow(T, 2), std::pow(T, 3),
      std::pow(T, 4), std::pow(T, 5), 0.0, 1.0, 2.0 * T, 3.0 * std::pow(T, 2),
      4.0 * std::pow(T, 3), 5.0 * std::pow(T, 4), 0.0, 0.0, 2.0, 6.0 * T,
      12.0 * std::pow(T, 2), 20.0 * std::pow(T, 3);
  Eigen::MatrixXd coeff_inv = coeff.inverse();
  Eigen::VectorXd process_param = coeff_inv * s_;
  *result = process_param;
  *min_jerk = 6.0 * process_param[3] * T +
              12.0 * process_param[4] * std::pow(T, 2) +
              20.0 * process_param[5] * std::pow(T, 3);
}

void ConflictAreaYieldOptModel::CalOptimalBoundaryValueByQuartic(
    const Eigen::VectorXd& sf, const double verify_t, Eigen::VectorXd* result,
    double* min_jerk) {
  // LOG_INFO("Polynomial curves are quartic!");
  Eigen::VectorXd s_(5);
  double min_jerk_{0.0};
  s_ << 0.0, ego_.sl_velocity.x(), sf[0], sf[1], sf[2];
  const double T = verify_t;
  Eigen::MatrixXd coeff(5, 5);
  coeff << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, T,
      std::pow(T, 2), std::pow(T, 3), std::pow(T, 4), 0.0, 1.0, 2.0 * T,
      3.0 * std::pow(T, 2), 4.0 * std::pow(T, 3), 0.0, 0.0, 2.0, 6.0 * T,
      12.0 * std::pow(T, 2);
  Eigen::MatrixXd coeff_inv = coeff.inverse();
  Eigen::VectorXd process_param = coeff_inv * s_;
  *result = process_param;
  for (int i = 0; i < std::floor(T / 0.1); ++i) {
    min_jerk_ += 6.0 * process_param[3] + 2.40 * process_param[4] * i;
  }
  *min_jerk = std::max(min_jerk_, 1e-1) / std::max(1e-4, std::floor(T / 0.1));
}

void ConflictAreaYieldOptModel::CalOptimalBoundaryValueByCubic(
    const Eigen::VectorXd& sf, const double verify_t, Eigen::VectorXd* result,
    double* min_jerk) {
  // LOG_INFO("Polynomial curves are cubic!");
  Eigen::VectorXd s_(4);
  double min_jerk_{0.0};
  s_ << 0.0, ego_.sl_velocity.x(), sf[0], sf[1];
  const double T = verify_t;
  Eigen::MatrixXd coeff(4, 4);
  coeff << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, T, std::pow(T, 2),
      std::pow(T, 3), 0.0, 1.0, 2.0 * T, 3.0 * std::pow(T, 2);
  Eigen::MatrixXd coeff_inv = coeff.inverse();
  Eigen::VectorXd process_param = coeff_inv * s_;
  *result = process_param;
  for (int i = 0; i < std::floor(T / 0.1); ++i) {
    min_jerk_ += 6.0 * process_param[3];
  }
  *min_jerk = std::max(min_jerk_, 1e-1) / std::max(1e-4, std::floor(T / 0.1));
}

// objective function
bool ConflictAreaYieldOptModel::ObjectiveFunctionBaseQuintic(
    const double x, double* value, double* verify_t, double* real_x,
    std::vector<double>* s_reality, std::vector<double>* t_reality,
    std::vector<double>* vel_reality, std::vector<double>* acc_reality) {
  double goal_s = conflict_area_bound_[3], goal_v = x, verify_t_;
  // RSS
  if (!CalFinalS(conflict_area_bound_[3], x, &goal_s)) {
    // back up
    goal_s = (conflict_area_bound_[3] > 0.4) ? (conflict_area_bound_[3] - 0.4)
                                             : conflict_area_bound_[3];
    goal_v = 0.0;
  }
  if (!VerifySpecialTime(goal_v, goal_s, &verify_t_)) {
    // LOG_WARN("When goal_v = {:3f}, not pass special t verify!", goal_v);
    return false;
  }
  uint64_t steps = std::floor(verify_t_ / time_step_);
  speed_limit_.clear();
  s_limit_.clear();
  acc_limit_.clear();
  for (int i = 0; i < steps; ++i) {
    if (i < upstream_speed_limit_.size()) {
      speed_limit_.emplace_back(upstream_speed_limit_[i]);
      s_limit_.emplace_back(upstream_s_limit_[i]);
      acc_limit_.emplace_back(upstream_a_limit_[i]);
    } else {
      speed_limit_.emplace_back(upstream_speed_limit_.back());
      s_limit_.emplace_back(upstream_s_limit_.back());
      acc_limit_.emplace_back(upstream_a_limit_.back());
    }
  }
  Eigen::VectorXd result(6);
  double min_jerk, t_to_junction, vel_diff, s_diff, veh_cross_time,
      value_ = 0.0;
  Eigen::VectorXd sf(3);
  sf << goal_s, goal_v, sf_a_;
  CalOptimalBoundaryValueByQuintic(sf, verify_t_, &result, &min_jerk);
  // value_ += rho_ * min_jerk;
  if (!CaltToJunction(goal_v, sf_a_, &t_to_junction)) {
    LOG_WARN("Calculate t to Junction Failed!");
    return false;
  }
  value_ += t_to_junction;
  std::vector<double> acc_reality_(std::floor(verify_t_ / time_step_),
                                   acc_limit_[0]);
  std::vector<double> vel_reality_(std::floor(verify_t_ / time_step_),
                                   speed_limit_[0]);
  std::vector<double> s_reality_(std::floor(verify_t_ / time_step_),
                                 s_limit_[0]);
  std::vector<double> t_reality_(std::floor(verify_t_ / time_step_), 0.0);
  Eigen::VectorXd coeff_a(4), coeff_s(6);
  coeff_a << result[2] * 2.0, result[3] * 6.0, result[4] * 12.0,
      result[5] * 20.0;
  coeff_s << result[0], result[1], result[2], result[3], result[4], result[5];
  if (std::floor(verify_t_ / time_step_) < 2) {
    // LOG_WARN("Special Time is Too Short!");
    return false;
  }
  for (int i = 0; i < std::floor(verify_t_ / time_step_) - 1; i++) {
    double time = time_step_ * (i + 1);
    Eigen::VectorXd time_a(4), time_s(6);
    time_a << 1.0, std::pow(time, 1), std::pow(time, 2), std::pow(time, 3);
    time_s << 1.0, std::pow(time, 1), std::pow(time, 2), std::pow(time, 3),
        std::pow(time, 4), std::pow(time, 5);
    double acc_rear = (coeff_a.transpose() * time_a)[0];
    double mix_s, mix_acc, mix_speed;
    CalMixOutput(acc_limit_[i + 1], acc_rear, s_reality_[i], vel_reality_[i],
                 &mix_acc, &mix_speed, &mix_s);  // match wtx's sequence (i + 1)
    acc_reality_[i] = mix_acc;
    s_reality_[i + 1] = mix_s;
    vel_reality_[i + 1] = mix_speed;
    t_reality_[i] = time;
  }
  double last_point = std::floor(verify_t_ / time_step_) - 1;
  double last_time = last_point * time_step_;
  Eigen::VectorXd time_a(4);
  time_a << 1.0, std::pow(last_time, 1), std::pow(last_time, 2),
      std::pow(last_time, 3);
  double acc_rear = (coeff_a.transpose() * time_a)[0];
  acc_reality_.back() = std::min(acc_rear, acc_limit_[last_point]);
  t_reality_[last_point] = last_time;

  if (!MakeUpperSpeedCons(vel_reality_, &vel_diff)) {
    LOG_WARN("Make Upper Speed Cons Failed!");
    return false;
  }
  if (!MakeUpperSCons(s_reality_, &s_diff)) {
    LOG_WARN("Make Upper S Cons Failed!");
    return false;
  }
  if (s_diff < -0.15) {
    // LOG_WARN("Violate of S Boundary Constraint (s_diff = {:3f})", s_diff);
    return false;
  }
  if (!CalOtherCrossTime(&veh_cross_time)) {
    // LOG_WARN("Calculate Cross Time Failed!");
    return false;
  }
  double cost_efficiency = time_step_ * std::floor(verify_t_ / time_step_) +
                           t_to_junction - veh_cross_time - yield_time_;
  value_ += std::abs(ego_.sl_velocity.x() - goal_v);
  if (cost_efficiency < -yield_time_) {
    value_ -= 100.0 * cost_efficiency;
  } else if (cost_efficiency < 0.0) {
    value_ -= cost_efficiency;
  }

  // // acc/dec
  // double cost_smooth = 0.0;
  // for (const auto acc : acc_reality_) {
  //   cost_smooth += std::abs(acc) * 1e+3;
  // }
  // value_ += cost_smooth;

  // double cost_speed_limit = vel_diff;
  // if (cost_speed_limit < 0) {
  //   value_ -= 1e+8 * cost_speed_limit;
  // }
  *real_x = goal_v;
  *value = value_;
  *verify_t = verify_t_;
  *t_reality = t_reality_;
  *s_reality = s_reality_;
  *vel_reality = vel_reality_;
  *acc_reality = acc_reality_;
  return true;
}

bool ConflictAreaYieldOptModel::ObjectiveFunctionBaseQuartic(
    const double x, double* value, double* verify_t, double* real_x,
    std::vector<double>* s_reality, std::vector<double>* t_reality,
    std::vector<double>* vel_reality, std::vector<double>* acc_reality) {
  double goal_s = conflict_area_bound_[3], goal_v = x, verify_t_;
  // RSS
  if (!CalFinalS(conflict_area_bound_[3], x, &goal_s)) {
    // back up
    goal_s = (conflict_area_bound_[3] > 0.4) ? (conflict_area_bound_[3] - 0.4)
                                             : conflict_area_bound_[3];
    goal_v = 0.0;
  }
  if (!VerifySpecialTime(goal_v, goal_s, &verify_t_)) {
    // LOG_WARN("When goal_v = {:3f}, not pass special t verify!", goal_v);
    return false;
  }
  uint64_t steps = std::floor(verify_t_ / time_step_);
  speed_limit_.clear();
  s_limit_.clear();
  acc_limit_.clear();
  for (int i = 0; i < steps; ++i) {
    if (i < upstream_speed_limit_.size()) {
      speed_limit_.emplace_back(upstream_speed_limit_[i]);
      s_limit_.emplace_back(upstream_s_limit_[i]);
      acc_limit_.emplace_back(upstream_a_limit_[i]);
    } else {
      speed_limit_.emplace_back(upstream_speed_limit_.back());
      s_limit_.emplace_back(upstream_s_limit_.back());
      acc_limit_.emplace_back(upstream_a_limit_.back());
    }
  }
  Eigen::VectorXd result(5);
  double min_jerk, t_to_junction, vel_diff, s_diff, veh_cross_time,
      value_ = 0.0;
  Eigen::VectorXd sf(3);
  sf << goal_s, goal_v, sf_a_;
  CalOptimalBoundaryValueByQuartic(sf, verify_t_, &result, &min_jerk);
  if (!CaltToJunction(goal_v, sf_a_, &t_to_junction)) {
    LOG_WARN("Calculate t to Junction Failed!");
    return false;
  }
  value_ += t_to_junction;
  std::vector<double> acc_reality_(std::floor(verify_t_ / time_step_),
                                   acc_limit_[0]);
  std::vector<double> vel_reality_(std::floor(verify_t_ / time_step_),
                                   speed_limit_[0]);
  std::vector<double> s_reality_(std::floor(verify_t_ / time_step_),
                                 s_limit_[0]);
  std::vector<double> t_reality_(std::floor(verify_t_ / time_step_), 0.0);
  Eigen::VectorXd coeff_a(3), coeff_s(5);
  coeff_a << result[2] * 2.0, result[3] * 6.0, result[4] * 12.0;
  coeff_s << result[0], result[1], result[2], result[3], result[4];
  if (std::floor(verify_t_ / time_step_) < 2) {
    // LOG_WARN("Special Time is Too Short!");
    return false;
  }
  for (int i = 0; i < std::floor(verify_t_ / time_step_) - 1; i++) {
    double time = time_step_ * (i + 1);
    Eigen::VectorXd time_a(3), time_s(5);
    time_a << 1.0, std::pow(time, 1), std::pow(time, 2);
    time_s << 1.0, std::pow(time, 1), std::pow(time, 2), std::pow(time, 3),
        std::pow(time, 4);
    double acc_rear = (coeff_a.transpose() * time_a)[0];
    double mix_s, mix_acc, mix_speed;
    CalMixOutput(acc_limit_[i + 1], acc_rear, s_reality_[i], vel_reality_[i],
                 &mix_acc, &mix_speed, &mix_s);  // match wtx's sequence (i + 1)
    acc_reality_[i] = mix_acc;
    s_reality_[i + 1] = mix_s;
    vel_reality_[i + 1] = mix_speed;
    t_reality_[i] = time;
  }
  double last_point = std::floor(verify_t_ / time_step_) - 1;
  double last_time = last_point * time_step_;
  Eigen::VectorXd time_a(3);
  time_a << 1.0, std::pow(last_time, 1), std::pow(last_time, 2);
  double acc_rear = (coeff_a.transpose() * time_a)[0];
  acc_reality_.back() = std::min(acc_rear, acc_limit_[last_point]);
  t_reality_[last_point] = last_time;

  if (!MakeUpperSpeedCons(vel_reality_, &vel_diff)) {
    LOG_WARN("Make Upper Speed Cons Failed!");
    return false;
  }
  if (!MakeUpperSCons(s_reality_, &s_diff)) {
    LOG_WARN("Make Upper S Cons Failed!");
    return false;
  }
  if (s_diff < -0.15) {
    // LOG_WARN("Violate of S Boundary Constraint (s_diff = {:3f})", s_diff);
    return false;
  }
  if (!CalOtherCrossTime(&veh_cross_time)) {
    // LOG_WARN("Calculate Cross Time Failed!");
    return false;
  }
  double cost_efficiency = time_step_ * std::floor(verify_t_ / time_step_) +
                           t_to_junction - veh_cross_time - yield_time_;
  value_ += std::abs(ego_.sl_velocity.x() - goal_v);
  if (cost_efficiency < -yield_time_) {
    value_ -= 100.0 * cost_efficiency;
  } else if (cost_efficiency < 0.0) {
    value_ -= cost_efficiency;
  }
  *real_x = goal_v;
  *value = value_;
  *verify_t = verify_t_;
  *t_reality = t_reality_;
  *s_reality = s_reality_;
  *vel_reality = vel_reality_;
  *acc_reality = acc_reality_;
  return true;
}

bool ConflictAreaYieldOptModel::ObjectiveFunctionBaseCubic(
    const double x, double* value, double* verify_t, double* real_x,
    std::vector<double>* s_reality, std::vector<double>* t_reality,
    std::vector<double>* vel_reality, std::vector<double>* acc_reality) {
  double goal_s = conflict_area_bound_[3], goal_v = x, verify_t_;
  // RSS
  if (!CalFinalS(conflict_area_bound_[3], x, &goal_s)) {
    // back up
    goal_s = (conflict_area_bound_[3] > 0.4) ? (conflict_area_bound_[3] - 0.4)
                                             : conflict_area_bound_[3];
    goal_v = 0.0;
  }
  if (!VerifySpecialTime(goal_v, goal_s, &verify_t_)) {
    return false;
  }
  uint64_t steps =
      std::min(static_cast<int>(std::floor(std::max(verify_t_, 2 * time_step_) /
                                           time_step_)),
               100);
  speed_limit_.clear();
  s_limit_.clear();
  acc_limit_.clear();
  for (int i = 0; i < steps; ++i) {
    if (i < upstream_speed_limit_.size()) {
      speed_limit_.emplace_back(upstream_speed_limit_[i]);
      s_limit_.emplace_back(upstream_s_limit_[i]);
      acc_limit_.emplace_back(upstream_a_limit_[i]);
    } else {
      speed_limit_.emplace_back(upstream_speed_limit_.back());
      s_limit_.emplace_back(upstream_s_limit_.back());
      acc_limit_.emplace_back(upstream_a_limit_.back());
    }
  }
  Eigen::VectorXd result(4);
  double min_jerk, t_to_junction, vel_diff, s_diff, veh_cross_time,
      value_ = 0.0;
  Eigen::VectorXd sf(3);
  sf << goal_s, goal_v, sf_a_;
  CalOptimalBoundaryValueByCubic(sf, verify_t_, &result, &min_jerk);
  if (!CaltToJunction(goal_v, sf_a_, &t_to_junction)) {
    LOG_WARN("Calculate t to Junction Failed!");
    return false;
  }
  value_ += t_to_junction;
  if (std::floor(verify_t_ / time_step_) < 2) {
    verify_t_ = 2 * time_step_;
  }
  std::vector<double> acc_reality_(
      std::min(static_cast<int>(std::floor(verify_t_ / time_step_)), 100),
      acc_limit_[0]);
  std::vector<double> vel_reality_(
      std::min(static_cast<int>(std::floor(verify_t_ / time_step_)), 100),
      speed_limit_[0]);
  std::vector<double> s_reality_(
      std::min(static_cast<int>(std::floor(verify_t_ / time_step_)), 100),
      s_limit_[0]);
  std::vector<double> t_reality_(
      std::min(static_cast<int>(std::floor(verify_t_ / time_step_)), 100), 0.0);
  Eigen::VectorXd coeff_a(2), coeff_s(4);
  coeff_a << result[2] * 2.0, result[3] * 6.0;
  coeff_s << result[0], result[1], result[2], result[3];
  for (int i = 0; i < steps - 1; i++) {
    double time = time_step_ * (i + 1);
    Eigen::VectorXd time_a(2), time_s(4);
    time_a << 1.0, std::pow(time, 1);
    time_s << 1.0, std::pow(time, 1), std::pow(time, 2), std::pow(time, 3);
    double acc_rear = (coeff_a.transpose() * time_a)[0];
    double mix_s, mix_acc, mix_speed;
    CalMixOutput(acc_limit_[i + 1], acc_rear, s_reality_[i], vel_reality_[i],
                 &mix_acc, &mix_speed, &mix_s);  // match wtx's sequence (i + 1)
    acc_reality_[i] = mix_acc;
    s_reality_[i + 1] = mix_s;
    vel_reality_[i + 1] = mix_speed;
    t_reality_[i + 1] = time;
  }
  double last_point = steps - 1;
  double last_time = last_point * time_step_;
  Eigen::VectorXd time_a(2);
  time_a << 1.0, std::pow(last_time, 1);
  double acc_rear = (coeff_a.transpose() * time_a)[0];
  acc_reality_.back() =
      std::max(std::min(acc_rear, acc_limit_[last_point]), -5.0);
  t_reality_[last_point] = last_time;
  if (!MakeUpperSpeedCons(vel_reality_, &vel_diff)) {
    LOG_WARN("Make Upper Speed Cons Failed!");
    return false;
  }
  if (!MakeUpperSCons(s_reality_, &s_diff)) {
    LOG_WARN("Make Upper S Cons Failed!");
    return false;
  }
  if (s_diff < -0.15) {
    // LOG_WARN("Violate of S Boundary Constraint (s_diff = {:3f})", s_diff);
    return false;
  }
  if (!CalOtherCrossTime(&veh_cross_time)) {
    LOG_WARN("Calculate Cross Time Failed!");
    return false;
  }
  double cost_efficiency = time_step_ * std::floor(verify_t_ / time_step_) +
                           t_to_junction - veh_cross_time - yield_time_;
  value_ -= 100.0 * std::abs(ego_.sl_velocity.x() - goal_v);
  if (cost_efficiency < -yield_time_) {
    value_ -= 100.0 * cost_efficiency;
  } else if (cost_efficiency < 0.0) {
    value_ -= cost_efficiency;
  }
  // LOG_INFO(
  //     "goal v is {:3f}, goal s is {:3f}, value is {:3f}, special time is
  //     {:3f}", goal_v, goal_s, value_, verify_t_);
  *real_x = goal_v;
  *value = value_;
  *verify_t = verify_t_;
  *t_reality = t_reality_;
  *s_reality = s_reality_;
  *vel_reality = vel_reality_;
  *acc_reality = acc_reality_;
  return true;
}

bool ConflictAreaYieldOptModel::CalFinalS(const double s_bound,
                                          const double goal_v,
                                          double* final_s) {
  // double final_s_ = s_bound -
  //                   (goal_v * ego_.react_time +
  //                    0.5 * std::pow(goal_v, 2) / ego_.max_longitudinal_brake)
  //                    -
  //                   2.4;
  double final_s_ = s_bound - 1.0;
  *final_s = final_s_;
  // if (final_s_ <= 0.0) {
  //   LOG_WARN("Don't Pass RSS Model!");
  //   return false;
  // }
  if (final_s_ <= 1.0) {
    LOG_WARN("Don't Pass RSS Model!");
    return false;
  }
  return true;
}

bool ConflictAreaYieldOptModel::CalMixOutput(
    const double acc_front, const double acc_rear, const double cur_s,
    const double cur_speed, double* mix_acc, double* mix_speed, double* mix_s) {
  double mix_acc_ = std::max(std::min(acc_front, acc_rear), -5.0);
  double mix_speed_, mix_s_;
  Utility::NewtonSecLaw(mix_acc_, cur_s, cur_speed, time_step_, &mix_speed_,
                        &mix_s_);
  *mix_acc = mix_acc_;
  *mix_speed = mix_speed_;
  *mix_s = mix_s_;
  return true;
}

bool ConflictAreaYieldOptModel::Init(const double rho, const double sf_a,
                                     const double yield_time,
                                     double* special_t) {
  rho_ = rho;
  sf_a_ = sf_a;
  yield_time_ = yield_time;
  if (!CalSpecialTime()) {
    LOG_WARN("Don't Should Yield!");
    return false;
  }
  // LOG_INFO("Yield's Special Time is {:3f}", special_t_[0]);
  // CalVehicleSpeedPrediction();
  *special_t = get_special_t();
  return true;
}

bool ConflictAreaYieldOptModel::Planner(const double upper_bound,
                                        const double loss_threshold,
                                        const double lower_bound,
                                        const double step) {
  // CalVehicleSpeedPrediction();
  state_trans_func_ = StateTransType::CUBIC;
  double min_x = std::numeric_limits<double>::infinity();
  double min_val = -std::numeric_limits<double>::infinity();
  double verify_t_match = std::numeric_limits<double>::infinity();
  for (double x = lower_bound; x <= upper_bound; x += step) {
    // LOG_INFO("x is {:3f}", x);
    double val, verify_t, real_x;
    std::vector<double> s_reality_, t_reality_, vel_reality_, acc_reality_;
    switch (state_trans_func_) {
      case StateTransType::QUINTIC:
        if (!ObjectiveFunctionBaseQuintic(x, &val, &verify_t, &real_x,
                                          &s_reality_, &t_reality_,
                                          &vel_reality_, &acc_reality_)) {
          continue;
        }
        break;
      case StateTransType::QUARTIC:
        if (!ObjectiveFunctionBaseQuartic(x, &val, &verify_t, &real_x,
                                          &s_reality_, &t_reality_,
                                          &vel_reality_, &acc_reality_)) {
          continue;
        }
        break;
      case StateTransType::CUBIC:
        if (!ObjectiveFunctionBaseCubic(x, &val, &verify_t, &real_x,
                                        &s_reality_, &t_reality_, &vel_reality_,
                                        &acc_reality_)) {
          continue;
        }
        break;
      default:
        LOG_ERROR("Lack necessary state trans type set up!");
        return false;
    }
    if (val > min_val) {
      min_x = real_x;
      min_val = val;
      verify_t_match = verify_t;
      expected_s_ = s_reality_;
      expected_v_ = vel_reality_;
      expected_a_ = acc_reality_;
      expected_t_ = t_reality_;
    }
  }
  // if (!expected_v_.empty()) {
  //   LOG_INFO("Winner Final Velocity: {:3f}, vel_reality is {:3f}", min_x,
  //            expected_v_.back());
  // }
  if (min_x > upper_bound) {
    LOG_ERROR("Planning Failed!");
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
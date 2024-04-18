#include "conflict_area_rush_model.h"

#include <cmath>

namespace neodrive {
namespace planning {
void ConflictAreaRushOptModel::CalVehicleSpeedPrediction() {
  Eigen::Vector3d xs(5);
  xs << -4.0, -3.0, -2.0, -1.0, 0.0;
  Utility::FittingQuartic(xs, past_vel_records_, &quartic_coeff_);
}

void ConflictAreaRushOptModel::CalVelocityUpper(const double R,
                                                const double width,
                                                const double height,
                                                const double speed_limit_upper,
                                                double* velocity_upper,
                                                double g, double mu) {
  double path_speed_limit =
      std::min(0.5 * g * width * R / height - 0.1, g * R * mu - 0.1);
  *velocity_upper = std::min(speed_limit_upper, path_speed_limit);
}

void ConflictAreaRushOptModel::CalInfluenceFromEgo(const double time,
                                                   const double velocity,
                                                   const double s,
                                                   const double comfort_brake,
                                                   double* max_acc) {
  double ego_velocity, ego_s, x, x_, t = 0.0;
  ego_velocity = ego_.sl_velocity.x() +
                 ego_.max_longitudinal_acc * time;  // worse-case hypothesis
  ego_s = ego_.sl_velocity.x() * time +
          0.5 * ego_.max_longitudinal_acc * std::pow(time, 2);
  x = conflict_area_bound_[3] - ego_.front_suspension - ego_s;
  x_ = -0.5 * std::pow(ego_velocity, 2) / comfort_brake;
  if (x_ < x) {
    t = 1e+8;
  } else {
    double safety_distance =
        ego_.sl_velocity.x() * ego_.react_time +
        0.5 * ego_.max_longitudinal_acc * std::pow(ego_.react_time, 2) +
        0.5 *
            std::pow(ego_.sl_velocity.x() +
                         ego_.max_longitudinal_acc * ego_.react_time,
                     2) /
            comfort_brake;
    x = conflict_area_bound_[3] - ego_.front_suspension - safety_distance;
    t = (x < 0.0) ? 1e-8 : (x / ego_.sl_velocity.x());
  }
  x = conflict_area_bound_[0] - other_agent_.front_suspension - s;
  *max_acc = 2.0 * (x - velocity * t) / std::pow(t, 2);
}

void ConflictAreaRushOptModel::CalVehSafetyDistance(const double max_velocity,
                                                    double* safety_distance) {
  *safety_distance =
      max_velocity * other_agent_.react_time +
      0.5 * std::pow(max_velocity, 2) / other_agent_.max_longitudinal_brake;
}

// sim_t < 0 -> simulate failed
bool ConflictAreaRushOptModel::ForwardSimulation(
    const double max_velocity, const double max_acc, const double s0,
    const double v0, const double forward_t, const double s_goal,
    bool* sim_success, double* sim_t) {
  double t2_max_vel = (max_velocity - v0) / (max_acc + 1e-8);
  double s, s_diff, a, b, c;
  if (t2_max_vel >= forward_t) {
    s = s0 + v0 * forward_t + 0.5 * max_acc * std::pow(forward_t, 2);
    if (s >= s_goal) {
      *sim_success = true;
      s_diff = s_goal - s0;
      a = 0.5 * max_acc;
      b = v0;
      c = -s_diff;
      if (!Utility::SolveQuadraticEquation(a, b, c, sim_t)) {
        LOG_WARN("Quadratic Equation No Root!");
        return false;
      }
    }
  } else {
    double s_accelerate_driving =
        v0 * t2_max_vel + 0.5 * max_acc * std::pow(t2_max_vel, 2);
    if (s0 + s_accelerate_driving >= s_goal) {
      *sim_success = true;
      s_diff = s_goal - s0;
      a = 0.5 * max_acc;
      b = v0;
      c = -s_diff;
      if (!Utility::SolveQuadraticEquation(a, b, c, sim_t)) {
        LOG_WARN("Quadratic Equation No Root!");
        return false;
      }
    } else {
      double t_constant_vel = forward_t - t2_max_vel;
      double s_constant_driving = max_velocity * t_constant_vel;
      s = s0 + s_accelerate_driving + s_constant_driving;
      if (s >= s_goal) {
        *sim_success = true;
        s_diff = s_goal - s0 - s_accelerate_driving;
        *sim_t = s_diff / max_velocity + t2_max_vel;
      }
    }
  }
  bool res = (*sim_t < 0.0) ? false : true;
  return res;
}

bool ConflictAreaRushOptModel::CalSpecialTime() {
  double s_left = conflict_area_bound_[0];
  auto CalRssDistance = [](const double distance, const VehicleInfo& ego,
                           const VehicleInfo& other_agent) -> double {
    double v_diff = std::max(
        0.0, other_agent.sl_velocity.x() +
                 other_agent.react_time * other_agent.max_longitudinal_acc -
                 std::max(0.0, ego.sl_velocity.x() *
                                   std::cos(std::abs(ego.heading -
                                                     other_agent.heading))));
    double dis_agent =
        (v_diff > 0) ? other_agent.sl_velocity.x() * other_agent.react_time +
                           0.5 * other_agent.max_longitudinal_acc *
                               std::pow(other_agent.react_time, 2) +
                           std::pow(v_diff, 2) /
                               (2.0 * other_agent.max_longitudinal_brake)
                     : 0.0;
    double adjust_t =
        2.0 * dis_agent /
        (other_agent.sl_velocity.x() +
         other_agent.react_time * other_agent.max_longitudinal_acc +
         std::max(0.0,
                  ego.sl_velocity.x() *
                      std::cos(std::abs(ego.heading - other_agent.heading))));
    double dis_ego =
        std::max(0.0,
                 ego.sl_velocity.x() *
                     std::cos(std::abs(ego.heading - other_agent.heading))) *
        adjust_t;
    return std::max(distance, dis_agent - dis_ego);
  };
  set_min_distance(key_agent_info_.param.rush_margin_distance);
  min_distance_ = CalRssDistance(min_distance_, ego_, other_agent_);
  if (s_left - min_distance_ < 0.0) {
    LOG_ERROR("Calculate Special Time Failed Due to S Left ({:3f})!",
              s_left - min_distance_);
    return false;
  }
  special_t_ =
      std::min((s_left - min_distance_) / other_agent_.sl_velocity.x(), 100.0);
  LOG_INFO("s left is {:3f}, min distance is {:3f}, velocity is {:3f}", s_left,
           min_distance_, other_agent_.sl_velocity.x());
  if (special_t_ < rush_time_) {
    LOG_ERROR("Rush Time Unable To Meet!");
    return false;
  }
  special_t_ -= rush_time_;
  return true;
}

bool ConflictAreaRushOptModel::CalSpecialTime(const double max_vel) {
  double s_ego2junction = conflict_area_bound_[3] - ego_.front_suspension;
  if (s_ego2junction < 0.0) return false;
  double veh_safety_distance = 0.0;
  CalVehSafetyDistance(max_vel, &veh_safety_distance);
  double t_ego2junction = -1.0;
  if (!Utility::SolveQuadraticEquation(0.5 * ego_.sl_accel.x(),
                                       ego_.sl_velocity.x(), -s_ego2junction,
                                       &t_ego2junction)) {
    LOG_WARN("Quadratic Equation No Root!");
    return false;
  }
  double s_left = conflict_area_bound_[0] - other_agent_.front_suspension;
  double s_goal = s_left - 0.1 * veh_safety_distance;
  double s, v, time = time_step_;

  while (true) {
    v = other_agent_.sl_velocity.x();
    if (v <= 0.0) {
      special_t_ = 1e+8;
      break;
    }
    // s = quartic_coeff_integral_.transpose() * time_item_integral;
    s = v * time;
    // forward simulation
    double max_a_temp, t_veh2junction, sim_t;
    bool sim_success = false;
    if (time < 1.0) {
      max_a_temp = 0.0;
      t_veh2junction = t_ego2junction - time;
      if (t_veh2junction < 0.0) {
        special_t_ = 1e+8;
        break;
      }
      // forward simulation
      ForwardSimulation(max_vel, max_a_temp, s, v, t_veh2junction, s_goal,
                        &sim_success, &sim_t);
      if (sim_success) {
        special_t_ = sim_t + time;
        break;
      }
      time += 0.1;
    } else {
      CalInfluenceFromEgo(time, v, s, -2.0, &max_a_temp);
      max_a_temp = std::max(max_a_temp, 0.0);
      max_a_temp = std::min(max_a_temp, other_agent_.max_longitudinal_acc);
      t_veh2junction = t_ego2junction - time;
      if (t_veh2junction < 0.0) {
        special_t_ = 1e+8;
        break;
      }
      // forward simulation
      ForwardSimulation(max_vel, max_a_temp, s, v, t_veh2junction, s_goal,
                        &sim_success, &sim_t);
      if (sim_success) {
        special_t_ = sim_t + time;
        break;
      }
      time += 0.1;
    }
  }
  return true;
}

bool ConflictAreaRushOptModel::CaltToJunction(const double v, const double acc,
                                              double* t) {
  double t_ = 0.0;
  double d = conflict_area_bound_[2] + ego_.rear_suspension -
             conflict_area_bound_[3] + ego_.front_suspension;
  if (acc < 0.0) {
    double d_ = -0.5 * std::pow(v, 2) / acc;
    if (d_ < d) {
      t_ = 1e+8;
    } else {
      double a = 0.5 * acc;
      double b = v;
      double c = -d;
      if (!Utility::SolveQuadraticEquation(a, b, c, &t_)) {
        LOG_WARN("Quadratic Equation No Root!");
        return false;
      }
    }
  } else {
    double a = 0.5 * acc;
    double b = v;
    double c = -d;
    if (!Utility::SolveQuadraticEquation(a, b, c, &t_)) {
      LOG_WARN("Quadratic Equation No Root!");
      return false;
    }
  }
  *t = t_;
  return true;
}

bool ConflictAreaRushOptModel::CalVehStateAtEgoCross(
    const double v, const double acc, const double t,
    std::vector<double>* veh_state) {
  double t_ = time_step_ * std::floor(t / time_step_);
  // Eigen::VectorXd time_item(5);
  // time_item << std::pow(t_, 0.0), std::pow(t_, 1), std::pow(t_, 2),
  //     std::pow(t_, 3), std::pow(t_, 4);
  // Eigen::VectorXd quartic_coeff_integral_(6), time_item_integral(6);
  // quartic_coeff_integral_ << other_agent_.sl_position.x(),
  // quartic_coeff_[0],
  //     quartic_coeff_[1] / 2.0, quartic_coeff_[2] / 3.0, quartic_coeff_[3]
  //     / 4.0, quartic_coeff_[4] / 5.0;
  // time_item_integral << std::pow(t_, 0), std::pow(t_, 1), std::pow(t_, 2),
  //     std::pow(t_, 3), std::pow(t_, 4), std::pow(t_, 5);
  // double s_veh = (quartic_coeff_integral_.transpose() *
  // time_item_integral)[0]; double v_veh = (quartic_coeff_.transpose() *
  // time_item)[0];
  double s_veh = other_agent_.sl_velocity.x() * t_;
  double v_veh = other_agent_.sl_velocity.x();
  if (s_veh < 0.0 || v_veh < 0.0) return false;
  veh_state->emplace_back(s_veh);
  veh_state->emplace_back(v_veh);
  return true;
}

// upper velocity
bool ConflictAreaRushOptModel::MakeUpperSpeedCons(
    const std::vector<double>& vel_reality, double* vel_diff) {
  double min_diff = 0.0;
  if (vel_reality.size() != speed_limit_.size()) {
    LOG_ERROR("Container size misalignment!");
    return false;
  }
  for (size_t i = 0; i < vel_reality.size(); ++i) {
    double diff = speed_limit_[i] - vel_reality[i];
    if (diff < min_diff) {
      min_diff = diff;
    }
  }
  *vel_diff = min_diff;
  return true;
}

bool ConflictAreaRushOptModel::MakeUpperSCons(
    const std::vector<double>& s_reality,
    const std::vector<double>& vel_reality) {
  if (s_reality.size() != s_limit_.size()) {
    LOG_ERROR("Container size misalignment!");
    return false;
  }
  for (size_t i = 0; i < s_reality.size(); ++i) {
    if (ViolateDistanceCons(vel_reality[i], s_reality[i], s_limit_[i])) {
      return false;
    }
  }
  return true;
}

bool ConflictAreaRushOptModel::CompleteDecision(
    const int limit_size, std::vector<double>& s_reality,
    std::vector<double>& v_reality, std::vector<double>& a_reality) {
  if (s_reality.empty()) {
    return false;
  }
  int size_diff = limit_size - s_reality.size();
  if (size_diff < 0) {
    return false;
  }
  for (int i = 0; i < size_diff; ++i) {
    a_reality.emplace_back(0.0);
    v_reality.emplace_back(v_reality.back());
    s_reality.emplace_back(s_reality.back() + v_reality.back() * time_step_);
  }
  return true;
}

void ConflictAreaRushOptModel::CalOptimalBoundaryValueByQuintic(
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

bool ConflictAreaRushOptModel::CalOtherCrossTime(double* cross_time) {
  double veh_cross_time = 1e+8, time = 0.0;
  double s_ = conflict_area_bound_[0] - other_agent_.front_suspension;
  // Eigen::VectorXd quartic_coeff_integral_(6);
  // quartic_coeff_integral_ << other_agent_.sl_position.x(),
  // quartic_coeff_[0],
  //     quartic_coeff_[1] / 2.0, quartic_coeff_[2] / 3.0, quartic_coeff_[3]
  //     / 4.0, quartic_coeff_[4] / 5.0;
  while (true) {
    // Eigen::VectorXd time_item(5);
    // time_item << std::pow(time, 0), std::pow(time, 1), std::pow(time, 2),
    //     std::pow(time, 3), std::pow(time, 4);
    // Eigen::VectorXd time_item_integral(6);
    // time_item_integral << std::pow(time, 0), std::pow(time, 1),
    //     std::pow(time, 2), std::pow(time, 3), std::pow(time, 4),
    //     std::pow(time, 5);
    // double v = quartic_coeff_.transpose() * time_item;
    double v = other_agent_.sl_velocity.x();
    if (v <= 0.0) break;
    // float s = quartic_coeff_integral_.transpose() * time_item_integral;
    float s = v * time;
    if (s >= s_) {
      veh_cross_time = time;
      break;
    }
    time += time_step_;
  }
  *cross_time = veh_cross_time;
  return true;
}

bool ConflictAreaRushOptModel::VerifySpecialTime(const double v_f,
                                                 double* verify_t) {
  double verify_t_ =
      2.0 * conflict_area_bound_[2] / (ego_.sl_velocity.x() + v_f + 1e-2);
  *verify_t = verify_t_;
  if (verify_t_ > special_t_) {
    return false;
  }
  return true;
}

bool ConflictAreaRushOptModel::ViolateDistanceCons(double v, double current_s,
                                                   double upper_s) {
  double buffer_distance = std::pow(v, 2) / (2.0 * ego_.max_longitudinal_brake);
  return (current_s + buffer_distance) > upper_s;
}

bool ConflictAreaRushOptModel::ObjectiveFunction(const double x, double* value,
                                                 double* verify_t) {
  double verify_t_;
  if (!VerifySpecialTime(x, &verify_t_)) {
    // LOG_WARN(
    //     "When x = {:3f}, not pass special t verify, verify time is {:3f}, "
    //     "special time is {:3f}!",
    //     x, verify_t_, special_t_);
    return false;
  }
  Eigen::VectorXd result(6);
  double sf_s = conflict_area_bound_[2];
  double min_jerk, t_to_junction, vel_diff, s_diff, veh_cross_time,
      value_ = 0.0;
  Eigen::VectorXd sf(3);
  sf << sf_s, x, sf_a_;
  CalOptimalBoundaryValueByQuintic(sf, verify_t_, &result, &min_jerk);
  value_ += rho_ * min_jerk;
  if (!CaltToJunction(x, sf_a_, &t_to_junction)) {
    LOG_WARN("Calculate t to Junction Failed!");
    return false;
  }
  value_ += t_to_junction;
  std::vector<double> vel_reality(std::floor(verify_t_ / time_step_), 0.0);
  std::vector<double> s_reality(std::floor(verify_t_ / time_step_), 0.0);
  std::vector<double> acc_reality(std::floor(verify_t_ / time_step_), 0.0);
  Eigen::VectorXd coeff_a(4), coeff_v(5), coeff_s(6);
  coeff_a << result[2] * 2.0, result[3] * 6.0, result[4] * 12.0,
      result[5] * 20.0;
  coeff_v << result[1], result[2] * 2.0, result[3] * 3.0, result[4] * 4.0,
      result[5] * 5.0;
  coeff_s << result[0], result[1], result[2], result[3], result[4], result[5];
  for (int i = 0; i < std::floor(verify_t_ / time_step_); i++) {
    double time = time_step_ * (i + 1);
    Eigen::VectorXd time_a(4), time_v(5), time_s(6);
    time_a << std::pow(time, 0), std::pow(time, 1), std::pow(time, 2),
        std::pow(time, 3);
    time_v << std::pow(time, 0), std::pow(time, 1), std::pow(time, 2),
        std::pow(time, 3), std::pow(time, 4);
    time_s << std::pow(time, 0), std::pow(time, 1), std::pow(time, 2),
        std::pow(time, 3), std::pow(time, 4), std::pow(time, 5);
    acc_reality[i] = (coeff_a.transpose() * time_a)[0];
    vel_reality[i] = (coeff_v.transpose() * time_v)[0];
    s_reality[i] = (coeff_s.transpose() * time_s)[0];
  }
  CompleteDecision(upstream_speed_limit_.size(), s_reality, vel_reality,
                   acc_reality);
  uint64_t steps = std::floor(verify_t_ / time_step_);
  speed_limit_.clear();
  s_limit_.clear();
  // The situation where yielding fails to solve (reasonable): there is a
  // blockage in front of the lane where the merging vehicle is located during
  // the merging process
  // The situation (risk) of failure to solve the problem of preemptive
  // execution: the window sent by the upstream for 5 seconds is not long
  // enough, which is less than a special time, resulting in the constraint of s
  // being unable to be overcome For the latter, reluctantly switch to another
  // line to do a back up
  for (int i = 0; i < std::max(steps, upstream_speed_limit_.size()); ++i) {
    if (i < upstream_speed_limit_.size()) {
      speed_limit_.emplace_back(upstream_speed_limit_[i]);
      s_limit_.emplace_back(upstream_s_limit_[i]);
    } else {
      speed_limit_.emplace_back(upstream_speed_limit_.back());
      s_limit_.emplace_back(upstream_s_limit_.back());
    }
  }
  if (!MakeUpperSpeedCons(vel_reality, &vel_diff)) {
    LOG_WARN("Make Upper Speed Cons Failed!");
    return false;
  }
  if (!MakeUpperSCons(s_reality, vel_reality)) {
    LOG_WARN("Make Upper S Cons Failed!");
    return false;
  }
  if (!CalOtherCrossTime(&veh_cross_time)) {
    LOG_WARN("Calculate Cross Time Failed!");
    return false;
  }
  std::vector<double> veh_state;
  if (!CalVehStateAtEgoCross(x, sf_a_, std::floor(verify_t_ / time_step_) * 0.1,
                             &veh_state)) {
    LOG_WARN("Calculate State At Ego Cross Failed!");
    return false;
  }
  double cost_rss = -0.5 * std::pow(veh_state[1], 2) /
                        (conflict_area_bound_[0] - veh_state[0]) +
                    other_agent_.max_longitudinal_brake;
  if (cost_rss < 0.0) {
    value_ -= 1e+5 * cost_rss;
  }
  double cost_speed_limit = vel_diff;
  if (cost_speed_limit < 0.0) {
    value_ -= 1e+8 * cost_speed_limit;
  }
  double cost_efficiency = -time_step_ * std::floor(verify_t_ / time_step_) -
                           t_to_junction + veh_cross_time - rush_time_;
  if (cost_efficiency < -rush_time_) {
    value_ -= 1e+5 * cost_efficiency;
  } else if (cost_efficiency < 0.0) {
    value_ -= 1e+3 * cost_efficiency;
  }
  // acc/dec
  double cost_smooth = 0.0;
  for (const auto acc : acc_reality) {
    cost_smooth += std::abs(acc) * 1e+3;
  }
  value_ += cost_smooth;

  *value = value_;
  *verify_t = verify_t_;
  return true;
}

bool ConflictAreaRushOptModel::Init(const double rho, const double sf_a,
                                    const double rush_time, double* special_t) {
  rho_ = rho;
  sf_a_ = sf_a;
  rush_time_ = rush_time;
  // CalSpecialTime(5.56);
  if (!CalSpecialTime()) {
    LOG_WARN("Calculate Special t Failed!");
    return false;
  }
  // CalVehicleSpeedPrediction();
  *special_t = get_special_t();
  return true;
}

bool ConflictAreaRushOptModel::Planner(double upper_bound, double lower_bound,
                                       const double step) {
  // CalVehicleSpeedPrediction();
  // if (!CalSpecialTime(5.56)) {
  //   LOG_WARN("Calculate Special t Failed!");
  //   return false;
  // }
  const auto& speed_limits =
      DataCenter::Instance()->behavior_speed_limits().speed_limits();
  const auto& speed_limits_enable =
      DataCenter::Instance()->behavior_speed_limits().speed_limits_enable();

  double soft_v_bound = std::numeric_limits<double>::infinity();
  double hard_v_bound = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < speed_limits_enable.size(); ++i) {
    if (!speed_limits_enable[i]) {
      continue;
    }
    const auto& speed_limit = speed_limits[i];
    if (speed_limit.constraint_type() == SpeedLimitType::SOFT &&
        speed_limit.upper_bounds_size() == 1) {
      soft_v_bound = std::min(soft_v_bound, speed_limit.upper_bounds().at(0));
    }

    if (speed_limit.constraint_type() == SpeedLimitType::HARD &&
        speed_limit.upper_bounds_size() == 1) {
      LOG_INFO("show infinite speed limit is [{}]",
               speed_limit.upper_bounds().at(0));
      hard_v_bound = std::min(hard_v_bound, speed_limit.upper_bounds().at(0));
    }
  }
  LOG_INFO("hard v bound is {}, soft v bound is {}!", hard_v_bound,
           soft_v_bound);
  if (ego_.sl_velocity.x() > hard_v_bound) {
    LOG_INFO("current velocity is {}, hard velocity bound is {}!",
             ego_.sl_velocity.x(), hard_v_bound);
    return false;
  }

  if (ego_.sl_velocity.x() > soft_v_bound) {
    LOG_INFO("current velocity is {}, soft velocity bound is {}!",
             ego_.sl_velocity.x(), soft_v_bound);
    upper_bound = soft_v_bound;
    lower_bound = std::max(upper_bound - 3.0, 0.0);
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_val = std::numeric_limits<double>::infinity();
  double verify_t_match = std::numeric_limits<double>::infinity();
  for (double x = lower_bound; x <= std::min(upper_bound, soft_v_bound);
       x += step) {
    double val, verify_t;
    if (!ObjectiveFunction(x, &val, &verify_t)) {
      continue;
    }
    if (val < min_val) {
      min_x = x;
      min_val = val;
      verify_t_match = verify_t;
    }
  }
  if (min_x > upper_bound) {
    LOG_ERROR("Planning Failed!");
    return false;
  }

  double min_jerk;
  Eigen::VectorXd opt_res(6);
  Eigen::VectorXd sf(6);
  float sf_s = conflict_area_bound_[2];
  sf << sf_s, min_x, sf_a_;
  CalOptimalBoundaryValueByQuintic(sf, verify_t_match, &opt_res, &min_jerk);
  Eigen::VectorXd coeff_s(6), coeff_v(5), coeff_a(4);
  coeff_s << opt_res[0], opt_res[1], opt_res[2], opt_res[3], opt_res[4],
      opt_res[5];
  coeff_v << opt_res[1], opt_res[2] * 2.0, opt_res[3] * 3.0, opt_res[4] * 4.0,
      opt_res[5] * 5.0;
  coeff_a << opt_res[2] * 2.0, opt_res[3] * 6.0, opt_res[4] * 12.0,
      opt_res[5] * 20.0;
  for (int i = 0; i <= std::floor(verify_t_match / time_step_); i++) {
    double time = time_step_ * i;
    Eigen::VectorXd time_s(6), time_v(5), time_a(4);
    time_a << std::pow(time, 0), std::pow(time, 1), std::pow(time, 2),
        std::pow(time, 3);
    time_v << std::pow(time, 0), std::pow(time, 1), std::pow(time, 2),
        std::pow(time, 3), std::pow(time, 4);
    time_s << std::pow(time, 0), std::pow(time, 1), std::pow(time, 2),
        std::pow(time, 3), std::pow(time, 4), std::pow(time, 5);
    expected_v_.emplace_back((coeff_v.transpose() * time_v)[0]);
    expected_s_.emplace_back((coeff_s.transpose() * time_s)[0]);
    expected_a_.emplace_back((coeff_a.transpose() * time_a)[0]);
    expected_t_.emplace_back(time);
  }
  if (!expected_a_.empty()) {
    expected_a_.back() = 0.0;
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
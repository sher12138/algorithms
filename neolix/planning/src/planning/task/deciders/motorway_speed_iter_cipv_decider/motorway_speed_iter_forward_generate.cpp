#include "motorway_speed_iter_forward_generate.h"

#include <algorithm>
#include <unordered_map>

#include "common_config/config/common_config.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/util/speed_planner_common.h"

using namespace std;
namespace neodrive {
namespace planning {
MotorwayHybirdPid::MotorwayHybirdPid(const IterDeductionEgoState& ego_state,
                                     double range_start_t, double range_end_t,
                                     double target_speed, double speed_limit,
                                     double target_acc) {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  ego_v_start_ = ego_state.ego_v;
  ego_a_start_ = ego_state.ego_a;
  ego_p_start_ = ego_state.ego_p;
  range_start_t_ = range_start_t;
  range_end_t_ = range_end_t;
  step_ = speed_iter_deduction_config.step_time;
  desire_max_a_ = speed_iter_deduction_config.pid_desire_max_a;
  v_set_ = std::min(DataCenter::Instance()->drive_strategy_max_speed(),
                    target_speed);
  target_acc_ = target_acc;
  LOG_INFO("PID V SET IS : {}", v_set_);
  speed_limit_ =
      std::min(DataCenter::Instance()->drive_strategy_max_speed(), speed_limit);
  // speed_limit_ = speed_limit_ < v_set_ ? speed_limit_ : (v_set_ + 0.5);
  p_ = speed_iter_deduction_config.pid_p;
}

DeductionEgoStateSequence MotorwayHybirdPid::CalPidProcess() {
  std::vector<double> ego_v_seris{ego_v_start_};
  std::vector<double> ego_t_seris{range_start_t_};
  std::vector<double> ego_p_seris{ego_p_start_};
  std::vector<double> ego_a_seris{ego_a_start_};
  if (range_end_t_ < range_start_t_) {
    hybird_pid_deduction_result_.Reset();
    LOG_INFO("start_t > end_t ,triger this !!!!!!!!!! !");
    return hybird_pid_deduction_result_;
  }
  int deduction_step = ((range_end_t_ - range_start_t_) < 0.1)
                           ? 1
                           : std::round((range_end_t_ - range_start_t_) / 0.1);
  double last_acc_cmd = ego_a_start_;

  for (int i = 0; i < deduction_step; i++) {
    if (v_set_ < 0.05) {
      v_set_ = 0.05;
    }
    double acc_cmd =
        desire_max_a_ * (1 - std::pow((ego_v_seris.back() / v_set_), 2));
    acc_cmd = std::max(acc_cmd, -4.0);
    double theta_a = p_ * (acc_cmd - last_acc_cmd);
    ego_a_seris.push_back(last_acc_cmd + theta_a);
    if (ego_a_seris.back() > target_acc_ && target_acc_ < -0.05) {
      ego_a_seris.pop_back();
      ego_a_seris.emplace_back(target_acc_);
    }
    last_acc_cmd = ego_a_seris.back();
    if (ego_v_seris.back() + step_ * last_acc_cmd <= 0.) {
      ego_v_seris.push_back(0.);
    } else {
      ego_v_seris.push_back(ego_v_seris.back() + step_ * last_acc_cmd);
    }
    if (ego_v_seris.back() > speed_limit_) {
      ego_v_seris.pop_back();
      ego_v_seris.push_back(speed_limit_);
      ego_a_seris.pop_back();
      if (ego_v_seris.size() >= 2) {
        double cal_limit =
            (ego_v_seris.back() - ego_v_seris[ego_v_seris.size() - 2]) / step_;
        cal_limit = std::max(cal_limit, -3.0);
        ego_a_seris.push_back(cal_limit);
      } else {
        ego_a_seris.push_back(0);
      }
    }
    ego_p_seris.push_back(ego_p_seris.back() + step_ * ego_v_seris.back());
    ego_t_seris.push_back(ego_t_seris.back() + step_);
  }
  hybird_pid_deduction_result_.Reset();
  hybird_pid_deduction_result_.deduction_ego_a_sequence = ego_a_seris;
  hybird_pid_deduction_result_.deduction_ego_p_sequence = ego_p_seris;
  hybird_pid_deduction_result_.deduction_ego_v_sequence = ego_v_seris;
  hybird_pid_deduction_result_.deduction_ego_t_sequence = ego_t_seris;
  return hybird_pid_deduction_result_;
}

MotorwayHybirdIDM::MotorwayHybirdIDM(
    const IterDeductionEgoState& ego_state_idm,
    const std::vector<ObsDecisionBound>& obs_low_points, bool if_virtual,
    bool if_vehicle, bool if_bump, double upper_limit, double target_speed,
    double target_acc) {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  ego_v_start_ = ego_state_idm.ego_v;
  ego_a_start_ = ego_state_idm.ego_a;
  ego_p_start_ = ego_state_idm.ego_p;
  start_theta_s_ = obs_low_points[0].obs_s - ego_p_start_;
  obs_follow_info_ = obs_low_points;
  if_virtual_ = if_virtual;
  if_vehicle_ = if_vehicle;

  range_start_t_ = obs_low_points[0].time;
  range_end_t_ = obs_low_points.back().time;
  step_ = speed_iter_deduction_config.step_time;
  p_ = speed_iter_deduction_config.idm_p;
  headway_ = speed_iter_deduction_config.headway;
  if (if_virtual_) {
    default_space_ = speed_iter_deduction_config.default_space_for_virtual;
  } else if (if_bump) {
    default_space_ = 1.0;
  } else {
    if (ego_v_start_ > FLAGS_planning_adc_stop_velocity_threshold ||
        obs_low_points.size() == 0 ||
        obs_low_points.at(0).obs_speed > kMathEpsilon || !if_vehicle_) {
      default_space_ = speed_iter_deduction_config.default_space;
    } else {
      default_space_ = speed_iter_deduction_config.default_space_for_stop;
      LOG_INFO("default_space = {:.2f}, obs v = {:.2f}, adc v = {:.2f}",
               default_space_, obs_low_points.at(0).obs_speed, ego_v_start_);
    }
  }
  desire_max_a_ = speed_iter_deduction_config.desire_max_a;
  upper_limit_ = upper_limit < 99.0 ? upper_limit : 100;

  LOG_INFO("show cruise speed limit is {}",
           DataCenter::Instance()->drive_strategy_max_speed());
  target_speed_ = std::min(DataCenter::Instance()->drive_strategy_max_speed(),
                           target_speed);
  target_speed_ = std::max(target_speed_, 0.1);
  target_acc_ = target_acc;
}

DeductionEgoStateSequence MotorwayHybirdIDM::CalHybirdIDMProcess() {
  std::vector<double> cal_obj_distance{start_theta_s_};
  std::vector<double> ego_v_seris{ego_v_start_};
  std::vector<double> ego_t_seris{range_start_t_};
  std::vector<double> ego_p_seris{ego_p_start_};
  std::vector<double> ego_a_seris{ego_a_start_};
  int deduction_step = ((range_end_t_ - range_start_t_) < 0.1)
                           ? 1
                           : std::round((range_end_t_ - range_start_t_) / 0.1);
  for (int i = 0; i < deduction_step; i++) {
    double headway_var = headway_ + obs_follow_info_[i].obs_speed / 8;
    double S_star = default_space_ + headway_var * ego_v_seris.back() +
                    ego_v_seris.back() *
                        (ego_v_seris.back() - obs_follow_info_[i].obs_speed) /
                        3.098;
    double adjust_para = 0.2;
    if (cal_obj_distance.back() <= 0) {
      adjust_para = -5.0;
    }
    double abs_dis;
    if (std::abs(cal_obj_distance.back()) < 0.02) {
      abs_dis = 0.02;
    } else {
      abs_dis = cal_obj_distance.back();
    }
    double acc_cmd =
        desire_max_a_ * (1 +
                         (obs_follow_info_[i].obs_speed - ego_v_seris.back()) /
                             (ego_v_seris.back() + 1.5) -
                         std::pow(ego_v_seris.back() / (target_speed_), 4) -
                         std::pow((S_star / abs_dis + adjust_para), 2));

    acc_cmd = std::max(acc_cmd, -4.0);
    ego_a_seris.push_back(ego_a_seris.back() +
                          p_ * (acc_cmd - ego_a_seris.back()));

    if (ego_a_seris.back() > target_acc_ && target_acc_ < -0.05) {
      ego_a_seris.pop_back();
      ego_a_seris.emplace_back(target_acc_);
    }

    if ((ego_v_seris.back() + step_ * ego_a_seris.back()) <= 0.) {
      ego_v_seris.push_back(0.);
    } else {
      ego_v_seris.push_back(ego_v_seris.back() + step_ * ego_a_seris.back());
    }
    if (ego_v_seris.back() > upper_limit_) {
      ego_v_seris.pop_back();
      ego_v_seris.push_back(upper_limit_);
      ego_a_seris.pop_back();
      if (ego_v_seris.size() >= 2) {
        double cal_limit =
            (ego_v_seris.back() - ego_v_seris[ego_v_seris.size() - 2]) / step_;
        cal_limit = std::max(cal_limit, -3.0);
        ego_a_seris.push_back(cal_limit);
      } else {
        ego_a_seris.push_back(0.);
      }
    }
    ego_t_seris.push_back(ego_t_seris.back() + step_);
    ego_p_seris.push_back(ego_p_seris.back() + step_ * ego_v_seris.back());
    cal_obj_distance.push_back(obs_follow_info_[i].obs_s - ego_p_seris.back());
  }
  hybird_idm_deduction_result_.Reset();
  hybird_idm_deduction_result_.deduction_ego_a_sequence = ego_a_seris;
  hybird_idm_deduction_result_.deduction_ego_p_sequence = ego_p_seris;
  hybird_idm_deduction_result_.deduction_ego_v_sequence = ego_v_seris;
  hybird_idm_deduction_result_.deduction_ego_t_sequence = ego_t_seris;
  return hybird_idm_deduction_result_;
}

MotorwayCheckDeductionCollisionCheck::MotorwayCheckDeductionCollisionCheck(
    const DeductionEgoStateSequence& ego_forward_deduction,
    const MotorwaySpeedObsExtend& obs_decision) {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  step_ = speed_iter_deduction_config.step_time;
  collision_buffer_ = speed_iter_deduction_config.collision_buffer;
  collision_buffer_l_ = speed_iter_deduction_config.collision_buffer_l;
  collision_buffer_u_ = speed_iter_deduction_config.collision_buffer_u;

  obs_decision_ = obs_decision;
  obs_is_virtual_ = false;
  obs_is_virtual_ = obs_decision_.IfVirtual();
  obs_is_reverse_ = false;
  obs_is_reverse_ = obs_decision_.IfReverse();
  ego_p_sequence_.clear();
  ego_t_sequence_.clear();
  ego_p_sequence_ = ego_forward_deduction.deduction_ego_p_sequence;
  ego_v_sequence_ = ego_forward_deduction.deduction_ego_v_sequence;
  ego_t_sequence_ = ego_forward_deduction.deduction_ego_t_sequence;
}

std::pair<double, int> MotorwayCheckDeductionCollisionCheck::FindClosestPoint(
    const std::vector<double>& list, double target) {
  int left = 0;
  int right = list.size() - 1;
  double closest = list[0];
  int closest_index = 0;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    double mid_value = list[mid];

    if (mid_value == target) {
      return std::make_pair(mid_value, mid);
    }

    if (std::abs(mid_value - target) < std::abs(closest - target)) {
      closest = mid_value;
      closest_index = mid;
    }

    if (mid_value < target) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }
  return std::make_pair(closest, closest_index);
}

std::pair<bool, int>
MotorwayCheckDeductionCollisionCheck::CheckAndCalCollisionPoint() {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  auto CalCollisionBuffer = [](double ego_speed, double thw, double obs_speed) {
    double a_ego_acc = 2.0;
    double react_time = 0.3;
    double a_ego_brake = 5.0;
    double a_obs_brake = 5.0;
    double rss_d =
        ego_speed * react_time + std::pow(0.5 * a_ego_acc * react_time, 2) +
        std::pow((ego_speed + react_time * a_ego_acc), 2) / (2 * a_ego_brake) -
        std::pow(obs_speed, 2) / (2 * a_obs_brake);
    rss_d = rss_d <= 0 ? 0 : rss_d;
    return std::min(ego_speed * thw, rss_d);
  };

  auto CalCollisionBufferForTake = [](double ego_speed, double thw,
                                      double obs_speed) {
    double a_obs_acc = 2.0;
    double react_time = 0.3;
    double a_ego_brake = 5.0;
    double a_obs_brake = 5.0;
    double rss_d =
        obs_speed * react_time + std::pow(0.5 * a_obs_acc * react_time, 2) +
        std::pow((obs_speed + react_time * a_obs_acc), 2) / (2 * a_obs_brake) -
        std::pow(ego_speed, 2) / (2 * a_ego_brake);
    rss_d = rss_d <= 0 ? 0 : rss_d;
    return std::max(ego_speed * thw, rss_d);
  };

  if (ego_t_sequence_.size() == 0) {
    return std::make_pair(false, 0);
  }
  if (ego_t_sequence_[0] >= obs_decision_.EndTime() ||
      ego_t_sequence_.back() <= obs_decision_.StartTime()) {
    return std::make_pair(false, 0);
  } else {
    double time_lap_start =
        std::max(obs_decision_.StartTime(), ego_t_sequence_[0]);
    double time_lap_end =
        std::min(obs_decision_.EndTime(), ego_t_sequence_.back());
    int num_points = std::round((time_lap_end - time_lap_start) / step_);
    num_points = (num_points < 1) ? 1 : num_points;
    std::vector<ObsDecisionBound> point_low_vector =
        obs_decision_.InterpolatePointsForLow();
    std::vector<ObsDecisionBound> point_upper_vector =
        obs_decision_.InterpolatePointsForUpper();
    std::vector<double> obs_point_time{};
    obs_point_time.clear();
    std::transform(point_low_vector.begin(), point_low_vector.end(),
                   std::back_inserter(obs_point_time),
                   [](const ObsDecisionBound& obsdecisionbound) {
                     return obsdecisionbound.time;
                   });
    bool has_overlap = false;
    for (int i = 0; i < num_points; i++) {
      std::pair<double, int> t_target_ego =
          FindClosestPoint(ego_t_sequence_, (time_lap_start + i * step_));
      double ego_pos = ego_p_sequence_[t_target_ego.second];
      double ego_v = ego_v_sequence_[t_target_ego.second];
      std::pair<double, int> t_target_obs =
          FindClosestPoint(obs_point_time, (time_lap_start + i * step_));
      double obs_pos_l = point_low_vector[t_target_obs.second].obs_s;
      double obs_pos_u = point_upper_vector[t_target_obs.second].obs_s;
      if (i == 0) {
        double collision_buffer_l_for_first = collision_buffer_l_;
        double collision_buffer_u_for_first = collision_buffer_u_;
        if (!obs_is_virtual_) {
          collision_buffer_l_for_first =
              collision_buffer_l_ +
              CalCollisionBuffer(
                  ego_v, speed_iter_deduction_config.collision_convergence_thw,
                  obs_decision_.ObsSpeed()) +
              0.1 * ego_v;

          collision_buffer_u_for_first =
              collision_buffer_u_ +
              CalCollisionBufferForTake(
                  ego_v, speed_iter_deduction_config.collision_convergence_ttw,
                  obs_decision_.ObsSpeed() + 0.4);
        }

        if (ego_pos <= (obs_pos_u + collision_buffer_u_for_first) &&
            ego_pos >= (obs_pos_l - collision_buffer_l_for_first)) {
          has_overlap = true;
          LOG_INFO("collision check ! first point has collisioni");
          return std::make_pair(true, t_target_ego.second);
        }
      }
      if (i == num_points - 1) {
        double collision_buffer_l_for_end = collision_buffer_l_;
        double collision_buffer_u_for_end = collision_buffer_u_;
        if (!obs_is_virtual_) {
          collision_buffer_l_for_end =
              collision_buffer_l_ +
              CalCollisionBuffer(
                  ego_v, speed_iter_deduction_config.collision_convergence_thw,
                  obs_decision_.ObsSpeed()) +
              0.1 * ego_v;

          collision_buffer_u_for_end =
              collision_buffer_u_ +
              CalCollisionBufferForTake(
                  ego_v, speed_iter_deduction_config.collision_convergence_ttw,
                  obs_decision_.ObsSpeed());
        }
        if (!obs_is_virtual_ && obs_is_reverse_) {
          collision_buffer_l_for_end =
              collision_buffer_l_ +
              CalCollisionBuffer(
                  ego_v, speed_iter_deduction_config.collision_convergence_thw,
                  obs_decision_.ObsSpeed()) +
              0.25 * ego_v + 0.4;
          collision_buffer_u_for_end =
              collision_buffer_u_ +
              CalCollisionBufferForTake(
                  ego_v, speed_iter_deduction_config.collision_convergence_thw,
                  obs_decision_.ObsSpeed() + 0.2);
        }
        if (ego_pos <= (obs_pos_u + collision_buffer_u_for_end) &&
            ego_pos >= (obs_pos_l - collision_buffer_l_for_end)) {
          has_overlap = true;
          LOG_INFO("collision check ! last point has collisioni");
          return std::make_pair(true, t_target_ego.second);
        }
      }
      if (ego_pos <= (obs_pos_u + collision_buffer_u_) &&
          ego_pos >= (obs_pos_l - collision_buffer_l_)) {
        has_overlap = true;
        return std::make_pair(true, t_target_ego.second);
      }
    }
    return std::make_pair(false, 0);
  }
}
}  // namespace planning
}  // namespace neodrive

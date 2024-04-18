#include "motorway_speed_iter_obs_decision_extend.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

MotorwaySpeedObsExtend::MotorwaySpeedObsExtend(
    int id, double start_t, double end_t, double start_s_l, double start_s_u,
    double end_s_l, double end_s_u, bool if_reverse, bool if_virtual,
    bool if_vehicle, bool if_bump, std::vector<STPoint> lower_points,
    std::vector<STPoint> upper_points, bool if_dynamic) {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  obs_id_ = id;
  start_t_ = start_t;
  end_t_ = end_t;
  start_s_l_ = start_s_l;
  start_s_u_ = start_s_u;
  end_s_l_ = end_s_l;
  end_s_u_ = end_s_u;
  obs_speed_ = (end_s_l - start_s_l) / (end_t - start_t);
  extend_speed_ = 0;
  back_extend_t_ = start_t_;
  back_extend_s_l_ = start_s_l_;
  back_extend_s_u_ = start_s_u_;
  if_reverse_ = if_reverse;
  step_ = speed_iter_deduction_config.step_time;
  if_virtual_ = if_virtual;
  if_vehicle_ = if_vehicle;
  if_bump_ = if_bump;
  lower_points_ = lower_points;
  upper_points_ = upper_points;
  if_dynamic_ = if_dynamic;
}

std::pair<double, int> MotorwaySpeedObsExtend::FindClosestPoint(
    const std::vector<STPoint>& list, double target) {
  int left = 0;
  int right = list.size() - 1;
  double closest = list[0].t();
  int closest_index = 0;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    double mid_value = list[mid].t();

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

std::vector<ObsDecisionBound> MotorwaySpeedObsExtend::InterpolatePoint(
    double x1, double y1, double x2, double y2, bool if_low) {
  std::vector<ObsDecisionBound> obs_decision_bound{};
  obs_decision_bound.clear();
  double slope;
  if (x1 == x2) {
    slope = 0;
  } else {
    slope = (y2 - y1) / (x2 - x1);
  }
  if (extend_ == true && if_reverse_ == false) {
    LOG_INFO(
        "extend start time for forward driving obs , id : {}, after "
        "extend "
        ",start t is : {}",
        obs_id_, back_extend_t_);
    double num_points_extend = std::round((start_t_ - back_extend_t_) / step_);
    for (int i = 0; i < num_points_extend; i++) {
      ObsDecisionBound obs_decision_bound_point{back_extend_t_ + i * step_, y1,
                                                extend_speed_};
      obs_decision_bound.emplace_back(obs_decision_bound_point);
    }
  }
  if (extend_ == true && if_reverse_ == true) {
    LOG_INFO(
        "extend start time for forward driving obs , id : {}, after "
        "extend "
        ",start t is : {}",
        obs_id_, back_extend_t_);
    double num_points_reverse = std::round((start_t_ - back_extend_t_) / step_);
    for (int i = 0; i < num_points_reverse; i++) {
      double s_value;
      if (y1 == start_s_u_) {
        s_value = back_extend_s_u_ + i * step_ * slope;
      } else {
        s_value = back_extend_s_l_ + i * step_ * slope;
      }
      ObsDecisionBound obs_decision_bound_point{back_extend_t_ + i * step_,
                                                s_value, obs_speed_};
      obs_decision_bound.emplace_back(obs_decision_bound_point);
    }
  }
  double num_points_orin = round((x2 - x1) / step_) + 1;
  for (int i = 0; i < num_points_orin; i++) {
    double time_orin = x1 + i * step_;
    double s_orin = y1 + (time_orin - x1) * slope;
    ObsDecisionBound obs_decision_bound_point{time_orin, s_orin, obs_speed_};
    obs_decision_bound.emplace_back(obs_decision_bound_point);
  }

  if (if_dynamic_) {
    int index_gap =
        10 * (lower_points_[0].t() - obs_decision_bound.front().time);
    int index = index_gap >= 0 ? index_gap : 0;

    for (int i = index; i < obs_decision_bound.size(); i++) {
      if (if_low) {
        auto find_index =
            FindClosestPoint(lower_points_, obs_decision_bound[i].time);
        obs_decision_bound[i].obs_s = lower_points_[find_index.second].s();
        // obs_decision_bound[i].time = lower_points_[find_index.second].t();
      } else {
        auto find_index =
            FindClosestPoint(upper_points_, obs_decision_bound[i].time);
        obs_decision_bound[i].obs_s = upper_points_[find_index.second].s();
        // obs_decision_bound[i].time = upper_points_[find_index.second].t();
      }
    }
  }
  // std::cout << "obs end time : " << obs_decision_bound.back().time
  //           << " , origin end t : " << lower_points_.back().t() << std::endl;

  return obs_decision_bound;
}

ObsBoundPolySeris MotorwaySpeedObsExtend::IterExtendStart() {
  const auto& speed_iter_deduction_config = config::PlanningConfig::Instance()
                                                ->planning_research_config()
                                                .speed_iter_deduction;
  ObsBoundPolySeris obs_bound_poly_seris;
  obs_bound_poly_seris.Reset();
  double extend_time = speed_iter_deduction_config.back_extend_time;
  if (start_t_ == end_t_) {
    LOG_INFO("osb decision is wrong ,time gap is 0 : {}");
  }
  if (back_extend_t_ > extend_time) {
    back_extend_t_ = back_extend_t_ - extend_time;
  } else {
    back_extend_t_ = 0;
  }
  back_extend_s_l_ = start_s_l_;
  back_extend_s_u_ = start_s_u_;
  if (if_reverse_) {
    back_extend_s_l_ = start_s_l_ + (end_s_l_ - start_s_l_) /
                                        (end_t_ - start_t_) *
                                        (back_extend_t_ - start_t_);
    back_extend_s_u_ = start_s_u_ + (end_s_u_ - start_s_u_) /
                                        (end_t_ - start_t_) *
                                        (back_extend_t_ - start_t_);
  }
  extend_ = true;
  obs_bound_poly_seris.lower_obs_decision_poly_seris =
      InterpolatePointsForLow();
  obs_bound_poly_seris.upper_obs_decision_poly_seris =
      InterpolatePointsForUpper();
  return obs_bound_poly_seris;
}

std::vector<ObsDecisionBound>
MotorwaySpeedObsExtend::InterpolatePointsForLow() {
  return InterpolatePoint(start_t_, start_s_l_, end_t_, end_s_l_, true);
}

std::vector<ObsDecisionBound>
MotorwaySpeedObsExtend::InterpolatePointsForUpper() {
  return InterpolatePoint(start_t_, start_s_u_, end_t_, end_s_u_, false);
}

void MotorwaySpeedObsExtend::ResetExtendOrder() { extend_ = false; }
}  // namespace planning
}  // namespace neodrive

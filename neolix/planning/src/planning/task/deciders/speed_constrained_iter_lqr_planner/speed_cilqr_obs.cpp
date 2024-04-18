#include "speed_cilqr_obs.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

SpeedCilqrObsProcess::SpeedCilqrObsProcess(double id, double start_t,
                                           double end_t, double start_s_l,
                                           double start_s_u, double end_s_l,
                                           double end_s_u, bool if_reverse,
                                           bool if_virtual, bool if_vehicle) {
  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;
  obs_id_ = id;
  start_t_ = round(start_t * 10.0) / 10.0;
  end_t_ = round(end_t * 10.0) / 10.0;
  start_s_l_ = start_s_l;
  start_s_u_ = start_s_u;
  end_s_l_ = end_s_l;
  end_s_u_ = end_s_u;
  obs_speed_ = (end_s_l - start_s_l) / (end_t - start_t);
  if_reverse_ = if_reverse;
  step_ = speed_cilqr_conf.step_time;
  if_virtual_ = if_virtual;
  if_vehicle_ = if_vehicle;
}

std::vector<ObsDecisionBound> SpeedCilqrObsProcess::InterpolatePoint(
    double x1, double y1, double x2, double y2) {
  std::vector<ObsDecisionBound> obs_decision_bound{};
  obs_decision_bound.clear();
  double slope;
  if (x1 == x2) {
    slope = 0;
  } else {
    slope = (y2 - y1) / (x2 - x1);
  }

  double num_points_orin = round((x2 - x1) / step_) + 1;
  for (int i = 0; i < num_points_orin; i++) {
    double time_orin = x1 + i * step_;
    double s_orin = y1 + (time_orin - x1) * slope;
    ObsDecisionBound obs_decision_bound_point{time_orin, s_orin, obs_speed_};
    obs_decision_bound.emplace_back(std::move(obs_decision_bound_point));
  }
  return obs_decision_bound;
}

std::vector<ObsDecisionBound> SpeedCilqrObsProcess::InterpolatePointsForLow() {
  return InterpolatePoint(start_t_, start_s_l_, end_t_, end_s_l_);
}

std::vector<ObsDecisionBound>
SpeedCilqrObsProcess::InterpolatePointsForUpper() {
  return InterpolatePoint(start_t_, start_s_u_, end_t_, end_s_u_);
}

}  // namespace planning
}  // namespace neodrive

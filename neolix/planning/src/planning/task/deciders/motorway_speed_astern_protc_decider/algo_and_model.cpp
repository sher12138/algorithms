#include "algo_and_model.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kObservetionTheta = 25 / 180.0 * M_PI;
constexpr double kSafeLateralDistance = 2.5;
constexpr double kSafeSpeed = 10.0 / 3.6;
constexpr double kMaxDeacc = 2.0;
}  // namespace

RssDistanceUtils::RssDistanceUtils(const Obstacle& obs,
                                   const EgoState& ego_state) {
  obs_ptr_ = &obs;
  ego_state_ptr_ = &ego_state;

  if (!DataCheck()) {
    LOG_ERROR("DataCheck error!");
    return;
  };

  CalRSSLateralDistance();
  CalRSSLongtiDistance();
  LOG_INFO("lateral_min_distance: {:.3f} longti_min_distance: {:.3f}",
           lateral_min_distance_, longti_min_distance_);
}

bool RssDistanceUtils::DataCheck() {
  if (ego_state_ptr_->duration < 0.0) {
    LOG_ERROR("ego_state_ptr_->duration < 0.0");
    return false;
  }
  if (ego_state_ptr_->alpha_max < 0.0) {
    LOG_ERROR("ego_state_ptr_->alpha_max < 0.0");
    return false;
  }
  if (ego_state_ptr_->beta_min < 0.0) {
    LOG_ERROR("ego_state_ptr_->beta_min < 0.0");
    return false;
  }
  if (ego_state_ptr_->beta_max < 0.0) {
    LOG_ERROR("ego_state_ptr_->beta_max < 0.0");
    return false;
  }
  if (ego_state_ptr_->speed < 0.0) {
    LOG_ERROR("ego_state_ptr_->speed < 0.0");
    return false;
  }
  if (obs_ptr_->speed() < 0.0) {
    LOG_ERROR("obs_ptr_->speed() < 0.0");
    return false;
  }
  return true;
}

void RssDistanceUtils::CalRSSLateralDistance() {
  double heading_diff =
      normalize_angle(ego_state_ptr_->heading - obs_ptr_->heading());
  bool direction = obs_ptr_->center_sl().l() > ego_state_ptr_->l ? true : false;

  double v_obs = obs_ptr_->speed() > 1
                     ? obs_ptr_->speed() * std::fabs(std::sin(heading_diff))
                     : 0.5;
  double t = std::max(
      std::pow(ego_state_ptr_->speed, 2) / 2 / ego_state_ptr_->beta_max, 0.5);
  lateral_min_distance_ = v_obs * t;
}

void RssDistanceUtils::CalRSSLongtiDistance() {
  double heading_diff =
      normalize_angle(ego_state_ptr_->heading - obs_ptr_->heading());

  double de_s1 = std::pow(ego_state_ptr_->speed + ego_state_ptr_->duration *
                                                      ego_state_ptr_->alpha_max,
                          2) /
                 (2 * ego_state_ptr_->beta_min);
  double de_s2 =
      std::pow(obs_ptr_->speed(), 2) / (2 * ego_state_ptr_->beta_mean);

  longti_min_distance_ =
      de_s1 + de_s2 * (std::cos(heading_diff) > 0. ? 1. : -1.);
}
RssDistanceUtils::~RssDistanceUtils() {}

ActionMap::ActionMap(RssDistanceUtils rss_info) {
  rss_info_ = std::move(rss_info);
  action_ = std::make_pair(std::numeric_limits<double>::max(), 0.);
  GenerateActions();
}

ActionMap::~ActionMap() {}

void ActionMap::GenerateActions() {
  LOG_INFO("obs min_l: {:.3f} max_l: {:.3f}", rss_info_.obs_ptr_->min_l(),
           rss_info_.obs_ptr_->max_l());

  double real_lateral_distance =
      std::fmin(
          std::fabs(rss_info_.obs_ptr_->min_l() - rss_info_.ego_state_ptr_->l),
          std::fabs(rss_info_.obs_ptr_->max_l() -
                    rss_info_.ego_state_ptr_->l)) -
      rss_info_.ego_state_ptr_->width / 2;
  double real_longti_distance =
      std::fabs(rss_info_.obs_ptr_->min_s() - rss_info_.ego_state_ptr_->s -
                rss_info_.ego_state_ptr_->length);

  double min_lateral_distance = rss_info_.GetLateralMinDistance();
  double min_longti_distance = rss_info_.GetLongtiMinDistance();

  GenerateSpeedLimit(min_lateral_distance, min_longti_distance,
                     real_lateral_distance, real_longti_distance);
}

bool ActionMap::GenerateSpeedLimit(const double min_lateral_distance,
                                   const double min_longti_distance,
                                   const double real_lateral_distance,
                                   const double real_longti_distance) {
  LOG_INFO(
      "min_lateral_distance: {:.3f} min_longti_distance: {:.3f} "
      "real_lateral_distance: {:.3f} real_longti_distance: {:.3f}",
      min_lateral_distance, min_longti_distance, real_lateral_distance,
      real_longti_distance);
  LOG_INFO("heading diff:{:.3f}",
           rss_info_.ego_state_ptr_->heading_diff / M_PI * 180);
  double heading_diff =
      normalize_angle(rss_info_.ego_state_ptr_->heading - rss_info_.obs_ptr_->heading());
  // static obs
  if (rss_info_.obs_ptr_->speed() < 0.1) {
    if (real_lateral_distance > kSafeLateralDistance) {
      LOG_INFO("far from distance > {:.3f}", kSafeLateralDistance);
      return false;
    }
  }
  // non static
  if ((rss_info_.obs_ptr_->center_sl().l() > rss_info_.ego_state_ptr_->l &&
       heading_diff > 0.) ||
      (rss_info_.obs_ptr_->center_sl().l() < rss_info_.ego_state_ptr_->l &&
       heading_diff < 0.)) {
    if (real_lateral_distance > kSafeLateralDistance) {
      LOG_INFO("far from distance > {:.3f}", kSafeLateralDistance);
      return false;
    }
  } else {
    if (real_lateral_distance > min_lateral_distance + 1.0) {
      LOG_INFO("ego real lateral distance {:.3f} > min lateral distance {:.3f}",
               real_lateral_distance, min_lateral_distance);

      action_ = std::make_pair(
          std::max(rss_info_.ego_state_ptr_->speed, kSafeSpeed), 0.0);
      return false;
    };
  }
  double safe_stop_dis{};
  GenerateSafeKeepDistance(safe_stop_dis);
  double max_safe_dis = safe_stop_dis + min_longti_distance;
  LOG_INFO("safe_stop_dis:{:.3f}, max_safe_dis:{:.3f}", safe_stop_dis,
           max_safe_dis);

  double need_a = 0.0;
  if (max_safe_dis < real_longti_distance) {
    need_a = std::pow(rss_info_.ego_state_ptr_->speed, 2) / 2 /
             (real_longti_distance - max_safe_dis + 1e-6);
    action_ = std::make_pair(
        std::min(action_.first - need_a * 0.1, action_.first), 0.0);
    if (action_.first < 0.0) {
      action_ = std::make_pair(0.0, 0.0);
    }
  } else {
    action_ = std::make_pair(action_.first - kMaxDeacc * 0.1, -kMaxDeacc);
  }
  return true;
}

void ActionMap::GenerateSafeKeepDistance(double& safe_stop_dis) {
  double max_dis =
      std::abs(rss_info_.obs_ptr_->max_l() - rss_info_.obs_ptr_->min_l()) / 2;

  safe_stop_dis = max_dis / std::tan(kObservetionTheta / 2);
  safe_stop_dis = std::sqrt(safe_stop_dis * safe_stop_dis -
                            std::pow(rss_info_.ego_state_ptr_->l, 2));
}

}  // namespace planning
}  // namespace neodrive

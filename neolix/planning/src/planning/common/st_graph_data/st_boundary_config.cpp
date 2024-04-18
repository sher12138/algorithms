#include "st_boundary_config.h"

#include "common_config/config/common_config.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

STBoundaryConfiguration::STBoundaryConfiguration() {}

void STBoundaryConfiguration::update_inner_parameters(const int mode) {
  current_mode_ = speed_planner_common::GetCurrentModeName(mode);
  frase_from_config();
  // if (mode == MultiLevelMode::MLM_HIGHSPEED) {
  //   static_obs_stop_dis_ = default_static_obs_stop_dis_;
  //   minimal_follow_time_ = default_minimal_follow_time_;
  // } else if (mode == MultiLevelMode::MLM_NORMAL) {
  //   static_obs_stop_dis_ = std::max(6.0, default_static_obs_stop_dis_ * 0.9);
  //   minimal_follow_time_ = std::max(default_minimal_follow_time_ * 0.9, 3.0);
  // } else if (mode == MultiLevelMode::MLM_PASSINGBY) {
  //   static_obs_stop_dis_ = std::max(5.0, default_static_obs_stop_dis_ * 0.8);
  //   minimal_follow_time_ = std::max(default_minimal_follow_time_ * 0.9, 3.0);
  // } else if (mode == MultiLevelMode::MLM_PASSINGTHROUGH) {
  //   static_obs_stop_dis_ = std::max(5.0, default_static_obs_stop_dis_ * 0.6);
  //   minimal_follow_time_ = std::max(default_minimal_follow_time_ * 0.9, 3.0);
  // } else if (mode == MultiLevelMode::MLM_STRONGPASSING) {
  // }
}

double STBoundaryConfiguration::boundary_buffer() const {
  return boundary_buffer_;
}

double STBoundaryConfiguration::minimal_follow_time() const {
  return minimal_follow_time_;
}

double STBoundaryConfiguration::minimal_yield_time() const {
  return minimal_yield_time_;
}

double STBoundaryConfiguration::centric_acceleration_limit() const {
  return centric_acceleration_limit_;
}

double STBoundaryConfiguration::kappa_threshold() const {
  return kappa_threshold_;
}

double STBoundaryConfiguration::default_characteristic_length() const {
  return default_characteristic_length_;
}
double STBoundaryConfiguration::max_speed() const { return max_speed_; }

double STBoundaryConfiguration::maximal_parking_speed() const {
  return maximal_parking_speed_;
}

double STBoundaryConfiguration::lowest_speed() const { return lowest_speed_; }

double STBoundaryConfiguration::nudge_lon_expand_dis() const {
  return nudge_lon_expand_dis_;
}
double STBoundaryConfiguration::nudge_lat_expand_dis() const {
  return nudge_lat_expand_dis_;
}
double STBoundaryConfiguration::static_obs_stop_dis() const {
  return static_obs_stop_dis_;
}
double STBoundaryConfiguration::dynamic_hard_dis() const {
  return dynamic_hard_dis_;
}
double STBoundaryConfiguration::static_obs_park_stop_dis() const {
  return static_obs_park_stop_dis_;
}
double STBoundaryConfiguration::static_obs_uturn_stop_dis() const {
  return static_obs_uturn_stop_dis_;
}
double STBoundaryConfiguration::virtual_obs_stop_dis() const {
  return virtual_obs_stop_dis_;
}
double STBoundaryConfiguration::max_decel_for_converse_obs() const {
  return max_decel_for_converse_obs_;
}
double STBoundaryConfiguration::dynamic_obs_st_boundary_buffer() const {
  return dynamic_obs_st_boundary_buffer_;
}
double STBoundaryConfiguration::dynamic_obs_st_boundary_yeild_buffer() const {
  return dynamic_obs_st_boundary_yeild_buffer_;
}
double STBoundaryConfiguration::dynamic_obs_st_boundary_overtake_buffer()
    const {
  return dynamic_obs_st_boundary_overtake_buffer_;
}
double STBoundaryConfiguration::dynamic_obs_use_polygon_dis_threshold() const {
  return dynamic_obs_use_polygon_dis_threshold_;
}

bool STBoundaryConfiguration::frase_from_config() {
  const auto& common_config =
      config::PlanningConfig::Instance()->plan_config().common;
  /// TODO(zhp): should be restruct
  max_speed_ = 6.3;
  max_speed_gate_ = max_speed_;
  maximal_parking_speed_ = common_config.parking_speed;
  centric_acceleration_limit_ = common_config.centric_accel_limit;
  static_obs_stop_dis_ = common_config.static_hard_distance;
  dynamic_hard_dis_ = common_config.dynamic_hard_distance;
  default_static_obs_stop_dis_ = static_obs_stop_dis_;

  const auto& config = config::PlanningConfig::Instance()
                           ->planning_research_config()
                           .dp_st_graph_config.st_boundary_config;
  boundary_buffer_ = config.boundary_buffer;
  minimal_follow_time_ = config.minimal_follow_time;
  default_minimal_follow_time_ = minimal_follow_time_;
  minimal_yield_time_ = config.minimal_yield_time;
  kappa_threshold_ = config.kappa_threshold;
  lowest_speed_ = config.lowest_speed;
  default_characteristic_length_ = config.default_characteristic_length;

  nudge_lon_expand_dis_ = config.nudge_lon_expand_dis;
  nudge_lat_expand_dis_ = config.nudge_lat_expand_dis;
  static_obs_park_stop_dis_ = config.static_obs_park_stop_dis;
  static_obs_uturn_stop_dis_ = config.static_obs_uturn_stop_dis;
  virtual_obs_stop_dis_ = config.virtual_obs_stop_dis;
  max_decel_for_converse_obs_ = config.max_decel_for_converse_obs;
  dynamic_obs_st_boundary_buffer_ = config.dynamic_obs_st_boundary_buffer;
  dynamic_obs_st_boundary_yeild_buffer_ =
      config.dynamic_obs_st_boundary_yeild_buffer;
  dynamic_obs_st_boundary_overtake_buffer_ =
      config.dynamic_obs_st_boundary_overtake_buffer;
  dynamic_obs_use_polygon_dis_threshold_ =
      config.dynamic_obs_use_polygon_dis_threshold;

  return true;
}
}  // namespace planning
}  // namespace neodrive

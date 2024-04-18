#pragma once

#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class STBoundaryConfiguration {
 public:
  STBoundaryConfiguration();
  void update_inner_parameters(const int mode = 1);
  double boundary_buffer() const;
  double minimal_follow_time() const;
  double minimal_yield_time() const;
  double centric_acceleration_limit() const;
  double kappa_threshold() const;
  double max_speed() const;
  double maximal_parking_speed() const;
  double lowest_speed() const;
  double default_characteristic_length() const;
  // dp_st
  double nudge_lon_expand_dis() const;
  double nudge_lat_expand_dis() const;
  double static_obs_stop_dis() const;
  double dynamic_hard_dis() const;
  double static_obs_park_stop_dis() const;
  double static_obs_uturn_stop_dis() const;
  double virtual_obs_stop_dis() const;
  double dynamic_obs_st_boundary_buffer() const;
  double dynamic_obs_st_boundary_yeild_buffer() const;
  double dynamic_obs_st_boundary_overtake_buffer() const;
  double dynamic_obs_use_polygon_dis_threshold() const;
  double max_decel_for_converse_obs() const;

 private:
  bool frase_from_config();
  std::string current_mode_{};
  double boundary_buffer_ = 0.1;
  double minimal_follow_time_ = 1.0;
  double default_minimal_follow_time_ = 1.0;
  double minimal_yield_time_ = 2.0;
  double centric_acceleration_limit_ = 0.35;
  double kappa_threshold_ = 0.01;
  double max_speed_ = 3.0;
  double max_speed_gate_ = 3.0;
  double maximal_parking_speed_ = 2.0;
  double lowest_speed_ = 0.5;
  double default_characteristic_length_ = 1.0;
  // used in dp_st
  double nudge_lon_expand_dis_ = 0.5;
  double nudge_lat_expand_dis_ = 0.2;
  double static_obs_stop_dis_ = 3.0;
  double dynamic_hard_dis_ = 0.5;
  double default_static_obs_stop_dis_ = 3.0;
  double static_obs_park_stop_dis_ = 0.2;
  double static_obs_uturn_stop_dis_ = 0.5;
  double virtual_obs_stop_dis_ = 0.3;

  double max_decel_for_converse_obs_ = 1.0;  // used in converse obs
  // when yield or follow an obs, default buffer
  double dynamic_obs_st_boundary_buffer_ = 1.0;

  double dynamic_obs_st_boundary_yeild_buffer_ = 3.0;

  double dynamic_obs_st_boundary_overtake_buffer_ = 1.0;
  /*if the distance between trajectory point and obstacle center
              and the distance between obstacle center and adc
              less than this value, use polygon calc st boundary mapper*/
  double dynamic_obs_use_polygon_dis_threshold_ = 10.0;
};

}  // namespace planning
}  // namespace neodrive

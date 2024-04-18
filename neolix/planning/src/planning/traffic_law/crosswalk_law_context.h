#pragma once

#include <unordered_map>

#include "common/math/segment2d.h"
#include "common/math/vec2d.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/common/path/path_point.h"

namespace neodrive {
namespace planning {

struct CrosswalkSignal {
  double start_route_s{0.0};
  double end_route_s{0.0};
  uint64_t crosswalk_id{0};
  bool valid{true};
};

struct CrosswalkDecisionContext {
  bool need_speed_limit{false};
  bool is_all_obs_static{true};
  double static_lateral_min_dis{1000.0};
  double static_longitudinal_min_dis{1000.0};
  double dynamic_lateral_min_dis{1000.0};
  double dynamic_longitudinal_min_dis{1000.0};
};

class CrosswalkLawContext {
 public:
  CrosswalkLawContext() = default;
  ~CrosswalkLawContext() = default;

  void Reset() {
    last_stop_time_ = 0.0;
    invalid_crosswalk_id_ = 0;
  }
  double LastStopTime() const { return last_stop_time_; }
  uint64_t InvalidCrosswalkId() const { return invalid_crosswalk_id_; }

  void SetLastStopTime(const double& last_stop_time) {
    last_stop_time_ = last_stop_time;
  }
  void SetInvalidCrosswalkId(const uint64_t invalid_crosswalk_id) {
    invalid_crosswalk_id_ = invalid_crosswalk_id;
  }

 private:
  double last_stop_time_ = 0.0;
  uint64_t invalid_crosswalk_id_ = 0;
};

}  // namespace planning
}  // namespace neodrive

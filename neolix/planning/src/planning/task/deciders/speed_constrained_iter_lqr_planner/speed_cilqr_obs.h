#pragma once
#include <cmath>

#include "cyber/common/macros.h"
#include "src/planning/common/data_center/outside_planner_data.h"

namespace neodrive {
namespace planning {
class SpeedCilqrObsProcess {
 public:
  ~SpeedCilqrObsProcess() = default;
  SpeedCilqrObsProcess() = default;
  SpeedCilqrObsProcess(double id, double start_t, double end_t,
                       double start_s_l, double start_s_u, double end_s_l,
                       double end_s_u, bool if_reverse, bool if_virtual,
                       bool if_vehicle);
  std::vector<ObsDecisionBound> InterpolatePoint(double x1, double y1,
                                                 double x2, double y2);

  std::vector<ObsDecisionBound> InterpolatePointsForLow();
  std::vector<ObsDecisionBound> InterpolatePointsForUpper();

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, start_t);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, end_t);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(int, obs_id);

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, start_s_l);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, end_s_l);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, obs_speed);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, start_s_u);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, end_s_u);

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, path_clear);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, if_virtual);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, if_reverse);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, if_vehicle);

  void SetPathClear() { path_clear_ = true; }

 private:
  double obs_id_{};
  double start_t_{};
  double end_t_{};
  double start_s_l_{};
  double start_s_u_{};
  double end_s_l_{};
  double end_s_u_{};
  double obs_speed_{};
  double step_{};
  bool path_clear_ = false;
  bool if_reverse_{};
  bool if_virtual_{};
  bool if_vehicle_{};
};

}  // namespace planning
}  // namespace neodrive
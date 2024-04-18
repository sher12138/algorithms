#pragma once
#include <cmath>

#include "src/planning/common/data_center/outside_planner_data.h"

namespace neodrive {
namespace planning {
class SpeedObsExtend {
 public:
  ~SpeedObsExtend() = default;
  SpeedObsExtend() = default;
  SpeedObsExtend(double id, double start_t, double end_t, double start_s_l,
                 double start_s_u, double end_s_l, double end_s_u,
                 bool if_reverse, bool if_virtual, bool if_vehicle,
                 bool if_bump, std::vector<STPoint> lower_points,
                 std::vector<STPoint> upper_points, bool if_dynamic);
  std::vector<ObsDecisionBound> InterpolatePoint(double x1, double y1,
                                                 double x2, double y2,
                                                 bool if_low);
  ObsBoundPolySeris IterExtendStart();
  std::vector<ObsDecisionBound> InterpolatePointsForLow();
  std::vector<ObsDecisionBound> InterpolatePointsForUpper();
  std::pair<double, int> FindClosestPoint(const std::vector<STPoint>& list,
                                          double target);
  double StartTime() const { return start_t_; };
  double EndTime() const { return end_t_; }
  double ExtendTime() const { return back_extend_t_; };
  double Id() const { return obs_id_; }
  void ResetExtendOrder();
  void Reset();
  void SaveLOGResults();
  double StartLowS() const { return start_s_l_; };
  double EndLowS() const { return end_s_l_; };
  double ObsSpeed() const { return obs_speed_; };
  double StartUpS() const { return start_s_u_; };
  double EndUpS() const { return end_s_u_; };
  void SetPathClear() { path_clear_ = true; }
  bool ShowPathClear() const { return path_clear_; }
  bool IfVirtual() const { return if_virtual_; }
  bool IfReverse() const { return if_reverse_; }
  bool IfVehicle() const { return if_vehicle_; }
  bool IfBump() const { return if_bump_; }

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::vector<STPoint>, lower_points);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::vector<STPoint>, upper_points);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, if_dynamic);

 private:
  double obs_id_{};
  double start_t_{};
  double end_t_{};
  double start_s_l_{};
  double start_s_u_{};
  double end_s_l_{};
  double end_s_u_{};
  double obs_speed_{};
  double extend_speed_{};
  double step_{};
  double back_extend_t_{};
  double back_extend_s_l_{};
  double back_extend_s_u_{};
  bool extend_ = false;
  bool path_clear_ = false;
  bool if_reverse_{};
  bool if_virtual_{};
  bool if_vehicle_{};
  bool if_bump_{};
  std::vector<STPoint> lower_points_{};
  std::vector<STPoint> upper_points_{};
  bool if_dynamic_{false};
};

}  // namespace planning
}  // namespace neodrive
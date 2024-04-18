#pragma once

#include "speed_limit.h"
#include "src/planning/common/speed/speed_point.h"
#include "src/planning/common/trajectory/trajectory_point.h"
#include "st_graph_boundary.h"
#include "st_graph_polyline.h"

namespace neodrive {
namespace planning {

class STGraphData {
 public:
  STGraphData() = default;
  ~STGraphData();

  double path_data_length() const;
  double current_speed() const;
  const TrajectoryPoint& init_point() const;
  const SpeedLimit& speed_limit() const;
  SpeedLimit* mutable_speed_limit();
  const std::vector<SpeedPoint>& guided_speed_data() const;
  const std::vector<STGraphBoundary>& obs_boundary() const;
  std::vector<STGraphBoundary>* mutable_obs_boundary();
  const std::vector<STGraphBoundary>& nudge_boundary() const;
  std::vector<STGraphBoundary>* mutable_nudge_boundary();
  const std::vector<STGraphPolyline>& caution_polyline() const;
  std::vector<STGraphPolyline>* mutable_caution_polyline();
  const std::vector<SpeedPoint>& last_speed_data() const;

  void set_path_data_length(const double path_data_length);
  void set_current_speed(const double current_speed);
  void set_init_point(const TrajectoryPoint& init_point);
  void set_speed_limit(const SpeedLimit& speed_limit);
  void set_guided_speed_data(const std::vector<SpeedPoint>& speed_vec);
  void set_obs_boundary(const std::vector<STGraphBoundary>& obs_boundary);
  void set_nudge_boundary(const std::vector<STGraphBoundary>& nudge_boundary);
  void set_caution_polyline(
      const std::vector<STGraphPolyline>& caution_polyline);
  void set_last_speed_data(const std::vector<SpeedPoint>& last_speed_data);

 private:
  double path_data_length_ = 0.0;
  double current_speed_ = 0.0;
  TrajectoryPoint init_point_;
  SpeedLimit speed_limit_;
  std::vector<SpeedPoint> guided_speed_data_;
  std::vector<STGraphBoundary> obs_boundary_;
  std::vector<STGraphBoundary> nudge_boundary_;
  std::vector<STGraphPolyline> caution_polyline_;

  std::vector<SpeedPoint> last_speed_data_;
};

}  // namespace planning
}  // namespace neodrive

#pragma once

#include "discretized_trajectory.h"
#include "planning.pb.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;
  ~PublishableTrajectory() = default;

  bool evaluate_absolute_time(const double abs_time, TrajectoryPoint& pt) const;

  bool evaluate_linear_approximation_absolute_time(const double abs_time,
                                                   TrajectoryPoint& pt) const;

  bool query_nearest_point_absolute_time(const double abs_time,
                                         std::size_t& index) const;

  double header_time() const;

  void set_header_time(const double header_time);

  TrajectoryPoint GetFirstPoint() const;

  void to_trajectory_protobuf(
      ReferenceLinePtr ref_line, const std::vector<PathPoint>& path_points,
      const std::vector<TrajectoryPoint>& stitch_trajectory,
      std::shared_ptr<neodrive::global::planning::ADCTrajectory>&
          adc_trajectory_ptr) const;

 private:
  double header_time_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive

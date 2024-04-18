#pragma once

#include "discretized_trajectory.h"
#include "prediction_obstacles.pb.h"

namespace neodrive {
namespace planning {

using neodrive::global::prediction::Trajectory_PredictorType;
class PredictionTrajectory : public DiscretizedTrajectory {
 public:
  PredictionTrajectory() = default;
  ~PredictionTrajectory() = default;

  double probability() const;

  double start_timestamp() const;

  size_t lane_id() const;

  Trajectory_PredictorType predictor_type() const;

  void set_probability(const double prob);

  void set_start_timestamp(const double ts);

  void adjust_timestamp_with(const double diff);

  TrajectoryPoint* trajectory_point_ptr(const std::size_t index);

  void set_lane_id(const size_t lane_id);

  void set_predictor_type(const Trajectory_PredictorType& type);

 private:
  double probability_{0.};
  double start_timestamp_{0.};
  size_t lane_id_{0};
  Trajectory_PredictorType predictor_type_{
      global::prediction::Trajectory_PredictorType_Unknown};
};

}  // namespace planning
}  // namespace neodrive

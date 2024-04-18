#include "prediction_trajectory.h"

namespace neodrive {
namespace planning {

double PredictionTrajectory::probability() const { return probability_; }

double PredictionTrajectory::start_timestamp() const {
  return start_timestamp_;
}

size_t PredictionTrajectory::lane_id() const { return lane_id_; }

Trajectory_PredictorType PredictionTrajectory::predictor_type() const {
  return predictor_type_;
}

void PredictionTrajectory::set_probability(const double prob) {
  probability_ = prob;
}

void PredictionTrajectory::set_start_timestamp(const double ts) {
  start_timestamp_ = ts;
}

void PredictionTrajectory::adjust_timestamp_with(const double diff) {
  start_timestamp_ += diff;
  for (auto& pt : trajectory_points_) {
    pt.set_relative_time(pt.relative_time() - diff);
  }
}

TrajectoryPoint* PredictionTrajectory::trajectory_point_ptr(
    const std::size_t index) {
  if (index >= trajectory_points_.size()) return nullptr;

  return &trajectory_points_[index];
}

void PredictionTrajectory::set_lane_id(const size_t lane_id) {
  lane_id_ = lane_id;
}

void PredictionTrajectory::set_predictor_type(
    const Trajectory_PredictorType& type) {
  predictor_type_ = type;
}

}  // namespace planning
}  // namespace neodrive

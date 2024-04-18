#include "st_graph_data.h"

namespace neodrive {
namespace planning {

STGraphData::~STGraphData() {
  guided_speed_data_.clear();
  obs_boundary_.clear();
  nudge_boundary_.clear();
  speed_limit_.mutable_speed_limits()->clear();
  last_speed_data_.clear();
}

double STGraphData::path_data_length() const { return path_data_length_; }
double STGraphData::current_speed() const { return current_speed_; }
const TrajectoryPoint& STGraphData::init_point() const { return init_point_; }
const SpeedLimit& STGraphData::speed_limit() const { return speed_limit_; }
SpeedLimit* STGraphData::mutable_speed_limit() { return &speed_limit_; }

const std::vector<SpeedPoint>& STGraphData::guided_speed_data() const {
  return guided_speed_data_;
}
const std::vector<STGraphBoundary>& STGraphData::obs_boundary() const {
  return obs_boundary_;
}
std::vector<STGraphBoundary>* STGraphData::mutable_obs_boundary() {
  return &obs_boundary_;
}
const std::vector<STGraphBoundary>& STGraphData::nudge_boundary() const {
  return nudge_boundary_;
}
std::vector<STGraphBoundary>* STGraphData::mutable_nudge_boundary() {
  return &nudge_boundary_;
}
const std::vector<STGraphPolyline>& STGraphData::caution_polyline() const {
  return caution_polyline_;
}
std::vector<STGraphPolyline>* STGraphData::mutable_caution_polyline() {
  return &caution_polyline_;
}
const std::vector<SpeedPoint>& STGraphData::last_speed_data() const {
  return last_speed_data_;
}

void STGraphData::set_path_data_length(const double path_data_length) {
  path_data_length_ = path_data_length;
}
void STGraphData::set_current_speed(const double current_speed) {
  current_speed_ = current_speed;
}
void STGraphData::set_init_point(const TrajectoryPoint& init_point) {
  init_point_ = init_point;
}
void STGraphData::set_speed_limit(const SpeedLimit& speed_limit) {
  speed_limit_ = speed_limit;
}

void STGraphData::set_guided_speed_data(
    const std::vector<SpeedPoint>& speed_vec) {
  guided_speed_data_ = speed_vec;
}
void STGraphData::set_obs_boundary(
    const std::vector<STGraphBoundary>& obs_boundary) {
  obs_boundary_ = obs_boundary;
}
void STGraphData::set_nudge_boundary(
    const std::vector<STGraphBoundary>& nudge_boundary) {
  nudge_boundary_ = nudge_boundary;
}
void STGraphData::set_caution_polyline(
    const std::vector<STGraphPolyline>& caution_polyline) {
  caution_polyline_ = caution_polyline;
}
void STGraphData::set_last_speed_data(
    const std::vector<SpeedPoint>& last_speed_data) {
  last_speed_data_ = last_speed_data;
}

}  // namespace planning
}  // namespace neodrive

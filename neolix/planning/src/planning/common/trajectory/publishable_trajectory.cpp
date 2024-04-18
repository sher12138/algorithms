#include "publishable_trajectory.h"

namespace neodrive {
namespace planning {
using neodrive::global::planning::ADCTrajectory;

bool PublishableTrajectory::evaluate_absolute_time(const double abs_time,
                                                   TrajectoryPoint& pt) const {
  return evaluate(abs_time - header_time_, pt);
}

bool PublishableTrajectory::evaluate_linear_approximation_absolute_time(
    const double abs_time, TrajectoryPoint& pt) const {
  return evaluate_linear_approximation(abs_time - header_time_, pt);
}

bool PublishableTrajectory::query_nearest_point_absolute_time(
    const double abs_time, std::size_t& index) const {
  return query_relative_time_lower_bound_index(abs_time - header_time_, index);
}

double PublishableTrajectory::header_time() const { return header_time_; }

void PublishableTrajectory::set_header_time(const double header_time) {
  header_time_ = header_time;
}

TrajectoryPoint PublishableTrajectory::GetFirstPoint() const {
  for (const auto& tp : trajectory_points_) {
    if (tp.s() > 0) {
      return tp;
    }
  }
  return TrajectoryPoint();
}

void PublishableTrajectory::to_trajectory_protobuf(
    ReferenceLinePtr ref_line, const std::vector<PathPoint>& path_points,
    const std::vector<TrajectoryPoint>& stitch_trajectory,
    std::shared_ptr<ADCTrajectory>& adc_trajectory_ptr) const {
  adc_trajectory_ptr->clear_adc_trajectory_point();
  adc_trajectory_ptr->clear_adc_path_point();
  if (trajectory_points_.empty()) return;

  ReferencePoint curr_pt{};
  ref_line->GetNearestRefPoint(
      Vec2d{trajectory_points_[0].x(), trajectory_points_[0].y()}, &curr_pt);

  auto lt = [](auto& a, auto& b) { return a.s() < b; };
  const auto& ref_pts = ref_line->ref_points();
  adc_trajectory_ptr->mutable_adc_trajectory_point()->Reserve(
      trajectory_points_.size());

  auto get_average_pitch = [&ref_pts](const auto it) {
    double sum_pitch = 0.0;
    int cnt = 0;
    for (auto iter = it;
         iter != ref_pts.end() &&
         iter->s() < it->s() + VehicleParam::Instance()->front_edge_to_center();
         ++iter) {
      sum_pitch += iter->pitch();
      cnt++;
    }
    for (auto iter = it;
         iter != ref_pts.begin() &&
         iter->s() > it->s() - VehicleParam::Instance()->back_edge_to_center();
         --iter) {
      sum_pitch += iter->pitch();
      cnt++;
    }
    return sum_pitch / static_cast<double>(cnt);
  };

  for (const auto& tp : trajectory_points_) {
    auto ptr_tp_pb = adc_trajectory_ptr->add_adc_trajectory_point();
    ptr_tp_pb->set_relative_time(tp.relative_time());
    ptr_tp_pb->set_x(tp.x());
    ptr_tp_pb->set_y(tp.y());
    ptr_tp_pb->set_theta(tp.theta());
    ptr_tp_pb->set_curvature(tp.kappa());
    ptr_tp_pb->set_curvature_change_rate(tp.dkappa());
    ptr_tp_pb->set_speed(tp.velocity());
    ptr_tp_pb->set_accumulated_s(tp.s());
    ptr_tp_pb->set_acceleration_s(tp.acceleration());
    if (auto it = std::lower_bound(ref_pts.begin(), ref_pts.end(),
                                   curr_pt.s() + tp.s(), lt);
        it != ref_pts.end()) {
      ptr_tp_pb->set_z(it->z());
      ptr_tp_pb->set_pitch(get_average_pitch(it));
    } else if (!ref_pts.empty()) {
      ptr_tp_pb->set_z(ref_pts.rbegin()->z());
      ptr_tp_pb->set_pitch(ref_pts.rbegin()->pitch());
    }
  }

  // path points
  std::vector<TrajectoryPoint> stitch_path_points{};
  for (std::size_t i = 0; i + 1 < stitch_trajectory.size(); ++i) {
    if (i == 0) {
      stitch_path_points.push_back(stitch_trajectory[i]);
      continue;
    }
    auto& p = stitch_trajectory[i];
    auto& last_p = stitch_path_points.back();
    // trick for dreamview's frontend bug
    if (std::hypot(p.x() - last_p.x(), p.y() - last_p.y()) > 0.05) {
      stitch_path_points.push_back(stitch_trajectory[i]);
    }
  }
  for (const auto& p : stitch_path_points) {
    auto ptr_path_point = adc_trajectory_ptr->add_adc_path_point();
    ptr_path_point->set_x(p.x());
    ptr_path_point->set_y(p.y());
    ptr_path_point->set_z(0.0);
    ptr_path_point->set_heading(p.theta());
    ptr_path_point->set_curvature(p.kappa());
  }
  for (const auto& p : path_points) {
    auto ptr_path_point = adc_trajectory_ptr->add_adc_path_point();
    ptr_path_point->set_x(p.x());
    ptr_path_point->set_y(p.y());
    ptr_path_point->set_z(0.0);
    ptr_path_point->set_heading(p.theta());
    ptr_path_point->set_curvature(p.kappa());
  }
}

}  // namespace planning
}  // namespace neodrive

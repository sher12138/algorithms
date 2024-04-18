#include "oblique_park.h"

namespace neodrive {
namespace planning {
ObliquePark::ObliquePark(cyberverse::ParkingSpaceInfoConstPtr park)
    : ParkingSpace(park) {
  if (!park) return;
  auto &points = park->Points();
  origin_pos_ = Vec3d(points[0].x, points[0].y, points[0].z);
}

double ObliquePark::Length() {
  if (!park_ptr_) return 0.0;
  auto &points = park_ptr_->Points();
  return std::hypot(points[0].y - points[1].y, points[0].x, points[1].x);
}

double ObliquePark::Width() {
  if (!park_ptr_) return 0.0;
  auto &points = park_ptr_->Points();
  return std::hypot(points[1].y - points[2].y, points[1].x, points[2].x);
}

double ObliquePark::GetDistToEnd() {
  if (!park_ptr_) return 0.0;
  return length_;
}

double ObliquePark::Heading() {
  if (!park_ptr_) return 0.0;
  return park_ptr_->Heading();
}

void ObliquePark::Solve() { ; }

void ObliquePark::Reset() { ; }

bool ObliquePark::CheckNeedStitchPath() { return false; }

ParkingPath &ObliquePark::GetStitchPath() {
  ParkingPath parking_path{};
  return parking_path;
}

}  // namespace planning
}  // namespace neodrive
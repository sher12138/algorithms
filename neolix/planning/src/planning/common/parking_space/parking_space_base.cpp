#include "src/planning/common/parking_space/parking_space_base.h"
#include "hdmap/hdmap.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/parking_space/park_util.h"
namespace neodrive {
namespace planning {
bool ParkingSpace::CheckOnParkingLane() {
  if (!overlap_lane_ptr_) return false;
  const auto& vehicle_state_utm = DataCenter::Instance()->vehicle_state_utm();
  common::math::Vec2d vehicle_position{vehicle_state_utm.X(),
                                       vehicle_state_utm.Y()};
  auto overlap_predecessor = GetOnlyPredecessor(overlap_lane_ptr_);
  auto overlap_successor = GetOnlySuccessor(overlap_lane_ptr_);
  return overlap_lane_ptr_->IsOnLane(vehicle_position) ||
         (overlap_predecessor &&
          overlap_predecessor->IsOnLane(vehicle_position)) ||
         (overlap_successor && overlap_successor->IsOnLane(vehicle_position));
}

bool ParkingSpace::IsInParkingSpace() const {
  if (!park_ptr_) return false;
  const auto& utm_pose = DataCenter::Instance()->vehicle_state_utm();
  return park_ptr_->Polygon().is_point_in({utm_pose.X(), utm_pose.Y()});
}

std::string ParkingSpace::GetParkId() {
  if (!park_ptr_) return {};
  return cyberverse::HDMap::Instance()->GetIdHashString(park_ptr_->Id());
}
}  // namespace planning
}  // namespace neodrive
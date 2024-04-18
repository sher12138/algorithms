#pragma once
#include "src/planning/common/parking_space/parking_space_base.h"
namespace neodrive {
namespace planning {
class ObliquePark : public ParkingSpace {
 public:
  ObliquePark(cyberverse::ParkingSpaceInfoConstPtr park);
  double Length() override;
  double Width() override;
  double Heading() override;
  void Solve() override;
  void Reset() override;
  bool CheckNeedStitchPath() override;
  double GetDistToEnd() override;
  ParkingPath &GetStitchPath();
};
}  // namespace planning
}  // namespace neodrive
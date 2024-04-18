#pragma once
#include "parking_space_base.h"
namespace neodrive {
namespace planning {
class OnlanePark : public ParkingSpace {
 public:
  OnlanePark(cyberverse::ParkingSpaceInfoConstPtr park);
  double Length() override;
  double Width() override;
  double Heading() override;
  void Solve() override;
  void Reset() override;
  bool CheckNeedStitchPath() override;
  ParkingPath &GetStitchPath();
  double GetDistToEnd() override;

 private:
  void GenerateParkInPath();
  void GenerateParkOutPath();

};
}  // namespace planning
}  // namespace neodrive
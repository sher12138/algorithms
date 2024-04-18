#pragma once
#include "src/planning/common/parking_space/parking_space_base.h"
namespace neodrive {
namespace planning {
class HorizontalPark : public ParkingSpace {
 public:
  HorizontalPark(cyberverse::ParkingSpaceInfoConstPtr park);
  double Length() override;
  double Width() override;
  double Heading() override;
  void Solve() override;
  void Reset() override;
  bool CheckNeedStitchPath() override;
  double GetDistToEnd() override;
  ParkingPath &GetStitchPath() override;

 private:
  void GenerateParkInPath();
  void GenerateParkOutPath();
  void CalculateKeyPoints();

 private:
  ParkingPath stitch_path;
  Vec3d point_a_rel_, point_b_rel_, point_c_rel_, point_O_rel_, point_d_rel_;
};
}  // namespace planning
}  // namespace neodrive
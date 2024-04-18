#pragma once
#include "src/planning/common/parking_space/parking_space_base.h"
namespace neodrive {
namespace planning {
class VerticalPark : public ParkingSpace {
 public:
  enum ParkinMethod { TYPE_C = 0, TYPE_R = 1, TYPE_L = 2 };
  VerticalPark(cyberverse::ParkingSpaceInfoConstPtr park);
  double Length() override;
  double Width() override;
  double Heading() override;
  void Solve() override;
  void Reset() override;
  double GetDistToEnd() override;
  bool CheckNeedStitchPath() override;
  ParkingPath &GetStitchPath();

 private:
  void SwitchParkinMethod();
  void GenerateParkInPathTypeR();
  void GenerateParkInPathTypeC();
  void GenerateParkInPathTypeL();
  void GenerateParkOutPath();
  void GenerateBCPathTypeC(const Vec3d &point_b_rel, const Vec3d &point_c_rel,
                           ParkingPath &path, double step = 0.02);

 private:
  ParkinMethod park_in_method_{TYPE_R};
  ParkingPath stitch_path{};
  Vec3d point_o_rel_, point_a_rel_, point_b_rel_, point_c_rel_, point_O_rel_,
      point_d_rel_;
  bool on_left_{false};
};
}  // namespace planning
}  // namespace neodrive
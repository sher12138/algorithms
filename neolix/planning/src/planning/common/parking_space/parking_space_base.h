#pragma once
#include "common/coordinate/coodrdinate_convertion.h"
#include "common/coordinate/coordinate_math.h"
#include "common/data_center/master_info.h"
#include "common/math/quintic_curve_fit.h"
#include "common/math/vec3d.h"
#include "hdmap/hdmap.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {
using ParkingPathType = std::vector<Vec3d>;
struct ParkingPath {
  enum EndCheckType {
    DistEndCheck,
    ParallelHeadingDiffCheck,
    VerticalHeadingDiffCheck
  };
  ParkingPath() {}
  ParkingPathType path_points{};
  std::vector<double> point_s{};
  MasterInfo::DriveDirection drive_direction;
  double stop_befor_end{2.0};
  bool is_park_in{true};
  bool is_need_stop{false};
  EndCheckType check_type{DistEndCheck};
  void Reset() {
    path_points.clear();
    point_s.clear();
    drive_direction = MasterInfo::DriveDirection();
    stop_befor_end = 2.0;
    is_park_in = true;
    is_need_stop = false;
    check_type = DistEndCheck;
  }
};

class ParkingSpace {
 public:
  ParkingSpace(cyberverse::ParkingSpaceInfoConstPtr park)
      : park_ptr_(park),
        special_parking_config_(config::PlanningConfig::Instance()
                                    ->scene_special_config()
                                    .parking) {}
  const Vec3d& OriginPos() { return origin_pos_; }
  virtual double Length() = 0;
  virtual double Width() = 0;
  virtual double Heading() = 0;
  virtual void Solve() = 0;
  virtual void Reset() = 0;
  virtual bool CheckNeedStitchPath() = 0;
  virtual ParkingPath& GetStitchPath() = 0;
  virtual double GetDistToEnd() = 0;
  cyberverse::ParkingSpaceInfoConstPtr OriginPark() { return park_ptr_; }
  cyberverse::LaneInfoConstPtr OverlapLane() { return overlap_lane_ptr_; }
  std::vector<ParkingPath>& ParkPath() { return park_path_; }
  std::string GetParkId();
  bool CheckOnParkingLane();
  bool IsInParkingSpace() const;
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_park_in);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_finished);

 public:
  static constexpr double kParkingPathExternDist = 2.0;
  static constexpr double kPointDuplicateDist = 0.1;
  static constexpr double kStopDist = 1.;

 protected:
  Vec3d origin_pos_;
  int path_index_{-2};
  double heading_{0.};
  double length_{0.};
  double width_{0.};
  bool is_park_in_{true};
  bool is_finished_{false};
  std::vector<ParkingPath> park_path_{};
  cyberverse::ParkingSpaceInfoConstPtr park_ptr_;
  cyberverse::LaneInfoConstPtr overlap_lane_ptr_;
  config::AutoSceneSpecialConfig::Parking special_parking_config_;
};

typedef std::shared_ptr<ParkingSpace> ParkingSpaceShrPtr;
}  // namespace planning
}  // namespace neodrive
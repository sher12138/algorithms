#pragma once

#include <cmath>

#include "common_config/config/auto_ego_car_config.h"
#include "src/planning/proxy/proxy_type.h"

namespace neodrive {
namespace planning {

class VehicleStateProxy {
 public:
  VehicleStateProxy() = default;
  ~VehicleStateProxy() = default;

  void SetVehicleStatus(const ChassisShrPtr &chassis);
  void SetVehicleStatus(const TwistStampedShrPtr &kinematics);

  void SetVehicleStatus(const PoseStampedShrPtr &center_pose);

  void SetVehicleStatus(const PoseStampedShrPtr &pose,
                        const TwistStampedShrPtr &twist);

 public:
  bool IsValid() const;

  double X() const;
  double Y() const;
  double Z() const;
  double CenterX() const;
  double CenterY() const;
  double CenterZ() const;
  double LinearVelocity() const;
  double AbsoluteLinearVelocity() const;
  double Heading() const;
  double Pitch() const;
  double TrajPitch() const;
  double Roll() const;
  double LinearAcceleration() const;
  std::array<double, 2> ImuAcceleration() const;
  double Curvature() const;
  double Timestamp() const;
  double Velocity3dX() const;
  double Velocity3dY() const;
  double Velocity3dZ() const;
  double SteerPercent() const;
  const Chassis &chassis() const { return *chassis_; }
  const PoseStamped &pose() const { return *pose_; }
  const TwistStamped &twist() const { return *twist_; }
  const neodrive::global::planning::ADCSignals &ADC() const;
  const neodrive::global::planning::ADCSignals::SignalType &Signal() const;
  neodrive::global::status::GearPosition Gear() const;
  neodrive::global::status::DrivingMode DrivingMode() const;
  bool IsStopped() const;
  void SetTrajPitch(const double map_pitch);

 private:
  void UpdateInnerStatus();

  ChassisShrPtr chassis_{std::make_shared<Chassis>()};
  bool chassis_valid_{false};
  PoseStampedShrPtr pose_{std::make_shared<PoseStamped>()};
  bool pose_valid_{false};
  TwistStampedShrPtr twist_{std::make_shared<TwistStamped>()};
  bool kinematics_valid_{false};
  TwistStampedShrPtr kinematics_{std::make_shared<TwistStamped>()};
  bool twist_valid_{false};
  PoseStampedShrPtr center_pose_{std::make_shared<PoseStamped>()};
  bool center_pose_valid_{false};
  TwistStampedShrPtr center_twist_{std::make_shared<TwistStamped>()};
  bool center_twist_valid_{false};

  double linear_velocity_{0.};
  double heading_{0.};
  double pitch_{0.};
  double roll_{0.};
  double linear_acceleration_{0.};
  std::array<double, 2> imu_acc_{0.};
  double curvature_{0.};
  double traj_pitch_{0.};
};

}  // namespace planning
}  // namespace neodrive

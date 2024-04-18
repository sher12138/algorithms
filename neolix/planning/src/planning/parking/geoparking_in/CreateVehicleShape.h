#pragma once

#include "PolygonRelation.h"
#include "src/planning/common/trajectory/trajectory_point.h"

namespace neodrive {
namespace planning {

struct CandiatePoint {
  TrajectoryPoint point;
  Polygon_RELATION rect;
};

class CreateVehicleShape {
 public:
  CreateVehicleShape() = default;

  ~CreateVehicleShape() = default;

  // bflag=T: use point.yaw;bflag=false:calculate yaw
  void CreateVehicleRect(std::vector<CandiatePoint> &path, const bool bflag,
                         const double enlarge = 0.1);

  // enlarge vehicle size by 0.2
  void CreateVehicleRect(CandiatePoint &path, const double enlarge = 0.2);

  // do not enlarge vehicle size
  void CreateVehicleRectLocal(CandiatePoint &path, const double enlarge = 0.0);

  bool LogPathArrayFile(std::vector<CandiatePoint> &path);
};

}  // namespace planning
}  // namespace neodrive

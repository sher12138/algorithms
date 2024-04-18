#include "CreateVehicleShape.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

void CreateVehicleShape::CreateVehicleRect(std::vector<CandiatePoint> &path,
                                           const bool bflag,
                                           const double enlarge) {
  int path_size = path.size();
  if (bflag == false) {  // need to calculate yaw first
    for (int j = 0; j < path_size; ++j) {
      CandiatePoint &curr_point = path[j];
      int cc1 = 0, cc2 = 0;
      double avgx1 = 0, avgy1 = 0, avgx2 = 0, avgy2 = 0;
      for (int k = std::max(0, j - 4); k <= j; ++k) {
        const CandiatePoint &tmp_point = path[k];
        ++cc1;
        avgx1 += tmp_point.point.x();
        avgy1 += tmp_point.point.y();
      }
      for (int k = j; k <= std::min(path_size - 1, j + 4); ++k) {
        const CandiatePoint &tmp_point = path[k];
        ++cc2;
        avgx2 += tmp_point.point.x();
        avgy2 += tmp_point.point.y();
      }

      avgx1 /= cc1;
      avgy1 /= cc1;
      avgx2 /= cc2;
      avgy2 /= cc2;
      // calculate yaw based on average of front four points and average of rear
      // four points
      double yaw = std::atan2(avgy2 - avgy1, avgx2 - avgx1);
      curr_point.point.set_theta(yaw);
    }
  }

  for (int j = 0; j < path_size; ++j) {
    Polygon_RELATION &curr_point = path[j].rect;
    Box2d box = VehicleParam::Instance()->get_adc_bounding_box(
        {path[j].point.x(), path[j].point.y()}, path[j].point.theta(), enlarge,
        enlarge, enlarge);
    box.get_all_corners(&curr_point.vertex_);
  }
  return;
}

void CreateVehicleShape::CreateVehicleRect(CandiatePoint &path,
                                           const double enlarge) {
  // calculate four corners
  Polygon_RELATION &curr_point = path.rect;
  Box2d box = VehicleParam::Instance()->get_adc_bounding_box(
      {path.point.x(), path.point.y()}, path.point.theta(), enlarge, enlarge,
      enlarge);
  box.get_all_corners(&curr_point.vertex_);
  return;
}

void CreateVehicleShape::CreateVehicleRectLocal(CandiatePoint &path,
                                                const double enlarge) {
  // calculate four corners
  Polygon_RELATION &curr_point = path.rect;
  Box2d box = VehicleParam::Instance()->get_adc_bounding_box(
      {path.point.x(), path.point.y()}, path.point.theta(), enlarge, enlarge,
      enlarge);
  box.get_all_corners(&curr_point.vertex_);
  return;
}

bool CreateVehicleShape::LogPathArrayFile(std::vector<CandiatePoint> &path) {
  std::ofstream fp("./log/SpiralTrajectoryLogFile.csv",
                   std::ios::out);  // only read
  if (!fp.is_open()) {
    return false;
  }
  fp.setf(std::ios::fixed, std::ios::floatfield);
  fp.precision(2);

  /*fp<<"x,y,yaw,\
  curvature,distance,direction,\
  segment index,vel,accel, \
  x1,y1,x2,y2,x3,y3,x4,y4"<<'\n';*/
  std::size_t path_size = path.size();
  for (std::size_t i = 0; i < path_size; ++i) {
    CreateVehicleRectLocal(path[i]);
    fp << path[i].point.x() << "," << path[i].point.y() << ","
       << path[i].point.theta() << "," << path[i].point.kappa() << ","
       << path[i].point.s() << "," << path[i].point.direction() << ","
       << path[i].point.segment_index() << "," << path[i].point.velocity()
       << "," << path[i].point.acceleration() << ",";
    std::size_t tmp_num = path[i].rect.vertex_.size();
    for (std::size_t j = 0; j < tmp_num; ++j)
      fp << path[i].rect.vertex_[j].x() << "," << path[i].rect.vertex_[j].y()
         << ",";
    fp << '\n';
  }
  fp << '\n';
  fp.close();
  fp.clear();
  return true;
}

}  // namespace planning
}  // namespace neodrive

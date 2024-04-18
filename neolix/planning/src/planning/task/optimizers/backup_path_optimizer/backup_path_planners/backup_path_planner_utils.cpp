#include "task/optimizers/backup_path_optimizer/backup_path_planners/backup_path_planner_utils.h"

namespace neodrive {
namespace planning {

namespace backup_utils {

void FillWayPointHeading(std::vector<BackupPathPlanner::WayPoint>* pts) {
  using std::size_t;
  int m = pts->size();
  std::vector<double> px(m + 1, 0), py(m + 1, 0);
  for (int i = 1; i <= m; ++i) {
    px[i] = px[i - 1] + (*pts)[i - 1].x;
    py[i] = py[i - 1] + (*pts)[i - 1].y;
  }

  for (int i = 1; i < m - 1; ++i) {
    int low = std::max<int>(0, i - 4);
    int high = std::min<int>(m - 1, i + 4);
    const double avgx1 = (px[i + 1] - px[low]) / (i - low + 1.);
    const double avgy1 = (py[i + 1] - py[low]) / (i - low + 1.);

    const double avgx2 = (px[high + 1] - px[i]) / (high - i + 1.);
    const double avgy2 = (py[high + 1] - py[i]) / (high - i + 1.);
    (*pts)[i].theta = std::atan2(avgy2 - avgy1, avgx2 - avgx1);
  }
  (*pts)[0].theta = (*pts)[1].theta;
  if (m > 2) (*pts)[m - 1].theta = (*pts)[m - 2].theta;
}

void FillWayPointLateral(std::vector<BackupPathPlanner::WayPoint>* pts) {
  if (pts->size() < 3) return;
  auto& ps = *pts;
  size_t m = pts->size();

  for (size_t i = 1; i < m - 1; ++i) ps[i].dl = ps[i].l - ps[i - 1].l;
  ps[0].dl = ps[1].dl;
  ps[m - 1].dl = ps[m - 2].dl;
  for (size_t i = 1; i < m - 1; ++i) ps[i].ddl = ps[i].dl - ps[i - 1].dl;
  ps[0].ddl = ps[1].ddl;
  ps[m - 1].ddl = ps[m - 2].ddl;
}

TrajectoryPoint Trans2Utm(const TrajectoryPoint& odom_veh) {
  auto& utm_pos = DataCenter::Instance()->environment().vehicle_state_proxy();
  auto& odom_pos =
      DataCenter::Instance()->environment().vehicle_state_odometry_proxy();

  double t_x{0.}, t_y{0.}, t_h{0.}, t_x1{0.}, t_y1{0.}, t_h1{0.};
  earth2vehicle(odom_pos.X(), odom_pos.Y(), odom_pos.Heading(), odom_veh.x(),
                odom_veh.y(), odom_veh.theta(), t_x, t_y, t_h);
  vehicle2earth(utm_pos.X(), utm_pos.Y(), utm_pos.Heading(), t_x, t_y,
                normalize_angle(t_h), t_x1, t_y1, t_h1);

  TrajectoryPoint utm_veh = odom_veh;
  utm_veh.set_x(t_x1), utm_veh.set_y(t_y1), utm_veh.set_theta(t_h1);

  return utm_veh;
}

}  // namespace backup_utils

}  // namespace planning
}  // namespace neodrive

#include "task/optimizers/backup_path_optimizer/backup_path_planners/reference_line_backup_path_planner.h"

#include "common/planning_gflags.h"
#include "common/visualizer_event/visualizer_event.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planners/backup_path_planner_utils.h"

namespace neodrive {
namespace planning {

void VisLinePoints(ReferenceLinePtr ref_line,
                   std::vector<BackupPathPlanner::WayPoint>* line,
                   const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d tp{};
  for (auto [x, y, z, s, l, dl, ddl, theta, kappa] : *line) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->set_radius(0.1);
    ref_line->GetPointInCartesianFrame({s, l}, &tp);
    set_pt(sphere->mutable_center(), tp);
  }
}

ReferenceLineBackupPathPlanner::ReferenceLineBackupPathPlanner()
    : BackupPathPlanner{"ReferenceLineBackupPathPlanner"} {}

bool ReferenceLineBackupPathPlanner::GeneratePath(
    TaskInfo& task_info, std::vector<BackupPathPlanner::WayPoint>* ans,
    double& gain) {
  LOG_INFO("{} works!", name());

  ReferenceLinePtr ref_line = task_info.reference_line();
  const TrajectoryPoint& veh =
      task_info.current_frame()->inside_planner_data().init_point;
  const InsidePlannerData& inside_data =
      task_info.current_frame()->inside_planner_data();
  const OutsidePlannerData& outside_data =
      task_info.current_frame()->outside_planner_data();
  const OutsidePlannerData& last_outside_data =
      task_info.last_frame()->outside_planner_data();
  const DecisionData& decision_data =
      task_info.current_frame()->planning_data().decision_data();

  // get start and end state(s0, ds0, l0, dl0, st, dst, lt, dlt)
  // 0 for start time, t for end time
  double s0{0.}, l0{0.};
  if (SLPoint p; ref_line->GetPointInFrenetFrame({veh.x(), veh.y()}, &p)) {
    std::tie(s0, l0) = std::pair{p.s(), p.l()};
  } else {
    return false;
  }
  const double T = FLAGS_planning_trajectory_time_length;
  const double st = s0 + 10;
  const double ds = veh.velocity() * 0.8;

  // trans to waypoint
  Vec2d tmp{};
  for (double t = 0; t < T + 1e-3; t += 0.1) {
    const double s = s0 + ds * t;
    ref_line->GetPointInCartesianFrame({s, l0}, &tmp);
    ans->push_back(BackupPathPlanner::WayPoint{
        .x = tmp.x(), .y = tmp.y(), .s = s, .l = l0});
  }
  backup_utils::FillWayPointHeading(ans);

  VisLinePoints(ref_line, ans, "ref_line rough path");
  LOG_INFO("{} finished!", name());
  return true;
}

}  // namespace planning
}  // namespace neodrive

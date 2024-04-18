#include "sim_planner_track_optimizer.h"

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/sim_planner/sim_map.h"

namespace neodrive {
namespace planning {
namespace visgraph {
void VisSimMapGraph(const std::vector<sim_planner::SimMapRoad>& sim_map) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event_odd = vis::EventSender::Instance()->GetEvent("odd_road_lane");
  event_odd->set_type(visualizer::Event::k3D);
  event_odd->add_attribute(visualizer::Event::kOdom);
  auto event_even = vis::EventSender::Instance()->GetEvent("even_road_lane");
  event_even->set_type(visualizer::Event::k3D);
  event_even->add_attribute(visualizer::Event::kOdom);
  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.05);
  };
  for (const auto& sim_road : sim_map) {
    for (const auto& sim_lane : sim_road.lanes) {
      int pts_size = sim_lane.pts.size();
      if (pts_size == 0) continue;
      Vec2d first_pt(sim_lane.pts.front().x, sim_lane.pts.front().y),
          mid_pt(sim_lane.pts[pts_size / 2].x, sim_lane.pts[pts_size / 2].y),
          last_pt(sim_lane.pts.back().x, sim_lane.pts.back().y);
      if (sim_road.road_id % 2 == 0) {
        set_pts(event_even, first_pt);
        set_pts(event_even, mid_pt);
        set_pts(event_even, last_pt);
      } else {
        set_pts(event_odd, first_pt);
        set_pts(event_odd, mid_pt);
        set_pts(event_odd, last_pt);
      }
    }
  }
}

void VisGraph(const std::vector<sim_planner::Vehicle>& sim_traj) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("simplanner_track");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& pt : sim_traj) {
    Vec2d xy_point = pt.state().vec_position;
    // LOG_INFO("xy points:{:.4f}, {:.4f}", xy_point.x(), xy_point.y());
    set_pts(event, xy_point);
  }
}

void VisTrackPointsGraph(
    const std::vector<sim_planner::SimMapPoint>& track_pts) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("track_points");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& pt : track_pts) {
    Vec2d xy_point(pt.x, pt.y);
    // LOG_INFO("xy points:{:.4f}, {:.4f}", xy_point.x(), xy_point.y());
    set_pts(event, xy_point);
  }
}

void VisAllTrajGraph(
    const std::vector<std::vector<sim_planner::Vehicle>>& sim_trajs,
    const std::vector<bool>& sim_res) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("all_traj_track");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.1);
  };
  for (int i = 0; i < sim_trajs.size(); ++i) {
    if (!sim_res[i]) continue;
    std::vector<sim_planner::Vehicle> sim_traj = sim_trajs[i];
    for (const auto& pt : sim_traj) {
      Vec2d xy_point = pt.state().vec_position;
      set_pts(event, xy_point);
    }
  }
}
}  // namespace visgraph
SimPlannerTrackOptimizer::SimPlannerTrackOptimizer() {
  name_ = "SimPlannerTrackOptimizer";
}
SimPlannerTrackOptimizer::~SimPlannerTrackOptimizer() { Reset(); }

ErrorCode SimPlannerTrackOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  sim_planner::SimLateralTrackManager::Snapshot snapshot;
  const auto& init_pt = task_info.frame()->inside_planner_data().init_point;
  if (!sim_planner::SimMap::Instance()->createSimMap(task_info)) {
    LOG_ERROR("CreateSimMap failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // visgraph::VisSimMapGraph(sim_planner::SimMap::Instance()->road_map());
  sim_planner::Task sim_task;
  sim_task.is_under_ctrl = true;
  sim_task.user_desired_vel = 10.0;
  sim_planner::State state{
      // .time_stamp = DataCenter::Instance()->init_frame_time(),
      .time_stamp = 0,
      .vec_position = Vec2d(init_pt.x(), init_pt.y()),
      .angle = init_pt.theta(),
      .curvature = init_pt.kappa(),
      .velocity = init_pt.velocity(),
      .acceleration = init_pt.acceleration(),
      .steer = DataCenter::Instance()->vehicle_state_odometry().SteerPercent()};

  std::vector<sim_planner::Vehicle> sim_traj{};
  std::vector<std::vector<sim_planner::Vehicle>> sim_trajs{};
  sim_traj.clear();
  sim_trajs.clear();
  LOG_INFO("Simplanner start run......");
  sim_planner::SimLateralTrackManager sim_lateral_track_manager;

  if (!sim_lateral_track_manager.run(0, sim_task, state, track_pts,
                                     all_lanes_id, &snapshot)) {
    LOG_ERROR("sim lateral track manager failed!");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("Simplanner track run finish.....");
  sim_trajs = snapshot.forward_trajs;
  std::vector<bool> sim_res = snapshot.sim_res;
  sim_traj = sim_trajs[snapshot.processed_winner_id];
  visgraph::VisGraph(sim_traj);
  visgraph::VisAllTrajGraph(sim_trajs, sim_res);
  visgraph::VisTrackPointsGraph(track_pts);
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
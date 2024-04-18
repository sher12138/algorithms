#include "refe_generator.h"

#include "navigation/common/navigation_context.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/math/util.h"
namespace neodrive {
namespace planning {
using neodrive::planning::NavigationContext;

TrajectoryPoint RefeLineGenerator::GetInitPoint(
    const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq) {
  TrajectoryPoint init_pt;
  if (lane_seq.empty() || lane_seq[0]->Points().empty()) return init_pt;
  auto &first_pt = lane_seq[0]->Points()[0];
  init_pt.set_x(first_pt.x());
  init_pt.set_y(first_pt.y());
  return init_pt;
}

TrajectoryPoint RefeLineGenerator::GetInitPoint(
    const VehicleStateProxy &vehicle_state_proxy) {
  TrajectoryPoint init_pt;
  init_pt.set_x(vehicle_state_proxy.X());
  init_pt.set_y(vehicle_state_proxy.Y());
  return init_pt;
}

void RefeLineGenerator::SwapObject(ReferenceLinePtr &left,
                                   ReferenceLinePtr &right) {
  ReferenceLinePtr tmp_ptr;
  tmp_ptr = left;
  left = right;
  right = tmp_ptr;
}

ReferenceLinePtr RefeLineGenerator::TransformToOdometry(
    const ReferenceLinePtr &utm_ref) {
  auto &utm_pos = DataCenter::Instance()->environment().vehicle_state_proxy();
  auto &odom_pos =
      DataCenter::Instance()->environment().vehicle_state_odometry_proxy();

  // reference line
  double t_x{0.}, t_y{0.}, t_h{0.}, t_x1{0.}, t_y1{0.}, t_h1{0.};
  auto pts = utm_ref->ref_points();
  for (auto &p : pts) {
    earth2vehicle(utm_pos.X(), utm_pos.Y(), utm_pos.Heading(), p.x(), p.y(),
                  p.heading(), t_x, t_y, t_h);
    vehicle2earth(odom_pos.X(), odom_pos.Y(), odom_pos.Heading(), t_x, t_y,
                  normalize_angle(t_h), t_x1, t_y1, t_h1);
    p.set_x(t_x1);
    p.set_y(t_y1);
    p.set_heading(normalize_angle(t_h1));
  }
  LOG_ERROR("enter createfrom");
  ReferenceLinePtr ret = std::make_shared<ReferenceLine>();
  ret->CreateFrom(
      pts,
      std::array{utm_ref->crosswalk_overlaps(), utm_ref->signal_overlaps(),
                 utm_ref->yield_sign_overlaps(), utm_ref->stop_sign_overlaps(),
                 utm_ref->junction_overlaps(), utm_ref->speed_bump_overlaps(),
                 utm_ref->clearzone_overlaps(), utm_ref->geo_fence_overlaps(),
                 utm_ref->barrier_gate_overlaps(),
                 utm_ref->parking_space_overlaps()},
      Vec2d{odom_pos.X(), odom_pos.Y()}, utm_ref->anchor_s(),
      utm_ref->routing_sequence_num());
  LOG_ERROR("create finish");
  return ret;
}

bool RefeLineGenerator::Generate(
    const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq,
    const TrajectoryPoint &init_pt) {
  if (ctx_->request_msg.is_updated) {
    LOG_INFO(
        "route request msg is updated, reset generator and history ref-line");
    generator_ = Generator(ctx_);
  }
  auto start_time = cyber::Time::Now().ToSecond();

  if (!generator_.Generate(lane_seq, init_pt, &utm_ref_)) {
    LOG_ERROR("Generate tar reference line failed!");
    generator_ = Generator(ctx_);
    return false;
  }
  auto end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("RefeLineGenerator::Generate use time: {:.4f}",
           end_time - start_time);

  return true;
}
}  // namespace planning
}  // namespace neodrive
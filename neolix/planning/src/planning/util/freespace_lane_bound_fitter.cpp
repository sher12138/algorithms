#include "freespace_lane_bound_fitter.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/math/curve_fit/polygon_curve_fit.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double kSqrRadiusCamera = 8.0 * 8.0;
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
constexpr double kSqrTol = 0.2 * 0.2;
using AD2 = std::array<double, 2>;

std::vector<AD2> GetLidarPoints(
    const neodrive::global::perception::Freespace& freespace) {
  std::vector<AD2> ans{};
  auto is_valid = [](auto& p) {
    return p.y() > 0 && p.y() < kLongRange &&
           p.x() * p.x() + p.y() * p.y() < kSqrRadiusLidar;
  };
  for (auto& p : freespace.freespace()) {
    if (is_valid(p)) ans.push_back({p.x(), p.y()});
  }

  return ans;
}

std::vector<AD2> GetCameraPoints(
    const neodrive::global::perception::SingleCameraSegmentation& camera_segments) {
  std::vector<AD2> ans{};
  auto is_valid = [](auto& p) {
    return p.y() > kLongRange &&
           p.x() * p.x() + p.y() * p.y() < kSqrRadiusCamera;
  };
  if (camera_segments.segments().empty()) {
    LOG_ERROR("camera_segments is empty!");
    return ans;
  }
  for (auto& f : camera_segments.segments()) {
    for (auto& p : f.bev_points()) {
      if (is_valid(p)) ans.push_back({p.x(), p.y()});
    }
  }

  return ans;
}

std::vector<std::array<AD2, 2>> GetDynamicObstacleVecs(
    const std::vector<Obstacle*>& obstacles, const AD2& imu_pos) {
  std::vector<std::vector<AD2>> obs_polygon{};
  for (auto obs : obstacles) {
    std::vector<AD2> polygon{};
    LOG_INFO("dynamic {}", obs->id());
    for (auto& p : obs->polygon_corners()) {
      if (double xx = p.x() - imu_pos[0], yy = p.y() - imu_pos[1];
          xx * xx + yy * yy < kSqrRadiusCamera)
        polygon.push_back({xx, yy});
    }
    if (polygon.size() > 1) {
      LOG_INFO("XXX consider obs {}", obs->id());
    }
    if (polygon.size() > 1) {
      obs_polygon.push_back(std::move(polygon));
    }
  }

  for (auto& poly : obs_polygon) {
    std::sort(poly.begin(), poly.end(),
              [](auto& a, auto& b) { return a[0] * b[1] - a[1] * b[0] > 0; });
  }

  std::vector<std::array<AD2, 2>> ans{};
  for (auto& poly : obs_polygon) {
    ans.push_back({poly.front(), poly.back()});
  }

  return ans;
}

std::vector<AD2> RemovePointsOutOfRange(
    const std::vector<std::array<AD2, 2>>& range_vecs,
    const std::vector<AD2>& pts) {
  auto cross = [](auto& v0, auto& v1) { return v0[0] * v1[1] - v1[0] * v0[1]; };
  std::vector<AD2> ans{};
  for (auto& p : pts) {
    ans.push_back(p);
    for (auto& [left, right] : range_vecs) {
      if (cross(p, left) > 0 && cross(p, right) < 0) {
        ans.pop_back();
        break;
      }
    }
  }

  return ans;
}

std::vector<AD2> TransToOdom(const std::vector<AD2>& free_pts) {
  auto& pose = DataCenter::Instance()
                   ->environment()
                   .perception_proxy()
                   .Perception()
                   .odom_pose();
  auto& loc = pose.position();
  auto& q = pose.orientation();
  const double theta =
      std::atan2(2. * (q.qw() * q.qz() + q.qx() * q.qy()),
                 1. - 2. * (q.qy() * q.qy() + q.qz() * q.qz()));
  const double st = std::sin(theta), ct = std::cos(theta);

  std::vector<AD2> ans{};
  for (auto& [x, y] : free_pts) {
    ans.push_back({x * ct - y * st + loc.x(), x * st + y * ct + loc.y()});
  }

  return ans;
}

std::vector<AD2> TransToFrenet(ReferenceLinePtr ref_line,
                               const std::vector<AD2>& free_pts) {
  std::vector<AD2> ans{};
  for (auto& [x, y] : free_pts) {
    SLPoint slp{};
    ref_line->GetPointInFrenetFrame({x, y}, &slp);
    ans.push_back({slp.s(), slp.l()});
  }

  return ans;
}

std::array<std::vector<AD2>, 2> SplitToUpperAndLower(
    const std::vector<AD2>& pts, const AD2& ego) {
  std::vector<AD2> upper{}, lower{};
  for (auto& p : pts) {
    if (p[1] > ego[1]) {
      upper.push_back(p);
    } else {
      lower.push_back(p);
    }
  }

  return {upper, lower};
}

std::vector<AD2> DownSample(const std::vector<AD2>& pts, const double sqr_tol) {
  std::vector<AD2> ans{};
  auto is_valid = [&sqr_tol](auto& p0, auto& p1) {
    return std::pow(p0[0] - p1[0], 2) + std::pow(p0[1] - p1[1], 2) > sqr_tol;
  };
  for (auto& p : pts) {
    if (ans.empty() || is_valid(p, ans.back())) {
      ans.push_back(p);
    }
  }

  return ans;
}

void VisOdomPoints(const std::vector<AD2>& pts, const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x);
    ans->set_y(y);
    ans->set_z(0);
  };

  for (auto [x, y] : pts) {
    auto sphere = event->mutable_sphere()->Add();
    set_pt(sphere->mutable_center(), x, y);
    sphere->set_radius(0.1);
  }
}

void VisOdomLine(const std::vector<AD2>& pts, const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x);
    ans->set_y(y);
    ans->set_z(0);
  };

  for (auto [x, y] : pts) {
    auto sphere = event->mutable_sphere()->Add();
    set_pt(sphere->mutable_center(), x, y);
    sphere->set_radius(0.1);
  }

  auto polyline = event->mutable_polyline()->Add();
  for (auto [x, y] : pts) {
    set_pt(polyline->add_point(), x, y);
  }
}

}  // namespace

FreespaceLaneBoundFitter::FreespaceLaneBoundFitter(
    ReferenceLinePtr ref_line,
    const neodrive::global::perception::Freespace& lidar_freespace,
    const neodrive::global::perception::SingleCameraSegmentation& camera_segments) {
  LOG_INFO("lidar freespace time:{}", lidar_freespace.header().timestamp_sec());
  auto lidar_pts = GetLidarPoints(lidar_freespace);
  auto camera_pts = GetCameraPoints(camera_segments);

  lidar_pts = TransToOdom(lidar_pts);
  camera_pts = TransToOdom(camera_pts);
  VisOdomPoints(lidar_pts, "lidar_points_considered");
  VisOdomPoints(camera_pts, "camera_points_considered");
  lidar_pts = TransToFrenet(ref_line, lidar_pts);
  camera_pts = TransToFrenet(ref_line, camera_pts);

  auto ego_pos = TransToFrenet(ref_line, TransToOdom({{0, 0}}))[0];
  auto [lidar_upper, lidar_lower] = SplitToUpperAndLower(lidar_pts, ego_pos);
  auto [camera_upper, camera_lower] = SplitToUpperAndLower(camera_pts, ego_pos);

  auto upper_pts = lidar_upper;
  upper_pts.insert(upper_pts.end(), camera_upper.begin(), camera_upper.end());
  sl_upper_pts_ = PolygonCurveFit{upper_pts}.GetLowerCurve();
  std::reverse(sl_upper_pts_.begin(), sl_upper_pts_.end());

  auto lower_pts = lidar_lower;
  lower_pts.insert(lower_pts.end(), camera_lower.begin(), camera_lower.end());
  sl_lower_pts_ = PolygonCurveFit{lower_pts}.GetUpperCurve();

  Vec2d tmp{};
  for (auto& [s, l] : sl_upper_pts_) {
    if (ref_line->GetPointInCartesianFrame({s, l}, &tmp)) {
      xy_upper_pts_.push_back({tmp.x(), tmp.y()});
    }
    LOG_INFO("usl({:.3f}, {:.3f}), xy({:.3f}, {:.3f})", s, l, tmp.x(), tmp.y());
  }
  for (auto& [s, l] : sl_lower_pts_) {
    if (ref_line->GetPointInCartesianFrame({s, l}, &tmp)) {
      xy_lower_pts_.push_back({tmp.x(), tmp.y()});
    }
    LOG_INFO("lsl({:.3f}, {:.3f}), xy({:.3f}, {:.3f})", s, l, tmp.x(), tmp.y());
  }
  VisOdomLine(xy_upper_pts_, "freespace_upper_bound");
  VisOdomLine(xy_lower_pts_, "freespace_lower_bound");
}

}  // namespace planning
}  // namespace neodrive

#include "src/planning/task/optimizers/freespace_path_optimizer/freespace_path_optimizer.h"

#include "src/common/angles/angles.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/task/optimizers/freespace_path_optimizer/hybrid_a_star.h"
#include "src/planning/task/optimizers/freespace_path_optimizer/iterative_anchering_path_smoother.h"
namespace neodrive {
namespace planning {

namespace {

constexpr double kStationVelErr = 0.1;
constexpr double kStationVelCnt = 30;
constexpr double kStationPosErr = 0.5;
constexpr double kExtendLength = 5.;
constexpr double kBoundBuffer = 0.7;

using AD2 = std::array<double, 2>;
using AD3 = std::array<double, 3>;

std::vector<AD3> BuildCurrentPath(const std::vector<AD3>& pts, const AD3& pt) {
  const size_t n = pts.size();
  double errs[n];
  for (size_t i = 0; i < n; ++i) {
    errs[i] = std::hypot(pts[i][0] - pt[0], pts[i][1] - pt[1]);
  }
  auto idx = std::distance(errs, std::min_element(errs, errs + n));
  LOG_INFO("stitch pt x = {:.4f}, y = {:.4f}", pt[0], pt[1]);
  std::vector<double> dir_match{pts[idx][0] - pt[0], pts[idx][1] - pt[1]};
  std::vector<double> dir_next{pts[idx + 1][0] - pt[0],
                               pts[idx + 1][1] - pt[1]};
  if (dir_next[0] * dir_match[0] + dir_next[1] * dir_match[1] < 0) {
    LOG_INFO("match traj point idx++");
    ++idx;
  }

  std::vector<AD3> ans{};
  // if (errs[idx] < 0.1) ++idx;
  for (size_t i = idx; i < n; ++i) {
    ans.push_back(pts[i]);
  }

  return ans;
}

std::vector<AD3> ToPathPoints(const HybridAStar::PiecePath& path) {
  std::vector<AD3> ans{};
  for (size_t i = 0; i < path.xs.size(); ++i) {
    ans.push_back({path.xs[i], path.ys[i], path.thetas[i]});
  }
  return ans;
}

void ExtendPath(const OccupyMap& om, std::vector<AD3>& path, const double len,
                const double step) {
  /// back
  double dx = step * std::cos(path.back()[2]);
  double dy = step * std::sin(path.back()[2]);
  for (double s = step; std::abs(s) < len; s += step) {
    const double nx = path.back()[0] + dx;
    const double ny = path.back()[1] + dy;
    const double theta = path.back()[2];
    if (!om.IsVehicleBoxOccupied({nx, ny, theta})) {
      path.push_back({nx, ny, theta});
    }
    // LOG_INFO("XXX add back ({}, {})", path.back()[0], path.back()[1]);
  }

  /// front
  std::reverse(path.begin(), path.end());
  dx = -step * std::cos(path.back()[2]);
  dy = -step * std::sin(path.back()[2]);
  for (double s = step; std::abs(s) < len; s += step) {
    path.push_back({path.back()[0] + dx, path.back()[1] + dy, path.back()[2]});
    // LOG_INFO("XXX add front ({}, {})", path.back()[0], path.back()[1]);
  }
  std::reverse(path.begin(), path.end());

  return;
}

struct TmpPoint {
  double x, y, s, theta, kappa;
};
std::vector<TmpPoint> BuildPathPoint(const std::vector<AD3>& pts) {
  if (pts.size() <= 1) return {};
  const size_t n = pts.size();
  double ds[n];
  for (size_t i = 1; i < n; ++i) {
    ds[i] = std::hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
  }

  double s = 0;
  std::vector<TmpPoint> ans{{pts[0][0], pts[0][1], 0, pts[0][2], 0}};
  for (size_t i = 1; i < n; ++i) {
    ans.push_back({pts[i][0], pts[i][1], s += ds[i], pts[i][2],
                   normalize_angle(pts[i][2] - pts[i - 1][2]) / ds[i]});
  }
  if (n > 1) ans[0].kappa = ans[1].kappa;

  return ans;
}

void VisPolyline(const std::vector<AD3>& polyline, const std::string& name,
                 const std::array<double, 4>& col) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("start VisPolyline: {}", name);
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  event->mutable_color()->set_r(col[0]);
  event->mutable_color()->set_g(col[1]);
  event->mutable_color()->set_b(col[2]);
  event->mutable_color()->set_a(col[3]);
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p[0]), ans->set_y(p[1]), ans->set_z(0);
  };

  auto pl = event->add_polyline();
  for (auto& p : polyline) {
    set_pt(pl->add_point(), p);
  }
}

void VisNodes(const std::vector<HybridAStar::Node>& nodes) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("start VisPolyline");
  auto event = vis::EventSender::Instance()->GetEvent("HybridAStarNodes");
  event->set_type(visualizer::Event::k3D);
  auto set_pt = [](auto p, auto x, auto y, auto z) {
    p->set_x(x), p->set_y(y), p->set_z(z);
  };
  for (auto& n : nodes) {
    auto sp = event->add_sphere();
    set_pt(sp->mutable_center(), n.xs.back(), n.ys.back(), 0);
    sp->set_radius(0.025);

    auto txt = event->add_text();
    set_pt(txt->mutable_position(), n.xs.back(), n.ys.back(), 0.01);
    txt->set_text("idx: " + std::to_string(n.idx) +
                  "\nforward: " + std::to_string(n.is_forward) +
                  "\nstr: " + std::to_string(n.steer) +
                  "\ng: " + std::to_string(n.go_cost) +
                  "\nh: " + std::to_string(n.heu_cost) +
                  "\np: " + std::to_string(n.parent));
    auto pl = event->add_polyline();
    if (n.idx >= 2) {
      auto& pn = nodes[n.parent];
      set_pt(pl->add_point(), pn.xs.back(), pn.ys.back(), 0);
    }
    for (size_t i = 0; i < n.xs.size(); ++i) {
      set_pt(pl->add_point(), n.xs[i], n.ys[i], 0);
    }
  }
}

void VisPaths(const std::vector<HybridAStar::PiecePath>& paths) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("start VisPolyline");

  std::vector<AD3> pts{};
  for (auto& path : paths) {
    LOG_INFO("point size {}", path.xs.size());
    for (size_t i = 0; i < path.xs.size(); ++i) {
      pts.push_back({path.xs[i], path.ys[i], 0});
    }
  }

  VisPolyline(pts, "HybridAStarPaths", {1, 0, 0, 1});
}

void CreateVirtualBox(TaskInfo& task_info, const AD3& tar, const bool dir,
                      const bool isfisheye = false) {
  const auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  const double width = 4, length = 2, height = 2;
  const double len =
      isfisheye ? 1
                : 1 + (dir ? ego_car_config.front_edge_to_base_link + 0.15
                           : ego_car_config.back_edge_to_base_link + 0.3);
  const double tx = tar[0] + len * std::cos(tar[2]) * (dir ? 1 : -1);
  const double ty = tar[1] + len * std::sin(tar[2]) * (dir ? 1 : -1);
  auto decision = task_info.current_frame()
                      ->mutable_planning_data()
                      ->mutable_decision_data();
  decision->create_virtual_obstacle({tx, ty}, length, height, width, tar[2],
                                    VirtualObstacle::FREESPACE_STATION);
}

double distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x1 - x2, y1 - y2);
}

bool PointToLineSegmentDistance(double tx, double ty, double l1x, double l1y,
                                double l2x, double l2y, double& s, double& l,
                                double& px, double& py) {
  const double x0 = DataCenter::Instance()->vehicle_state_odometry().X();
  const double y0 = DataCenter::Instance()->vehicle_state_odometry().Y();
  const double t0 =
      DataCenter::Instance()->vehicle_state_odometry().Heading() - M_PI / 2.0;
  const double st = std::sin(t0), ct = std::cos(t0);
  px = tx * ct - ty * st + x0;
  py = tx * st + ty * ct + y0;
  double line_length = distance(l1x, l1y, l2x, l2y);
  double u = ((px - l1x) * (l2x - l1x) + (py - l1y) * (l2y - l1y)) /
             std::pow(line_length, 2);
  if (u < 0.0 || u > 1.0) {
    double d1 = distance(px, py, l1x, l1y);
    double d2 = distance(px, py, l2x, l2y);
    l = d1 < d2 ? d1 : d2;

    double x = l1x + u * (l2x - l1x);
    double y = l1y + u * (l2y - l1y);
    s = distance(l1x, l1y, x, y);
    return false;
  } else {
    double x = l1x + u * (l2x - l1x);
    double y = l1y + u * (l2y - l1y);
    l = distance(px, py, x, y);
    s = distance(l1x, l1y, x, y);
    if (l > 5.0) return false;
    return true;
  }
}

struct Point {
  double x, y, theta;
};

struct Node {
  Point p;
  int depth;
  Node *left, *right;
};

void DeleteKDTree(Node*& kdTree) {
  if (kdTree != NULL) {
    DeleteKDTree(kdTree->left);
    DeleteKDTree(kdTree->right);
    delete kdTree;
    kdTree = NULL;
  }
}

double distance(Point p1, Point p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double distance(Point p, Node* node) { return distance(p, node->p); }

Node* BuildKDTree(std::vector<Point>& points, int depth = 0) {
  if (points.empty()) return nullptr;
  int axis = depth % 2;
  if (points.size() == 1) {
    return new Node{points[0], depth, nullptr, nullptr};
  }
  if (axis == 0) {
    std::sort(points.begin(), points.end(),
              [](auto& a, auto& b) { return a.x < b.x; });
  } else {
    std::sort(points.begin(), points.end(),
              [](auto& a, auto& b) { return a.y < b.y; });
  }
  int mid = points.size() / 2;
  Node* node = new Node{points[mid], depth, nullptr, nullptr};
  std::vector<Point> left_points(points.begin(), points.begin() + mid);
  std::vector<Point> right_points(points.begin() + mid + 1, points.end());
  node->left = BuildKDTree(left_points, depth + 1);
  node->right = BuildKDTree(right_points, depth + 1);
  return node;
}

void SearchKDTree(Node* node, Point p, double& x, double& y, double& theta,
                  AD2& collision_freespace, double& min_dis) {
  if (!node) return;
  double dis = distance(p, node);
  if (dis < min_dis) {
    x = p.x;
    y = p.y;
    theta = p.theta;
    min_dis = dis;
    collision_freespace = {node->p.x, node->p.y};
  }
  int axis = node->depth % 2;
  double d_axis = axis == 0 ? p.x - node->p.x : p.y - node->p.y;
  if (d_axis < 0) {
    SearchKDTree(node->left, p, x, y, theta, collision_freespace, min_dis);
    if (min_dis > std::fabs(d_axis)) {
      SearchKDTree(node->right, p, x, y, theta, collision_freespace, min_dis);
    }
  } else {
    SearchKDTree(node->right, p, x, y, theta, collision_freespace, min_dis);
    if (min_dis > std::fabs(d_axis)) {
      SearchKDTree(node->left, p, x, y, theta, collision_freespace, min_dis);
    }
  }
}

bool FindNearestNode(auto paths, std::vector<Point>& polygon,
                     AD3& collision_point) {
  LOG_INFO("start FindNearestNode.");
  Node* kdTree = BuildKDTree(polygon);
  double min_dis = 0.8;
  bool find = false;
  AD2 collision_freespace{};
  for (size_t i = 0; i < paths.xs.size(); ++i) {
    Point p{paths.xs[i], paths.ys[i], paths.thetas[i]};
    double collision_x, collision_y, collision_theta;
    double dis = INT_MAX;
    SearchKDTree(kdTree, p, collision_x, collision_y, collision_theta,
                 collision_freespace, dis);
    if (dis < min_dis) {
      collision_point = {collision_x, collision_y, collision_theta};
      min_dis = dis;
      find = true;
      LOG_INFO("collision_freespace: ({:.4f}, {:.4f})", collision_freespace[0],
               collision_freespace[1]);
      break;
    }
  }
  DeleteKDTree(kdTree);
  return find;
}

void GetPerceptionFreespace(TaskInfo& task_info, HybridAStar::PiecePath& path,
                            const double startx, const double starty) {
  const auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  const auto& fisheye = DataCenter::Instance()->mutable_camera_segments();
  if (fisheye == nullptr) {
    LOG_WARN("fisheye is nullptr");
    return;
  }
  LOG_INFO("fisheye segments size: {}", fisheye->segments().size());

  std::vector<AD2> fisheye_polygon{};
  for (auto& f : fisheye->segments()) {
    if (f.type() == neodrive::global::perception::SegmentType::ROAD) {
      for (auto& h : f.bev_points()) fisheye_polygon.push_back({h.x(), h.y()});
    }
  }
  LOG_INFO("camera freespace original size: {}", fisheye_polygon.size());

  auto imu_to_front =
      ego_car_config.front_edge_to_base_link - ego_car_config.imu_in_car_x;
  auto imu_to_back =
      ego_car_config.imu_in_car_x + ego_car_config.back_edge_to_base_link;

  std::vector<AD3> filter_polygon{};
  for (auto& p : fisheye_polygon) {
    if ((!path.is_forward && p[0] > -imu_to_back - kBoundBuffer) ||
        (path.is_forward && p[1] < imu_to_front + kBoundBuffer))
      continue;
    if (double s, l, x, y;
        PointToLineSegmentDistance(p[0], p[1], startx, starty, path.xs.back(),
                                   path.ys.back(), s, l, x, y))
      filter_polygon.push_back({x, y, s});
  }
  LOG_INFO("filter_polygon size: {}, path size: {}", filter_polygon.size(),
           path.xs.size());
  if (!filter_polygon.empty()) {
    // std::sort(filter_polygon.begin(), filter_polygon.end(),
    //           [](auto& a, auto& b) { return a[2] < b[2]; });
    VisPolyline(filter_polygon, "filter_polygon", {1, 0, 1, 1});

    std::vector<Point> polygon{};
    for (auto& p : filter_polygon) {
      polygon.push_back(Point{p[0], p[1]});
    }
    if (AD3 collision_point{};
        FindNearestNode(path, polygon, collision_point)) {
      LOG_INFO("Camera Segmentation Collision point: {:.4f}, {:.4f}, {:.4f}",
               collision_point[0], collision_point[1], collision_point[2]);
      CreateVirtualBox(task_info, collision_point, path.is_forward, false);
    }
  }
}
std::vector<std::vector<AD2>> GetHoles(HybridAStar::PiecePath& path) {
  std::vector<std::vector<AD2>> ans{};
  const auto& fisheye = DataCenter::Instance()->mutable_camera_segments();
  if (fisheye == nullptr) {
    LOG_WARN("fisheye is nullptr");
    return ans;
  }
  const double x0 = DataCenter::Instance()->vehicle_state_odometry().X();
  const double y0 = DataCenter::Instance()->vehicle_state_odometry().Y();
  const double t0 =
      DataCenter::Instance()->vehicle_state_odometry().Heading() - M_PI / 2.0;
  const double st = std::sin(t0), ct = std::cos(t0);
  const auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  auto imu_to_front =
      ego_car_config.front_edge_to_base_link - ego_car_config.imu_in_car_x;
  auto imu_to_back =
      ego_car_config.imu_in_car_x + ego_car_config.back_edge_to_base_link;
  LOG_INFO("imu_in_car_x = {}", ego_car_config.imu_in_car_x);

  for (auto& f : fisheye->segments()) {
    LOG_INFO("holes size: {}", f.holes().size());
    for (auto& h : f.holes()) {
      std::vector<AD2> hole{};
      for (auto& p : h.hole_points()) {
        if ((!path.is_forward && p.y() > -imu_to_back - kBoundBuffer) ||
            (path.is_forward && p.y() < imu_to_front + kBoundBuffer))
          continue;
        if (p.y() > -2)
          LOG_INFO("find close hole: ({:.4f}, {:.4f})", p.x(), p.y());
        hole.push_back(
            {p.x() * ct - p.y() * st + x0, p.x() * st + p.y() * ct + y0});
      }
      if (hole.size() > 0) ans.push_back(hole);
    }
  }

  return ans;
}

std::pair<bool, AD3> GetCollisionPose(const OccupyMap& om,
                                      const std::vector<AD3>& pts) {
  for (auto& p : pts)
    if (om.IsVehicleBoxOccupied(p)) return {true, p};
  return {false, {0, 0, 0}};
}

}  // namespace

FreespacePathOptimizer::FreespacePathOptimizer() {}

void FreespacePathOptimizer::SaveTaskResults(TaskInfo& task_info) {}

void FreespacePathOptimizer::Reset() {}

ErrorCode FreespacePathOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO("Get in freespace optimizer");
  auto data = task_info.current_frame()->mutable_inside_planner_data();
  if (data->curr_scenario_state != ScenarioState::BACK_OUT)
    return ErrorCode::PLANNING_OK;
  const auto& back_out_config =
      config::PlanningConfig::Instance()->plan_config().back_out;
  const auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  auto freesapce_start_time = cyber::Time::Now().ToSecond();
  auto start_time = cyber::Time::Now().ToSecond();
  auto end_time = cyber::Time::Now().ToSecond();
  /// Generate path
  static std::vector<HybridAStar::PiecePath> paths{};
  static std::vector<std::vector<AD3>> path_pts{};
  static size_t path_idx = 0;
  auto& start = data->init_point;
  static auto hy = HybridAStar{
      {.node_one_shot_gap = 1, .tar_s_bias = back_out_config.tar_s_bias}};

  static OccupyMap om{
      {.rectangle_half_width =
           ego_car_config.width / 2 + back_out_config.lat_safe_buffer,
       .rectangle_front_half_length = ego_car_config.front_edge_to_base_link,
       .rectangle_back_half_length = ego_car_config.back_edge_to_base_link +
                                     back_out_config.lon_safe_buffer},
      data->polygon,
      data->holes};
  if (paths.empty() || data->is_replan) {
    LOG_INFO("XXX replan. paths size: {}, is_replan: {}", paths.size(),
             data->is_replan);
    auto& end = data->target_point;
    LOG_INFO("XXX curr {} {} {}", start.x(), start.y(), start.theta());
    start_time = cyber::Time::Now().ToSecond();
    om = {
        {.rectangle_half_width =
             ego_car_config.width / 2 + back_out_config.lat_safe_buffer,
         .rectangle_front_half_length = ego_car_config.front_edge_to_base_link,
         .rectangle_back_half_length = ego_car_config.back_edge_to_base_link +
                                       back_out_config.lon_safe_buffer},
        data->polygon,
        data->holes};
    end_time = cyber::Time::Now().ToSecond();
    LOG_INFO("om XXX use time: {:.4f}", end_time - start_time);
    LOG_INFO("Built map");
    start_time = cyber::Time::Now().ToSecond();
    paths = hy.GeneratePath(om, {start.x(), start.y(), start.theta()},
                            {end.x(), end.y(), end.theta()}, task_info);
    end_time = cyber::Time::Now().ToSecond();
    LOG_INFO("HybridAStar XXX use time: {:.4f}", end_time - start_time);
    path_idx = 0;
    LOG_INFO("Built path size {}", paths.size());

    start_time = cyber::Time::Now().ToSecond();
    path_pts.clear();
    for (auto& p : paths) {
      LOG_INFO("current path direction: {} ", p.is_forward);
      auto curr = ToPathPoints(p);
      if (!p.is_forward) std::reverse(curr.begin(), curr.end());
      bool isend = &p == &paths.back();
      curr = IterativeAncheringPathSmoother{}.Smooth(om, curr, isend);
      if (curr.empty()) return ErrorCode::PLANNING_ERROR_FAILED;
      // for (auto& [x, y, h] : curr) LOG_INFO("pt ({}, {}, {})", x, y, h);
      ExtendPath(om, curr, kExtendLength, 0.1);
      if (!p.is_forward) std::reverse(curr.begin(), curr.end());
      path_pts.push_back(curr);
    }
    end_time = cyber::Time::Now().ToSecond();
    LOG_INFO("Genreate Path XXX use time: {:.4f}", end_time - start_time);
  }

  om.VisMap();
  VisNodes(hy.GetNodes());
  hy.Vis2dGrid();
  LOG_INFO("path size {}", paths.size());
  if (path_idx >= paths.size()) {
    LOG_INFO("replan failed!");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  VisPaths(paths);
  LOG_INFO("XXX generated path");

  /// Split path
  static int stop_cnt = 0;
  stop_cnt = std::abs(data->vel_v) < kStationVelErr ? stop_cnt + 1 : 0;
  auto dis = [&]() {
    return std::hypot(data->init_point.x() - paths[path_idx].xs.back(),
                      data->init_point.y() - paths[path_idx].ys.back());
  };
  LOG_INFO("XXX stopcnt {}/{}, distance {}/{}", stop_cnt, kStationVelCnt, dis(),
           kStationPosErr);
  if (path_idx + 1 < paths.size() && stop_cnt > kStationVelCnt &&
      dis() < kStationPosErr) {
    LOG_INFO("step on");
    ++path_idx;
    stop_cnt = 0;
  }
  LOG_INFO("path index {} / {}", path_idx, paths.size() - 1);
  start_time = cyber::Time::Now().ToSecond();
  auto pts = BuildCurrentPath(path_pts[path_idx],
                              {start.x(), start.y(), start.theta()});
  end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("BuildCurrentPath XXX use time: {:.4f}", end_time - start_time);
  LOG_INFO("pts size: {}", pts.size());
  VisPolyline(path_pts[path_idx], "smoothed_path", {0, 0, 1, 1});

  start_time = cyber::Time::Now().ToSecond();
  // Create target
  AD3 tar{paths[path_idx].xs.back(), paths[path_idx].ys.back(),
          paths[path_idx].thetas.back()};
  CreateVirtualBox(task_info, tar, paths[path_idx].is_forward);
  // Search Freespace Collision
  if (!paths[path_idx].is_forward) {
    GetPerceptionFreespace(task_info, paths[path_idx], data->init_point.x(),
                           data->init_point.y());
    // Search Holes Collision
    auto holes = GetHoles(paths[path_idx]);
    OccupyMap free_map{
        {.rectangle_half_width =
             ego_car_config.width / 2 + back_out_config.lat_safe_buffer,
         .rectangle_front_half_length = ego_car_config.front_edge_to_base_link,
         .rectangle_back_half_length = ego_car_config.back_edge_to_base_link +
                                       back_out_config.lon_safe_buffer},
        data->polygon,
        holes};
    if (auto [f, pose] = GetCollisionPose(free_map, pts); f) {
      LOG_INFO("Find camera holes, set virtual obs!");
      CreateVirtualBox(task_info, pose, paths[path_idx].is_forward);
    }
  }
  end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("CreateVirtualBox XXX use time: {:.4f}", end_time - start_time);
  if (!paths[path_idx].is_forward) {
    data_center_->mutable_master_info()->set_drive_direction(
        MasterInfo::DriveDirection::DRIVE_BACKWARD);
    data_center_->current_frame()
        ->mutable_inside_planner_data()
        ->is_reverse_driving = true;
  }

  /// Save path
  std::vector<PathPoint> out_path_pts{};
  if (!paths[path_idx].is_forward) {
    LOG_INFO("current path is backward");
  } else {
    LOG_INFO("current path is forward");
  }
  LOG_INFO("XXX final path point (x, y, s, theta, kappa):");
  for (auto [x, y, s, theta, kappa] : BuildPathPoint(pts)) {
    if (!paths[path_idx].is_forward) kappa = -kappa;
    out_path_pts.emplace_back(Vec2d{x, y}, theta, kappa, 0, 0, s);
    // LOG_INFO("({}, {}, {}, {}, {})", x, y, s, theta, kappa);
  }
  auto out_data = task_info.current_frame()->mutable_outside_planner_data();
  out_data->path_data->set_path(out_path_pts);
  out_data->path_succeed_tasks += 1;
  auto freesapce_end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("freespace XXX use time: {:.4f}",
           freesapce_end_time - freesapce_start_time);
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive

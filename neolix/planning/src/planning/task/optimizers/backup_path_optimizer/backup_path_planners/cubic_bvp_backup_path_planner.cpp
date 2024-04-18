#include "task/optimizers/backup_path_optimizer/backup_path_planners/cubic_bvp_backup_path_planner.h"

#include <Eigen/Core>
#include <algorithm>
#include <numeric>

#include "common/planning_gflags.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planners/backup_path_planner_utils.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double lat_sample_dis = 0.2;
using AD2 = std::array<double, 2>;
constexpr double veh_r = 1;

constexpr double kBound = 1.0;
constexpr double kObstacle = 10.;
constexpr double kLaneThr = 0.7;
constexpr double kPathLength = 10.;
constexpr double kDt = 0.1;

std::vector<double> SampleLateral(const double left_width,
                                  const double right_width, const double dl) {
  std::vector<double> ans{0.};
  for (double lat = dl; lat < left_width; lat += dl) ans.push_back(lat);
  for (double lat = dl; lat < right_width; lat += dl) ans.push_back(-lat);
  return ans;
}

std::array<double, 4> CubicPolynomialParam(const double l0, const double dl0,
                                           const double l1, const double dl1,
                                           const double S) {
  Eigen::MatrixXd b(4, 1);
  b << l0, dl0, l1, dl1;

  const double S2 = S * S, S3 = S2 * S;
  Eigen::MatrixXd A(4, 4);
  A << 1, 0, 0, 0, 0, 1, 0, 0, 1, S, S2, S3, 0, 1, 2 * S, 3 * S2;
  auto x = A.inverse() * b;

  return {x(0, 0), x(1, 0), x(2, 0), x(3, 0)};
}

std::vector<AD2> GeneratePathCurve(const std::array<double, 4>& cubic_param,
                                   const double s0, const double ds0,
                                   const double dt, const double T) {
  auto [a0, a1, a2, a3] = cubic_param;
  std::vector<AD2> ans{};
  for (double t = 0; t < T + 1e-2; t += dt) {
    if (const double s = t * ds0; s < kPathLength) {
      const double s2 = s * s, s3 = s2 * s;
      ans.push_back({s + s0, a0 + a1 * s + a2 * s2 + a3 * s3});
    } else {
      ans.push_back({s + s0, ans.back()[1]});
    }
  }
  return ans;
}

struct ThreeCircularDisk {
  AD2 c0{0, 0};
  AD2 c1{0, 0};
  AD2 c2{0, 0};
  double r{0};
  double minx{0};
  double maxx{0};
};

ThreeCircularDisk CalcThreeCircularDisk(const AD2& center, const double heading,
                                        const double width,
                                        const double length) {
  auto vec = Vec2d::create_unit_vec(heading) * length / 3;
  auto p0 = Vec2d{center[0], center[1]};
  auto p1 = p0 + vec;
  auto p2 = p0 - vec;
  const double r = std::sqrt(std::pow(length / 6, 2) + std::pow(width / 2, 2));

  return {{p0.x(), p0.y()}, {p1.x(), p1.y()}, {p2.x(), p2.y()}, r, 0, 0};
}

std::vector<ThreeCircularDisk> CalcSortedThreeCircularDisks(
    ReferenceLinePtr ref_line, const DecisionData& decision_data,
    const double s0, const double st) {
  std::vector<ThreeCircularDisk> all{};
  // get three circular center and radius
  for (auto obs : decision_data.static_obstacle()) {
    auto box = obs->bounding_box();
    all.push_back(CalcThreeCircularDisk({box.center_x(), box.center_y()},
                                        box.heading(), box.width(),
                                        box.length()));
  }

  // trans to frenet
  std::vector<ThreeCircularDisk> ans{};
  SLPoint p{};
  for (auto& [c0, c1, c2, r, minx, maxx] : all) {
    ref_line->GetPointInFrenetFrame({c0[0], c0[1]}, &p);
    std::tie(c0[0], c0[1]) = std::pair{p.s(), p.l()};
    ref_line->GetPointInFrenetFrame({c1[0], c1[1]}, &p);
    std::tie(c1[0], c1[1]) = std::pair{p.s(), p.l()};
    ref_line->GetPointInFrenetFrame({c2[0], c2[1]}, &p);
    std::tie(c2[0], c2[1]) = std::pair{p.s(), p.l()};
    minx = std::min(c1[0], c2[0]);
    maxx = std::max(c1[0], c2[0]);
    if (st < minx || s0 > maxx) continue;
    ans.push_back({c0, c1, c2, r, minx, maxx});
  }

  std::sort(ans.begin(), ans.end(),
            [](auto& a, auto& b) { return a.minx < b.minx; });
  return ans;
}

std::array<std::vector<double>, 2> SqrDisToLeftRightObstacle(
    const std::vector<AD2>& pts, std::vector<ThreeCircularDisk>& obs) {
  auto dist = [](const auto& a, const auto& p) {
    auto sqr =
        std::min({std::pow(a.c0[0] - p[0], 2) + std::pow(a.c0[1] - p[1], 2),
                  std::pow(a.c1[0] - p[0], 2) + std::pow(a.c1[1] - p[1], 2),
                  std::pow(a.c2[0] - p[0], 2) + std::pow(a.c2[1] - p[1], 2)});
    return std::max(sqr -= veh_r * veh_r + a.r * a.r, 1e-5);
  };

  // sliding window, l for start index, r for the index after end index
  int l = 0, r = 0, m = obs.size();
  std::vector<double> left(pts.size(), 1e5), right(pts.size(), 1e5);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    auto [lon, lat] = pts[i];
    auto [mins, maxs] = std::pair{lon - veh_r, lon + veh_r};
    while (r < m && maxs > obs[r].minx) ++r;
    while (l < r && mins > obs[l].maxx) ++l;
    if (l == r) continue;

    std::vector<double> dists{};
    for (int j = l; j < r; ++j) dists.push_back(dist(obs[j], AD2{lon, lat}));
    auto min_idx = std::distance(dists.begin(),
                                 std::min_element(dists.begin(), dists.end()));
    if (auto idx = l + min_idx; obs[idx].c0[1] < lat) {
      right[i] = dists[min_idx];
    } else {
      left[i] = dists[min_idx];
    }
  }

  return {left, right};
}

std::vector<AD2> CalcReferenceLaneBound(const std::vector<AD2>& pts,
                                        ReferenceLinePtr ref_line) {
  std::vector<AD2> ans{};
  ReferencePoint ref_p{};
  for (auto [s, l] : pts) {
    ref_line->GetNearestRefPoint(SLPoint{s, l}, &ref_p);
    ans.push_back({ref_p.left_bound(), -ref_p.right_bound()});
  }

  return ans;
}

std::array<std::vector<double>, 2> SqrDisToLeftRightBound(
    const std::vector<AD2>& pts, const std::vector<AD2>& ref_bounds) {
  std::vector<double> left(pts.size(), 1e5), right(pts.size(), 1e5);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    left[i] = std::max(ref_bounds[i][0] - pts[i][1], 1e-5);
    right[i] = std::max(pts[i][1] - ref_bounds[i][1], 1e-5);
  }
  for (auto& n : left) n *= n;
  for (auto& n : right) n *= n;

  return {left, right};
}

std::vector<double> CalcCosts(
    const std::vector<std::array<std::vector<double>, 4>>& dists) {
  std::vector<double> ans{};
  for (const auto& [ld_obs, rd_obs, ld_bound, rd_bound] : dists) {
    double sum_left{0}, sum_right{0};
    for (std::size_t i = 0; i < ld_obs.size(); ++i) {
      sum_left += ld_obs[i] < ld_bound[i] ? kObstacle / (ld_obs[i] + 1e-5)
                                          : kBound / (ld_bound[i] + 1e-5);
      sum_right += rd_obs[i] < rd_bound[i] ? kObstacle / (rd_obs[i] + 1e-5)
                                           : kBound / (rd_bound[i] + 1e-5);
    }
    const double cost = sum_left + sum_right;
    ans.push_back(cost);
    LOG_DEBUG("sum left dis {}, sum right dis {}, sum {}", sum_left, sum_right,
              cost);
  }
  return ans;
}

}  // namespace

CubicBvpBackupPathPlanner::CubicBvpBackupPathPlanner()
    : BackupPathPlanner{"CubicBvpBackupPathPlanner"} {}

bool CubicBvpBackupPathPlanner::GeneratePath(
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
  LOG_INFO("plan from xy({}, {}), sl({}, {})", veh.x(), veh.y(), s0, l0);
  ReferencePoint ref_p0{};
  if (!ref_line->GetNearestRefPoint(s0, &ref_p0)) return false;
  const double vel = std::clamp(veh.velocity(), 1., 10.);
  const double dl0 = vel * std::sin(veh.theta() - ref_p0.heading());
  const double ds0 = vel * std::cos(veh.theta() - ref_p0.heading());

  const double T = FLAGS_planning_trajectory_time_length;
  ReferencePoint ref_pt{};
  if (!ref_line->GetNearestRefPoint(s0 + kPathLength, &ref_pt)) return false;
  double left_width{ref_pt.left_bound()}, right_width{ref_pt.right_bound()};
  LOG_INFO("origin left_width {}, right_width {}", left_width, right_width);
  if (std::abs(ref_pt.left_lane_bound() - ref_pt.left_road_bound()) < 0.3) {
    left_width = std::min(ref_pt.left_road_bound(), left_width);
  }
  if (std::abs(ref_pt.right_lane_bound() - ref_pt.right_road_bound()) < 0.3) {
    right_width = std::min(ref_pt.right_road_bound(), right_width);
  }
  LOG_INFO("final left_width {}, right_width {}", left_width, right_width);

  auto end_ls = SampleLateral(left_width - kLaneThr, right_width - kLaneThr,
                              lat_sample_dis);
  std::sort(end_ls.begin(), end_ls.end(), std::greater<double>());

  // sample lt to generate all curve
  std::vector<std::vector<AD2>> sample_lines{};
  for (auto end_l : end_ls) {
    sample_lines.push_back(GeneratePathCurve(
        CubicPolynomialParam(l0, dl0, end_l, 0, kPathLength), s0, vel, kDt, T));
  }

  // costs calculation
  auto obs_disks = CalcSortedThreeCircularDisks(ref_line, decision_data, s0,
                                                s0 + kPathLength);
  auto ref_bounds = CalcReferenceLaneBound(sample_lines[0], ref_line);
  std::vector<std::array<std::vector<double>, 4>> dists{};
  for (auto& line : sample_lines) {
    auto [l_dis_obs, r_dis_obs] = SqrDisToLeftRightObstacle(line, obs_disks);
    auto [l_dis_bound, r_dis_bound] = SqrDisToLeftRightBound(line, ref_bounds);
    dists.push_back({std::move(l_dis_obs), std::move(r_dis_obs),
                     std::move(l_dis_bound), std::move(r_dis_bound)});
  }
  auto costs = CalcCosts(dists);
  auto idx = std::distance(costs.begin(),
                           std::min_element(costs.begin(), costs.end()));
  LOG_INFO("sample size {}, select {}", costs.size(), idx);

  // trans to waypoint
  for (auto [s, l] : sample_lines[idx]) {
    Vec2d tmp{};
    ref_line->GetPointInCartesianFrame({s, l}, &tmp);
    ans->push_back(BackupPathPlanner::WayPoint{
        .x = tmp.x(), .y = tmp.y(), .s = s, .l = l});
  }
  backup_utils::FillWayPointHeading(ans);
  return true;
}

}  // namespace planning
}  // namespace neodrive

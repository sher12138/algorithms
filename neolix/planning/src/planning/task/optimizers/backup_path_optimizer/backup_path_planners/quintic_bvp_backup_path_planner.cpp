#include "task/optimizers/backup_path_optimizer/backup_path_planners/quintic_bvp_backup_path_planner.h"

#include <Eigen/Core>
#include <algorithm>
#include <numeric>

#include "common/planning_gflags.h"
#include "common/visualizer_event/visualizer_event.h"
#include "math/curve1d/quintic_polynomial_curve1d.h"
#include "math/frame_conversion/sl_analytic_transformation.h"
#include "planning/planning_map/planning_map.h"
#include "src/planning/common/data_center/data_center.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planners/backup_path_planner_utils.h"

#define toStr(name) (#name)
namespace neodrive {
namespace planning {

namespace {
using AD2 = std::array<double, 2>;

constexpr double lat_sample_dis = 0.2;
constexpr double veh_r = 1;

constexpr double kDeltaS = 1.;
constexpr double kClose = 100.;
constexpr double kBound = 1.;
constexpr double kObstacle = 10.;
constexpr double kLaneThr = 0.7;
constexpr double kPathLength = 10.;
constexpr double kMinPathLengthFinal = 30.;
constexpr double kDynamicForget = 1e8;
constexpr double kDt = 0.1;

enum EgoPos {
  INLANE = 0,
  LEFTLANE = 1,
  RIGHTLANE = 2,
  LEFTROAD = 3,
  RIGHTROAD = 4
} ego_pos;
constexpr char *ego_pos_str[] = {"INLANE", "LEFTLANE", "RIGHTLANE", "LEFTROAD",
                                 "RIGHTROAD"};
enum CurbUse {
  USE_LEFTCURB = 0,
  USE_RIGHTCURB = 1,
  USE_BOTHCURB = 2,
  USE_NONE = 3
} curb_pos;
constexpr char *curb_pos_str[] = {"USE_LEFTCURB", "USE_RIGHTCURB",
                                  "USE_BOTHCURB", "USE_NONE"};

std::vector<double> SampleLateral(const double left_width,
                                  const double right_width, const double dl) {
  std::vector<double> ans{0.};
  for (double lat = dl; lat < left_width; lat += dl) ans.push_back(lat);
  for (double lat = dl; lat < right_width; lat += dl) ans.push_back(-lat);
  return ans;
}

std::vector<double> SampleLateral(const double left_width,
                                  const double right_width, const double l0,
                                  const ReferencePoint &ref_pt,
                                  const bool is_left_side, const AD2 side_curbs,
                                  EgoPos &ego_pos, CurbUse &curb_pos,
                                  double &r_lim, double &l_lim) {
  std::vector<double> ans{l0};
  double right_lim1, left_lim1, right_lim2, left_lim2;

  // 1.ref bound judge
  if (is_left_side) {
    bool is_allowed_detour_in_reverse_lane =
        DataCenter::Instance()
            ->mutable_master_info()
            ->mutable_reverse_lane_detour_context()
            ->is_allowed_detour_in_reverse_lane;

    ego_pos =
        (l0 >= ref_pt.left_road_bound()) ? EgoPos::LEFTROAD : EgoPos::LEFTLANE;
    right_lim1 =
        (l0 >= ref_pt.left_road_bound() && !is_allowed_detour_in_reverse_lane)
            ? ref_pt.left_road_bound()
            : 0.0 - (right_width - kLaneThr);
    left_lim1 = l0 + (l0 - right_lim1);
  } else {
    ego_pos = (l0 <= -ref_pt.right_road_bound()) ? EgoPos::RIGHTROAD
                                                 : EgoPos::RIGHTLANE;
    left_lim1 = (l0 <= -ref_pt.right_road_bound())
                    ? -ref_pt.right_road_bound()
                    : 0.0 + (left_width - kLaneThr);
    right_lim1 = l0 - (left_lim1 - l0);
  }
  LOG_INFO("ref bound judge: left_lim: {:.3f}, right_lim: {:.3f}", left_lim1,
           right_lim1);

  // 2.vis curb judge
  left_lim2 = std::min(left_lim1, side_curbs[0]);
  right_lim2 = std::max(right_lim1, side_curbs[1]);
  auto equal = [](auto &a, auto &b) { return (a - b) <= 1e-5; };
  if (equal(right_lim1, right_lim2) && equal(left_lim1, left_lim2))
    curb_pos = CurbUse::USE_NONE;
  else {
    if (!equal(right_lim1, right_lim2) && !equal(left_lim1, left_lim2))
      curb_pos = CurbUse::USE_BOTHCURB;
    else if (equal(left_lim1, left_lim2))
      curb_pos = CurbUse::USE_RIGHTCURB;
    else
      curb_pos = CurbUse::USE_LEFTCURB;
  }
  LOG_INFO("vis curb judge: left_lim: {:.3f}, right_lim: {:.3f}", left_lim2,
           right_lim2);

  //  3.final check & output
  r_lim = right_lim2;
  l_lim = left_lim2;
  if (ego_pos == EgoPos::LEFTROAD || ego_pos == EgoPos::RIGHTROAD) {
    double left_lim3 = std::min(left_lim2, l0 + (left_width - kLaneThr)),
           right_lim3 = std::max(right_lim2, l0 - (right_width - kLaneThr));
    r_lim = right_lim3;
    l_lim = left_lim3;
    LOG_INFO("outside road final check: left_lim: {:.3f}, right_lim: {:.3f}",
             left_lim2, right_lim2);
  }

  // sample
  double ratio{1.5};
  for (double lat = l0 - ratio * lat_sample_dis; lat > r_lim;
       lat -= ratio * lat_sample_dis)
    ans.push_back(lat);
  for (double lat = l0 + ratio * lat_sample_dis; lat < l_lim;
       lat += ratio * lat_sample_dis)
    ans.push_back(lat);
  return ans;
}

std::vector<AD2> GeneratePathCurve(const std::vector<double> &cubic_param,
                                   const double s0, const double ds,
                                   const double len) {
  const double a0 = cubic_param[0], a1 = cubic_param[1], a2 = cubic_param[2],
               a3 = cubic_param[3], a4 = cubic_param[4], a5 = cubic_param[5];
  std::vector<AD2> ans{};
  for (double s = 0.; s < len + 1e-2; s += ds) {
    if (s < kPathLength) {
      const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
      ans.push_back(
          {s + s0, a0 + a1 * s + a2 * s2 + a3 * s3 + a4 * s4 + a5 * s5});
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

ThreeCircularDisk CalcThreeCircularDisk(const AD2 &center, const double heading,
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
    ReferenceLinePtr ref_line, const DecisionData &decision_data,
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
  for (auto &[c0, c1, c2, r, minx, maxx] : all) {
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
            [](auto &a, auto &b) { return a.minx < b.minx; });
  return ans;
}

std::array<std::vector<double>, 2> SqrDisToLeftRightObstacle(
    const std::vector<AD2> &pts, std::vector<ThreeCircularDisk> &obs) {
  auto dist = [](const auto &a, const auto &p) {
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

std::pair<std::vector<double>, double> CalcLastPathLateralAndLiveTime(
    const std::vector<AD2> &pts, const PathData *path_data) {
  if (!path_data) {
    LOG_INFO("path_data is empty!");
    return {std::vector<double>(pts.size(), 0), 1e5};
  }

  static std::vector<FrenetFramePoint> path_points =
      path_data->frenet_path().points();
  if (path_points.empty()) {
    LOG_INFO("path_points is empty!");
    return {std::vector<double>(pts.size(), 0), 1e5};
  }
  static double tic = common::util::TimeLogger::GetCurrentTimeMillisecond();
  static double born = tic;

  auto toc = common::util::TimeLogger::GetCurrentTimeMillisecond();
  auto dt = (toc - tic) / 1000;
  tic = toc;
  if (dt > 0.2) {
    path_points = path_data->frenet_path().points();
    born = toc;
  }
  auto live_time = (toc - born) / 1000;
  LOG_INFO("tic {}, toc {}, live {}", tic, toc, live_time);

  size_t j = 0;
  std::vector<double> ls{};
  for (auto [s, l] : pts) {
    while (j < path_points.size() - 1 && (&path_points[j]) &&
           path_points[j].s() < s)
      ++j;
    if (!&path_points[j]) ls.push_back(path_points[j].l());
  }

  return {ls, live_time};
}

std::vector<AD2> CalcReferenceLaneBound(const std::vector<AD2> &pts,
                                        ReferenceLinePtr ref_line) {
  std::vector<AD2> ans{};
  ReferencePoint ref_p{};
  for (auto [s, l] : pts) {
    ref_line->GetNearestRefPoint(s, &ref_p);
    ans.push_back({ref_p.left_bound(), -ref_p.right_bound()});
  }

  return ans;
}

std::array<std::vector<double>, 2> SqrDisToLeftRightBound(
    const std::vector<AD2> &pts, const std::vector<AD2> &ref_bounds) {
  std::vector<double> left(pts.size(), 1e5), right(pts.size(), 1e5);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    left[i] = std::max(ref_bounds[i][0] - pts[i][1], 1e-5);
    right[i] = std::max(pts[i][1] - ref_bounds[i][1], 1e-5);
  }
  for (auto &n : left) n *= n;
  for (auto &n : right) n *= n;

  return {left, right};
}

std::vector<double> CalcDynamicCosts(
    std::vector<std::vector<double>> &line_errs, const double t) {
  for (auto &l : line_errs) {
    for (auto &e : l) e *= e;
  }

  const double step = std::abs(t / kDt);
  double ratio = kDynamicForget / (step * step + 1);
  if (step > 300) ratio = 0;

  std::vector<double> ans{};
  for (auto &l : line_errs) {
    ans.push_back(std::accumulate(l.begin(), l.end(), 0.) * ratio);
    LOG_DEBUG("err {:.3f}, ratio {:.3f}",
              std::accumulate(l.begin(), l.end(), 0.), ratio);
    LOG_DEBUG("sum dynamic cost {:.3f}", ans.back());
  }

  return ans;
}

void VisLinePoints(ReferenceLinePtr ref_line,
                   const std::vector<std::vector<AD2>> &sample_lines,
                   const std::string &name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto &line : sample_lines) {
    Vec2d tp{};
    for (auto [s, l] : line) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->set_radius(0.1);
      ref_line->GetPointInCartesianFrame({s, l}, &tp);
      set_pt(sphere->mutable_center(), tp);
    }
  }
}

void VisLimPoint(ReferenceLinePtr ref_line, double &lim, double s0, double len,
                 const std::string &name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d tp{};
  double s{s0};
  while (s < s0 + len) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->set_radius(0.08);
    ref_line->GetPointInCartesianFrame({s, lim}, &tp);
    set_pt(sphere->mutable_center(), tp);
    s += 0.5;
  }
}

std::vector<double> operator+(const std::vector<double> &lhs,
                              const std::vector<double> &rhs) {
  if (lhs.size() != rhs.size()) return {};

  std::vector<double> ans(lhs.size(), 0);
  for (size_t i = 0; i < lhs.size(); ++i) ans[i] = lhs[i] + rhs[i];
  return ans;
}

std::vector<double> operator-(const std::vector<AD2> &lhs,
                              const std::vector<double> &rhs) {
  if (lhs.size() != rhs.size()) return {};

  std::vector<double> ans(lhs.size(), 0);
  for (size_t i = 0; i < lhs.size(); ++i) ans[i] = lhs[i][1] - rhs[i];
  return ans;
}

void GetLeftRightWidth(ReferencePoint &ref_pt, double &left_width,
                       double &right_width) {
  left_width = ref_pt.left_bound();
  right_width = ref_pt.right_bound();
  if (std::abs(ref_pt.left_lane_bound() - ref_pt.left_road_bound()) < 0.3) {
    left_width = std::min(ref_pt.left_road_bound(), left_width);
  }
  if (std::abs(ref_pt.right_lane_bound() - ref_pt.right_road_bound()) < 0.3) {
    right_width = std::min(ref_pt.right_road_bound(), right_width);
  }
  LOG_DEBUG("ref_s {:.3f}, left_width {:.3f}, right_width {:.3f}", ref_pt.s(),
            left_width, right_width);
}

void GetLeftRightNarrowestWidth(TaskInfo &task_info, double front_dis,
                                double back_dis, double &left_width,
                                double &right_width) {
  left_width = task_info.curr_referline_pt().left_bound();
  right_width = task_info.curr_referline_pt().right_bound();

  auto &ref_points = task_info.reference_line()->ref_points();
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    if (ref_points[i].s() < task_info.curr_sl().s() - back_dis) {
      continue;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + front_dis) {
      break;
    }
    if (std::abs(ref_points[i].left_lane_bound() -
                 ref_points[i].left_road_bound()) < 0.3) {
      left_width = std::min(ref_points[i].left_road_bound(), left_width);
    }
    if (std::abs(ref_points[i].right_lane_bound() -
                 ref_points[i].right_road_bound()) < 0.3) {
      right_width = std::min(ref_points[i].right_road_bound(), right_width);
    }
  }
}

std::vector<double> CalcStaticCosts(
    ReferenceLinePtr ref_line, double l0, const double observe_ref_l,
    double r_lim, double l_lim, std::vector<std::vector<AD2>> &sample_lines,
    const std::vector<PieceBoundary> &lat_boundaries, EgoPos &ego_pos,
    CurbUse &curb_pos, std::vector<ThreeCircularDisk> &obs_disks) {
  std::vector<double> static_costs{};
  // ego in lane or out lane
  if (ego_pos == EgoPos::INLANE) {
    auto ref_bounds = CalcReferenceLaneBound(sample_lines[0], ref_line);
    for (auto &line : sample_lines) {
      // 1. obs constrain and bound constrain
      auto [l_dis_obs, r_dis_obs] = SqrDisToLeftRightObstacle(line, obs_disks);
      auto [l_dis_bound, r_dis_bound] =
          SqrDisToLeftRightBound(line, ref_bounds);
      double sum_left{0.0}, sum_right{0.0};
      for (std::size_t i = 0; i < l_dis_obs.size(); ++i) {
        sum_left += l_dis_obs[i] < l_dis_bound[i]
                        ? kObstacle / (l_dis_obs[i] + 1e-5)
                        : kBound / (l_dis_bound[i] + 1e-5);
        sum_right += r_dis_obs[i] < r_dis_bound[i]
                         ? kObstacle / (r_dis_obs[i] + 1e-5)
                         : kBound / (r_dis_bound[i] + 1e-5);
      }
      LOG_DEBUG("left cost: {:.3f}, right cost: {:.3f}, sum cost: {:.3f}",
                sum_left, sum_right, sum_left + sum_right);

      // 2. close constrain
      double sum_close{0.0};
      double dis_ref = std::abs(observe_ref_l - line.back()[1]);
      sum_close = kClose * dis_ref;
      LOG_DEBUG("close cost: {:.3f}", sum_close);

      // sample line cost
      static_costs.push_back(sum_left + sum_right + sum_close);
      LOG_DEBUG("sample line cost: {:.3f}", sum_left + sum_right + sum_close);
    }
  } else {
    for (auto &line : sample_lines) {
      // obs constrain
      double sum_left{0}, sum_right{0};
      auto [l_dis_obs, r_dis_obs] = SqrDisToLeftRightObstacle(line, obs_disks);
      for (std::size_t i = 0; i < l_dis_obs.size(); ++i) {
        double dis_l = l_dis_obs[i] + 1e-5, dis_r = r_dis_obs[i] + 1e-5;
        double dis_bound =
            (ego_pos == EgoPos::LEFTROAD || ego_pos == EgoPos::LEFTLANE)
                ? std::abs(l0 - r_lim)
                : std::abs(l0 - l_lim);
        double left_cost = (dis_l < dis_bound) ? kObstacle / dis_l : 0.0;
        double right_cost = (dis_r < dis_bound) ? kObstacle / dis_r : 0.0;
        sum_left += left_cost;
        sum_right += right_cost;
      }
      LOG_DEBUG(
          "left obs cost: {:.3f}, right obs cost: {:.3f}, sum obs cost: {:.3f}",
          sum_left, sum_right, sum_left + sum_right);

      // close constrain
      auto cal_close_cost = [](double close_l, double sample_l,
                               std::string cost_name) {
        double close_cost = kClose * std::abs(close_l - sample_l);
        LOG_DEBUG("close cost type:{}", cost_name);
        return close_cost;
      };
      double sum_close{0};
      if (l_lim > observe_ref_l && r_lim < observe_ref_l) {
        sum_close =
            cal_close_cost(observe_ref_l, line.back()[1], "return ref l");
      } else {
        sum_close = cal_close_cost((r_lim + l_lim) / 2, line.back()[1],
                                   "along middle limit");
        // sum_close = cal_close_cost(l0, line.back()[1], "along current l");
      }
      LOG_DEBUG("close cost: {:.3f}", sum_close);

      // sample line cost
      static_costs.push_back(sum_left + sum_right + sum_close);
      LOG_DEBUG("sample line cost: {:.3f}", sum_left + sum_right + sum_close);
    }
  }
  return static_costs;
}

}  // namespace

QuinticBvpBackupPathPlanner::QuinticBvpBackupPathPlanner()
    : BackupPathPlanner{"QuinticBvpBackupPathPlanner"} {}

bool QuinticBvpBackupPathPlanner::GeneratePath(
    TaskInfo &task_info, std::vector<BackupPathPlanner::WayPoint> *ans,
    double &gain) {
  LOG_INFO("{} works!", name());

  ReferenceLinePtr ref_line = task_info.reference_line();
  const TrajectoryPoint &veh =
      task_info.current_frame()->inside_planner_data().init_point;
  const InsidePlannerData &inside_data =
      task_info.current_frame()->inside_planner_data();
  const OutsidePlannerData &outside_data =
      task_info.current_frame()->outside_planner_data();
  const OutsidePlannerData &last_outside_data =
      task_info.last_frame()->outside_planner_data();
  const DecisionData &decision_data =
      task_info.current_frame()->planning_data().decision_data();
  const TrajectoryPoint &utm_veh = backup_utils::Trans2Utm(veh);

  // ref_l
  const double &observe_ref_l =
      outside_data.path_observe_ref_l_info.observe_ref_l;

  // get start and end state(s0, ds0, l0, dl0, st, dst, lt, dlt)
  // 0 for start time, t for end time
  double s0{0.}, l0{0.};
  if (SLPoint p; ref_line->GetPointInFrenetFrame({veh.x(), veh.y()}, &p)) {
    std::tie(s0, l0) = std::pair{p.s(), p.l()};
  } else {
    return false;
  }
  ReferencePoint ref_p0 = task_info.curr_referline_pt();
  double dl0 = SLAnalyticTransformation::calculate_lateral_derivative(
      ref_p0.heading(), veh.theta(), l0, ref_p0.kappa());
  double ddl0 =
      DataCenter::Instance()->master_info().is_use_position_stitch()
          ? 0.0
          : SLAnalyticTransformation::calculate_second_order_lateral_derivative(
                ref_p0.heading(), veh.theta(), ref_p0.kappa(), veh.kappa(),
                ref_p0.dkappa(), l0);

  // adjust init state when encountering high curvature lanes
  path_planner_common::AdjustInitState(ref_line, inside_data, &outside_data,
                                       dl0, ddl0);

  const double vel = std::clamp(veh.velocity(), 1., 10.);
  const double T = FLAGS_planning_trajectory_time_length;
  const double path_len = std::min(std::max(vel * T, kPathLength),
                                   ref_line->ref_points().back().s() - s0);
  LOG_INFO(
      "plan from xy({:.3f}, {:.3f})  s {:.3f}, l {:.3f}, dl {:.3f}, ddl: "
      "{:.3f} ",
      veh.x(), veh.y(), s0, l0, dl0, ddl0);

  // width
  double left_width{0.0}, right_width{0.0};
  GetLeftRightNarrowestWidth(task_info, 2.0, path_len, left_width, right_width);
  LOG_INFO("narrowest width l/r {:.3f} {:.3f}", left_width, right_width);
  left_width = std::max(left_width, std::abs(observe_ref_l));
  right_width = std::max(right_width, std::abs(observe_ref_l));
  LOG_INFO("update width l/r {:.3f} {:.3f}, by ref_l:{:.3f}", left_width,
           right_width, observe_ref_l);

  // vis curb info
  AD2 side_curbs{std::numeric_limits<double>::max(),
                 -std::numeric_limits<double>::max()};
  SLPoint sl0{s0, l0};
  VisualLaneCombineEgoLane{}.GetVisCurbBound(sl0, ref_line, side_curbs);

  // sample lateral
  double left_bound = ref_p0.left_bound(), right_bound = -ref_p0.right_bound(),
         left_lane_bound = ref_p0.left_lane_bound(),
         right_lane_bound = -ref_p0.right_lane_bound(),
         left_road_bound = ref_p0.left_road_bound(),
         right_road_bound = -ref_p0.right_road_bound();
  double l_lim, r_lim;
  std::vector<double> end_ls{l0};
  if (l0 > right_lane_bound && l0 < left_lane_bound) {
    ego_pos = EgoPos::INLANE;
    end_ls = SampleLateral(left_width - kLaneThr, right_width - kLaneThr,
                           lat_sample_dis);
    LOG_INFO("lat sample points size:{}, ego pos:{}", end_ls.size(),
             ego_pos_str[ego_pos]);
  } else {
    bool is_left_side{l0 >= left_lane_bound};
    end_ls = SampleLateral(left_width, right_width, l0, ref_p0, is_left_side,
                           side_curbs, ego_pos, curb_pos, r_lim, l_lim);
    VisLimPoint(ref_line, l_lim, s0, kPathLength, "left_lim");
    VisLimPoint(ref_line, r_lim, s0, kPathLength, "right_lim");
    LOG_INFO("lat sample points size:{}, ego pos:{}, use curb:{}",
             end_ls.size(), ego_pos_str[ego_pos], curb_pos_str[curb_pos]);
  }
  std::sort(end_ls.begin(), end_ls.end());

  // basic print
  LOG_INFO(
      "left bound: {:.3f}, left lane bound: {:.3f}, left road bound: {:.3f}",
      left_bound, left_lane_bound, left_road_bound);
  LOG_INFO(
      "right bound: {:.3f}, right lane bound: {:.3f}, right road bound: "
      "{:.3f}",
      right_bound, right_lane_bound, right_road_bound);
  for (auto samle_l : end_ls) {
    LOG_INFO("lat sample l:{:.3f}", samle_l);
  }

  // generate sample curve
  std::vector<std::vector<AD2>> sample_lines{};
  for (auto end_l : end_ls) {
    auto coef = QuinticPolynomialCurve1d{l0, 0, 0, end_l, 0., 0., kPathLength}
                    .PolyCoef();
    sample_lines.push_back(GeneratePathCurve(coef, s0, vel * kDt, kPathLength));
  }
  VisLinePoints(ref_line, sample_lines, "quintic_sample_lines");

  // static cost
  std::vector<ThreeCircularDisk> obs_disks = CalcSortedThreeCircularDisks(
      ref_line, decision_data, s0, s0 + kPathLength);
  auto static_costs =
      CalcStaticCosts(ref_line, l0, observe_ref_l, r_lim, l_lim, sample_lines,
                      outside_data.road_obs_path_shrink_boundries, ego_pos,
                      curb_pos, obs_disks);

  // dynamic cost
  auto [ls, live_time] = CalcLastPathLateralAndLiveTime(
      sample_lines[0], &inside_data.last_path_data);
  std::vector<std::vector<double>> dy_errs{};
  for (auto &line : sample_lines) dy_errs.push_back(line - ls);
  auto dynamic_costs = CalcDynamicCosts(dy_errs, live_time);

  // select
  auto costs = static_costs + dynamic_costs;
  auto idx = std::distance(costs.begin(),
                           std::min_element(costs.begin(), costs.end()));
  LOG_INFO("sample size {}, select {}", costs.size(), idx);

  // gain
  auto get_gain = [](double size, double speed) {
    double gain1 = std::max(20 / size, 1.0);
    double gain2 = std::max(10 / speed, 1.0);
    double gain = std::clamp(0.5 * gain1 + 0.5 * gain2, 1.0, 50.0);
    LOG_INFO(
        "gain:{:.4f}, gain1:{:.4f}, gain2:{:.4f}, size:{:.4f}, speed:{:.4f}",
        gain, gain1, gain2, size, speed);
    return gain;
  };
  gain = get_gain(costs.size(), vel);

  // select
  auto &sel_line = sample_lines[idx];
  while (sel_line.back()[0] - sel_line.front()[0] < path_len) {
    sel_line.push_back({sel_line.back()[0] + vel * kDt, sel_line.back()[1]});
  }
  VisLinePoints(ref_line, {sel_line}, "final_select_line");

  // trans to waypoint
  for (auto [s, l] : sample_lines[idx]) {
    Vec2d tmp{};
    ref_line->GetPointInCartesianFrame({s, l}, &tmp);
    ans->push_back(BackupPathPlanner::WayPoint{
        .x = tmp.x(), .y = tmp.y(), .s = s, .l = l});
  }
  backup_utils::FillWayPointLateral(ans);

  LOG_INFO("{} finished!", name());
  return true;
}

}  // namespace planning
}  // namespace neodrive

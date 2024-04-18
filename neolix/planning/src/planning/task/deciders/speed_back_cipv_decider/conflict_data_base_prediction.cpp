#include "conflict_data_base_prediction.h"

#include "src/planning/common/math/util.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {

namespace {

void VisObsPredictionSegment(const std::vector<PredictionCurves>& curves) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("PrediceitonCurves");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };

  for (const auto& curve : curves) {
    for (const auto& pt : curve.left_curves) {
      set_pts(event, pt);
    }
    for (const auto& pt : curve.right_curves) {
      set_pts(event, pt);
    }
  }
}

void VisObsTrajectory(const std::vector<Vec2d>& traj) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("PrediceitonTrajectory");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };

  for (const auto& pt : traj) {
    set_pts(event, pt);
  }
}

}  // namespace

BackDataBasePrediction::BackDataBasePrediction()
    : BackDataInterface("ConflictDataBaseOnPrediction") {}

std::vector<ConnectionConflictInfo>
BackDataBasePrediction::ComputeConflictMergeInData(TaskInfo& task_info) {
  LOG_INFO("Start calc merge in data base prediction!");
  std::vector<ConnectionConflictInfo> ans{};
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if ((!HasMergingArea(task_info))) {
    LOG_INFO("No Merging Area!");
    return ans;
  }
  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return ans;
  }

  PredictionCurves ego_curves;
  std::vector<Vec2d> ego_traj;
  if (!CalcEgoCurves(task_info, ego_curves, ego_traj)) {
    LOG_ERROR("Calc ego curves failed!");
    return ans;
  }
  VisObsTrajectory(ego_traj);

  auto CalcRightOfWay =
      [](const uint64_t obs_id,
         const TrafficConflictZoneContext& tczc) -> RightOfWay {
    bool agent_main = true;
    const auto& merging_ids = tczc.merging_ids;
    for (auto iter = merging_ids.begin(); iter != merging_ids.end(); iter++) {
      if (iter->id == obs_id) {
        if (iter->orientation != ConflictLaneContext::mTurnType::Straight) {
          agent_main = false;
        }
      }
    }
    bool ego_main = false;
    const auto& current_lane = tczc.current_lane;
    if (current_lane.orientation == ConflictLaneContext::mTurnType::Straight) {
      ego_main = true;
    }
    if (ego_main && agent_main) {
      return RightOfWay::EQUAL;
    } else if (!ego_main && !agent_main) {
      return RightOfWay::EQUAL;
    } else if (ego_main && !agent_main) {
      return RightOfWay::ADVANTAGE;
    } else if (!ego_main && agent_main) {
      return RightOfWay::UNADVANTAGE;
    }
    return RightOfWay::UNKNOWN;
  };

  const auto& merging_ids = traffic_conflict_zone_context.merging_ids;
  std::unordered_set<uint64_t> merging_lane_ids;
  for (auto iter = merging_ids.begin(); iter != merging_ids.end(); iter++) {
    merging_lane_ids.insert(iter->id);
  }
  for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
    if (!interactive_agent_ids_.count(obs->id())) {
      continue;
    }
    if (merging_lane_ids.find(obs->matched_lane_id()) ==
        merging_lane_ids.end()) {
      LOG_INFO("obs {} is not in merging lane id!", obs->id());
      continue;
    }
    ConnectionConflictInfo c;
    c.lane_id = obs->matched_lane_id();
    CalAgentInfo(obs, &c.agent);
    c.agent.id = obs->id();
    c.way_right = CalcRightOfWay(c.agent.id, traffic_conflict_zone_context);
    SetConflictParam(&c);
    std::vector<PredictionCurves> agent_curves;
    std::vector<Vec2d> agent_traj;
    if (!CalcAgentCurves(obs, agent_curves, agent_traj)) {
      LOG_ERROR("Calc agent (id is {}) curves failed!", c.agent.id);
      std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
      c.conflict_area_bound = cab;
      ans.emplace_back(c);
      continue;
    }
    VisObsTrajectory(agent_traj);
    std::array<math::AD2, 4> meeting_pts;
    math::AD2 ego_rd{0.0, 0.0}, agent_rd{0.0, 0.0};
    CalcMeeting(ego_curves, agent_curves.front(), meeting_pts);
    CalcConflictZone(ego_traj, agent_traj, meeting_pts, ego_rd, agent_rd);
    c.conflict_area_bound = {agent_rd[0], agent_rd[1], ego_rd[1], ego_rd[0]};

    agent_curves.emplace_back(ego_curves);
    VisObsPredictionSegment(agent_curves);

    ans.emplace_back(c);
  }

  return ans;
}

std::vector<ConnectionConflictInfo>
BackDataBasePrediction::ComputeConflictMeetingData(TaskInfo& task_info) {
  LOG_INFO("Start calc meeting data base prediction!");
  std::vector<ConnectionConflictInfo> ans{};
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return ans;
  }

  PredictionCurves ego_curves;
  std::vector<Vec2d> ego_traj;
  if (!CalcEgoCurves(task_info, ego_curves, ego_traj)) {
    LOG_ERROR("Calc ego curves failed!");
    return ans;
  }
  VisObsTrajectory(ego_traj);

  for (int i = 0; i < 2; ++i) {
    if (traffic_conflict_zone_context.meeting_geoinfo[i]) {
      auto mg = traffic_conflict_zone_context.meeting_geoinfo[i];
      for (auto it = mg->begin(); it != mg->end(); it++) {
        auto& mlc = it->first;
        // dynamic obs
        for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
          if (!interactive_agent_ids_.count(obs->id())) {
            continue;
          }
          // if (obs->matched_lane_id() != mlc.lane_id) {
          //   LOG_INFO("Obs {} is not in meeting lane id!", obs->id());
          //   continue;
          // }
          if (std::abs(obs->matched_lane_heading_deviation()) >=
              M_PI_2 * 0.67) {
            LOG_INFO("ignore obs id is {}, heading diff is {}", obs->id(),
                     std::abs(obs->matched_lane_heading_deviation()));
            continue;
          }
          ConnectionConflictInfo c;
          c.lane_id = obs->matched_lane_id();
          c.way_right = RightOfWay::UNKNOWN;
          CalAgentInfo(obs, &c.agent);
          c.agent.id = obs->id();
          SetConflictParam(&c);
          std::vector<PredictionCurves> agent_curves;
          std::vector<Vec2d> agent_traj;
          if (!CalcAgentCurves(obs, agent_curves, agent_traj)) {
            LOG_ERROR("Calc agent (id is {}) curves failed!", c.agent.id);
            std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
            c.conflict_area_bound = cab;
            ans.emplace_back(c);
            continue;
          }
          VisObsTrajectory(agent_traj);

          std::array<math::AD2, 4> meeting_pts;
          math::AD2 ego_rd{0.0, 0.0}, agent_rd{0.0, 0.0};
          CalcMeeting(ego_curves, agent_curves.front(), meeting_pts);
          CalcConflictZone(ego_traj, agent_traj, meeting_pts, ego_rd, agent_rd);
          LOG_INFO("Conflict area bound is {}, {}, {}, {}", agent_rd[0],
                   agent_rd[1], ego_rd[1], ego_rd[0]);
          c.conflict_area_bound = {agent_rd[0], agent_rd[1], ego_rd[1],
                                   ego_rd[0]};

          agent_curves.emplace_back(ego_curves);
          VisObsPredictionSegment(agent_curves);
          ans.emplace_back(c);
        }
      }
    }
  }
  return ans;
}

void BackDataBasePrediction::CalcMeeting(const PredictionCurves& ego,
                                         const PredictionCurves& agent,
                                         std::array<math::AD2, 4>& res) {
  const auto& src0 = ego.left_curves;
  const auto& src1 = ego.right_curves;
  const auto& src2 = agent.left_curves;
  const auto& src3 = agent.right_curves;
  std::array<std::vector<math::AD2>, 4> pts;

  for (int j = 0; j < src0.size(); j++) {
    pts[0].push_back({src0[j].x(), src0[j].y()});
  }
  for (int j = 0; j < src1.size(); j++) {
    pts[1].push_back({src1[j].x(), src1[j].y()});
  }
  for (int j = 0; j < src2.size(); j++) {
    pts[2].push_back({src2[j].x(), src2[j].y()});
  }
  for (int j = 0; j < src3.size(); j++) {
    pts[3].push_back({src3[j].x(), src3[j].y()});
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 2; j < 4; j++) {
      std::array<double, 4> dis{math::Distance(pts[i].front(), pts[j].front()),
                                math::Distance(pts[i].front(), pts[j].back()),
                                math::Distance(pts[i].back(), pts[j].front()),
                                math::Distance(pts[i].back(), pts[j].back())};
      std::pair<int, double> idx{-1, 1e9};
      for (int k = 0; k < 4; k++) {
        if (idx.second > dis[k]) {
          idx.second = dis[k];
          idx.first = k;
        }
      }
      res[i * 2 + j - 2] = math::FindIntersection(
          pts[i], pts[j], idx.first & 0x02, idx.first & 0x01);
    }
  }
  std::array<int, 5> is_find{0, 0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    is_find[i] = math::Sign(res[i][0]) == 0 && math::Sign(res[i][1]) == 0;
    is_find[4] += is_find[i];
  }
}

void BackDataBasePrediction::CalcConflictZone(const std::vector<Vec2d>& ego,
                                              const std::vector<Vec2d>& agent,
                                              std::array<math::AD2, 4>& res,
                                              math::AD2& ego_rd,
                                              math::AD2& agent_rd) {
  auto GetResDistance = [](const auto& path, const auto& res, auto& rd) {
    std::array<std::pair<int, double>, 4> proj_pts = {
        {{0, 1e9}, {0, 1e9}, {0, 1e9}, {0, 1e9}}};
    for (int i = 1; i < path.size(); i++) {
      math::LineSegment seg({path[i - 1].x(), path[i - 1].y()},
                            {path[i].x(), path[i].y()});
      for (int j = 0; j < 4; j++) {
        double dis = math::Distance(res[j], seg);
        if (dis < proj_pts[j].second) {
          proj_pts[j].second = dis;
          proj_pts[j].first = i;
        }
      }
    }
    for (int j = 0; j < 4; j++) {
      double s = 0.0;
      for (int i = 1; i < proj_pts[j].first; i++) {
        s += path[i - 1].distance_to(path[i]);
      }
      s +=
          math::Dot(
              {res[j][0] - path[proj_pts[j].first - 1].x(),
               res[j][1] - path[proj_pts[j].first - 1].y()},
              {path[proj_pts[j].first].x() - path[proj_pts[j].first - 1].x(),
               path[proj_pts[j].first].y() - path[proj_pts[j].first - 1].y()}) /
          path[proj_pts[j].first].distance_to(path[proj_pts[j].first - 1]);
      rd[0] = std::min(rd[0], s);
      rd[1] = std::max(rd[1], s);
    }
  };

  ego_rd[0] = agent_rd[0] = 1e9;
  ego_rd[1] = agent_rd[1] = 0;
  GetResDistance(ego, res, ego_rd);
  GetResDistance(agent, res, agent_rd);
}

std::vector<ConnectionConflictInfo>
BackDataBasePrediction::ComputeConflictCustomData(TaskInfo& task_info) {
  std::vector<ConnectionConflictInfo> ans{};
  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return ans;
  }

  PredictionCurves ego_curves;
  std::vector<Vec2d> ego_traj;
  if (!CalcEgoCurves(task_info, ego_curves, ego_traj)) {
    LOG_ERROR("Calc ego curves failed!");
    return ans;
  }
  VisObsTrajectory(ego_traj);

  for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
    if (!interactive_agent_ids_.count(obs->id())) {
      continue;
    }
    ConnectionConflictInfo c;
    c.lane_id = obs->matched_lane_id();
    CalAgentInfo(obs, &c.agent);
    c.agent.id = obs->id();
    c.way_right = RightOfWay::UNKNOWN;
    SetConflictParam(&c);
    std::vector<PredictionCurves> agent_curves;
    std::vector<Vec2d> agent_traj;
    if (!CalcAgentCurves(obs, agent_curves, agent_traj)) {
      LOG_ERROR("Calc agent (id is {}) curves failed!", c.agent.id);
      std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
      c.conflict_area_bound = cab;
      ans.emplace_back(c);
      continue;
    }
    VisObsTrajectory(agent_traj);
    std::array<math::AD2, 4> meeting_pts;
    math::AD2 ego_rd{0.0, 0.0}, agent_rd{0.0, 0.0};
    CalcMeeting(ego_curves, agent_curves.front(), meeting_pts);
    CalcConflictZone(ego_traj, agent_traj, meeting_pts, ego_rd, agent_rd);
    c.conflict_area_bound = {agent_rd[0], agent_rd[1], ego_rd[1], ego_rd[0]};

    agent_curves.emplace_back(ego_curves);
    VisObsPredictionSegment(agent_curves);

    ans.emplace_back(c);
  }

  return ans;
}

void BackDataBasePrediction::CalcCircle(const double width, const double length,
                                        const size_t n, double& r, double& d) {
  r = std::sqrt(std::pow(0.5 * length / n, 2) + std::pow(0.5 * width, 2));
  d = length / (n - 1) -
      2.0 / (n - 1) * std::sqrt(std::pow(r, 2) - std::pow(0.5 * width, 2));
}

void BackDataBasePrediction::CalcCirclePoint(
    const Vec2d& center_pt, const double heading, const double r,
    const double d, PredictionCurves& first_circle,
    PredictionCurves& second_circle, PredictionCurves& third_circle) const {
  std::vector<std::vector<double>> unit_vec(1, std::vector<double>(2));
  unit_vec[0][0] = std::cos(heading);
  unit_vec[0][1] = std::sin(heading);
  std::vector<std::vector<double>> unit_left_vec(1, std::vector<double>(2));
  unit_left_vec[0][0] = -std::sin(heading);
  unit_left_vec[0][1] = std::cos(heading);
  std::vector<std::vector<double>> unit_right_vec(1, std::vector<double>(2));
  unit_right_vec[0][0] = std::sin(heading);
  unit_right_vec[0][1] = -std::cos(heading);
  Vec2d first_center_pt(center_pt.x() + d * unit_vec[0][0],
                        center_pt.y() + d * unit_vec[0][1]);
  Vec2d second_center_pt(center_pt.x() - d * unit_vec[0][0],
                         center_pt.y() - d * unit_vec[0][1]);
  first_circle.left_curves.emplace_back(
      Vec2d(first_center_pt.x() + r * unit_left_vec[0][0],
            first_center_pt.y() + r * unit_left_vec[0][1]));
  first_circle.right_curves.emplace_back(
      Vec2d(first_center_pt.x() + r * unit_right_vec[0][0],
            first_center_pt.y() + r * unit_right_vec[0][1]));
  second_circle.left_curves.emplace_back(
      Vec2d(center_pt.x() + r * unit_left_vec[0][0],
            center_pt.y() + r * unit_left_vec[0][1]));
  second_circle.right_curves.emplace_back(
      Vec2d(center_pt.x() + r * unit_right_vec[0][0],
            center_pt.y() + r * unit_right_vec[0][1]));
  third_circle.left_curves.emplace_back(
      Vec2d(second_center_pt.x() + r * unit_left_vec[0][0],
            second_center_pt.y() + r * unit_left_vec[0][1]));
  third_circle.right_curves.emplace_back(
      Vec2d(second_center_pt.x() + r * unit_right_vec[0][0],
            second_center_pt.y() + r * unit_right_vec[0][1]));
}

bool BackDataBasePrediction::CalcAgentCurves(
    const Obstacle* obs, std::vector<PredictionCurves>& agent_curves,
    std::vector<Vec2d>& agent_traj) {
  LOG_INFO("Start calc agent curves!");
  const auto& pred_traj = obs->uniform_trajectory();
  if (pred_traj.num_of_points() < 2) {
    LOG_ERROR("trajectory points less 2.");
    return false;
  }
  double r = 0.0;
  double d = 0.0;
  CalcCircle(obs->width(), obs->length(), 3, r, d);
  PredictionCurves first_circles, second_circles, third_circles;
  for (std::size_t i = 0; i < pred_traj.trajectory_points().size(); ++i) {
    TrajectoryPoint point{};
    if (!pred_traj.trajectory_point_at(i, point)) {
      LOG_ERROR("trajectory point at {} failed", i);
      continue;
    }
    CalcCirclePoint(Vec2d(point.x(), point.y()), point.theta(), r, d,
                    first_circles, second_circles, third_circles);
    agent_traj.emplace_back(point.coordinate());
  }
  agent_curves.emplace_back(first_circles);
  agent_curves.emplace_back(second_circles);
  agent_curves.emplace_back(third_circles);
  LOG_INFO("Agent's traj size is {}, curves size is {}!", agent_traj.size(),
           agent_curves.front().left_curves.size());
  return true;
}

bool BackDataBasePrediction::CalcEgoCurves(TaskInfo& task_info,
                                           PredictionCurves& ego_curves,
                                           std::vector<Vec2d>& ego_traj) {
  LOG_INFO("Start calc ego curves!");
  const auto& outside_data = task_info.last_frame()->outside_planner_data();
  const auto& path_points = outside_data.path_data->path().path_points();
  if (path_points.size() < 3) {
    LOG_INFO("Path points is less than 3!");
    return false;
  }
  double r = 0.0;
  double d = 0.0;
  CalcCircle(VehicleParam::Instance()->width(),
             VehicleParam::Instance()->length(), 3, r, d);
  PredictionCurves first_circles, second_circles, third_circles;
  for (const auto& pt : path_points) {
    CalcCirclePoint(pt.coordinate(), pt.theta(), r, d, first_circles,
                    second_circles, third_circles);
    ego_traj.emplace_back(pt.coordinate());
  }
  ego_curves = first_circles;
  return true;
}

}  // namespace planning
}  // namespace neodrive

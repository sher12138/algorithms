#include "speed_crosswalk_pedestrian_protect_decider.h"

#include <unordered_map>

#include "common/visualizer_event/visualizer_event.h"
#include "hdmap/hdmap.h"
#include "hdmap/hdmap_info.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using LaneObjectsInfo = std::vector<std::tuple<uint64_t, double, double>>;

namespace neodrive {
namespace planning {

namespace {
void VisCrossWalkBoundary(const ReferenceLinePtr ref_line, const Boundary& bdry,
                          const std::string& event_name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(event_name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  Vec2d pt{};
  auto polygon = event->mutable_polygon()->Add();
  ref_line->GetPointInCartesianFrame({bdry.start_s(), bdry.end_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame({bdry.start_s(), bdry.start_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame({bdry.end_s(), bdry.start_l()}, &pt);
  set_pt(polygon->add_point(), pt);
  ref_line->GetPointInCartesianFrame({bdry.end_s(), bdry.end_l()}, &pt);
  set_pt(polygon->add_point(), pt);
}
}  // namespace

SpeedCrosswalkPedestrainProtectDecider::
    SpeedCrosswalkPedestrainProtectDecider() {
  name_ = "SpeedCrosswalkPedestrainProtectDecider";
}

void SpeedCrosswalkPedestrainProtectDecider::GetPedestrianCheckArea(
    TaskInfo& task_info) {
  if (if_next_to_crosswalk_) {
    LOG_INFO("need generate crosswalk check area");
    // 先找到crosswalk 的左右l值
    double crosswalk_middle_s =
        0.5 * (crosswalk_range.first + crosswalk_range.second);
    ReferencePoint ref_pt;
    task_info.reference_line()->GetNearestRefPoint(crosswalk_middle_s, &ref_pt);
    auto lane_id = ref_pt.hd_map_lane_id();

    LaneObjectsInfo lane_cross_walk_seq =
        PlanningMap::Instance()->GetLaneCrosswalks(lane_id);

    for (auto& crosswalk_info : lane_cross_walk_seq) {
      crosswalk_id_vec_.push_back(std::get<0>(crosswalk_info));
      auto crosswalk_ptr = cyberverse::HDMap::Instance()->GetCrosswalkById(
          std::get<0>(crosswalk_info));
      if (crosswalk_ptr != nullptr) {
        auto& crosswalk_polygon = crosswalk_ptr->Polygon();
        crosswalk_polygen_vec_.emplace_back(crosswalk_polygon);
      }
    }
    if (crosswalk_polygen_vec_.empty()) {
      if_crosswalk_boundary_ = false;
    } else {
      if_crosswalk_boundary_ = true;
      LOG_INFO("crosswalk polygen size is : {}", crosswalk_polygen_vec_.size());
      CrosswalkPoly2Boundary(crosswalk_polygen_vec_, task_info);
      VisCrossWalkBoundary(task_info.reference_line(), crosswalk_boundary_,
                           "crosswalk_boundary");
    }
  }
}

void SpeedCrosswalkPedestrainProtectDecider::CrosswalkPoly2Boundary(
    const std::vector<common::math::ShmPolygon2d>& crosswalk_polygen_vec,
    TaskInfo& task_info) {
  auto odom2veh = [](Vec2d& pt, double heading) -> std::pair<Vec2d, double> {
    auto& odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x() - odom_pose.position().x();
    double dy = pt.y() - odom_pose.position().y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_},
            heading - odom_yaw};
  };
  auto veh2odom = [](Vec2d& pt, double heading) -> std::pair<Vec2d, double> {
    auto& odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{-dy * sin_ + dx * cos_ + odom_pose.position().x(),
             dx * sin_ + dy * cos_ + odom_pose.position().y()},
            heading + odom_yaw};
  };
  auto utm2veh = [](Vec2d& pt, double heading) -> std::pair<Vec2d, double> {
    auto& utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x() - utm_pose.position().x();
    double dy = pt.y() - utm_pose.position().y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - utm_yaw};
  };
  auto veh2utm = [](Vec2d& pt, double heading) -> std::pair<Vec2d, double> {
    auto& utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{-dy * sin_ + dx * cos_ + utm_pose.position().x(),
             dx * sin_ + dy * cos_ + utm_pose.position().y()},
            heading + utm_yaw};
  };
  auto Utm2Odom = [&utm2veh, &veh2odom](
                      Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto tmp = utm2veh(pt, heading);
    return veh2odom(tmp.first, tmp.second);
  };

  double min_s{10000000.0};
  int poly_index = 0;
  std::vector<std::vector<SLPoint>> crosswalk_sl_vec{};

  for (const auto& crosswalk_polygen : crosswalk_polygen_vec) {
    std::vector<SLPoint> boundary_points{};

    for (int i = 0; i < crosswalk_polygen.points().size(); i++) {
      SLPoint curr_sl;
      auto utm_xy = Utm2Odom(Vec2d(crosswalk_polygen.points()[i].x(),
                                   crosswalk_polygen.points()[i].y()),
                             atan2(crosswalk_polygen.points()[i].y(),
                                   crosswalk_polygen.points()[i].x()));
      if (!task_info.reference_line()->GetPointInFrenetFrame(
              {utm_xy.first.x(), utm_xy.first.y()}, &curr_sl)) {
        LOG_ERROR("GetPointInFrenetFrame failed");
      }

      boundary_points.emplace_back(curr_sl);
    }
    crosswalk_sl_vec.emplace_back(boundary_points);
  }
  std::sort(
      crosswalk_sl_vec.begin(), crosswalk_sl_vec.end(),
      [](const std::vector<SLPoint>& vec1, const std::vector<SLPoint>& vec2) {
        double min_s1 =
            std::min({vec1[0].s(), vec1[1].s(), vec1[2].s(), vec1[3].s()});
        double min_s2 =
            std::min({vec2[0].s(), vec2[1].s(), vec2[2].s(), vec2[3].s()});
        return min_s1 < min_s2;
      });
  corsswalk_boundary_points_ = crosswalk_sl_vec.front();
  SLPoint curr_ego_sl;
  task_info.reference_line()->GetPointInFrenetFrame(
      {vehicle_state_.X(), vehicle_state_.Y()}, &curr_ego_sl);
  for (int i = 0; i < crosswalk_sl_vec.size(); i++) {
    double max_s =
        std::max({crosswalk_sl_vec[i][0].s(), crosswalk_sl_vec[i][1].s(),
                  crosswalk_sl_vec[i][2].s(), crosswalk_sl_vec[i][3].s()});
    if (max_s < curr_ego_sl.s()) {
      continue;
    }

    corsswalk_boundary_points_.clear();
    corsswalk_boundary_points_ = crosswalk_sl_vec[i];
    break;
  }
  Boundary boundary{std::min({corsswalk_boundary_points_[0].s(),
                              corsswalk_boundary_points_[1].s(),
                              corsswalk_boundary_points_[2].s(),
                              corsswalk_boundary_points_[3].s()}),
                    std::max({corsswalk_boundary_points_[0].s(),
                              corsswalk_boundary_points_[1].s(),
                              corsswalk_boundary_points_[2].s(),
                              corsswalk_boundary_points_[3].s()}),
                    std::min({corsswalk_boundary_points_[0].l(),
                              corsswalk_boundary_points_[1].l(),
                              corsswalk_boundary_points_[2].l(),
                              corsswalk_boundary_points_[3].l()}),
                    std::max({corsswalk_boundary_points_[0].l(),
                              corsswalk_boundary_points_[1].l(),
                              corsswalk_boundary_points_[2].l(),
                              corsswalk_boundary_points_[3].l()})};
  std::vector<SLPoint> cornerpoint{
      SLPoint{boundary.start_s(), boundary.start_l()},
      SLPoint{boundary.start_s(), boundary.end_l()},
      SLPoint{boundary.end_s(), boundary.start_l()},
      SLPoint{boundary.end_s(), boundary.end_l()}};
  for (auto& cross_sl : cornerpoint) {
    LOG_INFO("cross wall sl point  : {} {}", cross_sl.s(), cross_sl.l());
  }
  crosswalk_boundary_ = boundary;
}

SpeedCrosswalkPedestrainProtectDecider::
    ~SpeedCrosswalkPedestrainProtectDecider() {}

ErrorCode SpeedCrosswalkPedestrainProtectDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    LOG_INFO("path successed tasks is 0, skip rest tasks.");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  LOG_INFO(">>>> Crosswalk pedestrian protect work normal");
  if (!Init(task_info)) {
    LOG_ERROR("Crosswalk pedestrian protect Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Crosswalk pedestrian protectr Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void SpeedCrosswalkPedestrainProtectDecider::SetSpeedLimit(double speed_limit,
                                                           TaskInfo& task_info,
                                                           bool if_hard) {
  neodrive::global::planning::SpeedLimit internal_speed_limit{};
  internal_speed_limit.set_source_type(SpeedLimitType::PEDESTRAIN);
  internal_speed_limit.add_upper_bounds(speed_limit);
  if (if_hard) {
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
  } else {
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
  }
  internal_speed_limit.set_acceleration(0.0);
  LOG_INFO(
      "pedestrian protect {} limit speed: speed = {:.2f}, acc = {:.2f}",
      SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
      speed_limit, 0.0);

  data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
      internal_speed_limit);
}

void SpeedCrosswalkPedestrainProtectDecider::Reset() {
  crosswalk_polygen_vec_.clear();
  crosswalk_id_vec_.clear();
  if_next_to_crosswalk_ = false;
  if_crosswalk_boundary_ = false;
  corsswalk_boundary_points_.clear();
  crosswalk_boundary_.reset();
  crosswalk_id_vec_.clear();
};

bool SpeedCrosswalkPedestrainProtectDecider::Init(TaskInfo& task_info) {
  LOG_INFO(">>>> step into Check CIPV Decider");

  adc_current_s_ =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();

  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  Reset();

  if (IfInCrosswalkCheckArea(task_info)) {
    GetPedestrianCheckArea(task_info);
  }

  return true;
}

bool SpeedCrosswalkPedestrainProtectDecider::IfInCrosswalkCheckArea(
    TaskInfo& task_info) {
  LOG_INFO("Start crosswalk check");
  for (const auto& junction_overlap :
       task_info.reference_line()->junction_overlaps()) {
    if (junction_overlap.object_id != 0 &&
        adc_front_edge_s_ > junction_overlap.start_s - 5.0 &&
        adc_front_edge_s_ < junction_overlap.end_s) {
      LOG_INFO("Ego is in junction");

      std::vector<std::pair<double, double>> crosswalk_overlap_s_pair{};
      for (const auto& crosswalk_overlap :
           task_info.reference_line()->crosswalk_overlaps()) {
        if (crosswalk_overlap.object_id != 0 &&
            crosswalk_overlap.end_s < junction_overlap.end_s &&
            crosswalk_overlap.start_s > junction_overlap.start_s) {
          crosswalk_overlap_s_pair.push_back(
              {crosswalk_overlap.start_s, crosswalk_overlap.end_s});
        }
      }

      if (crosswalk_overlap_s_pair.empty()) {
        return false;
      }
      if (!crosswalk_overlap_s_pair.empty()) {
        std::sort(
            crosswalk_overlap_s_pair.begin(), crosswalk_overlap_s_pair.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        for (const auto& crosswalk : crosswalk_overlap_s_pair) {
          LOG_INFO("show road crosswalk s range is : {} : {}", crosswalk.first,
                   crosswalk.second);
          if (crosswalk.first - 5.0 < adc_front_edge_s_ &&
              crosswalk.second > adc_front_edge_s_) {
            if_next_to_crosswalk_ = true;
            LOG_INFO(
                "adc exactly in crosswalk check area , ego front s is : {} , "
                "crosswalk "
                "s "
                "range is : {} : {}",
                adc_front_edge_s_, crosswalk.first, crosswalk.second);
            crosswalk_range.first = crosswalk.first;
            crosswalk_range.second = crosswalk.second;
            return true;
          }
        }
      }
      break;
    }
  }
  return false;
}

bool SpeedCrosswalkPedestrainProtectDecider::Process(TaskInfo& task_info) {
  if (!ProcessProtect(task_info)) {
    LOG_INFO("ProcessObsProtect error.");
    return false;
  }

  return true;
}

bool SpeedCrosswalkPedestrainProtectDecider::ProcessProtect(
    TaskInfo& task_info) {
  if (if_crosswalk_boundary_ &&
      (task_info.current_frame()->inside_planner_data().init_point.velocity() <
       2.0)) {
    CheckDynamicPedestrianInCheckArea(task_info);
    CheckStaticPedestrianInCheckArea(task_info);
  }
  return true;
}

void SpeedCrosswalkPedestrainProtectDecider::CheckStaticPedestrianInCheckArea(
    TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& static_obs_vec = task_info.current_frame()
                                   ->planning_data()
                                   .decision_data()
                                   .static_obstacle();

  LOG_INFO("check all static obstacle num: [{}].", static_obs_vec.size());
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  SLPoint curr_sl;
  task_info.reference_line()->GetPointInFrenetFrame(
      {vehicle_state_.X(), vehicle_state_.Y()}, &curr_sl);
  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (nullptr == static_obs_vec[i]) {
      LOG_DEBUG("static obstacle is nullptr .");
      continue;
    }
    if (static_obs_vec[i]->type() != Obstacle::ObstacleType::PEDESTRIAN) {
      continue;
    }
    if (static_obs_vec[i]->PolygonBoundary().end_s() < curr_sl.s()) continue;
    if (!crosswalk_boundary_.has_overlap(
            static_obs_vec[i]->PolygonBoundary())) {
      continue;
    }
    LOG_INFO(
        "Pedestrian id: [{}] is in crosswalk check area , obs start l is "
        "{} "
        ", end l is {}",
        static_obs_vec[i]->id(), static_obs_vec[i]->PolygonBoundary().start_l(),
        static_obs_vec[i]->PolygonBoundary().end_l());
    // 剩下的障碍物都是在斑马线上的了
    auto outside_planner_data_ptr =
        task_info.current_frame()->mutable_outside_planner_data();
    auto& adc_boundaries =
        outside_planner_data_ptr->speed_obstacle_context.adc_sl_boundaries;
    for (auto& ego_poly : adc_boundaries) {
      if (ego_poly.has_overlap(static_obs_vec[i]->PolygonBoundary())) {
        LOG_INFO(
            "static pedestrian has overlap with path ,id is: [{}],set speed "
            "limit 0",
            static_obs_vec[i]->id());
        SetSpeedLimit(0.0, task_info, true);
        break;
      }
    }
    double min_obs_l =
        std::min((static_obs_vec[i]->PolygonBoundary().start_l() - curr_sl.l()),
                 (static_obs_vec[i]->PolygonBoundary().end_l() - curr_sl.l()));
    if (std::abs(min_obs_l) < 0.8) {
      LOG_INFO(
          "static pedestrian to close to path ,id is: [{}],set speed "
          "limit 0",
          static_obs_vec[i]->id());
      SetSpeedLimit(0.0, task_info, true);
    } else if (std::abs(min_obs_l) < 1.8) {
      LOG_INFO(
          "static pedestrian to near to path ,id is: [{}],set speed "
          "limit 3kph",
          static_obs_vec[i]->id());
      SetSpeedLimit(0.83, task_info, false);
    }
  }
}

void SpeedCrosswalkPedestrainProtectDecider::CheckDynamicPedestrianInCheckArea(
    TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();

  LOG_INFO("check all dynamic obstacle num: [{}].", dynamic_obs_vec.size());
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();

  SLPoint curr_sl;
  task_info.reference_line()->GetPointInFrenetFrame(
      {vehicle_state_.X(), vehicle_state_.Y()}, &curr_sl);

  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (nullptr == dynamic_obs_vec[i]) {
      LOG_DEBUG("dynamic obstacle is nullptr .");
      continue;
    }
    if (dynamic_obs_vec[i]->type() != Obstacle::ObstacleType::PEDESTRIAN) {
      continue;
    }
    if (dynamic_obs_vec[i]->PolygonBoundary().end_s() < curr_sl.s()) continue;
    if (crosswalk_boundary_.has_overlap(
            dynamic_obs_vec[i]->PolygonBoundary())) {
      PathPoint closest_pt{};
      double path_heading_near_obs =
          path.query_closest_point(dynamic_obs_vec[i]->center(), closest_pt)
              ? closest_pt.theta()
              : inside_data.vel_heading;
      double heading_diff = normalize_angle(
          dynamic_obs_vec[i]->velocity_heading() - path_heading_near_obs);
      LOG_INFO(
          "Pedestrian id: [{}] is in crosswalk check area , obs start l is "
          "{} "
          ", end l is {}",
          dynamic_obs_vec[i]->id(),
          dynamic_obs_vec[i]->PolygonBoundary().start_l(),
          dynamic_obs_vec[i]->PolygonBoundary().end_l());
      // starting judge
      if (heading_diff * 180 / 3.14 < 0.0 &&
          heading_diff * 180 / 3.14 >= (-180.0)) {
        if (dynamic_obs_vec[i]->PolygonBoundary().end_l() < curr_sl.l()) {
          LOG_INFO(
              "Pedestrian id: [{}] is in crosswalk check area , heading diff "
              "is "
              ": "
              "{} , he is going right, he is already right",
              dynamic_obs_vec[i]->id(), heading_diff * 180 / 3.14);
          if (std::abs(curr_sl.l() -
                       dynamic_obs_vec[i]->PolygonBoundary().end_l()) > 1.2) {
            LOG_INFO("Because  Pedestrian id : [ {} ] is too right ,ignore",
                     dynamic_obs_vec[i]->id());
            continue;
          } else {
            SetSpeedLimit(0.0, task_info, true);
            LOG_INFO(
                "Because  Pedestrian id : [ {} ] is right but not far ,limit "
                "speed 0",
                dynamic_obs_vec[i]->id());
          }

        } else {
          LOG_INFO(
              "Pedestrian id: [{}] is in crosswalk check area , heading diff "
              "is "
              ": "
              "{} , he is going right, he is in left",
              dynamic_obs_vec[i]->id(), heading_diff * 180 / 3.14);

          if (std::abs(dynamic_obs_vec[i]->PolygonBoundary().start_l() -
                       curr_sl.l()) >
              std::max(5.0, 2 * std::cos(heading_diff) *
                                dynamic_obs_vec[i]->velocity_heading())) {
            LOG_INFO("Because  Pedestrian id : [ {} ] is too left ,ignore",
                     dynamic_obs_vec[i]->id());
            continue;
          } else {
            SetSpeedLimit(0.0, task_info, true);
            LOG_INFO(
                "Because  Pedestrian id : [ {} ] is left but not far ,limit "
                "speed 0",
                dynamic_obs_vec[i]->id());
          }
        }
      } else {
        if (dynamic_obs_vec[i]->PolygonBoundary().start_l() > curr_sl.l()) {
          LOG_INFO(
              "Pedestrian id: [{}] is in crosswalk check area , heading diff "
              "is "
              ": "
              "{} , he is going left, he is already left",
              dynamic_obs_vec[i]->id(), heading_diff * 180 / 3.14);

          if (std::abs(dynamic_obs_vec[i]->PolygonBoundary().start_l() -
                       curr_sl.l()) > 1.2) {
            LOG_INFO("Because  Pedestrian id : [ {} ] is too left ,ignore",
                     dynamic_obs_vec[i]->id());
            continue;
          } else {
            SetSpeedLimit(0.0, task_info, true);
            LOG_INFO(
                "Because  Pedestrian id : [ {} ] is left but not far ,limit "
                "speed 0",
                dynamic_obs_vec[i]->id());
          }
        } else {
          LOG_INFO(
              "Pedestrian id: [{}] is in crosswalk check area , heading diff "
              "is "
              ": "
              "{} , he is going left , he is in right",
              dynamic_obs_vec[i]->id(), heading_diff * 180 / 3.14);
          if (std::abs((dynamic_obs_vec[i]->PolygonBoundary().end_l() -
                        curr_sl.l())) >
              std::max(5.0, 2 * std::cos(heading_diff) *
                                dynamic_obs_vec[i]->velocity_heading())) {
            LOG_INFO("Because  Pedestrian id : [ {} ] is too right ,ignore",
                     dynamic_obs_vec[i]->id());
            continue;
          } else {
            SetSpeedLimit(0.0, task_info, true);
            LOG_INFO(
                "Because  Pedestrian id : [ {} ] is right but not far ,limit "
                "speed 0",
                dynamic_obs_vec[i]->id());
          }
        }
      }
    }
  }
}

void SpeedCrosswalkPedestrainProtectDecider::SaveTaskResults(
    TaskInfo& task_info) {}

}  // namespace planning
}  // namespace neodrive

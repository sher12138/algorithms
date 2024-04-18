#include "visual_lane_combine_ego_lane.h"

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/reference_line/reference_line_util.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;
using neodrive::global::perception::camera::LaneLineType;

namespace neodrive {
namespace planning {

VisualLaneCombineEgoLane::VisualLaneCombineEgoLane() {
  data_center_ = DataCenter::Instance();
}

void VisMatchLinePoints(
    ReferenceLinePtr ref_line,
    const std::vector<std::vector<SLPoint>> match_lanes_sl) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("vis_curbs");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  for (auto& line : match_lanes_sl) {
    Vec2d tp{};
    for (auto sl : line) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->set_radius(0.1);
      ref_line->GetPointInCartesianFrame({sl.s(), sl.l()}, &tp);
      set_pt(sphere->mutable_center(), tp);
    }
  }
}

bool VisualLaneCombineEgoLane::GetVisCurbBound(
    SLPoint& ego_sl, ReferenceLinePtr reference_line,
    std::array<double, 2>& side_lat) {
  /// valid check
  if (data_center_->last_frame() == nullptr) {
    LOG_INFO("last_frame is nullptr, skip.");
    return false;
  }
  auto camera_lane_config = config::PlanningConfig::Instance()
                                ->planning_research_config()
                                .camera_lane_config;
  std::size_t start_index{0}, end_index{0};
  if (!reference_line->GetStartEndIndexBySLength(
          ego_sl.s(), camera_lane_config.junction_pre_dis, &start_index,
          &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return false;
  }

  // compute need_match_lanes
  const auto& perception_lanes =
      data_center_->environment().perception_lanes_proxy().camera_lanes_line();
  if (perception_lanes.empty()) {
    LOG_INFO("perception_lanes empty, skip.");
    return false;
  } else {
    LOG_INFO("perception_lanes size:{}", perception_lanes.size());
  }
  need_match_lanes_type_.clear();
  need_match_lanes_pos_type_.clear();
  need_match_lanes_sl_.clear();
  for (const auto& lane : perception_lanes) {
    LOG_INFO("lane's type:{} , lane's pos type:{} ", lane.type(),
             lane.pos_type());

    // 1.Filter unreasonable perception lines
    auto type = lane.type();
    if (type != LaneLineType::ROAD_CURB) continue;
    auto pos_type = lane.pos_type();  // for reference only, not accurate
    if (!(pos_type == CameraLaneLinePositionType::EGO_LEFT ||
          pos_type == CameraLaneLinePositionType::EGO_RIGHT)) {
      continue;
    }

    // 2. Perception lines to frenet
    std::vector<SLPoint> sl_points{};
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
    auto odom_x = loc.x();
    auto odom_y = loc.y();
    for (const auto& point : lane.bev_image_point_set()) {
      SLPoint sl_point{};
      if (!reference_line->GetPointInFrenetFrame(
              {point.x() * ct - point.y() * st + odom_x,
               point.x() * st + point.y() * ct + odom_y},
              &sl_point)) {
        continue;
      }
      if (sl_point.s() > ego_sl.s() + camera_lane_config.bev_2d_use_length ||
          sl_point.s() < ego_sl.s()) {
        continue;
      }
      if (sl_points.empty()) {
        sl_points.emplace_back(sl_point);
      } else {
        if (sl_point.s() > sl_points.back().s() + 0.01) {
          sl_points.emplace_back(sl_point);
        }
      }
    }
    if (sl_points.size() < 4) {
      continue;
    }
    need_match_lanes_type_.emplace_back(type);
    need_match_lanes_pos_type_.emplace_back(pos_type);
    need_match_lanes_sl_.emplace_back(sl_points);
  }
  if (need_match_lanes_type_.empty() || need_match_lanes_pos_type_.empty() ||
      need_match_lanes_sl_.empty()) {
    LOG_INFO("lane_line_positon_type could not match, skip.");
    return false;
  }
  for (int k = 0; k < need_match_lanes_pos_type_.size(); ++k) {
    LOG_INFO("chosen lines:  type:{}, pos_type:{}, front l:{}",
             need_match_lanes_type_[k], need_match_lanes_pos_type_[k],
             need_match_lanes_sl_[k].at(0).l());
  }
  VisMatchLinePoints(reference_line, need_match_lanes_sl_);

  // Get closest lateral position
  std::vector<double> lat_vec{ego_sl.l()};
  for (std::size_t index = 0; index < need_match_lanes_sl_.size(); ++index) {
    double min_dis_l{std::numeric_limits<double>::max()};
    std::size_t min_dis_index{0};
    for (std::size_t k = 0; k < need_match_lanes_sl_[index].size(); ++k) {
      double sample_l = need_match_lanes_sl_[index].at(k).l();
      double dis_l = std::abs(sample_l - ego_sl.l());
      if (min_dis_l > dis_l) {
        min_dis_l = dis_l;
        min_dis_index = k;
      }
    }
    auto closest_sl = need_match_lanes_sl_[index].at(min_dis_index);
    LOG_INFO("match line closest s:{:.4f}, l:{:.4f}   index:{}", closest_sl.s(),
             closest_sl.l(), min_dis_index);
    lat_vec.push_back(closest_sl.l());
  }

  // output
  std::sort(lat_vec.begin(), lat_vec.end());
  auto index = std::distance(
      lat_vec.begin(), std::find(lat_vec.begin(), lat_vec.end(), ego_sl.l()));
  if (index == 0) {
    side_lat[0] = lat_vec[index + 1];
    LOG_INFO("left_l:{:.4f}, ego_l:{:.4f}", side_lat[0], ego_sl.l());
  } else if (index == lat_vec.size() - 1) {
    side_lat[1] = lat_vec[index - 1];
    LOG_INFO("ego_l:{:.4f}, right_l:{:.4f}", ego_sl.l(), side_lat[1]);
  } else {
    side_lat[0] = lat_vec[index + 1];
    side_lat[1] = lat_vec[index - 1];
    LOG_INFO("left_l:{:.4f}, ego_l:{:.4f}, right_l:{:.4f}", side_lat[0],
             ego_sl.l(), side_lat[1]);
  }

  return true;
}

bool VisualLaneCombineEgoLane::Combine(TaskInfo& task_info) {
  /// valid check
  if (data_center_->last_frame() == nullptr) {
    LOG_INFO("last_frame is nullptr, skip.");
    return false;
  }
  auto camera_lane_config = config::PlanningConfig::Instance()
                                ->planning_research_config()
                                .camera_lane_config;
  // front has junction should not use perception lane
  auto reference_line_raw = task_info.reference_line_raw();
  auto reference_line = task_info.reference_line();
  std::size_t start_index{0}, end_index{0};
  if (!reference_line->GetStartEndIndexBySLength(
          task_info.curr_sl().s(), camera_lane_config.junction_pre_dis,
          &start_index, &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return false;
  }
  Segment2d cut_ref_seg({reference_line->ref_points()[start_index].s(), 0.},
                        {reference_line->ref_points()[end_index].s(), 0.});
  for (const auto& [junction_ptr, overlap] : reference_line->junctions()) {
    if (junction_ptr == nullptr) continue;
    if (junction_ptr->Type() != static_cast<uint32_t>(JunctionType::IN_ROAD)) {
      Segment2d junction_seg({overlap.start_s, 0.}, {overlap.end_s, 0.});
      if (cut_ref_seg.has_intersect(junction_seg)) {
        LOG_INFO("front has junction and not is IN_ROAD type, skip.");
        return true;
      }
    }
  }
  // last frame is lane borrow scenario should not use perception lane
  bool last_frame_borrow_status =
      data_center_->last_frame()->inside_planner_data().is_lane_borrowing ||
      data_center_->last_frame()->inside_planner_data().is_prepare_borrowing;
  if (last_frame_borrow_status) {
    LOG_INFO("last_frame is borrow, skip.");
    return false;
  }
  // compute need_match_lanes
  const auto& perception_lanes =
      data_center_->environment().perception_lanes_proxy().camera_lanes_line();
  if (perception_lanes.empty()) {
    LOG_INFO("perception_lanes empty, skip.");
    return false;
  }

  /// compute perception match infos
  // compute perception pose/heading
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
  auto odom_x = loc.x();
  auto odom_y = loc.y();
  // compute need_match infos
  need_match_lanes_type_.clear();
  need_match_lanes_pos_type_.clear();
  need_match_lanes_sl_.clear();
  for (const auto& lane : perception_lanes) {
    auto pos_type = lane.pos_type();
    if (!(pos_type == CameraLaneLinePositionType::EGO_LEFT ||
          pos_type == CameraLaneLinePositionType::EGO_RIGHT ||
          pos_type == CameraLaneLinePositionType::ADJACENT_LEFT ||
          pos_type == CameraLaneLinePositionType::ADJACENT_RIGHT)) {
      continue;
    }
    std::vector<SLPoint> sl_points{};
    for (const auto& point : lane.bev_image_point_set()) {
      SLPoint sl_point{};
      if (!reference_line->GetPointInFrenetFrame(
              {point.x() * ct - point.y() * st + odom_x,
               point.x() * st + point.y() * ct + odom_y},
              &sl_point)) {
        continue;
      }
      if (sl_point.s() >
              task_info.curr_sl().s() + camera_lane_config.bev_2d_use_length ||
          sl_point.s() < task_info.curr_sl().s()) {
        continue;
      }
      if (sl_points.empty()) {
        sl_points.emplace_back(sl_point);
      } else {
        if (sl_point.s() > sl_points.back().s() + 0.01) {
          sl_points.emplace_back(sl_point);
        }
      }
    }
    if (sl_points.size() < 4) {
      continue;
    }
    need_match_lanes_type_.emplace_back(lane.type());
    need_match_lanes_pos_type_.emplace_back(lane.pos_type());
    need_match_lanes_sl_.emplace_back(sl_points);
  }
  if (need_match_lanes_type_.empty() || need_match_lanes_pos_type_.empty() ||
      need_match_lanes_sl_.empty()) {
    LOG_INFO("lane_line_positon_type could not match, skip.");
    return false;
  }

  /// compute ego lane infos
  ref_s_vec_.clear();
  ref_left_lane_vec_.clear();
  ref_left_road_vec_.clear();
  ref_right_lane_vec_.clear();
  ref_right_road_vec_.clear();
  for (std::size_t index = start_index; index < end_index; ++index) {
    const auto& point = reference_line->ref_points()[index];
    if (point.s() - task_info.curr_sl().s() >
        camera_lane_config.bev_2d_use_length) {
      break;
    }
    ref_s_vec_.emplace_back(point.s());
    ref_left_lane_vec_.emplace_back(point.left_lane_bound());
    ref_left_road_vec_.emplace_back(point.left_road_bound());
    ref_right_lane_vec_.emplace_back(-point.right_lane_bound());
    ref_right_road_vec_.emplace_back(-point.right_road_bound());
  }
  if (ref_s_vec_.size() < 4) {
    LOG_INFO("refence_line s_vec size < 4, skip.");
    return false;
  }
  tk::spline ref_left_lane_spline{}, ref_left_road_spline{},
      ref_right_lane_spline{}, ref_right_road_spline{};
  ref_left_lane_spline.set_points(ref_s_vec_, ref_left_lane_vec_);
  ref_left_road_spline.set_points(ref_s_vec_, ref_left_road_vec_);
  ref_right_lane_spline.set_points(ref_s_vec_, ref_right_lane_vec_);
  ref_right_road_spline.set_points(ref_s_vec_, ref_right_road_vec_);

  /// match perception lane with ego lane
  // sum_cost: (l - l_i_perception_lane)^2 / num_points
  // matched string: "LEFT_LANE"/"LEFT_ROAD"/"RIGHT_LANE"/"RIGHT_ROAD"
  auto min_index = [](const auto& cost) {
    std::size_t index{cost.size()};
    double min_cost{std::numeric_limits<double>::max()};
    for (std::size_t i = 0; i < cost.size(); ++i) {
      if (cost[i].second < min_cost) {
        min_cost = cost[i].second;
        index = i;
      }
    }
    return index;
  };
  auto matched_string = [](const auto& type) {
    if (type == EgoRoadLaneType::LEFT_ROAD) return "LEFT_ROAD";
    if (type == EgoRoadLaneType::LEFT_LANE) return "LEFT_LANE";
    if (type == EgoRoadLaneType::RIGHT_LANE) return "RIGHT_LANE";
    if (type == EgoRoadLaneType::RIGHT_ROAD) return "RIGHT_ROAD";
    return "NONE";
  };

  matched_type_index_vec_.clear();
  for (std::size_t index = 0; index < need_match_lanes_sl_.size(); ++index) {
    std::vector<std::pair<EgoRoadLaneType, double>> matched_cost = {
        {EgoRoadLaneType::LEFT_ROAD, 0.},
        {EgoRoadLaneType::LEFT_LANE, 0.},
        {EgoRoadLaneType::RIGHT_LANE, 0.},
        {EgoRoadLaneType::RIGHT_ROAD, 0.}};
    for (const auto& sl_point : need_match_lanes_sl_[index]) {
      matched_cost[0].second +=
          std::fabs(sl_point.l() - ref_left_road_spline(sl_point.s()));
      matched_cost[1].second +=
          std::fabs(sl_point.l() - ref_left_lane_spline(sl_point.s()));
      matched_cost[2].second +=
          std::fabs(sl_point.l() - ref_right_lane_spline(sl_point.s()));
      matched_cost[3].second +=
          std::fabs(sl_point.l() - ref_right_road_spline(sl_point.s()));
    }
    std::size_t matched_min_index = min_index(matched_cost);
    double min_cost{10.};
    if (matched_min_index < matched_cost.size()) {
      min_cost = matched_cost[matched_min_index].second /
                 need_match_lanes_sl_[index].size();
    }
    LOG_INFO(
        "index, matched_min_index, min_cost, matched_string: {}, {}, {:.3f}, "
        "{}",
        index, matched_min_index, min_cost,
        matched_string(matched_cost[matched_min_index].first));
    if (min_cost > camera_lane_config.bev_2d_average_value_threshold) {
      continue;
    }
    matched_type_index_vec_.emplace_back(
        std::make_pair(matched_cost[matched_min_index].first, index));
  }

  // modify reference_line_raw/reference_line road/lane bound
  //   const auto& reference_line = task_info.reference_line();
  for (const auto& matched_string_index : matched_type_index_vec_) {
    LOG_INFO("matched_string_index: {}",
             matched_string(matched_string_index.first));
    std::vector<double> lane_s_vec{}, lane_l_vec{};
    for (const auto& sl_point :
         need_match_lanes_sl_[matched_string_index.second]) {
      lane_s_vec.emplace_back(sl_point.s());
      lane_l_vec.emplace_back(sl_point.l());
    }
    if (lane_s_vec.size() < 4) {
      continue;
    }
    tk::spline lane_sl_spline{};
    lane_sl_spline.set_points(lane_s_vec, lane_l_vec);

    std::size_t matched_start_index = ref_line_util::BinarySearchIndex(
        reference_line->ref_points(), lane_s_vec.front());
    std::size_t matched_end_index = ref_line_util::BinarySearchIndex(
        reference_line->ref_points(), lane_s_vec.back());

    for (std::size_t index = matched_start_index; index < matched_end_index;
         ++index) {
      double set_bound =
          lane_sl_spline(reference_line->ref_points().at(index).s());
      double left_road_bound =
          (matched_string_index.first == EgoRoadLaneType::LEFT_ROAD)
              ? std::abs(set_bound)
              : reference_line->ref_points().at(index).left_road_bound();
      double right_road_bound =
          (matched_string_index.first == EgoRoadLaneType::RIGHT_ROAD)
              ? std::abs(set_bound)
              : reference_line->ref_points().at(index).right_road_bound();
      double right_lane_bound =
          (matched_string_index.first == EgoRoadLaneType::RIGHT_LANE)
              ? std::abs(set_bound)
              : reference_line->ref_points().at(index).right_lane_bound();
      double left_lane_bound =
          (matched_string_index.first == EgoRoadLaneType::LEFT_LANE)
              ? std::abs(set_bound)
              : reference_line->ref_points().at(index).left_lane_bound();
      reference_line->SetIndexRoadBound(index, left_road_bound,
                                        right_road_bound);
      reference_line->SetIndexLaneBound(index, left_lane_bound,
                                        right_lane_bound);

      const auto& point_raw = reference_line_raw->ref_points().at(index);
      const auto& point = reference_line->ref_points().at(index);
      LOG_INFO(
          "front: s, l_r_b, l_l_b, r_l_b, r_r_b: {:.3f}, {:.3f}, {:.3f}, "
          "{:.3f}, "
          "{:.3f}",
          point_raw.s(), point_raw.left_road_bound(),
          point_raw.left_lane_bound(), point_raw.right_lane_bound(),
          point_raw.right_road_bound());
      LOG_INFO(
          "back:  s, l_r_b, l_l_b, r_l_b, r_r_b: {:.3f}, {:.3f}, {:.3f}, "
          "{:.3f}, "
          "{:.3f}",
          point.s(), point.left_road_bound(), point.left_lane_bound(),
          point.right_lane_bound(), point.right_road_bound());
    }
  }

  // check lane bound > road bound
  for (std::size_t index = start_index; index < end_index; ++index) {
    const auto& point = reference_line->ref_points()[index];
    if (point.s() - task_info.curr_sl().s() >
        camera_lane_config.bev_2d_use_length) {
      LOG_INFO("break s: {:.3f}", point.s());
      break;
    }
    double odom_left_lane_bound = point.left_lane_bound();
    double odom_left_road_bound = point.left_road_bound();
    double odom_right_lane_bound = point.right_lane_bound();
    double odom_right_road_bound = point.right_road_bound();
    if (odom_left_lane_bound > odom_left_road_bound + kMathEpsilon ||
        odom_right_lane_bound > odom_right_road_bound + kMathEpsilon) {
      reference_line->SetIndexRoadBound(
          index, std::fmax(odom_left_lane_bound, odom_left_road_bound),
          std::fmax(odom_right_lane_bound, odom_right_road_bound));
    }

    const auto& point_odom = reference_line->ref_points().at(index);
    reference_line->SetIndexBound(index, point_odom.left_lane_bound(),
                                  point_odom.right_lane_bound());

    reference_line_raw->SetIndexLaneBound(index, point_odom.left_lane_bound(),
                                          point_odom.right_lane_bound());
    reference_line_raw->SetIndexBound(index, point_odom.left_lane_bound(),
                                      point_odom.right_lane_bound());
    reference_line_raw->SetIndexRoadBound(index, point_odom.left_road_bound(),
                                          point_odom.right_road_bound());

    const auto& point_utm = reference_line_raw->ref_points().at(index);
    const auto& point_odom_2 = reference_line->ref_points().at(index);

    LOG_INFO(
        "utm:  s, l_r_b, l_l_b, l_b, r_b, r_l_b, r_r_b: {:.3f}, {:.3f}, "
        "{:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}, "
        "{:.3f}",
        point_utm.s(), point_utm.left_road_bound(), point_utm.left_lane_bound(),
        point_utm.left_bound(), point_utm.right_bound(),
        point_utm.right_lane_bound(), point_utm.right_road_bound());
    LOG_INFO(
        "odom: s, l_r_b, l_l_b, l_b, r_b, r_l_b, r_r_b: {:.3f}, {:.3f}, "
        "{:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}, "
        "{:.3f}",
        point_odom_2.s(), point_odom_2.left_road_bound(),
        point_odom_2.left_lane_bound(), point_odom_2.left_bound(),
        point_odom_2.right_bound(), point_odom_2.right_lane_bound(),
        point_odom_2.right_road_bound());
  }

  return true;
}

void VisualLaneCombineEgoLane::SetCurbLines(TaskInfo& task_info) {
  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  const auto& perception_lanes =
      data_center_->environment().perception_lanes_proxy().camera_lanes_line();
  auto* camera_curb_lines_ptr = data_center_->mutable_environment()
                                    ->mutable_perception_lanes_proxy()
                                    ->mutable_camera_curb_lines();
  camera_curb_lines_ptr->clear();
  LOG_INFO("perception_lanes size = {}", perception_lanes.size());
  double vel = inside_planner_data.vel_v;
  double check_dis = VehicleParam::Instance()->front_edge_to_center() +
                     vel * vel / 4.0 +
                     config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .speed_slow_down_decider_config.curb_check_distance;
  for (const auto& lane : perception_lanes) {
    if (lane.type() != LaneLineType::ROAD_CURB) {
      continue;
    }
    if (lane.odometry_point_set_size() < 3) {
      LOG_INFO("odometry_point_set size < 3, do not check.");
      continue;
    }
    std::vector<Segment2d> curb_segments;
    Vec2d start_point{lane.odometry_point_set(0).x(),
                      lane.odometry_point_set(0).y()};
    Vec2d end_point;
    for (int index = 0; index < lane.odometry_point_set_size(); index++) {
      const auto& odom_curb_point = lane.odometry_point_set(index);
      const auto& bev_curb_point = lane.bev_image_point_set(index);
      end_point.set_x(odom_curb_point.x());
      end_point.set_y(odom_curb_point.y());
      if (start_point.distance_sqr_to(end_point) < 0.01) {
        continue;
      }
      if (std::pow(bev_curb_point.x(), 2) + std::pow(bev_curb_point.y(), 2) <
          std::pow(check_dis, 2)) {
        Segment2d segment(start_point, end_point);
        curb_segments.emplace_back(segment);
      }
      start_point.set_x(odom_curb_point.x());
      start_point.set_y(odom_curb_point.y());
    }
    camera_curb_lines_ptr->emplace_back(curb_segments);
  }
  if (camera_curb_lines_ptr->empty()) {
    LOG_INFO("camera_curb_lines is empty.");
  }
}

}  // namespace planning
}  // namespace neodrive

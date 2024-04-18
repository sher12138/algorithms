#include "visual_lane_check_utm_heading.h"

#include "src/planning/common/math/segment2d.h"
#include "src/planning/common/math/util.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/curve1d/spline.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

VisualLaneCheckUtmHeading::VisualLaneCheckUtmHeading() {
  data_center_ = DataCenter::Instance();
}

bool VisualLaneCheckUtmHeading::Check() {
  if (data_center_->last_frame() == nullptr) {
    LOG_INFO("last_frame is nullptr, skip.");
    return true;
  }
  if (data_center_->task_info_list().empty()) {
    LOG_INFO("last task_info_list is empty, skip.");
    return true;
  }
  if (!data_center_->task_info_list().front().reference_line()) {
    LOG_INFO("last reference_line is nullptr, skip.");
    return true;
  }
  if (!data_center_->task_info_list().front().reference_line_raw()) {
    LOG_INFO("last reference_line_raw is nullptr, skip.");
    return true;
  }
  // last frame is lane borrow scenario should not use perception lane
  bool last_frame_borrow_status =
      data_center_->last_frame()->inside_planner_data().is_lane_borrowing ||
      data_center_->last_frame()->inside_planner_data().is_prepare_borrowing;
  if (last_frame_borrow_status) {
    LOG_INFO("last_frame is borrow, skip.");
    return true;
  }
  // perception lanes
  const auto& perception_lanes =
      data_center_->environment().perception_lanes_proxy().camera_lanes_line();
  if (perception_lanes.empty()) {
    LOG_INFO("perception_lanes empty, skip.");
    return true;
  }
  // EGO_LEFT/EGO_RIGHT bev positions
  std::size_t ego_lane_count{0};
  for (const auto& lane : perception_lanes) {
    if (lane.pos_type() == CameraLaneLinePositionType::EGO_LEFT ||
        lane.pos_type() == CameraLaneLinePositionType::EGO_RIGHT) {
      ego_lane_count++;
    }
  }
  if (ego_lane_count < 1) {
    LOG_INFO("ego_lane_count: {}, skip.", ego_lane_count);
    return true;
  }
  // junction check
  auto camera_lane_config = config::PlanningConfig::Instance()
                                ->planning_research_config()
                                .camera_lane_config;
  const auto& reference_line =
      data_center_->task_info_list().front().reference_line();
  const auto& last_init_s =
      data_center_->last_frame()->inside_planner_data().init_sl_point.s();
  std::size_t start_index{0}, end_index{0};
  if (!reference_line->GetStartEndIndexBySLength(
          last_init_s, camera_lane_config.junction_pre_dis, &start_index,
          &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return true;
  }
  Segment2d cut_ref_seg({reference_line->ref_points()[start_index].s(), 0.},
                        {reference_line->ref_points()[end_index].s(), 0.});
  for (const auto& [junction_ptr, overlap] : reference_line->junctions()) {
    if (junction_ptr == nullptr) continue;
    if (junction_ptr->Type() != static_cast<uint32_t>(JunctionType::IN_ROAD)) {
      Segment2d junction_seg({overlap.start_s, 0.}, {overlap.end_s, 0.});
      if (cut_ref_seg.has_intersect(junction_seg)) {
        LOG_INFO("front has junction and not is IN_ROAD type.");
        return true;
      }
    }
  }
  // clear data
  left_lane_points_.clear();
  right_lane_points_.clear();
  last_reference_points_.clear();
  // perception lanes from imu to ego
  for (const auto& lane : perception_lanes) {
    if (lane.pos_type() == CameraLaneLinePositionType::EGO_LEFT) {
      for (const auto& pt : lane.bev_image_point_set()) {
        if (pt.y() < 0.0) continue;
        if (pt.y() > camera_lane_config.check_heading_front_use_length) break;
        if (!left_lane_points_.empty()) {
          if (pt.y() > left_lane_points_.back().x() + kMathEpsilon) {
            left_lane_points_.push_back({pt.y(), pt.x()});
          }
        } else {
          left_lane_points_.push_back({pt.y(), pt.x()});
        }
        LOG_INFO("bev left->ego: {:.3f}, {:.3f}", left_lane_points_.back().x(),
                 left_lane_points_.back().y());
      }
    }
    if (lane.pos_type() == CameraLaneLinePositionType::EGO_RIGHT) {
      for (const auto& pt : lane.bev_image_point_set()) {
        if (pt.y() < 0.0) continue;
        if (pt.y() > camera_lane_config.check_heading_front_use_length) break;
        if (!right_lane_points_.empty()) {
          if (pt.y() > right_lane_points_.back().x() + kMathEpsilon) {
            right_lane_points_.push_back({pt.y(), pt.x()});
          }
        } else {
          right_lane_points_.push_back({pt.y(), pt.x()});
        }
        LOG_INFO("bev right->ego: {:.3f}, {:.3f}",
                 right_lane_points_.back().x(), right_lane_points_.back().y());
      }
    }
  }
  // 3. last utm reference_line trans to ego
  const auto& pts =
      data_center_->task_info_list().front().reference_line_raw()->ref_points();
  const auto utm_pos = data_center_->vehicle_state_utm();
  double t_x{0.}, t_y{0.}, t_h{0.};
  for (const auto& p : pts) {
    if (p.s() < last_init_s) continue;
    if (p.s() > last_init_s + camera_lane_config.check_heading_front_use_length)
      break;
    earth2vehicle(utm_pos.X(), utm_pos.Y(), utm_pos.Heading(), p.x(), p.y(),
                  p.heading(), t_x, t_y, t_h);
    last_reference_points_.push_back({t_x, t_y});
    LOG_INFO("last_ref from utm to ego <x, y>: {:.3f}, {:.3f}",
             last_reference_points_.back().x(),
             last_reference_points_.back().y());
  }
  if (last_reference_points_.size() < 10) {
    LOG_INFO("last_reference_points size < 10, skip.");
    return true;
  }
  // 4. collision check with segment
  bool left_lane_valid = left_lane_points_.size() >= 4;
  tk::spline left_lane_spline{};
  if (left_lane_valid) {
    std::vector<double> left_lane_x_vec{}, left_lane_y_vec{};
    for (const auto& p : left_lane_points_) {
      left_lane_x_vec.push_back(p.x());
      left_lane_y_vec.push_back(p.y());
    }
    left_lane_spline.set_points(left_lane_x_vec, left_lane_y_vec,
                                tk::spline::spline_type::linear);
  }
  bool right_lane_valid = right_lane_points_.size() >= 4;
  tk::spline right_lane_spline{};
  if (right_lane_valid) {
    std::vector<double> right_lane_x_vec{}, right_lane_y_vec{};
    for (const auto& p : right_lane_points_) {
      right_lane_x_vec.push_back(p.x());
      right_lane_y_vec.push_back(p.y());
    }
    right_lane_spline.set_points(right_lane_x_vec, right_lane_y_vec,
                                 tk::spline::spline_type::linear);
  }

  auto arcos_degree = [](const double a, const double b, const double c) {
    return 180.0 / M_PI * std::acos((b * b + c * c - a * a) / (2 * b * c));
  };
  double left_delta_degree{0.}, right_delta_degree{0.};
  for (std::size_t index = 0; index + 10 < last_reference_points_.size();
       index += 10) {
    Segment2d ref_segment(last_reference_points_[index],
                          last_reference_points_[index + 10]);
    LOG_INFO("ref_segment: start[{:.3f}, {:.3f}], end[{:.3f}, {:.3f}]",
             ref_segment.start().x(), ref_segment.start().y(),
             ref_segment.end().x(), ref_segment.end().y());
    if (left_lane_valid) {
      for (double start_s = -camera_lane_config.check_heading_back_use_length,
                  end_s = start_s + 1.0;
           start_s < camera_lane_config.check_heading_front_use_length;
           start_s += 1.0, end_s = start_s + 1.0) {
        Segment2d left_segment(Vec2d(start_s, left_lane_spline(start_s)),
                               Vec2d(end_s, left_lane_spline(end_s)));
        Vec2d intersect_point{};
        if (ref_segment.get_intersect(left_segment, &intersect_point)) {
          LOG_INFO("collide with left_lane_points");
          LOG_INFO("left_segment: start[{:.3f}, {:.3f}], end[{:.3f}, {:.3f}]",
                   left_segment.start().x(), left_segment.start().y(),
                   left_segment.end().x(), left_segment.end().y());
          left_delta_degree = arcos_degree(
              (ref_segment.start() - left_segment.start()).length(),
              (ref_segment.start() - intersect_point).length(),
              (left_segment.start() - intersect_point).length());
          break;
        }
      }
    }
    if (right_lane_valid) {
      for (double start_s = -camera_lane_config.check_heading_back_use_length,
                  end_s = start_s + 1.0;
           start_s < camera_lane_config.check_heading_front_use_length;
           start_s += 1.0, end_s = start_s + 1.0) {
        Segment2d right_segment(Vec2d(start_s, right_lane_spline(start_s)),
                                Vec2d(end_s, right_lane_spline(end_s)));
        Vec2d intersect_point{};
        if (ref_segment.get_intersect(right_segment, &intersect_point)) {
          LOG_INFO("collide with right_lane_points");
          LOG_INFO("right_segment: start[{:.3f}, {:.3f}], end[{:.3f}, {:.3f}]",
                   right_segment.start().x(), right_segment.start().y(),
                   right_segment.end().x(), right_segment.end().y());
          right_delta_degree = arcos_degree(
              (ref_segment.start() - right_segment.start()).length(),
              (ref_segment.start() - intersect_point).length(),
              (right_segment.start() - intersect_point).length());
          break;
        }
      }
    }
  }
  bool utm_update =
      (left_delta_degree <=
       camera_lane_config.check_heading_delta_degree + kMathEpsilon) &&
      (right_delta_degree <=
       camera_lane_config.check_heading_delta_degree + kMathEpsilon);
  LOG_INFO(
      "utm_update[{}], left_delta_degree, right_delta_degree: {:.3f}, {:.3f}",
      utm_update, left_delta_degree, right_delta_degree);

  return utm_update;
}

}  // namespace planning
}  // namespace neodrive
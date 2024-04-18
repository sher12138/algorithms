#include "parking_spot_decider.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

void ParkingSpotDecider::SetParkingDataInfo(
    ParkingDataInfo *const parking_data_info,
    ParkingConfig *const parking_config) {
  parking_data_info_ = parking_data_info;
  p_park_config_ = parking_config;
}

ParkingDataInfo *ParkingSpotDecider::GetParkingDataInfo() {
  return parking_data_info_;
}

bool ParkingSpotDecider::SetParkingReferenceLine(
    ReferenceLinePtr const refer_line) {
  parking_data_info_->SetParkingRefLine(refer_line);
  return true;
}

bool ParkingSpotDecider::Process(ReferenceLinePtr const refer_line,
                                 std::array<Vec2d, 4> &vertices) {
  if (parking_data_info_ == nullptr || p_park_config_ == nullptr) {
    LOG_ERROR("parking_data_info_ == nullptr || p_park_config_ == nullptr");
    return false;
  }
  if (refer_line == nullptr) {
    LOG_ERROR("parking_refer_line is null");
    return false;
  }
  if (refer_line->ref_points().empty() || vertices.empty()) {
    LOG_ERROR(
        "parking_refer_line is invalid [{}] or parking spot is invalid [{}]",
        refer_line->ref_points().size(), vertices.size());
    return false;
  }
  if (!SetParkingReferenceLine(refer_line)) {
    LOG_ERROR("SetParkingReferenceLine failed");
    return false;
  }

  // next will change vertices sequence
  std::array<Vec2d, 4> vertices_updated(vertices);
  if (!CalculateAnchorPoint(vertices, vertices_updated)) {
    LOG_ERROR("CalculateAnchorPoint failed");
    return false;
  }

  if (!SetParkingSpotType(vertices_updated)) {
    LOG_ERROR("set_parking_spot failed");
    return false;
  }
  if (!ParkingSpotValid(vertices_updated)) {
    LOG_ERROR("ParkingSpotValid failed");
    return false;
  }
  if (!SetParkingSpotEndPose(vertices_updated)) {
    LOG_ERROR("SetParkingSpotEndPose failed");
    return false;
  }

  // update refer_line bound
  if (!ReferLineReorganize(refer_line, vertices)) {
    LOG_ERROR("ReferLineReorganize failed");
    return false;
  }

  // vector of different obstacle consisting of vertice points.The
  // obstacle and the vertices order are in counter-clockwise order
  std::vector<std::vector<Vec2d>> roi_boundary;
  LOG_INFO("vertices size : {}, refer_line size : {}", vertices.size(),
           refer_line->ref_points().size());
  if (!GetParkingBoundary(vertices, refer_line, &roi_boundary)) {
    LOG_ERROR("failed to create boundary");
    return false;
  }

  if (!FormulateBoundaryConstraints(roi_boundary)) {
    LOG_ERROR("failed to FormulateBoundaryConstraints");
    return false;
  }

  return true;
}

// 1. input vertices: orgin corners order in earth coordinate;
// 2. ouput vertices: anchor corners order in earth coordinate, order is
//    left_top, left_down, right_down, right top; spot_corners is same;
// 3. ouput new_vertices: spot anchor corners order in anchor coordinate;
bool ParkingSpotDecider::CalculateAnchorPoint(
    std::array<Vec2d, 4> &vertices, std::array<Vec2d, 4> &new_vertices) {
  if (vertices.size() != 4 || new_vertices.size() != 4) {
    LOG_ERROR("vertices.size()[{}]!=4", vertices.size());
    return false;
  }
  // let's check the carport is left or right
  std::array<Vec2d, 4> vertices_bak(vertices);
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  double vel_x = 0.0, vel_y = 0.0, vel_theta = 0.0;
  vel_x = parking_data_info_->CurrGps().x();
  vel_y = parking_data_info_->CurrGps().y();
  vel_theta = parking_data_info_->CurrHeading();
  for (size_t i = 0; i < vertices_bak.size(); ++i) {
    earth2vehicle(vel_x, vel_y, vel_theta, vertices_bak[i].x(),
                  vertices_bak[i].y(), 0.0, tmp_x, tmp_y, tmp_theta);
    vertices_bak[i].set_x(tmp_x);
    vertices_bak[i].set_y(tmp_y);
  }
  // find index 0 left_top, left_down
  size_t min_x_tmp = 0;
  size_t min_x_2nd_tmp = 0;
  double min_x = vertices_bak[0].x();
  for (size_t i = 0; i < vertices_bak.size(); ++i) {
    if (min_x > vertices_bak[i].x()) {
      min_x = vertices_bak[i].x();
      min_x_tmp = i;
    }
  }
  if (min_x_tmp == 0) {
    min_x_2nd_tmp = vertices_bak[1].x() <= vertices_bak[3].x() ? 1 : 3;
  } else if (min_x_tmp == 3) {
    min_x_2nd_tmp = vertices_bak[0].x() <= vertices_bak[2].x() ? 0 : 2;
  } else {
    min_x_2nd_tmp =
        vertices_bak[min_x_tmp - 1].x() <= vertices_bak[min_x_tmp + 1].x()
            ? min_x_tmp - 1
            : min_x_tmp + 1;
  }
  int left_right_type = 0;
  // check both y is on one side
  if (vertices_bak[min_x_tmp].y() > 0.0 &&
      vertices_bak[min_x_2nd_tmp].y() > 0.0) {
    left_right_type = 1;
  } else if (vertices_bak[min_x_tmp].y() < 0.0 &&
             vertices_bak[min_x_2nd_tmp].y() < 0.0) {
    left_right_type = 2;
  } else {
    LOG_WARN("currently do not support this left_right type, y1 {:.4f}, y2: {:.4f}",
             vertices_bak[min_x_tmp].y(), vertices_bak[min_x_2nd_tmp].y());
    return false;
  }
  parking_data_info_->SetLeftRightType(left_right_type);

  // calculate each point index
  size_t index_left_top = 0, index_left_down = 0;
  size_t index_right_top = 0, index_right_down = 0;
  std::vector<size_t> index_vec{0, 1, 2, 3};

  index_vec.erase(
      std::remove(std::begin(index_vec), std::end(index_vec), min_x_tmp),
      std::end(index_vec));
  index_vec.erase(
      std::remove(std::begin(index_vec), std::end(index_vec), min_x_2nd_tmp),
      std::end(index_vec));
  if (index_vec.size() != 2) {
    LOG_ERROR("index vector calculated wrong");
    return false;
  }
  if (left_right_type == 1) {
    // carport is left of vehicle
    /*
     *  * **********************************************
     *
     *                               <-------
     *
     *  ****************A          ******************
     *                  *        *
     *                  *        *
     *                  **********
     */
    index_right_top = min_x_tmp;
    index_right_down = min_x_2nd_tmp;

    if (vertices_bak[index_vec.front()].y() <
        vertices_bak[index_vec.back()].y()) {
      index_left_top = index_vec.front();
      index_left_down = index_vec.back();
    } else {
      index_left_top = index_vec.back();
      index_left_down = index_vec.front();
    }
    if (vertices_bak[min_x_tmp].y() < vertices_bak[min_x_2nd_tmp].y()) {
      index_right_top = min_x_tmp;
      index_right_down = min_x_2nd_tmp;
    } else {
      index_right_top = min_x_2nd_tmp;
      index_right_down = min_x_tmp;
    }
  } else if (left_right_type == 2) {
    // carport is right of vehicle
    /*
     * **********************************************
     *
     *    ---->
     *
     *  ****************A          ******************
     *                  *        *
     *                  *        *
     *                  **********
     * */
    index_left_top = min_x_tmp;
    index_left_down = min_x_2nd_tmp;

    if (vertices_bak[index_vec.front()].y() <
        vertices_bak[index_vec.back()].y()) {
      index_right_top = index_vec.back();
      index_right_down = index_vec.front();
    } else {
      index_right_top = index_vec.front();
      index_right_down = index_vec.back();
    }
    if (vertices_bak[min_x_tmp].y() < vertices_bak[min_x_2nd_tmp].y()) {
      index_left_top = min_x_2nd_tmp;
      index_left_down = min_x_tmp;
    }
  } else {
    LOG_ERROR("left_right type is not defined");
    return false;
  }
  // already get the index of left_top, right_top
  // let's set the anchor point
  ReferencePoint anchor_pt;
  anchor_pt.set_x(vertices[index_left_top].x());
  anchor_pt.set_y(vertices[index_left_top].y());
  Vec2d heading_vec;
  heading_vec = vertices[index_right_top] - vertices[index_left_top];
  double heading = heading_vec.angle();
  anchor_pt.set_heading(heading);
  parking_data_info_->SetAnchorPointCoordinate(anchor_pt);

  // next, we need to transform the CurrGps, vertices to anchor coordinate
  tmp_x = tmp_y = tmp_theta = 0.0;
  vel_x = anchor_pt.x();
  vel_y = anchor_pt.y();
  vel_theta = anchor_pt.heading();
  // enum method
  std::vector<size_t> carport_list{index_left_top, index_left_down,
                                   index_right_down, index_right_top};
  LOG_INFO("carport left_top: {}, left_down: {}, right_down: {}, right_top: {}",
           index_left_top, index_left_down, index_right_down, index_right_top);
  for (size_t i = 0; i < carport_list.size(); ++i) {
    size_t k = carport_list[i];
    earth2vehicle(vel_x, vel_y, vel_theta, vertices[k].x(), vertices[k].y(),
                  0.0, tmp_x, tmp_y, tmp_theta);
    new_vertices[i].set_x(tmp_x);
    new_vertices[i].set_y(tmp_y);
  }
  // CurrGps
  earth2vehicle(vel_x, vel_y, vel_theta, parking_data_info_->CurrGps().x(),
                parking_data_info_->CurrGps().y(),
                parking_data_info_->CurrHeading(), tmp_x, tmp_y, tmp_theta);
  TrajectoryPoint curr_gps_pos;
  curr_gps_pos.set_x(tmp_x);
  curr_gps_pos.set_y(tmp_y);
  curr_gps_pos.set_theta(tmp_theta);
  parking_data_info_->SetCurrPosOnAnchor(curr_gps_pos);

  // pay attention: changed vertices here
  std::array<Vec2d, 4> vertices_reorginized(vertices);
  std::vector<Vec2d> spot_corners;
  for (size_t i = 0; i < carport_list.size(); ++i) {
    vertices[i] = vertices_reorginized[carport_list[i]];
    spot_corners.push_back(vertices_reorginized[carport_list[i]]);
  }
  if (spot_corners.size() != 4) {
    LOG_ERROR("spot_corners.size()!=4");
    return false;
  }
  parking_data_info_->SetSpotVertices(spot_corners);

  return true;
}

bool ParkingSpotDecider::SetParkingSpotType(
    const std::array<Vec2d, 4> &vertices) {
  if (vertices.size() != 4) {
    LOG_ERROR("vertices.size({}) != 4", vertices.size());
    return false;
  }
  auto left_top = vertices[0];
  auto right_top = vertices[3];
  // rotate the points to have the lane to be horizontal to x axis positive
  // direction and scale them base on the origin point
  Vec2d heading_vec = right_top - left_top;
  parking_data_info_->SetOriginHeading(heading_vec.angle());
  parking_data_info_->SetOriginPoint({left_top.x(), left_top.y()});
  // check parking spot type
  if (parking_data_info_->ParkSpotType() == 1 ||
      parking_data_info_->ParkSpotType() == 2 ||
      parking_data_info_->ParkSpotType() == 3) {
    return true;
  }

  Vec2d origin_point = parking_data_info_->OriginPoint();
  double origin_heading = parking_data_info_->OriginHeading();
  // bak all vertices to origin coor
  left_top_origin_ = vertices[0];
  left_down_origin_ = vertices[1];
  right_down_origin_ = vertices[2];
  right_top_origin_ = vertices[3];
  // End pose is set in normalized boundary
  left_top_origin_ -= origin_point;
  left_top_origin_.self_rotate(-origin_heading);
  left_down_origin_ -= origin_point;
  left_down_origin_.self_rotate(-origin_heading);
  right_top_origin_ -= origin_point;
  right_top_origin_.self_rotate(-origin_heading);
  right_down_origin_ -= origin_point;
  right_down_origin_.self_rotate(-origin_heading);
  double lon_dis = left_top_origin_.y() - left_down_origin_.y() +
                   right_top_origin_.y() - right_down_origin_.y();
  lon_dis *= 0.5;
  double lat_dis = right_top_origin_.x() - left_top_origin_.x() +
                   right_down_origin_.x() - left_down_origin_.x();
  lat_dis *= 0.5;
  // need to insert rectangle inside and check the type of spot
  if (lat_dis >= lon_dis + 1.0) {
    // parallel
    parking_data_info_->SetParkSpotType(2);
    LOG_INFO("carport is paralle, lat_dis is : {:.4f}, lon_dis is : {:.4f}",
             lat_dis, lon_dis);
    return true;
  }
  // perpendicular or inclined
  Vec2d heading_cal_vec;
  heading_cal_vec = left_top_origin_ - left_down_origin_;
  double angle_1 = heading_cal_vec.angle();
  heading_cal_vec = right_top_origin_ - right_down_origin_;
  double angle_2 = heading_cal_vec.angle();
  angle_1 = normalize_angle(angle_1);
  angle_2 = normalize_angle(angle_2);
  // trick
  angle_1 = std::abs(std::abs(angle_1) - M_PI_2);
  angle_2 = std::abs(std::abs(angle_2) - M_PI_2);

  double threshold = M_PI * 20.0 / 180.0;

  if (angle_1 > threshold && angle_2 > threshold) {
    parking_data_info_->SetParkSpotType(3);
    LOG_INFO("carport is inclinded, angle1: {:.4f}, angle2: {:.4f}", angle_1,
             angle_2);
  } else {
    parking_data_info_->SetParkSpotType(1);
    LOG_INFO("carport is perpendicular");
  }
  return true;
}

// true: valid
bool ParkingSpotDecider::ParkingSpotValid(
    const std::array<Vec2d, 4> &vertices) {
  if (!(parking_data_info_->ParkSpotType() == 1 ||
        parking_data_info_->ParkSpotType() == 2 ||
        parking_data_info_->ParkSpotType() == 3)) {
    LOG_ERROR("park spot type is not defined");
    return false;
  }

  // mchan fix this bug ?
  double lat_dis = left_top_origin_.y() - left_down_origin_.y() +
                   right_top_origin_.y() - right_down_origin_.y();
  lat_dis *= 0.5;
  double lon_dis = right_top_origin_.x() - left_top_origin_.x() +
                   right_down_origin_.x() - left_down_origin_.x();
  lon_dis *= 0.5;

  // upper is not correct
  // check valid spot?
  if (parking_data_info_->ParkSpotType() == 1 &&
      lat_dis < VehicleParam::Instance()->width() + 0.5) {
    // perpendicular
    LOG_ERROR("park spot width is too narrow: {:.4f}", lat_dis);
    return false;
  } else if (parking_data_info_->ParkSpotType() == 2 &&
             lon_dis < VehicleParam::Instance()->length() + 0.5) {
    // parallel
    LOG_ERROR("park spot length is too narrow: {:.4f}", lon_dis);
    return false;
  } else if (parking_data_info_->ParkSpotType() == 3 &&
             lat_dis < VehicleParam::Instance()->width() + 0.5) {
    // inclined
    LOG_ERROR("park spot width is too narrow: {:.4f}", lat_dis);
    return false;
  }

  return true;
}

bool ParkingSpotDecider::SetParkingSpotEndPose(
    const std::array<Vec2d, 4> &vertices) {
  if (parking_data_info_->FinalPosFlag()) {
    LOG_INFO("use pre defined final pos");
    TrajectoryPoint pre_pos = parking_data_info_->PreDefinedFinalPos();
    // transform to anchor
    double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
    double vel_x = 0.0, vel_y = 0.0, vel_theta = 0.0;
    vel_x = parking_data_info_->AnchorPointCoordinate().x();
    vel_y = parking_data_info_->AnchorPointCoordinate().y();
    vel_theta = parking_data_info_->AnchorPointCoordinate().heading();

    earth2vehicle(vel_x, vel_y, vel_theta, pre_pos.x(), pre_pos.y(),
                  pre_pos.theta(), tmp_x, tmp_y, tmp_theta);
    tmp_theta = normalize_angle(tmp_theta);
    std::vector<double> parking_end_pose{tmp_x, tmp_y, tmp_theta, 0.0};
    parking_data_info_->SetParkingEndPose(parking_end_pose);
    TrajectoryPoint final_pos;
    final_pos.set_x(parking_end_pose[0]);
    final_pos.set_y(parking_end_pose[1]);
    final_pos.set_theta(parking_end_pose[2]);
    parking_data_info_->SetFinalPosOnAnchor(final_pos);
    return true;
  }

  std::array<Vec2d, 4> vertices_bak(vertices);

  // perpendicular parking spot
  if (parking_data_info_->ParkSpotType() == 1) {
    SetPerpenSpotEndPose(vertices_bak, false);
  } else if (parking_data_info_->ParkSpotType() == 2) {
    SetParallelSpotEndPose(vertices_bak);
  } else if (parking_data_info_->ParkSpotType() == 3) {
    SetInclinedSpotEndPose(vertices_bak, false);
  } else {
    LOG_ERROR("ParkSpotType is not defined");
    return false;
  }

  return true;
}

bool ParkingSpotDecider::SetPerpenSpotEndPose(
    const std::array<Vec2d, 4> &vertices, const bool &head_tail_in) {
  // perpendicular parking spot
  double parking_spot_heading = (left_down_origin_ - left_top_origin_).angle();
  double end_x = (left_top_origin_.x() + right_top_origin_.x()) / 2.0;
  double end_y = 0.0;

  const double parking_depth_buffer =
      p_park_config_->common_config().park_spot_depth_buffer_;

  const double top_to_down_distance =
      left_top_origin_.y() - left_down_origin_.y();
  if (parking_spot_heading > kMathEpsilon) {
    if (head_tail_in) {
      end_y = left_down_origin_.y() -
              (std::max(3.0 * -top_to_down_distance / 4.0,
                        VehicleParam::Instance()->front_edge_to_center()) +
               parking_depth_buffer);
    } else {
      end_y = left_down_origin_.y() -
              (std::max(-top_to_down_distance / 4.0,
                        VehicleParam::Instance()->back_edge_to_center()) +
               parking_depth_buffer);
    }
  } else {
    if (head_tail_in) {
      end_y = left_down_origin_.y() +
              (std::max(3.0 * top_to_down_distance / 4.0,
                        VehicleParam::Instance()->front_edge_to_center()) +
               parking_depth_buffer);
    } else {
      end_y = left_down_origin_.y() +
              (std::max(top_to_down_distance / 4.0,
                        VehicleParam::Instance()->back_edge_to_center()) +
               parking_depth_buffer);
    }
  }
  std::vector<double> parking_end_pose{end_x, end_y, parking_spot_heading, 0.0};
  if (!head_tail_in) {
    parking_end_pose[2] = normalize_angle(parking_spot_heading + M_PI);
  }
  parking_data_info_->SetParkingEndPose(parking_end_pose);

  TrajectoryPoint final_pos;
  final_pos.set_x(parking_end_pose[0]);
  final_pos.set_y(parking_end_pose[1]);
  final_pos.set_theta(parking_end_pose[2]);
  parking_data_info_->SetFinalPosOnAnchor(final_pos);

  return true;
}

bool ParkingSpotDecider::SetParallelSpotEndPose(
    const std::array<Vec2d, 4> &vertices) {
  // parallel spot
  double parking_spot_heading = (right_top_origin_ - left_top_origin_).angle();
  double end_x = 0.0;
  double end_y = (left_top_origin_.y() + left_down_origin_.y()) * 0.5;

  const double parking_depth_buffer =
      p_park_config_->common_config().park_spot_depth_buffer_;

  const double front_to_back_distance =
      right_top_origin_.x() - left_top_origin_.x();

  end_x = left_top_origin_.x() +
          (std::max(1.0 * front_to_back_distance / 4.0,
                    VehicleParam::Instance()->back_edge_to_center()) +
           parking_depth_buffer);
  std::vector<double> parking_end_pose{end_x, end_y, parking_spot_heading, 0.0};
  parking_data_info_->SetParkingEndPose(parking_end_pose);
  TrajectoryPoint final_pos;
  final_pos.set_x(parking_end_pose[0]);
  final_pos.set_y(parking_end_pose[1]);
  final_pos.set_theta(parking_end_pose[2]);
  parking_data_info_->SetFinalPosOnAnchor(final_pos);

  return true;
}

bool ParkingSpotDecider::SetInclinedSpotEndPose(
    const std::array<Vec2d, 4> &vertices, const bool &head_tail_in) {
  if (vertices.size() != 4) {
    LOG_ERROR("vertices.size() != 4");
    return false;
  }
  // transform to perpen
  auto left_top = left_top_origin_;
  auto right_top = right_top_origin_;
  auto left_down = left_down_origin_;
  auto right_down = right_down_origin_;

  Vec2d heading_vec = left_down - left_top;
  Vec2d heading_vec2 = right_down - right_top;

  double origin_heading = heading_vec.angle();
  double origin_heading2 = heading_vec2.angle();
  origin_heading = normalize_angle(origin_heading);
  origin_heading2 = normalize_angle(origin_heading2);
  origin_heading = (origin_heading + origin_heading2) * 0.5;
  origin_heading = normalize_angle(origin_heading);
  double inclined_heading = origin_heading;

  origin_heading += M_PI_2;
  origin_heading = normalize_angle(origin_heading);
  auto origin_pt = left_top;
  left_top -= origin_pt;
  left_top.self_rotate(-origin_heading);
  right_top -= origin_pt;
  right_top.self_rotate(-origin_heading);
  left_down -= origin_pt;
  left_down.self_rotate(-origin_heading);
  right_down -= origin_pt;
  right_down.self_rotate(-origin_heading);
  // sanity check
  if (std::abs(left_down.y()) < 0.01 || std::abs(right_down.y()) < 0.01) {
    LOG_ERROR(
        "inclined down pt are close to 0, left_down: {:.4f}, right_down: {:.4f}",
        left_down.y(), right_down.y());
    return false;
  }
  if (left_down.y() * right_down.y() < 0.0) {
    LOG_ERROR(
        "inclined down pt are oppisite, left_down: {:.4f}, right_down: {:.4f}",
        left_down.y(), right_down.y());
    return false;
  }
  // final pos
  double end_x = 0.0;
  end_x = left_top.x() + std::min(right_top.x(), right_down.x());
  end_x *= 0.5;
  double end_y = 0.0;

  const double parking_depth_buffer =
      p_park_config_->common_config().park_spot_depth_buffer_;

  double top_to_down_distance = 0.0;
  double min_down_y = std::max(left_down.y(), right_down.y());
  top_to_down_distance = left_top.y() - min_down_y;
  if (inclined_heading > kMathEpsilon) {
    if (head_tail_in) {
      end_y = min_down_y -
              (std::max(3.0 * -top_to_down_distance / 4.0,
                        VehicleParam::Instance()->front_edge_to_center()) +
               parking_depth_buffer);
    } else {
      end_y = min_down_y -
              (std::max(-top_to_down_distance / 4.0,
                        VehicleParam::Instance()->back_edge_to_center()) +
               parking_depth_buffer);
    }
  } else {
    if (head_tail_in) {
      end_y = min_down_y +
              (std::max(3.0 * top_to_down_distance / 4.0,
                        VehicleParam::Instance()->front_edge_to_center()) +
               parking_depth_buffer);
    } else {
      end_y = min_down_y +
              (std::max(top_to_down_distance / 4.0,
                        VehicleParam::Instance()->back_edge_to_center()) +
               parking_depth_buffer);
    }
  }
  // transform back
  Vec2d final_pt(end_x, end_y);

  final_pt.self_rotate(origin_heading);
  final_pt += origin_pt;
  std::vector<double> parking_end_pose{final_pt.x(), final_pt.y(),
                                       inclined_heading, 0.0};
  if (!head_tail_in) {
    parking_end_pose[2] = normalize_angle(inclined_heading + M_PI);
  }
  parking_data_info_->SetParkingEndPose(parking_end_pose);
  TrajectoryPoint final_pos;
  final_pos.set_x(parking_end_pose[0]);
  final_pos.set_y(parking_end_pose[1]);
  final_pos.set_theta(parking_end_pose[2]);
  parking_data_info_->SetFinalPosOnAnchor(final_pos);

  return true;
}

bool ParkingSpotDecider::GetParkingBoundary(
    const std::array<Vec2d, 4> &vertices, const ReferenceLinePtr refer_line,
    std::vector<std::vector<Vec2d>> *const roi_parking_boundary) {
  auto [left_top, left_down, right_down, right_top] = vertices;

  double left_top_s = 0.0, left_top_l = 0.0;
  double right_top_s = 0.0, right_top_l = 0.0;
  // get projection
  // we should check left_top and right_top whether can project on refer_line
  SLPoint left_top_sl;
  if (!refer_line->GetPointInFrenetFrame(left_top, &left_top_sl)) {
    LOG_ERROR("left_top_sl porejection failed");
    return false;
  }
  SLPoint right_top_sl;
  if (!refer_line->GetPointInFrenetFrame(right_top, &right_top_sl)) {
    LOG_ERROR("right_top_sl porejection failed");
    return false;
  }
  left_top_s = left_top_sl.s();
  left_top_l = left_top_sl.l();
  right_top_s = right_top_sl.s();
  right_top_l = right_top_sl.l();

  // it's different with left_top_origin_
  Vec2d origin_point;
  auto anchor_pt = parking_data_info_->AnchorPointCoordinate();
  origin_point.set_x(anchor_pt.x());
  origin_point.set_y(anchor_pt.y());
  double origin_heading = anchor_pt.heading();

  // rotate
  left_top -= origin_point;
  left_top.self_rotate(-origin_heading);
  left_down -= origin_point;
  left_down.self_rotate(-origin_heading);
  right_top -= origin_point;
  right_top.self_rotate(-origin_heading);
  right_down -= origin_point;
  right_down.self_rotate(-origin_heading);
  const double center_line_s = (left_top_s + right_top_s) / 2.0;

  std::vector<Vec2d> left_lane_boundary;
  std::vector<Vec2d> right_lane_boundary;
  // The pivot points on the central lane, mapping with the key points on
  // the left lane boundary.
  std::vector<Vec2d> center_lane_boundary_left;
  // The pivot points on the central lane, mapping with the key points on
  // the right lane boundary.
  std::vector<Vec2d> center_lane_boundary_right;
  // The s-value for the anchor points on the center_lane_boundary_left.
  std::vector<double> center_lane_s_left;
  // The s-value for the anchor points on the center_lane_boundary_right.
  std::vector<double> center_lane_s_right;
  // The left-half road width between the pivot points on the
  // center_lane_boundary_left and key points on the
  // left_lane_boundary.
  std::vector<double> left_lane_road_width;
  // The right-half road width between the pivot points on the
  // center_lane_boundary_right and key points on the
  // right_lane_boundary.
  std::vector<double> right_lane_road_width;

  GetRoadBoundary(refer_line, center_line_s, origin_point, origin_heading,
                  &left_lane_boundary, &right_lane_boundary,
                  &center_lane_boundary_left, &center_lane_boundary_right,
                  &center_lane_s_left, &center_lane_s_right,
                  &left_lane_road_width, &right_lane_road_width);
  // If smaller than zero, the parking spot is on the right of the lane
  // Left, right, down or opposite of the boundary is decided when viewing the
  // parking spot upward
  const double average_l = (left_top_l + right_top_l) / 2.0;
  std::vector<Vec2d> boundary_points;

  // TODO, Write a half-boundary formation function and call it twice
  // to avoid duplicated manipulations on the left and right sides
  if (average_l < 0) {
    // if average_l is lower than zero, the parking spot is on the right
    // lane boundary and assume that the lane half width is average_l
    LOG_INFO("average_l is less than 0 in OpenSpaceROI");
    size_t point_size = right_lane_boundary.size();
    for (size_t i = 0; i < point_size; ++i) {
      right_lane_boundary[i].self_rotate(origin_heading);
      right_lane_boundary[i] += origin_point;
      right_lane_boundary[i] -= center_lane_boundary_right[i];
      right_lane_boundary[i] /= right_lane_road_width[i];
      right_lane_boundary[i] *= (-average_l);
      right_lane_boundary[i] += center_lane_boundary_right[i];
      right_lane_boundary[i] -= origin_point;
      right_lane_boundary[i].self_rotate(-origin_heading);
    }

    LOG_INFO("right lane boundary begin pt : x {:.4f}, y {:.4f}",
             right_lane_boundary[0].x(), right_lane_boundary[0].y());
    auto point_left_to_left_top_connor_s = std::lower_bound(
        center_lane_s_right.begin(), center_lane_s_right.end(), left_top_s);
    size_t point_left_to_left_top_connor_index = std::distance(
        center_lane_s_right.begin(), point_left_to_left_top_connor_s);
    point_left_to_left_top_connor_index =
        point_left_to_left_top_connor_index == 0
            ? point_left_to_left_top_connor_index
            : point_left_to_left_top_connor_index - 1;
    auto point_left_to_left_top_connor_itr =
        right_lane_boundary.begin() + point_left_to_left_top_connor_index;
    auto point_right_to_right_top_connor_s = std::upper_bound(
        center_lane_s_right.begin(), center_lane_s_right.end(), right_top_s);
    size_t point_right_to_right_top_connor_index = std::distance(
        center_lane_s_right.begin(), point_right_to_right_top_connor_s);
    auto point_right_to_right_top_connor_itr =
        right_lane_boundary.begin() + point_right_to_right_top_connor_index;

    std::copy(right_lane_boundary.begin(), point_left_to_left_top_connor_itr,
              std::back_inserter(boundary_points));

    std::vector<Vec2d> parking_spot_boundary{left_top, left_down, right_down,
                                             right_top};

    std::copy(parking_spot_boundary.begin(), parking_spot_boundary.end(),
              std::back_inserter(boundary_points));

    std::copy(point_right_to_right_top_connor_itr, right_lane_boundary.end(),
              std::back_inserter(boundary_points));

    std::reverse_copy(left_lane_boundary.begin(), left_lane_boundary.end(),
                      std::back_inserter(boundary_points));

    // reinsert the initial point to the back to from closed loop
    boundary_points.push_back(right_lane_boundary.front());

    // disassemble line into line2d segments
    for (size_t i = 0; i < point_left_to_left_top_connor_index; ++i) {
      std::vector<Vec2d> segment{right_lane_boundary[i],
                                 right_lane_boundary[i + 1]};
      roi_parking_boundary->push_back(segment);
    }

    std::vector<Vec2d> left_stitching_segment{
        right_lane_boundary[point_left_to_left_top_connor_index], left_top};
    roi_parking_boundary->push_back(left_stitching_segment);

    std::vector<Vec2d> left_parking_spot_segment{left_top, left_down};
    std::vector<Vec2d> down_parking_spot_segment{left_down, right_down};
    std::vector<Vec2d> right_parking_spot_segment{right_down, right_top};

    roi_parking_boundary->emplace_back(left_parking_spot_segment);
    roi_parking_boundary->emplace_back(down_parking_spot_segment);
    roi_parking_boundary->emplace_back(right_parking_spot_segment);

    std::vector<Vec2d> right_stitching_segment{
        right_top, right_lane_boundary[point_right_to_right_top_connor_index]};
    roi_parking_boundary->emplace_back(right_stitching_segment);

    size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
    for (size_t i = point_right_to_right_top_connor_index;
         i < right_lane_boundary_last_index; ++i) {
      std::vector<Vec2d> segment{right_lane_boundary[i],
                                 right_lane_boundary[i + 1]};
      roi_parking_boundary->emplace_back(segment);
    }

    size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
    for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
      std::vector<Vec2d> segment{left_lane_boundary[i],
                                 left_lane_boundary[i - 1]};
      roi_parking_boundary->emplace_back(segment);
    }

  } else {
    // if average_l is higher than zero, the parking spot is on the left
    // lane boundary and assume that the lane half width is average_l
    LOG_INFO("average_l is greater than 0 in OpenSpaceROI");
    size_t point_size = left_lane_boundary.size();
    for (size_t i = 0; i < point_size; ++i) {
      left_lane_boundary[i].self_rotate(origin_heading);
      left_lane_boundary[i] += origin_point;
      left_lane_boundary[i] -= center_lane_boundary_left[i];
      left_lane_boundary[i] /= left_lane_road_width[i];
      left_lane_boundary[i] *= average_l;
      left_lane_boundary[i] += center_lane_boundary_left[i];
      left_lane_boundary[i] -= origin_point;
      left_lane_boundary[i].self_rotate(-origin_heading);
      LOG_INFO("left_lane_boundary[{}]: x: {:.4f},y: {:.4f}", i,
               left_lane_boundary[i].x(), left_lane_boundary[i].y());
    }

    auto point_right_to_right_top_connor_s = std::lower_bound(
        center_lane_s_left.begin(), center_lane_s_left.end(), right_top_s);
    size_t point_right_to_right_top_connor_index = std::distance(
        center_lane_s_left.begin(), point_right_to_right_top_connor_s);
    if (point_right_to_right_top_connor_index > 0) {
      --point_right_to_right_top_connor_index;
    }
    auto point_right_to_right_top_connor_itr =
        left_lane_boundary.begin() + point_right_to_right_top_connor_index;

    auto point_left_to_left_top_connor_s = std::upper_bound(
        center_lane_s_left.begin(), center_lane_s_left.end(), left_top_s);
    size_t point_left_to_left_top_connor_index = std::distance(
        center_lane_s_left.begin(), point_left_to_left_top_connor_s);
    auto point_left_to_left_top_connor_itr =
        left_lane_boundary.begin() + point_left_to_left_top_connor_index;

    std::copy(right_lane_boundary.begin(), right_lane_boundary.end(),
              std::back_inserter(boundary_points));

    std::reverse_copy(point_left_to_left_top_connor_itr,
                      left_lane_boundary.end(),
                      std::back_inserter(boundary_points));

    std::vector<Vec2d> parking_spot_boundary{left_top, left_down, right_down,
                                             right_top};
    std::copy(parking_spot_boundary.begin(), parking_spot_boundary.end(),
              std::back_inserter(boundary_points));

    std::reverse_copy(left_lane_boundary.begin(),
                      point_right_to_right_top_connor_itr,
                      std::back_inserter(boundary_points));

    // reinsert the initial point to the back to from closed loop
    boundary_points.push_back(right_lane_boundary.front());

    // disassemble line into line2d segments
    size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
    for (size_t i = 0; i < right_lane_boundary_last_index; ++i) {
      std::vector<Vec2d> segment{right_lane_boundary[i],
                                 right_lane_boundary[i + 1]};
      roi_parking_boundary->emplace_back(segment);
    }

    size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
    for (size_t i = left_lane_boundary_last_index;
         i > point_left_to_left_top_connor_index; i--) {
      std::vector<Vec2d> segment{left_lane_boundary[i],
                                 left_lane_boundary[i - 1]};
      roi_parking_boundary->emplace_back(segment);
    }

    std::vector<Vec2d> left_stitching_segment{
        left_lane_boundary[point_left_to_left_top_connor_index], left_top};
    roi_parking_boundary->emplace_back(left_stitching_segment);

    std::vector<Vec2d> left_parking_spot_segment{left_top, left_down};
    std::vector<Vec2d> down_parking_spot_segment{left_down, right_down};
    std::vector<Vec2d> right_parking_spot_segment{right_down, right_top};
    roi_parking_boundary->emplace_back(left_parking_spot_segment);
    roi_parking_boundary->emplace_back(down_parking_spot_segment);
    roi_parking_boundary->emplace_back(right_parking_spot_segment);

    std::vector<Vec2d> right_stitching_segment{
        right_top, left_lane_boundary[point_right_to_right_top_connor_index]};
    roi_parking_boundary->emplace_back(right_stitching_segment);

    for (size_t i = point_right_to_right_top_connor_index; i > 0; --i) {
      std::vector<Vec2d> segment{left_lane_boundary[i],
                                 left_lane_boundary[i - 1]};
      roi_parking_boundary->emplace_back(segment);
    }
  }

  // Fuse line segments into convex contraints
  if (!FuseLineSegments(roi_parking_boundary)) {
    LOG_ERROR("FuseLineSegments failed in parking ROI");
    return false;
  }

  // Get xy boundary
  auto xminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
  std::vector<double> ROI_xy_boundary{xminmax.first->x(), xminmax.second->x(),
                                      yminmax.first->y(), yminmax.second->y()};

  std::vector<double> ROI_xy_boundary_;
  ROI_xy_boundary_.assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());
  parking_data_info_->SetROIXyBoundary(ROI_xy_boundary_);

  Vec2d vehicle_xy = parking_data_info_->CurrGps();
  vehicle_xy -= origin_point;
  vehicle_xy.self_rotate(-origin_heading);

  LOG_INFO("vehicle_xy : x {:.4f}, y {:.4f}", vehicle_xy.x(), vehicle_xy.y());
  LOG_INFO(
      "ROI_xy_boundary : x_min {:.4f}, x_max {:.4f}, y_min {:.4f}, "
      "y_max {:.4f}",
      ROI_xy_boundary[0], ROI_xy_boundary[1], ROI_xy_boundary[2],
      ROI_xy_boundary[3]);

  Vec2d ref_start, ref_end;
  ref_start.set_x(refer_line->ref_points().front().x());
  ref_start.set_y(refer_line->ref_points().front().y());
  ref_start -= origin_point;
  ref_start.self_rotate(-origin_heading);

  ref_end.set_x(refer_line->ref_points().back().x());
  ref_end.set_y(refer_line->ref_points().back().y());
  ref_end -= origin_point;
  ref_end.self_rotate(-origin_heading);
  LOG_INFO("reference line length : {:.4f}", refer_line->length());

  LogParkingBoundary(boundary_points, vehicle_xy, ref_start, ref_end);

  if (vehicle_xy.x() < ROI_xy_boundary[0] ||
      vehicle_xy.x() > ROI_xy_boundary[1] ||
      vehicle_xy.y() < ROI_xy_boundary[2] ||
      vehicle_xy.y() > ROI_xy_boundary[3]) {
    LOG_ERROR("vehicle outside of xy boundary of parking ROI");
    return false;
  }
  return true;
}

void ParkingSpotDecider::LogParkingBoundary(const std::vector<Vec2d> &round_pts,
                                            const Vec2d &veh_xy,
                                            const Vec2d &ref_start,
                                            const Vec2d &ref_end) {
  FILE *save;
  if (save = fopen("../test_data/output/parking/parking_boundary.csv", "w")) {
    // parking bound
    for (size_t i = 0; i < round_pts.size(); ++i) {
      Vec2d tmp_pt = round_pts[i];
      fprintf(save, "%.4f,%.4f\n", tmp_pt.x(), tmp_pt.y());
    }

    // veh_xy
    fprintf(save, "%.4f, %.4f\n", veh_xy.x(), veh_xy.y());
    fprintf(save, "%.4f, %.4f\n", ref_start.x(), ref_start.y());
    fprintf(save, "%.4f, %.4f\n", ref_end.x(), ref_end.y());

    fclose(save);
  }
}

void ParkingSpotDecider::GetRoadBoundary(
    const ReferenceLinePtr refer_line, const double center_line_s,
    const Vec2d &origin_point, const double origin_heading,
    std::vector<Vec2d> *left_lane_boundary,
    std::vector<Vec2d> *right_lane_boundary,
    std::vector<Vec2d> *center_lane_boundary_left,
    std::vector<Vec2d> *center_lane_boundary_right,
    std::vector<double> *center_lane_s_left,
    std::vector<double> *center_lane_s_right,
    std::vector<double> *left_lane_road_width,
    std::vector<double> *right_lane_road_width) {
  double start_s = center_line_s - 20.0;
  // config_.open_space_roi_decider_config().roi_longitudinal_range_start();
  double end_s = center_line_s + 20.0;
  // config_.open_space_roi_decider_config().roi_longitudinal_range_end();

  ReferencePoint start_point = refer_line->get_reference_point(start_s);

  double last_check_point_heading = start_point.heading();
  double index = 0.0;
  double check_point_s = start_s;

  // For the road boundary, add key points to left/right side boundary
  // separately. Iterate s_value to check key points at a step of
  // roi_line_segment_length. Key points include: start_point, end_point, points
  // where path curvature is large, points near left/right road-curb corners
  while (check_point_s <= end_s) {
    ReferencePoint check_point = refer_line->get_reference_point(check_point_s);
    double check_point_heading = check_point.heading();
    bool is_center_lane_heading_change =
        std::abs(normalize_angle(check_point_heading -
                                 last_check_point_heading)) > 0.3;
    // config_.open_space_roi_decider_config().roi_line_segment_min_angle();
    last_check_point_heading = check_point_heading;

    LOG_INFO("is is_center_lane_heading_change: {}",
             is_center_lane_heading_change);
    // Check if the current center-lane checking-point is start point || end
    // point || or point with larger curvature. If yes, mark it as an anchor
    // point.
    bool is_anchor_point = std::abs(check_point_s - start_s) < 1e-2 ||
                           std::abs(check_point_s - end_s) < 1e-2 ||
                           is_center_lane_heading_change;
    // Add key points to the left-half boundary
    AddBoundaryKeyPoint(refer_line, check_point_s, start_s, end_s,
                        is_anchor_point, true, center_lane_boundary_left,
                        left_lane_boundary, center_lane_s_left,
                        left_lane_road_width);
    // Add key points to the right-half boundary
    AddBoundaryKeyPoint(refer_line, check_point_s, start_s, end_s,
                        is_anchor_point, false, center_lane_boundary_right,
                        right_lane_boundary, center_lane_s_right,
                        right_lane_road_width);
    if (std::abs(check_point_s - end_s) < 1e-2) {
      break;
    }
    index += 1.0;
    check_point_s = start_s + index * 1.0;
    // config_.open_space_roi_decider_config().roi_line_segment_length();
    check_point_s = check_point_s >= end_s ? end_s : check_point_s;
  }

  size_t left_point_size = left_lane_boundary->size();
  size_t right_point_size = right_lane_boundary->size();
  for (size_t i = 0; i < left_point_size; ++i) {
    left_lane_boundary->at(i) -= origin_point;
    left_lane_boundary->at(i).self_rotate(-origin_heading);
  }
  for (size_t i = 0; i < right_point_size; ++i) {
    right_lane_boundary->at(i) -= origin_point;
    right_lane_boundary->at(i).self_rotate(-origin_heading);
  }
  return;
}

void ParkingSpotDecider::AddBoundaryKeyPoint(
    const ReferenceLinePtr refer_line, const double check_point_s,
    const double start_s, const double end_s, const bool is_anchor_point,
    const bool is_left_curb, std::vector<Vec2d> *center_lane_boundary,
    std::vector<Vec2d> *curb_lane_boundary, std::vector<double> *center_lane_s,
    std::vector<double> *road_width) {
  // Check if current central-lane checking point's mapping on the left/right
  // road boundary is a key point. The road boundary point is a key point if
  // one of the following two confitions is satisfied:
  // 1. the current central-lane point is an anchor point: (a start/end point
  // or the point on path with large curvatures)
  // 2. the point on the left/right lane boundary is close to a curb corner
  // As indicated below:
  // (#) Key Point Type 1: Lane anchor points
  // (*) Key Point Type 2: Curb-corner points
  //                                                         #
  // Path Direction -->                                     /    /   #
  // Left Lane Boundary   #--------------------------------#    /   /
  //                                                           /   /
  // Center Lane          - - - - - - - - - - - - - - - - - - /   /
  //                                                             /
  // Right Lane Boundary  #--------*                 *----------#
  //                                \               /
  //                                 *-------------*

  // road width changes slightly at the turning point of a path
  // TODO: 1. consider distortion introduced by curvy road; 2. use both
  // round boundaries for single-track road; 3. longitudinal range may not be
  // symmetric
  const double previous_distance_s = std::min(1.0, check_point_s - start_s);
  const double next_distance_s = std::min(1.0, end_s - check_point_s);

  ReferencePoint current_check_point =
      refer_line->get_reference_point(check_point_s);
  ReferencePoint previous_check_point =
      refer_line->get_reference_point(check_point_s - previous_distance_s);
  ReferencePoint next_check_point =
      refer_line->get_reference_point(check_point_s + next_distance_s);

  double current_check_point_heading = current_check_point.heading();
  double current_road_width = is_left_curb
                                  ? current_check_point.left_road_bound()
                                  : current_check_point.right_road_bound();
  // If the current center-lane checking point is an anchor point, then add
  // current left/right curb boundary point as a key point
  if (is_anchor_point) {
    double point_vec_cos =
        is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                     : std::cos(current_check_point_heading - M_PI / 2.0);
    double point_vec_sin =
        is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                     : std::sin(current_check_point_heading - M_PI / 2.0);
    Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos,
                                  current_road_width * point_vec_sin);
    curb_lane_point.set_x(curb_lane_point.x() + current_check_point.x());
    curb_lane_point.set_y(curb_lane_point.y() + current_check_point.y());
    center_lane_boundary->push_back(
        {current_check_point.x(), current_check_point.y()});
    curb_lane_boundary->push_back(curb_lane_point);
    center_lane_s->push_back(check_point_s);
    road_width->push_back(current_road_width);
    return;
  }
  double previous_road_width = is_left_curb
                                   ? previous_check_point.left_road_bound()
                                   : previous_check_point.right_road_bound();
  double next_road_width = is_left_curb ? next_check_point.left_road_bound()
                                        : next_check_point.right_road_bound();
  double previous_curb_segment_angle =
      (current_road_width - previous_road_width) / previous_distance_s;
  double next_segment_angle =
      (next_road_width - current_road_width) / next_distance_s;
  double current_curb_point_delta_theta =
      next_segment_angle - previous_curb_segment_angle;
  // If the delta angle between the previous curb segment and the next curb
  // segment is large (near a curb corner), then add current curb_lane_point
  // as a key point.
  //  config_.open_space_roi_decider_config()
  //      .curb_heading_tangent_change_upper_limit()
  if (std::abs(current_curb_point_delta_theta) > 1.0) {
    double point_vec_cos =
        is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                     : std::cos(current_check_point_heading - M_PI / 2.0);
    double point_vec_sin =
        is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                     : std::sin(current_check_point_heading - M_PI / 2.0);
    Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos,
                                  current_road_width * point_vec_sin);
    curb_lane_point.set_x(curb_lane_point.x() + current_check_point.x());
    curb_lane_point.set_y(curb_lane_point.y() + current_check_point.y());
    center_lane_boundary->push_back(
        {current_check_point.x(), current_check_point.y()});

    curb_lane_boundary->push_back(curb_lane_point);
    center_lane_s->push_back(check_point_s);
    road_width->push_back(current_road_width);
  }
  return;
}

bool ParkingSpotDecider::FuseLineSegments(
    std::vector<std::vector<Vec2d>> *line_segments_vec) {
  static constexpr double kEpsilon = 1.0e-8;
  auto cur_segment = line_segments_vec->begin();
  while (cur_segment != line_segments_vec->end() - 1) {
    auto next_segment = cur_segment + 1;
    auto cur_last_point = cur_segment->back();
    auto next_first_point = next_segment->front();
    // Check if they are the same points
    if (cur_last_point.distance_to(next_first_point) > kEpsilon) {
      ++cur_segment;
      continue;
    }
    if (cur_segment->size() < 2 || next_segment->size() < 2) {
      LOG_ERROR("Single point line_segments vec not expected");
      return false;
    }
    size_t cur_segments_size = cur_segment->size();
    auto cur_second_to_last_point = cur_segment->at(cur_segments_size - 2);
    auto next_second_point = next_segment->at(1);
    if (cross_prod(cur_second_to_last_point, cur_last_point,
                   next_second_point) < 0.0) {
      cur_segment->push_back(next_second_point);
      next_segment->erase(next_segment->begin(), next_segment->begin() + 2);
      if (next_segment->empty()) {
        line_segments_vec->erase(next_segment);
      }
    } else {
      ++cur_segment;
    }
  }
  return true;
}

bool ParkingSpotDecider::FormulateBoundaryConstraints(
    const std::vector<std::vector<Vec2d>> &roi_parking_boundary) {
  // Gather vertice needed by warm start and distance approach
  if (!LoadObstacleInVertices(roi_parking_boundary)) {
    LOG_ERROR("fail at LoadObstacleInVertices()");
    return false;
  }
  // Transform vertices into the form of Ax>b
  if (!LoadObstacleInHyperPlanes()) {
    LOG_ERROR("fail at LoadObstacleInHyperPlanes()");
    return false;
  }
  return true;
}

bool ParkingSpotDecider::LoadObstacleInVertices(
    const std::vector<std::vector<Vec2d>> &roi_parking_boundary) {
  // load vertices for parking boundary (not need to repeat the first
  // vertice to get close hull)
  size_t parking_boundaries_num = roi_parking_boundary.size();
  size_t perception_obstacles_num = 0;
  std::vector<std::vector<Vec2d>> obstacles_verties_vec;

  parking_data_info_->SetObstaclesVerticesVec(obstacles_verties_vec);
  LOG_INFO("parking_boundaries_num : {}", parking_boundaries_num);
  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    parking_data_info_->MutableObstaclesVerticesVec()->push_back(
        roi_parking_boundary[i]);
  }
  LOG_INFO("obstacles_vertices_vec size : {}",
           parking_data_info_->MutableObstaclesVerticesVec()->size());
  Eigen::MatrixXi parking_boundaries_obstacles_edges_num(parking_boundaries_num,
                                                         1);
  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    if (roi_parking_boundary[i].size() < 1) {
      LOG_ERROR("roi_parking_boundary[i].size() < 1");
      return false;
    }
    parking_boundaries_obstacles_edges_num(i, 0) =
        static_cast<int>(roi_parking_boundary[i].size()) - 1;
  }
  Eigen::MatrixXi obstacles_edges_num;

  // need add perception obs here
  if (parking_data_info_->ConsiderObstcleFlag()) {
    // load vertices for perception obstacles(repeat the first vertice at the
    // last to form closed convex hull)

    //    for (auto &obstacle : obs_list) {
    //      if (FilterOutObstacle(obstacle)) {
    //        continue;
    //      }
    //      ++perception_obstacles_num;

    //      Box2d original_box = obstacle.bounding_box();
    //      original_box.shift(-1.0 * origin_point_);
    //      original_box.longitudinal_extend(0.2);
    //      original_box.lateral_extend(0.2);

    //      // TODO(Jinyun): Check correctness of ExpandByDistance() in polygon
    //      // Polygon2d buffered_box(original_box);
    //      // buffered_box = buffered_box.ExpandByDistance(
    //      //
    //      config_.open_space_roi_decider_config().perception_obstacle_buffer());
    //      // TODO(Runxin): Rotate from origin instead
    //      // original_box.RotateFromCenter(-1.0 * origin_heading);
    //      std::vector<Vec2d> vertices_ccw;
    //      original_box.get_all_corners(&vertices_ccw);
    //      std::vector<Vec2d> vertices_cw;
    //      while (!vertices_ccw.empty()) {
    //        auto current_corner_pt = vertices_ccw.back();
    //        current_corner_pt.self_rotate(-1.0 * origin_heading_);
    //        vertices_cw.push_back(current_corner_pt);
    //        vertices_ccw.pop_back();
    //      }
    //      // As the perception obstacle is a closed convex set, the first
    //      vertice
    //      // is repeated at the end of the vector to help transform all four
    //      edges
    //      // to inequality constraint
    //      vertices_cw.push_back(vertices_cw.front());
    //      obstacles_vertices_vec_.push_back(vertices_cw);
    //    }

    //    // obstacle boundary box is used, thus the edges are set to be 4
    //    Eigen::MatrixXi perception_obstacles_edges_num =
    //        4 * Eigen::MatrixXi::Ones(perception_obstacles_num, 1);

    //    obstacles_edges_num_.resize(parking_boundaries_obstacles_edges_num.rows()
    //    +
    //                                    perception_obstacles_edges_num.rows(),
    //                                1);
    //    obstacles_edges_num_ << parking_boundaries_obstacles_edges_num,
    //        perception_obstacles_edges_num;
  } else {
    obstacles_edges_num.resize(parking_boundaries_obstacles_edges_num.rows(),
                               1);
    obstacles_edges_num << parking_boundaries_obstacles_edges_num;
  }
  parking_data_info_->SetObstaclesEdgesNum(obstacles_edges_num);
  LOG_INFO("obstacles_edges_num size : {} x {}", obstacles_edges_num.rows(),
           obstacles_edges_num.cols());
  LOG_INFO("parking_boundaries_num : {}, perception_obstacles_num : {}",
           parking_boundaries_num, perception_obstacles_num);
  parking_data_info_->SetObstaclesNum(parking_boundaries_num +
                                      perception_obstacles_num);
  return true;
}

bool ParkingSpotDecider::LoadObstacleInHyperPlanes() {
  auto obstacles_edges_num_ = parking_data_info_->ObstaclesEdgesNum();
  Eigen::MatrixXd obstacles_A =
      Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 2);
  Eigen::MatrixXd obstacles_b =
      Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 1);

  // vertices using H-representation
  if (!GetHyperPlanes(parking_data_info_->ObstaclesNum(),
                      parking_data_info_->ObstaclesEdgesNum(),
                      parking_data_info_->ObstaclesVerticesVec(), &obstacles_A,
                      &obstacles_b)) {
    LOG_ERROR("Fail to present obstacle in hyperplane");
    return false;
  }
  parking_data_info_->SetObstaclesA(obstacles_A);
  parking_data_info_->SetObstaclesB(obstacles_b);
  return true;
}

bool ParkingSpotDecider::GetHyperPlanes(
    const size_t &obstacles_num, const Eigen::MatrixXi &obstacles_edges_num,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all) {
  LOG_INFO("obstacles_num : {}, obstacles_vertices_vec.size() : {}",
           obstacles_num, obstacles_vertices_vec.size());
  if (obstacles_num != obstacles_vertices_vec.size()) {
    LOG_ERROR("obstacles_num != obstacles_vertices_vec.size()");
    return false;
  }

  A_all->resize(obstacles_edges_num.sum(), 2);
  b_all->resize(obstacles_edges_num.sum(), 1);

  int counter = 0;
  double kEpsilon = 1.0e-5;
  // start building H representation
  for (size_t i = 0; i < obstacles_num; ++i) {
    size_t current_vertice_num = obstacles_edges_num(i, 0);
    Eigen::MatrixXd A_i(current_vertice_num, 2);
    Eigen::MatrixXd b_i(current_vertice_num, 1);

    // take two subsequent vertices, and computer hyperplane
    for (size_t j = 0; j < current_vertice_num; ++j) {
      Vec2d v1 = obstacles_vertices_vec[i][j];
      Vec2d v2 = obstacles_vertices_vec[i][j + 1];

      Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);
      // find hyperplane passing through v1 and v2
      if (std::abs(v1.x() - v2.x()) < kEpsilon) {
        if (v2.y() < v1.y()) {
          A_tmp << 1, 0;
          b_tmp << v1.x();
        } else {
          A_tmp << -1, 0;
          b_tmp << -v1.x();
        }
      } else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
        if (v1.x() < v2.x()) {
          A_tmp << 0, 1;
          b_tmp << v1.y();
        } else {
          A_tmp << 0, -1;
          b_tmp << -v1.y();
        }
      } else {
        Eigen::MatrixXd tmp1(2, 2);
        tmp1 << v1.x(), 1, v2.x(), 1;
        Eigen::MatrixXd tmp2(2, 1);
        tmp2 << v1.y(), v2.y();
        ab = tmp1.inverse() * tmp2;
        double a = ab(0, 0);
        double b = ab(1, 0);

        if (v1.x() < v2.x()) {
          A_tmp << -a, 1;
          b_tmp << b;
        } else {
          A_tmp << a, -1;
          b_tmp << -b;
        }
      }

      // store vertices
      A_i.block(j, 0, 1, 2) = A_tmp.transpose();
      b_i.block(j, 0, 1, 1) = b_tmp;
    }

    A_all->block(counter, 0, A_i.rows(), 2) = A_i;
    b_all->block(counter, 0, b_i.rows(), 1) = b_i;
    counter += static_cast<int>(current_vertice_num);
  }
  return true;
}

// bool ParkingSpotDecider::FilterOutObstacle(const Obstacle &obstacle) {
//  if (obstacle.is_virtual()) {
//    return true;
//  }

//  const auto &obstacle_box = obstacle.bounding_box();
//  auto obstacle_center_xy = obstacle_box.center();

//  // xy_boundary in xmin, xmax, ymin, ymax.
//  obstacle_center_xy -= origin_point_;
//  obstacle_center_xy.self_rotate(-origin_heading_);
//  if (obstacle_center_xy.x() < ROI_xy_boundary_[0] ||
//      obstacle_center_xy.x() > ROI_xy_boundary_[1] ||
//      obstacle_center_xy.y() < ROI_xy_boundary_[2] ||
//      obstacle_center_xy.y() > ROI_xy_boundary_[3]) {
//    return true;
//  }

//  // Translate the end pose back to world frame with endpose in x, y, phi, v
//  Vec2d end_pose_x_y(parking_end_pose_[0], parking_end_pose_[1]);
//  end_pose_x_y.self_rotate(origin_heading_);
//  end_pose_x_y += origin_point_;

//  // Get vehicle state
//  Vec2d vehicle_x_y(CurrGps.x(), CurrGps.y());

//  // Use vehicle position and end position to filter out obstacle
//  const double vehicle_center_to_obstacle =
//      obstacle_box.distance_to(vehicle_x_y);
//  const double end_pose_center_to_obstacle =
//      obstacle_box.distance_to(end_pose_x_y);
//  const double filtering_distance = 1000.0;
//  if (vehicle_center_to_obstacle > filtering_distance &&
//      end_pose_center_to_obstacle > filtering_distance) {
//    return true;
//  }
//  return false;
//}

}  // namespace planning
}  // namespace neodrive

#include "parking_spot_decider.h"

namespace neodrive {
namespace planning {

// reorganize refer lines bound
// vertices: UTM(x,y)
bool ParkingSpotDecider::ReferLineReorganize(ReferenceLinePtr const refer_line,
                                             std::array<Vec2d, 4> &vertices) {
  if (vertices.size() != 4) {
    LOG_ERROR("vertices.size()!=4");
    return false;
  }
  if (refer_line->ref_points().size() < 4) {
    LOG_ERROR("refer_line->ref_points().size()<4");
    return false;
  }
  SLPoint tmp_sl;
  std::vector<Vec2d> sl_vec;
  for (size_t i = 0; i < vertices.size(); ++i) {
    if (!refer_line->GetPointInFrenetFrame(vertices[i], &tmp_sl)) {
      LOG_ERROR("get projection failed");
      return false;
    }
    sl_vec.push_back(tmp_sl);
  }

  if (!ReferLineBoundAdapt(refer_line, sl_vec)) {
    LOG_ERROR("ReferLineBoundAdapt failed");
    return false;
  }

  if (!GetCarportLimitations(
          refer_line, vertices,
          *parking_data_info_->MutableLimitCarportCoorGlobal(),
          *parking_data_info_->MutableLimitRoadOppsiteGlobal(),
          *parking_data_info_->MutableLimitRoadRearGlobal(),
          *parking_data_info_->MutableLimitRoadFrontGlobal())) {
    LOG_ERROR("get_carport_limitations failed");
    return false;
  }
  return true;
}

// reorganize refer lines bound
// vertice: SL(x,y)
bool ParkingSpotDecider::ReferLineBoundAdapt(ReferenceLinePtr const refer_line,
                                             std::vector<Vec2d> &vertice) {
  if (vertice.size() != 4) {
    LOG_ERROR("vertice.size()!=4");
    return false;
  }
  if (refer_line->ref_points().size() < 4) {
    LOG_ERROR("refer_line->ref_points().size()<4");
    return false;
  }
  // sort by x
  std::sort(vertice.begin(), vertice.end(),
            [](auto &a, auto &b) { return a.x() < b.x(); });

  // check the shape of vertices
  double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, x3 = 0.0, y3 = 0.0, x4 = 0.0,
         y4 = 0.0;
  if (vertice[0].y() < vertice[1].y()) {
    x1 = vertice[0].x();
    y1 = vertice[0].y();
    x2 = vertice[1].x();
    y2 = vertice[1].y();
  } else {
    x1 = vertice[1].x();
    y1 = vertice[1].y();
    x2 = vertice[0].x();
    y2 = vertice[0].y();
  }
  if (vertice[2].y() < vertice[3].y()) {
    x3 = vertice[3].x();
    y3 = vertice[3].y();
    x4 = vertice[2].x();
    y4 = vertice[2].y();
  } else {
    x3 = vertice[2].x();
    y3 = vertice[2].y();
    x4 = vertice[3].x();
    y4 = vertice[3].y();
  }
  // check carport left or right
  bool left = false, right = false;
  if (y2 < 0 || abs(y2) < abs(y1)) {
    left = false;
    right = true;
  } else if (y1 > 0 || abs(y2) > abs(y1)) {
    left = true;
    right = false;
  }
  // build 4 lines
  // warning, parallel to y-axis
  double k1 = 0.0, b1 = 0.0, k2 = 0.0, b2 = 0.0, k3 = 0.0, b3 = 0.0, k4 = 0.0,
         b4 = 0.0;
  bool line1_vertical = false, line2_vertical = false;
  bool line3_vertical = false, line4_vertical = false;
  if (std::abs(x1 - x2) < 0.01) {
    line1_vertical = true;
  } else {
    k1 = (y1 - y2) / (x1 - x2);
    b1 = y1 - (((y1 - y2) * x1) / (x1 - x2));
  }
  if (std::abs(x2 - x3) < 0.01) {
    line2_vertical = true;
  } else {
    k2 = (y2 - y3) / (x2 - x3);
    b2 = y2 - (((y2 - y3) * x2) / (x2 - x3));
  }
  if (std::abs(x3 - x4) < 0.01) {
    line3_vertical = true;
  } else {
    k3 = (y3 - y4) / (x3 - x4);
    b3 = y3 - (((y3 - y4) * x3) / (x3 - x4));
  }
  if (std::abs(x4 - x1) < 0.01) {
    line4_vertical = true;
  } else {
    k4 = (y4 - y1) / (x4 - x1);
    b4 = y4 - (((y4 - y1) * x4) / (x4 - x1));
  }

  size_t m1 = 0, m2 = 0;
  double y = 0.0;
  if (!refer_line->get_index_by_s(vertice[0].x(), &m1)) return false;
  if (!refer_line->get_index_by_s(vertice[3].x(), &m2)) return false;
  if (left && (x1 > x2) && (x3 > x4)) {
    if (!refer_line->get_index_by_s(vertice[1].x(), &m1)) return false;
    if (!refer_line->get_index_by_s(vertice[2].x(), &m2)) return false;
  } else if (left && (x1 > x2) && (x3 < x4)) {
    if (!refer_line->get_index_by_s(vertice[1].x(), &m1)) return false;
    if (!refer_line->get_index_by_s(vertice[3].x(), &m2)) return false;
  } else if (left && (x1 < x2) && (x3 > x4)) {
    if (!refer_line->get_index_by_s(vertice[0].x(), &m1)) return false;
    if (!refer_line->get_index_by_s(vertice[2].x(), &m2)) return false;
  } else if (right && (x1 < x2) && (x3 < x4)) {
    if (!refer_line->get_index_by_s(vertice[1].x(), &m1)) return false;
    if (!refer_line->get_index_by_s(vertice[2].x(), &m2)) return false;
  } else if (right && (x1 < x2) && (x3 > x4)) {
    if (!refer_line->get_index_by_s(vertice[1].x(), &m1)) return false;
    if (!refer_line->get_index_by_s(vertice[3].x(), &m2)) return false;
  } else if (right && (x1 > x2) && (x3 < x4)) {
    if (!refer_line->get_index_by_s(vertice[0].x(), &m1)) return false;
    if (!refer_line->get_index_by_s(vertice[2].x(), &m2)) return false;
  }

  for (size_t i = m1; i < m2; ++i) {
    auto &pt = refer_line->mutable_ref_points()->at(i);
    if (left) {
      // carport is left of vehicle
      if (pt.s() >= vertice[0].x() && pt.s() <= vertice[1].x()) {
        if (line1_vertical) {
          y = y2;
        } else {
          y = k1 * pt.s() + b1;
        }
        pt.set_left_bound(std::abs(y));
      } else if (pt.s() > vertice[1].x() && pt.s() <= vertice[2].x()) {
        if (line2_vertical) {
          LOG_ERROR("line2 is vertical, something is wrong, return");
          return false;
        } else {
          y = k2 * pt.s() + b2;
        }
        pt.set_left_bound(std::abs(y));
      } else if (pt.s() > vertice[2].x() && pt.s() <= vertice[3].x()) {
        if (line3_vertical) {
          y = y3;
        } else {
          y = k3 * pt.s() + b3;
        }
        pt.set_left_bound(std::abs(y));
      }
    } else {
      // carport is right of vehicle
      if (pt.s() >= vertice[0].x() && pt.s() <= vertice[1].x()) {
        if (line1_vertical) {
          y = y1;
        } else {
          y = k1 * pt.s() + b1;
        }
        pt.set_right_bound(std::abs(y));
      } else if (pt.s() > vertice[1].x() && pt.s() < vertice[2].x()) {
        if (line4_vertical) {
          LOG_ERROR("line4 is vertical, something is wrong");
          return false;
        } else {
          y = k4 * pt.s() + b4;
        }
        pt.set_right_bound(std::abs(y));
      } else if (pt.s() > vertice[2].x() && pt.s() <= vertice[3].x()) {
        if (line3_vertical) {
          y = y4;
        } else {
          y = k3 * pt.s() + b3;
        }
        pt.set_right_bound(std::abs(y));
      }
    }
  }

  return true;
}

bool ParkingSpotDecider::GetCarportLimitations(
    const ReferenceLinePtr refer_line, std::array<Vec2d, 4> &vertices,
    std::vector<Vec2d> &limit_carport_coor_global,
    std::vector<Vec2d> &limit_road_oppsite_global,
    std::vector<Vec2d> &limit_road_rear_global,
    std::vector<Vec2d> &limit_road_front_global) {
  if (parking_data_info_->LeftRightType() == 0) {
    LOG_ERROR("carport is not initialized");
    return false;
  }
  limit_carport_coor_global.clear();
  limit_road_oppsite_global.clear();
  limit_road_rear_global.clear();
  limit_road_front_global.clear();

  std::vector<Vec2d> vec_carport;
  std::vector<Vec2d> vec_road_oppsite;
  std::vector<Vec2d> vec_road;

  for (size_t i = 0; i < vertices.size(); ++i) {
    vec_carport.push_back(vertices[i]);
  }

  Vec2d tmp_lower_pt;
  Vec2d tmp_upper_pt;
  // extract bound (x,y) from whole refer_line
  size_t num_size = refer_line->ref_points().size();
  double tmp_val = 0.0;
  for (size_t i = 0; i < num_size; ++i) {
    auto pt = refer_line->ref_points().at(i);
    tmp_lower_pt.set_x(0);
    tmp_lower_pt.set_y(-pt.right_bound());
    tmp_upper_pt.set_x(0);
    tmp_upper_pt.set_y(pt.left_bound());

    tmp_lower_pt.self_rotate(pt.heading());
    tmp_upper_pt.self_rotate(pt.heading());
    tmp_val = tmp_lower_pt.x() + pt.x();
    tmp_lower_pt.set_x(tmp_val);
    tmp_val = tmp_lower_pt.y() + pt.y();
    tmp_lower_pt.set_y(tmp_val);
    tmp_val = tmp_upper_pt.x() + pt.x();
    tmp_upper_pt.set_x(tmp_val);
    tmp_val = tmp_upper_pt.y() + pt.y();
    tmp_upper_pt.set_y(tmp_val);
    // tmp_upper_pt.set_x(pt.right_bound() * sin(pt.heading()) + pt.x());
    // tmp_upper_pt.set_y(pt.right_bound() * cos(pt.heading()) + pt.y());
    // tmp_lower_pt.set_x(-pt.right_bound() * sin(pt.heading()) + pt.x());
    // tmp_lower_pt.set_y(-pt.right_bound() * cos(pt.heading()) + pt.y());
    vec_road_oppsite.push_back(tmp_upper_pt);
    vec_road.push_back(tmp_lower_pt);
  }
  // sanity check
  if (num_size != vec_road_oppsite.size() || num_size != vec_road.size()) {
    LOG_ERROR(
        "num_size != vec_road_oppsite.size() || num_size != vec_road.size()");
    return false;
  }

  // test refer line and bound function
  LogReferLineBound(vec_road, vec_road_oppsite, refer_line->ref_points());

  bool ret = false;
  if (parking_data_info_->LeftRightType() == 1) {
    ret = CalcParkingRoadboundKeyPts(
        refer_line, parking_data_info_->LeftRightType(), vec_road,
        vec_road_oppsite, vec_carport, limit_road_oppsite_global,
        limit_road_rear_global, limit_carport_coor_global,
        limit_road_front_global);
  } else {
    ret = CalcParkingRoadboundKeyPts(
        refer_line, parking_data_info_->LeftRightType(), vec_road_oppsite,
        vec_road, vec_carport, limit_road_oppsite_global,
        limit_road_rear_global, limit_carport_coor_global,
        limit_road_front_global);
  }
  if (ret == false) {
    LOG_ERROR("CalcParkingRoadboundKeyPts failed");
    return false;
  }
  if (limit_road_oppsite_global.empty() || limit_road_rear_global.empty() ||
      limit_carport_coor_global.empty() || limit_road_front_global.empty()) {
    LOG_ERROR("CalcParkingRoadboundKeyPts failed, something is empty");
    return false;
  }
  // transform to anchor coor
  std::vector<Vec2d> tmp_vec_anchor;
  double vel_x = parking_data_info_->AnchorPointCoordinate().x();
  double vel_y = parking_data_info_->AnchorPointCoordinate().y();
  double vel_yaw = parking_data_info_->AnchorPointCoordinate().heading();
  double tmp_x = 0.0, tmp_y = 0.0, tmp_yaw = 0.0;

  tmp_vec_anchor.clear();
  for (size_t i = 0; i < limit_road_oppsite_global.size(); ++i) {
    auto &pt = limit_road_oppsite_global[i];
    earth2vehicle(vel_x, vel_y, vel_yaw, pt.x(), pt.y(), 0.0, tmp_x, tmp_y,
                  tmp_yaw);
    tmp_vec_anchor.push_back({tmp_x, tmp_y});
  }
  parking_data_info_->SetLimitRoadOppsiteAnchor(tmp_vec_anchor);

  tmp_vec_anchor.clear();
  for (size_t i = 0; i < limit_road_rear_global.size(); ++i) {
    auto &pt = limit_road_rear_global[i];
    earth2vehicle(vel_x, vel_y, vel_yaw, pt.x(), pt.y(), 0.0, tmp_x, tmp_y,
                  tmp_yaw);
    tmp_vec_anchor.push_back({tmp_x, tmp_y});
  }
  parking_data_info_->SetLimitRoadRearAnchor(tmp_vec_anchor);

  tmp_vec_anchor.clear();
  for (size_t i = 0; i < limit_carport_coor_global.size(); ++i) {
    auto &pt = limit_carport_coor_global[i];
    earth2vehicle(vel_x, vel_y, vel_yaw, pt.x(), pt.y(), 0.0, tmp_x, tmp_y,
                  tmp_yaw);
    tmp_vec_anchor.push_back({tmp_x, tmp_y});
  }
  parking_data_info_->SetLimitCarportCoorAnchor(tmp_vec_anchor);

  tmp_vec_anchor.clear();
  for (size_t i = 0; i < limit_road_front_global.size(); ++i) {
    auto &pt = limit_road_front_global[i];
    earth2vehicle(vel_x, vel_y, vel_yaw, pt.x(), pt.y(), 0.0, tmp_x, tmp_y,
                  tmp_yaw);
    tmp_vec_anchor.push_back({tmp_x, tmp_y});
  }
  parking_data_info_->SetLimitRoadFrontAnchor(tmp_vec_anchor);

  return true;
}

bool ParkingSpotDecider::CalcParkingRoadboundKeyPts(
    const ReferenceLinePtr reference_line, const int &left_right_flag,
    const std::vector<Vec2d> &not_park_side_road_bound,
    const std::vector<Vec2d> &park_side_road_bound,
    const std::vector<Vec2d> &parking_space,
    std::vector<Vec2d> &limit_road_oppsite_global,
    std::vector<Vec2d> &limit_road_rear_global,
    std::vector<Vec2d> &limit_carport_coor_global,
    std::vector<Vec2d> &limit_road_front_global) {
  if (not_park_side_road_bound.size() < 2 || park_side_road_bound.size() < 2 ||
      parking_space.size() < 4) {
    LOG_ERROR("input data err!");
    return false;
  }
  // get limit_road_oppsite_global
  if (!ExtractRoadboundKeyPts(not_park_side_road_bound,
                              limit_road_oppsite_global)) {
    LOG_ERROR("limit_road_oppsite_global get failed");
    return false;
  }

  // part 1, fill extended parking space cornner pts limit_carport_coor_global.
  // find parking space two rear and two front corner pts
  std::size_t min_s_index = 0;
  std::size_t second_min_s_index = 0;
  double min_s = std::numeric_limits<double>::max();
  double second_min_s = std::numeric_limits<double>::max();
  std::size_t max_s_index = 0;
  std::size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  std::vector<SLPoint> parking_space_sl;
  for (std::size_t i = 0; i < parking_space.size(); ++i) {
    SLPoint sl_pt;
    if (!reference_line->GetPointInFrenetFrame(parking_space[i], &sl_pt)) {
      LOG_ERROR("get frenet frame fail!");
      return false;
    }
    parking_space_sl.push_back(sl_pt);
    if (sl_pt.s() < min_s) {
      second_min_s = min_s;
      second_min_s_index = min_s_index;
      min_s = sl_pt.s();
      min_s_index = i;
    } else if (sl_pt.s() >= min_s && sl_pt.s() < second_min_s) {
      second_min_s = sl_pt.s();
      second_min_s_index = i;
    }
    if (sl_pt.s() > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = sl_pt.s();
      max_s_index = i;
    } else if (sl_pt.s() <= max_s && sl_pt.s() > second_max_s) {
      second_max_s = sl_pt.s();
      second_max_s_index = i;
    }
  }
  // find four parking sl pts
  // four pts: rear two + front two
  std::vector<SLPoint> park_sl_pts{
      parking_space_sl[min_s_index], parking_space_sl[second_min_s_index],
      parking_space_sl[max_s_index], parking_space_sl[second_max_s_index]};
  // record four corr pts index
  std::size_t front_corr_index0 = 0;
  std::size_t front_corr_index1 = 0;
  std::size_t rear_corr_index2 = 0;
  std::size_t rear_corr_index3 = 0;
  // four pts' corresponding road bound pts;
  std::vector<SLPoint> corr_sl_pts(4);
  bool b_front_corr_index0 = false;
  bool b_front_corr_index1 = false;
  bool b_rear_corr_index0 = false;
  bool b_rear_corr_index1 = false;

  if (left_right_flag == 1) {
    // rear 2 pts
    for (std::size_t i = 0; i < park_side_road_bound.size(); ++i) {
      SLPoint sl_pt;
      if (!reference_line->GetPointInFrenetFrame(park_side_road_bound[i],
                                                     &sl_pt)) {
        LOG_ERROR("get frenet frame fail!");
        return false;
      }
      // rear
      if ((sl_pt.s() >= park_sl_pts[2].s()) &&
          (sl_pt.l() <= park_sl_pts[2].l()) && !b_rear_corr_index0) {
        corr_sl_pts[2] = sl_pt;
        rear_corr_index2 = i;
        b_rear_corr_index0 = true;
      }
      if ((sl_pt.s() >= park_sl_pts[3].s()) &&
          (sl_pt.l() <= park_sl_pts[3].l()) && !b_rear_corr_index1) {
        corr_sl_pts[3] = sl_pt;
        rear_corr_index3 = i;
        b_rear_corr_index1 = true;
      }
      if (b_rear_corr_index0 && b_rear_corr_index1) break;
    }
    // front 2 pts
    for (std::size_t i = park_side_road_bound.size() - 1; i > 0; i--) {
      SLPoint sl_pt;
      if (!reference_line->GetPointInFrenetFrame(park_side_road_bound[i],
                                                     &sl_pt)) {
        LOG_ERROR("get frenet frame fail!");
        return false;
      }
      // front
      if ((sl_pt.s() <= park_sl_pts[1].s()) &&
          (sl_pt.l() <= park_sl_pts[1].l()) && !b_front_corr_index1) {
        corr_sl_pts[1] = sl_pt;
        front_corr_index1 = i;
        b_front_corr_index1 = true;
      }
      if ((sl_pt.s() <= park_sl_pts[0].s()) &&
          (sl_pt.l() <= park_sl_pts[0].l()) && !b_front_corr_index0) {
        corr_sl_pts[0] = sl_pt;
        front_corr_index0 = i;
        b_front_corr_index0 = true;
      }
      if (b_front_corr_index0 && b_front_corr_index1) break;
    }

  } else {
    // rear 2 pts
    for (std::size_t i = 0; i < park_side_road_bound.size(); ++i) {
      SLPoint sl_pt;
      if (!reference_line->GetPointInFrenetFrame(park_side_road_bound[i],
                                                     &sl_pt)) {
        LOG_INFO("park side road bound[{}] : x {:.4f}, y {:.4f}, theta {:.4f}", i,
                 park_side_road_bound[i].x(), park_side_road_bound[i].y(),
                 park_side_road_bound[i].angle());
        LOG_ERROR("get frenet frame fail!");
        return false;
      }
      // rear
      if ((sl_pt.s() >= park_sl_pts[2].s()) &&
          (sl_pt.l() >= park_sl_pts[2].l()) && !b_rear_corr_index0) {
        corr_sl_pts[2] = sl_pt;
        rear_corr_index2 = i;
        b_rear_corr_index0 = true;
      }
      if ((sl_pt.s() >= park_sl_pts[3].s()) &&
          (sl_pt.l() >= park_sl_pts[3].l()) && !b_rear_corr_index1) {
        corr_sl_pts[3] = sl_pt;
        rear_corr_index3 = i;
        b_rear_corr_index1 = true;
      }
      if (b_rear_corr_index0 && b_rear_corr_index1) break;
    }
    // front 2 pts
    for (std::size_t i = park_side_road_bound.size() - 1; i > 0; i--) {
      SLPoint sl_pt;
      if (!reference_line->GetPointInFrenetFrame(park_side_road_bound[i],
                                                     &sl_pt)) {
        LOG_ERROR("get frenet frame fail!");
        return false;
      }
      // front
      if ((sl_pt.s() <= park_sl_pts[1].s()) &&
          (sl_pt.l() >= park_sl_pts[1].l()) && !b_front_corr_index1) {
        corr_sl_pts[1] = sl_pt;
        front_corr_index1 = i;
        b_front_corr_index1 = true;
      }
      if ((sl_pt.s() <= park_sl_pts[0].s()) &&
          (sl_pt.l() >= park_sl_pts[0].l()) && !b_front_corr_index0) {
        corr_sl_pts[0] = sl_pt;
        front_corr_index0 = i;
        b_front_corr_index0 = true;
      }
      if (b_front_corr_index0 && b_front_corr_index1) break;
    }
  }
  // valid
  if (!(b_front_corr_index0 && b_front_corr_index1)) {
    if (!b_front_corr_index0 && !b_front_corr_index1) {
      LOG_ERROR("get carport failed");
      return false;
    }
    if (!b_front_corr_index0) {
      corr_sl_pts[0] = corr_sl_pts[1];
      front_corr_index0 = front_corr_index1;
    } else {
      corr_sl_pts[1] = corr_sl_pts[0];
      front_corr_index1 = front_corr_index0;
    }
  }
  if (!(b_rear_corr_index0 && b_rear_corr_index1)) {
    if (!b_rear_corr_index0 && !b_rear_corr_index1) {
      LOG_ERROR("get carport failed");
      return false;
    }
    if (!b_rear_corr_index0) {
      corr_sl_pts[2] = corr_sl_pts[3];
      rear_corr_index2 = rear_corr_index3;
    } else {
      corr_sl_pts[3] = corr_sl_pts[2];
      rear_corr_index3 = rear_corr_index2;
    }
  }
  // decide four space pts whether extend and extend how much
  const double EXTEND_VALUE = 5.0;  // m
  double tmp_delta = 0.0;
  double tmp_delta_1 = 0.0;
  tmp_delta = park_sl_pts[0].s() - corr_sl_pts[0].s();
  tmp_delta_1 = tmp_delta;
  tmp_delta = park_sl_pts[1].s() - corr_sl_pts[1].s();
  tmp_delta_1 = fmin(tmp_delta_1, tmp_delta);

  if (tmp_delta_1 >= EXTEND_VALUE) {
    park_sl_pts[0].set_s(park_sl_pts[0].s() - EXTEND_VALUE);
    park_sl_pts[1].set_s(park_sl_pts[1].s() - EXTEND_VALUE);
  } else if (tmp_delta > 0.0 && tmp_delta < EXTEND_VALUE) {
    park_sl_pts[0].set_s(park_sl_pts[0].s() - tmp_delta);
    park_sl_pts[1].set_s(park_sl_pts[1].s() - tmp_delta);
  }

  tmp_delta = corr_sl_pts[2].s() - park_sl_pts[2].s();
  tmp_delta_1 = tmp_delta;
  tmp_delta = corr_sl_pts[3].s() - park_sl_pts[3].s();
  tmp_delta_1 = fmin(tmp_delta_1, tmp_delta);

  if (tmp_delta >= EXTEND_VALUE) {
    park_sl_pts[2].set_s(park_sl_pts[2].s() + EXTEND_VALUE);
    park_sl_pts[3].set_s(park_sl_pts[3].s() + EXTEND_VALUE);
  } else if (tmp_delta > 0.0 && tmp_delta < EXTEND_VALUE) {
    park_sl_pts[2].set_s(park_sl_pts[2].s() + tmp_delta);
    park_sl_pts[3].set_s(park_sl_pts[3].s() + tmp_delta);
  }
  // tranform into cartesian frame
  /*
   *  0**3
   *   *  *
   *   1**2
   * change to
   *  0**1
   *  *  *
   *  3**2
   *
   *
   */
  limit_carport_coor_global.clear();
  if (left_right_flag == 1) {
    if (park_sl_pts[0].l() > park_sl_pts[1].l())
      std::swap(park_sl_pts[0], park_sl_pts[1]);
    if (park_sl_pts[2].l() < park_sl_pts[3].l())
      std::swap(park_sl_pts[2], park_sl_pts[3]);
  } else {
    if (park_sl_pts[0].l() < park_sl_pts[1].l())
      std::swap(park_sl_pts[0], park_sl_pts[1]);
    if (park_sl_pts[2].l() > park_sl_pts[3].l())
      std::swap(park_sl_pts[2], park_sl_pts[3]);
    std::swap(park_sl_pts[1], park_sl_pts[3]);
  }

  for (auto sl_pt : park_sl_pts) {
    // limit_carport_coor_global
    Vec2d xy_pt;
    if (!reference_line->get_point_in_cartesian_frame(sl_pt, &xy_pt)) {
      LOG_ERROR("get cartesian frame fail!");
      return false;
    }
    limit_carport_coor_global.push_back(xy_pt);
  }

  // rear segment extract
  size_t tmp_rear = std::min(front_corr_index0, front_corr_index1);
  std::vector<Vec2d> rear_part;
  rear_part.insert(rear_part.end(), park_side_road_bound.begin(),
                   park_side_road_bound.begin() + tmp_rear);
  if (!ExtractRoadboundKeyPts(rear_part, limit_road_rear_global)) {
    LOG_ERROR("limit_road_rear_global failed");
    return false;
  }

  tmp_rear = std::max(rear_corr_index2, rear_corr_index3);
  rear_part.clear();
  rear_part.insert(rear_part.end(), park_side_road_bound.begin() + tmp_rear,
                   park_side_road_bound.end());
  if (!ExtractRoadboundKeyPts(rear_part, limit_road_front_global)) {
    LOG_ERROR("limit_road_front_global failed");
    return false;
  }

  if (left_right_flag == 1) {
    // reorginize the sequence
    std::reverse(limit_road_oppsite_global.begin(),
                 limit_road_oppsite_global.end());
    std::vector<Vec2d> transfer_vec = limit_road_front_global;
    limit_road_front_global = limit_road_rear_global;
    std::reverse(limit_road_front_global.begin(),
                 limit_road_front_global.end());
    limit_road_rear_global = transfer_vec;
    std::reverse(limit_road_rear_global.begin(), limit_road_rear_global.end());
    std::reverse(limit_carport_coor_global.begin(),
                 limit_carport_coor_global.end());
    std::swap(limit_carport_coor_global[1], limit_carport_coor_global[3]);
  }
  return true;
}

bool ParkingSpotDecider::ExtractRoadboundKeyPts(
    const std::vector<Vec2d> &road_bound, std::vector<Vec2d> &key) {
  if (road_bound.size() < 2) {
    LOG_ERROR("road_bound.size() < 2");
    return false;
  }
  const double RESULUTION = 0.2;
  // part 0, fill oppsite side road bound inflection pts
  // key.
  key.clear();
  key.push_back(road_bound.front());
  std::size_t ii = 0;
  // TODO(wyc): this can be more elegant
  while (ii < road_bound.size() - 2) {
    std::size_t jj = ii + 2;
    // Vec2d start_pt = not_park_side_road_bound[ii];
    // Vec2d end_pt = not_park_side_road_bound[ii+1];
    while (jj < road_bound.size()) {
      // build segment
      // TODO(wyc): use const reference other than object
      const Vec2d &start_pt = road_bound[ii];
      const Vec2d &end_pt = road_bound[jj - 1];
      Segment2d seg(start_pt, end_pt);
      const Vec2d &check_pt = road_bound[jj];
      // use cross prod to calc pt to line dis
      double dis = cross_prod(start_pt, end_pt, check_pt) / seg.length();
      // dis over 0.2m, means check pt not in seg line,
      // the end pt is a inflection pt.
      if (dis > RESULUTION) {
        key.push_back(end_pt);
        // update index, jj - 1 is the inflection pt.
        ii = jj - 1;
        break;
      }
      jj += 1;
    }
    // if jj go through limit_road_oppsite_global, need to break
    if (jj >= road_bound.size() - 1) break;
  }
  key.push_back(road_bound.back());
  return true;
}

void ParkingSpotDecider::LogReferLineBound(
    const std::vector<Vec2d> &vec_road,
    const std::vector<Vec2d> &vec_road_oppsite,
    const std::vector<ReferencePoint> &refer_pts) {
  if (refer_pts.size() != vec_road_oppsite.size() ||
      refer_pts.size() != vec_road.size()) {
    LOG_ERROR(
        "num_size != vec_road_oppsite.size() || num_size != vec_road.size()");
    return;
  }
  FILE *save;
  if (save = fopen("../test_data/output/refer_bound_test.csv", "w")) {
    for (size_t i = 0; i < refer_pts.size(); ++i) {
      auto &pt = refer_pts.at(i);
      fprintf(save, "%f,%f,%f, %f,%f, %f,%f, %f,%f\n ", pt.x(), pt.y(),
              pt.heading(), pt.left_bound(), pt.right_bound(),
              vec_road_oppsite[i].x(), vec_road_oppsite[i].y(), vec_road[i].x(),
              vec_road[i].y());
    }
  }
  if (save != nullptr) {
    fclose(save);
  }
}

}  // namespace planning
}  // namespace neodrive

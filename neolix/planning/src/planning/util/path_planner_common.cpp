#include "path_planner_common.h"

namespace neodrive {
namespace planning {
namespace path_planner_common {

bool CalculateRoadObsBoundary(const std::size_t planner_type,
                              const double static_max_speed,
                              const InsidePlannerData &inside_data,
                              const DecisionData &decision_data,
                              const std::vector<FrenetFramePoint> &frenet_path,
                              PathObstacleContext &path_obstacle_context,
                              std::vector<PieceBoundary> &lateral_boundaries,
                              OutsidePlannerData *const outside_data) {
  return true;
}

bool CalcFinalTrajValidLength(const std::size_t planner_type,
                              const PathObstacleContext &path_obstacle_context,
                              const ReferenceLinePtr &reference_line,
                              const std::vector<FrenetFramePoint> &frenet_path,
                              double *valid_length) {
  if (valid_length == nullptr) return false;
  std::vector<Boundary> adc_boundarys;
  std::vector<Box2d> adc_boxes;
  std::vector<Box2d> adc_boxes_sl;
  std::vector<ReferencePoint> reference_points;
  std::vector<std::vector<Vec2d>> adc_box_xy_pts_vec;
  std::vector<std::vector<Vec2d>> adc_box_sl_pts_vec;
  CalcAdcSampleDataForValidLength(frenet_path, reference_line, adc_boundarys,
                                  adc_boxes, adc_boxes_sl, adc_box_xy_pts_vec,
                                  adc_box_sl_pts_vec, reference_points);
  if (!GetWithoutRoadBoundCollisionLength(reference_points, frenet_path,
                                          valid_length)) {
    LOG_ERROR("GetWithoutRoadBoundCollisionLength err");
  }
  LOG_DEBUG("road without collision len {:.3f}", *valid_length);
  if (!GetWithoutObsCollisionLength(planner_type, path_obstacle_context,
                                    adc_boundarys, adc_boxes, adc_boxes_sl,
                                    adc_box_xy_pts_vec, adc_box_sl_pts_vec,
                                    frenet_path, valid_length)) {
    LOG_ERROR("GetWithoutObsCollisionLength err");
  }
  LOG_DEBUG("obs without collision len {:.3f}", *valid_length);
  return true;
}

bool CutOffFinalTrajValidLength(const DiscretizedPath &path,
                                double *valid_length) {
  if (valid_length == nullptr) {
    return false;
  }
  *valid_length = 0.0;
  const double KAPPA_THRESH = 0.7;
  for (std::size_t i = 0; i < path.path_points().size(); ++i) {
    PathPoint pt = path.path_points().at(i);
    if (pt.kappa() > KAPPA_THRESH) break;
    *valid_length = pt.s() - path.path_points().front().s();
  }
  return true;
}

bool UpdateHistoryValidLength(const double valid_length,
                              std::vector<double> &history_valid_len) {
  const std::size_t HISTORY_NUM = 10;
  if (history_valid_len.size() > HISTORY_NUM) {
    return false;
  }
  if (history_valid_len.size() == HISTORY_NUM) {
    history_valid_len.erase(history_valid_len.begin());
  }
  history_valid_len.push_back(valid_length);
  return true;
}

bool ExtractObsCorrespondingSRange(
    const std::vector<SLPoint> &curve_trajectory_points,
    const PathObstacleDecision &obs_decision, std::size_t &start_index,
    std::size_t &end_index) {
  if (curve_trajectory_points.empty()) {
    LOG_ERROR("input empty vector, err");
    return false;
  }
  constexpr double expand_s = 1.639;  // m
  double start_s = std::fmax(curve_trajectory_points.front().s(),
                             obs_decision.boundary.start_s() - expand_s);
  double end_s = std::fmin(curve_trajectory_points.back().s(),
                           obs_decision.boundary.end_s() + expand_s);
  // calc check range
  for (std::size_t i = 0; i < curve_trajectory_points.size(); i++) {
    if (curve_trajectory_points[i].s() < start_s) start_index = i;
    if (curve_trajectory_points[i].s() > end_s) {
      end_index = i;
      break;
    }
  }
  return true;
}

bool OnlyProvideOriginReferenceLine(const ReferenceLinePtr &reference_line,
                                    const InsidePlannerData &inside_data,
                                    const FrenetFramePoint &frenet_init_point,
                                    PathData *const path_data) {
  if (path_data == nullptr) {
    LOG_ERROR("intput nullptr, err");
    return false;
  }
  if (!FLAGS_planning_only_provide_reference_line) {
    return true;
  }
  LOG_INFO("provide_origin_reference_line for control testing");

  double path_output_length =
      std::fmax(30.0, inside_data.init_point.velocity() * 8.0);
  LOG_DEBUG("path_output_length {}", path_output_length);
  std::size_t start_index{0}, end_index{0};
  if (!reference_line->GetStartEndIndexBySLength(frenet_init_point.s(),
                                                 path_output_length,
                                                 &start_index, &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return false;
  }
  const auto &points = reference_line->ref_points();

  std::vector<FrenetFramePoint> frenet_path{};
  std::vector<PathPoint> path_points{};
  path_data->mutable_reference_points()->clear();
  double start_s = points.front().s();
  for (std::size_t i = start_index; i <= end_index; ++i) {
    FrenetFramePoint sl_pt(points[i].s(), 0.0, 0.0, 0.0);
    frenet_path.push_back(sl_pt);
    PathPoint path_point;
    path_point.set_s(points[i].s() - start_s);
    path_point.set_x(points[i].x());
    path_point.set_y(points[i].y());
    path_point.set_theta(points[i].heading());
    path_point.set_kappa(points[i].kappa());
    path_point.set_dkappa(points[i].dkappa());
    path_points.push_back(path_point);
    path_data->mutable_reference_points()->push_back(points[i]);
  }

  path_data->set_frenet_path(FrenetFramePath(frenet_path));
  path_data->set_path(DiscretizedPath(path_points));

  if (frenet_path.size() != path_points.size()) {
    LOG_INFO("frenet_path.size() {} != path_points.size() {}",
             frenet_path.size(), path_points.size());
    return false;
  }
  for (std::size_t i = 0; i < frenet_path.size(); ++i) {
    LOG_DEBUG("{}, s {}, l {}, x {}, y {}", i, frenet_path[i].s(),
              frenet_path[i].l(), path_points[i].x(), path_points[i].y());
  }
  return true;
}

void CalcAdcSampleDataForValidLength(
    const std::vector<FrenetFramePoint> &frenet_path,
    const ReferenceLinePtr &reference_line,
    std::vector<Boundary> &adc_boundarys, std::vector<Box2d> &adc_boxes,
    std::vector<Box2d> &adc_boxes_sl,
    std::vector<std::vector<Vec2d>> &adc_box_xy_pts_vec,
    std::vector<std::vector<Vec2d>> &adc_box_sl_pts_vec,
    std::vector<ReferencePoint> &reference_points) {
  const double enlarge_buffer = 0.0;
  const double veh_sl_buffer = FLAGS_planning_path_adc_enlarge_buffer;
  for (auto pt : frenet_path) {
    double s = pt.s();
    double l = pt.l();
    double dl = pt.dl();

    Box2d adc_bounding_box_sl = VehicleParam::Instance()->get_adc_bounding_box(
        {s, l}, std::atan(dl), veh_sl_buffer, veh_sl_buffer, veh_sl_buffer);

    Vec2d adc_position_cartesian;
    SLPoint sl_pt(s, l);
    reference_line->GetPointInCartesianFrame(sl_pt, &adc_position_cartesian);

    ReferencePoint reference_point;
    if (!reference_line->GetNearestRefPoint(s, &reference_point)) {
      LOG_ERROR("GetNearestRefPoint fail");
    }

    double one_minus_kappa_r_d = 1 - reference_point.kappa() * l;
    double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
    double theta = normalize_angle(delta_theta + reference_point.heading());

    Box2d adc_bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
        {adc_position_cartesian.x(), adc_position_cartesian.y()}, theta,
        enlarge_buffer, enlarge_buffer, enlarge_buffer);
    Boundary adc_boundary;
    CalcBoundary(reference_point, adc_bounding_box, sl_pt, adc_boundary);
    reference_points.push_back(reference_point);
    adc_boundarys.push_back(adc_boundary);
    adc_boxes.push_back(adc_bounding_box);
    adc_boxes_sl.push_back(adc_bounding_box_sl);
    std::vector<Vec2d> adc_box_xy_pts;
    adc_bounding_box.get_all_corners(&adc_box_xy_pts);
    adc_box_xy_pts_vec.push_back(adc_box_xy_pts);
    std::vector<Vec2d> adc_box_sl_pts;
    adc_bounding_box_sl.get_all_corners(&adc_box_sl_pts);
    adc_box_sl_pts_vec.push_back(adc_box_sl_pts);
  }
}

void CalcBoundary(const ReferencePoint &reference_point,
                  const Box2d &bounding_box, const SLPoint &center_sl,
                  Boundary &boundary) {
  double heading_diff =
      std::fabs(bounding_box.heading() - reference_point.heading());
  double impact_width =
      bounding_box.half_length() * fabs(std::sin(heading_diff)) +
      bounding_box.half_width() * fabs(std::cos(heading_diff));
  double impact_length =
      bounding_box.half_length() * fabs(std::cos(heading_diff)) +
      bounding_box.half_width() * fabs(std::sin(heading_diff));
  boundary.set_start_s(center_sl.s() - impact_length);
  boundary.set_end_s(center_sl.s() + impact_length);
  boundary.set_start_l(center_sl.l() - impact_width);
  boundary.set_end_l(center_sl.l() + impact_width);
}

bool GetWithoutObsCollisionLength(
    const std::size_t &planner_type,
    const PathObstacleContext &path_obstacle_context,
    const std::vector<Boundary> &adc_boundarys,
    const std::vector<Box2d> &adc_boxes, const std::vector<Box2d> &adc_boxes_sl,
    const std::vector<std::vector<Vec2d>> &adc_box_xy_pts_vec,
    const std::vector<std::vector<Vec2d>> &adc_box_sl_pts_vec,
    const std::vector<FrenetFramePoint> &frenet_path,
    double *const valid_length) {
  if (valid_length == nullptr) return false;
  double tmp_length = 0.0;
  bool use_sl_check_collision = true;
  bool is_collided = false;
  // use different collision check
  if (planner_type == FLAGS_planning_piecewise_planner) {
    for (std::size_t i = 0; i < adc_boundarys.size(); ++i) {
      Boundary adc_boundary = adc_boundarys[i];
      // Box2d adc_box = adc_boxes[i];
      std::vector<Vec2d> adc_box_xy_pts = adc_box_xy_pts_vec[i];
      for (std::size_t j = 0;
           j < path_obstacle_context.obstacle_decision.size(); ++j) {
        auto &obs_decision = path_obstacle_context.obstacle_decision[j];
        // skip virtual obs
        if (obs_decision.obstacle_boundary.obstacle.is_virtual()) continue;
        if (obs_decision.decision_type == Decision::DecisionType::GO_RIGHT ||
            obs_decision.decision_type == Decision::DecisionType::GO_LEFT ||
            obs_decision.decision_type == Decision::DecisionType::YIELD_DOWN) {
          bool overlap_origin_boundary =
              adc_boundary.has_overlap(obs_decision.boundary);
          if (overlap_origin_boundary) {
            std::vector<Vec2d> obs_poly =
                obs_decision.obstacle_boundary.obstacle.polygon().points();
            if (PolygonCollision(adc_box_xy_pts, obs_poly)) {
              LOG_ERROR("obstacle polygon collision, id [{}]",
                        obs_decision.obstacle_boundary.obstacle.id());
              is_collided = true;
              break;
            }
          }
        }
      }
      if (is_collided) break;
      // update valid length
      tmp_length = frenet_path[i].s() - frenet_path.front().s();
    }
  } else {
    for (std::size_t i = 0; i < adc_boundarys.size(); ++i) {
      Boundary adc_boundary = adc_boundarys[i];
      Box2d adc_box = adc_boxes[i];
      std::vector<Vec2d> adc_box_sl_pts = adc_box_sl_pts_vec[i];
      for (std::size_t j = 0;
           j < path_obstacle_context.obstacle_decision.size(); ++j) {
        auto &obs_decision = path_obstacle_context.obstacle_decision[j];
        // skip virtual obs
        if (obs_decision.obstacle_boundary.obstacle.is_virtual()) continue;
        if (adc_box_sl_pts.size() <= 3 ||
            obs_decision.obstacle_box_sl.size() <= 3) {
          use_sl_check_collision = false;
        } else {
          use_sl_check_collision = true;
        }
        if (obs_decision.decision_type == Decision::DecisionType::GO_RIGHT ||
            obs_decision.decision_type == Decision::DecisionType::GO_LEFT ||
            obs_decision.decision_type == Decision::DecisionType::YIELD_DOWN) {
          bool overlap_origin_boundary =
              adc_boundary.has_overlap(obs_decision.boundary);
          if (overlap_origin_boundary) {
            if (use_sl_check_collision) {
              if (PolygonCollision(adc_box_sl_pts,
                                   obs_decision.obstacle_box_sl)) {
                is_collided = true;
                LOG_INFO("obstacle_box_sl collision, id {}",
                         obs_decision.obstacle_boundary.obstacle.id());
                break;
              }
            } else {
              if (adc_box.has_overlap(obs_decision.obstacle_box)) {
                is_collided = true;
                LOG_INFO("obstacle_box collision, id {}",
                         obs_decision.obstacle_boundary.obstacle.id());
                break;
              }
            }
          }
        }
      }
      if (is_collided) break;
      // update valid length
      tmp_length = frenet_path[i].s() - frenet_path.front().s();
    }
  }
  *valid_length = fmin(tmp_length, *valid_length);
  return true;
}

bool GetWithoutRoadBoundCollisionLength(
    const std::vector<ReferencePoint> &reference_points,
    const std::vector<FrenetFramePoint> &frenet_path, double *valid_length) {
  if (valid_length == nullptr) return false;
  double tmp_length = 0.0;
  double adc_width = VehicleParam::Instance()->width();
  double shrink_distance = FLAGS_planning_road_border_remain_threshold * 0.5;
  for (std::size_t i = 0; i < frenet_path.size(); ++i) {
    double right_remain_dist = reference_points[i].right_road_bound() -
                               shrink_distance + frenet_path[i].l() -
                               adc_width / 2.0;
    double left_remain_dist = reference_points[i].left_road_bound() -
                              shrink_distance - frenet_path[i].l() -
                              adc_width / 2.0;
    if (right_remain_dist < 0.0 || left_remain_dist < 0.0) {
      break;
    }
    tmp_length = frenet_path[i].s() - frenet_path.front().s();
  }
  *valid_length = fmin(tmp_length, *valid_length);
  return true;
}

void AdjustInitState(const ReferenceLinePtr &reference_line,
                     const InsidePlannerData &inside_data,
                     const OutsidePlannerData *outside_data, double &init_dl,
                     double &init_ddl) {
  const auto &frenet_init_point = outside_data->frenet_init_point;
  const auto &reference_point = outside_data->init_point_ref_point;
  const auto &init_point = inside_data.init_point;
  LOG_INFO("origin l:{:.4f}, dl:{:.4f}, ddl:{:.4f}", frenet_init_point.l(),
           init_dl, init_ddl);
  const auto &config = config::PlanningConfig::Instance()
                           ->planning_research_config()
                           .third_order_spline_path_optimizer_config;
  LOG_INFO("is_indoor:{}", inside_data.is_indoor);
  if (config.init_test_fix_flag) {
    const auto &ego_car_config =
        neodrive::common::config::CommonConfig::Instance()->ego_car_config();
    double current_heading = inside_data.vel_heading;

    // for judge
    bool is_replan =
        DataCenter::Instance()->master_info().is_use_position_stitch();
    bool is_ref_kappa_high =
        std::abs(reference_point.kappa()) > config.init_test_delta_kappa_thersh;
    LOG_INFO("is_replan:{}, is_ref_kappa_high:{}", is_replan,
             is_ref_kappa_high);

    static char str_buffer[256];
    sprintf(
        str_buffer,
        "[ADJUST_INIT_STATE][is_replan:%d][is_high_kappa:%d, ref_kappa:%.4f, "
        "abs_kappa_thresh:%.4f]",
        is_replan, is_ref_kappa_high, reference_point.kappa(),
        config.init_test_delta_kappa_thersh);
    DataCenter::Instance()->SetMonitorString(str_buffer,
                                             MonitorItemSource::CURVES_PASSING);

    // replan & high kappa
    if (is_replan && is_ref_kappa_high) {
      // due to heading is normalize_angle, do not know the direction
      auto find_edge_sl = [&](bool is_front, SLPoint &edge_sl) {
        double edge_to_planning_center = ego_car_config.front_edge_to_center;
        if (is_front) {
          edge_to_planning_center = ego_car_config.front_edge_to_center;
        } else {
          edge_to_planning_center = ego_car_config.back_edge_to_center;
        }

        double delta_x = edge_to_planning_center * std::cos(current_heading),
               delta_y = edge_to_planning_center * std::sin(current_heading);
        Vec2d edge_xy1{init_point.x() + delta_x, init_point.y() + delta_y};
        Vec2d edge_xy2{init_point.x() - delta_x, init_point.y() - delta_y};
        SLPoint edge_sl1, edge_sl2;
        if (!reference_line->GetPointInFrenetFrameWithLastS(
                edge_xy1, frenet_init_point.s(), &edge_sl1)) {
          LOG_WARN("edge_xy1 get frenet frame fail, is_front:{}", is_front);
          return false;
        }
        if (!reference_line->GetPointInFrenetFrameWithLastS(
                edge_xy2, frenet_init_point.s(), &edge_sl2)) {
          LOG_WARN("edge_xy2 get frenet frame fail, is_front:{}", is_front);
          return false;
        }
        if (is_front) {
          edge_sl = (edge_sl1.s() >= edge_sl2.s()) ? edge_sl1 : edge_sl2;
        } else {
          edge_sl = (edge_sl1.s() >= edge_sl2.s()) ? edge_sl2 : edge_sl1;
        }

        LOG_DEBUG("edge_s1:{:.4f}, edge_s2:{:.4f}, edge_s:{:.4f}", edge_sl1.s(),
                  edge_sl2.s(), edge_sl.s());
        return true;
      };

      // front & back edge
      SLPoint front_edge_sl, back_edge_sl;
      if (!find_edge_sl(true, front_edge_sl)) return;
      if (!find_edge_sl(false, back_edge_sl)) return;
      LOG_INFO(
          "front_edge_sl:({:.4f},{:.4f}), back_edge_sl:({:.4f},{:.4f}), "
          "init_point_s:({:.4f},{:.4f}) ",
          front_edge_sl.s(), front_edge_sl.l(), back_edge_sl.s(),
          back_edge_sl.l(), frenet_init_point.s(), frenet_init_point.l());

      //  spline
      bool can_create_spline = (front_edge_sl.s() > frenet_init_point.s() &&
                                frenet_init_point.s() > back_edge_sl.s());
      tk::spline ego_sl_spline{};
      if (can_create_spline) {
        ego_sl_spline.set_points(
            {back_edge_sl.s(), frenet_init_point.s(), front_edge_sl.s()},
            {back_edge_sl.l(), frenet_init_point.l(), front_edge_sl.l()},
            tk::spline::spline_type::linear);
      }

      // search index
      std::size_t start_index, end_index;
      if (!reference_line->GetStartEndIndexBySLength(
              back_edge_sl.s(), front_edge_sl.s() - back_edge_sl.s(),
              &start_index, &end_index)) {
        LOG_WARN("GetStartEndIndexBySLength fail!");
        return;
      }

      // get heading
      double heading_delta = 1000000;
      std::size_t nearest_heading_index = end_index;
      for (std::size_t i = start_index; i <= end_index; ++i) {
        auto ref_point = reference_line->ref_points().at(i);
        LOG_INFO(
            "  index:{}, s:{:.4f}, heading:{:.4f}, heading_delt:{:.4f}, "
            "normalize_heading_delt:{:.4f}, abs_normalize_heading_delt:{:.4f}",
            i, ref_point.s(), ref_point.heading(),
            ref_point.heading() - current_heading,
            normalize_angle(ref_point.heading() - current_heading),
            std::abs(normalize_angle(ref_point.heading() - current_heading)));

        if (std::abs(normalize_angle(ref_point.heading() - current_heading)) <
            heading_delta) {
          heading_delta =
              std::abs(normalize_angle(ref_point.heading() - current_heading));
          nearest_heading_index = i;
        }
      }
      auto fix_ref_pt =
          can_create_spline
              ? reference_line->ref_points().at(nearest_heading_index)
              : reference_point;

      double fix_l = can_create_spline ? ego_sl_spline(fix_ref_pt.s())
                                       : frenet_init_point.l();

      LOG_INFO(
          "can_create_spline:{}, nearest_index:{}, nearest_heading:{:.4f}, "
          "current_heading:{:.4f}, fix_ref_pt_s:{:.4f}, fix_l:{:.4f}",
          can_create_spline, nearest_heading_index,
          reference_line->ref_points().at(nearest_heading_index).heading(),
          current_heading, fix_ref_pt.s(), fix_l);

      init_dl = SLAnalyticTransformation::calculate_lateral_derivative(
          fix_ref_pt.heading(), init_point.theta(), frenet_init_point.l(),
          fix_ref_pt.kappa());
      init_ddl = 0.0;
      LOG_INFO("adjust l:{:.4f}, dl:{:.4f}, ddl:{:.4f}", fix_l, init_dl,
               init_ddl);
    }
  }
}

}  // namespace path_planner_common
}  // namespace planning
}  // namespace neodrive

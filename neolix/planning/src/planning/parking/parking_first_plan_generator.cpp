#include "parking_first_plan_generator.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {
ParkingFirstPlanGenerator::ParkingFirstPlanGenerator() {
  // TODO(wyc): smart pointer assign
  park_optimizer_ =
      std::make_unique<ParkingTrajectoryOptimizer>(parking_config_);

  iterative_anchoring_smoother_ = std::make_unique<IterativeAnchoringSmoother>(
      parking_config_.iterative_anchor_config(),
      parking_config_.common_config());
}

// TODO(wyc): no need to manage manually
ParkingFirstPlanGenerator::~ParkingFirstPlanGenerator() { Reset(); }

void ParkingFirstPlanGenerator::Reset() {
  park_in_out_ = false;
  park_spot_type_ = 0;
  head_tail_in_ = 0;
  trajectory_points_.clear();
}
std::vector<TrajectoryPoint> ParkingFirstPlanGenerator::get_results() {
  return trajectory_points_;
}

bool ParkingFirstPlanGenerator::Generate(
    TrajectoryPoint& start_pos, /*ReferenceLinePtr& parking_ref_line,*/ // mchan doubt : parking_ref_line is unnesesarry
    ParkingDataInfo& park_data_info, bool park_in_out
    /*std::array<Vec2d, 4>& spot_corners, bool park_in_out,
    int park_spot_type, int head_tail_in*/) {
  // if (parking_ref_line == nullptr) {
  //   LOG_ERROR("parking_ref_line is nullptr");
  //   return false;
  // }
  // if (parking_ref_line->ref_points().size() < 4) {
  //   LOG_ERROR("parking_ref_line size < 4");
  //   return false;
  // }
  // if (spot_corners.size() != 4) {
  //   LOG_ERROR("spot_corners size != 4");
  //   return false;
  // }

  // parking_ref_line_ = parking_ref_line;
  park_data_info_ = park_data_info;
  // start_pos_ = start_pos;
  park_in_out_ = park_in_out;
  // park_spot_type_ = park_spot_type;
  // head_tail_in_ = head_tail_in;

  // // common setting
  // park_data_info_.set_curr_gps({start_pos_.x(), start_pos_.y()});
  // park_data_info_.set_curr_heading(start_pos_.theta());
  // park_data_info_.set_final_pos_flag(false);
  // if (park_in_out && !ParkOutStartFinalPosAdjust(
  //                        start_pos_, parking_ref_line, spot_corners)) {
  //   LOG_ERROR("Failed to init park out parameters");
  //   return false;
  // }

  // park_spot_decider_.set_parking_data_info(&park_data_info_,
  // &parking_config_); if (!park_spot_decider_.process(parking_ref_line_,
  // spot_corners)) {
  //   LOG_ERROR("park_spot_decider_.process failed");
  //   return false;
  // }

  if (!park_in_out) {
    if (!GenerateParkIn()) {
      LOG_ERROR("Generate_Park_in failed");
      return false;
    }
  } else {
    if (!GenerateParkOut()) {
      LOG_ERROR("Generate_Park_out failed");
      return false;
    }
  }

  // LogParkingResults(trajectory_points_);

  return true;
}

bool ParkingFirstPlanGenerator::GenerateParkIn() {
  ParkingFirstPlanConfig firt_config =
      parking_config_.first_plan_park_in_config();
  bool ret = false;
  if (firt_config.first_plan_type == "geometric") {
    if (firt_config.geo_use_smoothing) {
      LOG_INFO("geometric_park_in_with_smooth");
      ret = GeometricParkInWithSmooth();
    } else {
      LOG_INFO("geometric_park_in_without_smooth");
      ret = GeometricParkInWithoutSmooth();
    }
  } else if (firt_config.first_plan_type == "hybrid a") {
    LOG_INFO("hybrid_a_star_park_in");
    ret = HybridAStarParkIn();
  } else {
    LOG_ERROR("park in planner type not defined [{}]",
              firt_config.first_plan_type);
  }
  if (!ret) {
    LOG_ERROR("Generate_Park_in failed, type: {}", firt_config.first_plan_type);
    return false;
  }
  return true;
}

bool ParkingFirstPlanGenerator::GeometricParkInWithoutSmooth() {
  TrajectoryPoint curr_pos = park_data_info_.CurrPosOnAnchor();
  TrajectoryPoint carport_pos = park_data_info_.FinalPosOnAnchor();
  std::vector<Vec2d> carport_coor = park_data_info_.LimitCarportCoorAnchor();
  std::vector<Vec2d> road_oppsite_coor =
      park_data_info_.LimitRoadOppsiteAnchor();
  std::vector<Vec2d> road_rear_coor = park_data_info_.LimitRoadRearAnchor();
  std::vector<Vec2d> road_front_coor = park_data_info_.LimitRoadFrontAnchor();

  std::vector<TrajectoryPoint> candidate_pos;

  switch (park_data_info_.ParkSpotType()) {
    case 0:
      LOG_ERROR("parking spot type is not defined");
      break;
    case 1:
      if (!ppp_in_.Initialize(curr_pos, carport_pos, carport_coor,
                              road_oppsite_coor, road_rear_coor,
                              road_front_coor)) {
        LOG_ERROR("ppp_in init failed");
        break;
      }
      if (!ppp_in_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                            park_data_info_.HeadTailIn())) {
        LOG_ERROR("ppp_in Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 2:
      if (!pp_in_.Initialize(curr_pos, carport_pos, carport_coor,
                             road_oppsite_coor, road_rear_coor,
                             road_front_coor)) {
        LOG_ERROR("pp_in init failed");
        break;
      }
      if (!pp_in_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                           park_data_info_.HeadTailIn())) {
        LOG_ERROR("pp_in Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 3:
      LOG_WARN("inclined not developed yet");
      break;
    default:
      LOG_ERROR("parking spot type is not defined");
      break;
  }
  if (candidate_pos.empty()) {
    LOG_ERROR("geometric_park_in_without_smooth failed");
    return false;
  }
  trajectory_points_.clear();
  double vel_x = park_data_info_.AnchorPointCoordinate().x();
  double vel_y = park_data_info_.AnchorPointCoordinate().y();
  double vel_yaw = park_data_info_.AnchorPointCoordinate().heading();
  double tmp_x = 0.0, tmp_y = 0.0, tmp_yaw = 0.0;

  TrajectoryPoint tmp_pt;
  for (size_t i = 0; i < candidate_pos.size(); ++i) {
    tmp_pt = candidate_pos[i];
    vehicle2earth(vel_x, vel_y, vel_yaw, tmp_pt.x(), tmp_pt.y(), tmp_pt.theta(),
                  tmp_x, tmp_y, tmp_yaw);
    tmp_pt.set_x(tmp_x);
    tmp_pt.set_y(tmp_y);
    tmp_pt.set_theta(tmp_yaw);
    trajectory_points_.push_back(tmp_pt);
  }

  return true;
}

bool ParkingFirstPlanGenerator::GeometricParkInWithSmooth() {
  TrajectoryPoint curr_pos = park_data_info_.CurrPosOnAnchor();
  TrajectoryPoint carport_pos = park_data_info_.FinalPosOnAnchor();
  std::vector<Vec2d> carport_coor = park_data_info_.LimitCarportCoorAnchor();
  std::vector<Vec2d> road_oppsite_coor =
      park_data_info_.LimitRoadOppsiteAnchor();
  std::vector<Vec2d> road_rear_coor = park_data_info_.LimitRoadRearAnchor();
  std::vector<Vec2d> road_front_coor = park_data_info_.LimitRoadFrontAnchor();

  std::vector<TrajectoryPoint> candidate_pos;

  switch (park_data_info_.ParkSpotType()) {
    case 0:
      LOG_ERROR("parking spot type is not defined");
      break;
    case 1:
      if (!ppp_in_.Initialize(curr_pos, carport_pos, carport_coor,
                              road_oppsite_coor, road_rear_coor,
                              road_front_coor)) {
        LOG_ERROR("ppp_in init failed");
        break;
      }
      if (!ppp_in_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                            park_data_info_.HeadTailIn())) {
        LOG_ERROR("ppp_in Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 2:
      if (!pp_in_.Initialize(curr_pos, carport_pos, carport_coor,
                             road_oppsite_coor, road_rear_coor,
                             road_front_coor)) {
        LOG_ERROR("pp_in init failed");
        break;
      }
      if (!pp_in_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                           park_data_info_.HeadTailIn())) {
        LOG_ERROR("pp_in Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 3:
      LOG_WARN("inclined not developed yet");
      break;
    default:
      LOG_ERROR("parking spot type is not defined");
      break;
  }
  if (candidate_pos.empty()) {
    LOG_ERROR("geometric_park_in_without_smooth failed");
    return false;
  }

  // smoothing
  std::vector<std::vector<TrajectoryPoint>> part_traj;
  if (!TrajectoryPartition(candidate_pos, part_traj)) {
    LOG_ERROR("TrajectoryPartition failed");
    return false;
  }
  if (part_traj.empty()) {
    LOG_ERROR("TrajectoryPartition failed");
    return false;
  }
  candidate_pos.clear();
  DiscretizedTrajectory smoothed_trajectory;
  for (size_t i = 0; i < part_traj.size(); ++i) {
    if (part_traj[i].empty()) {
      LOG_ERROR("TrajectoryPartition [{}] is empty", i);
      return false;
    }
    if (!iterative_anchoring_smoother_->Smooth(
            part_traj[i], park_data_info_.ObstaclesVerticesVec(),
            &smoothed_trajectory)) {
      LOG_ERROR("iterative_anchoring_smoother failed, use raw data");
      // return false;
      double front_s = part_traj[i].front().s();
      for (size_t j = 0; j < part_traj[i].size(); ++j) {
        smoothed_trajectory.mutable_trajectory_points()->push_back(
            part_traj[i][j]);
        smoothed_trajectory.mutable_trajectory_points()->back().set_s(
            smoothed_trajectory.mutable_trajectory_points()->back().s() -
            front_s);
      }
    }
    double last_s = 0.0;
    if (i != 0 && !candidate_pos.empty()) {
      last_s = candidate_pos.back().s();
    }
    for (size_t j = 0; j < smoothed_trajectory.trajectory_points().size();
         ++j) {
      TrajectoryPoint tra_pt;
      if (!smoothed_trajectory.trajectory_point_at(j, tra_pt)) {
        LOG_ERROR("trajectory_point_at {} failed", j);
        continue;
      }
      candidate_pos.push_back(tra_pt);
      candidate_pos.back().set_s(candidate_pos.back().s() + last_s);
    }
  }

  trajectory_points_.clear();

  // transform to anchor
  double vel_x = park_data_info_.AnchorPointCoordinate().x();
  double vel_y = park_data_info_.AnchorPointCoordinate().y();
  double vel_yaw = park_data_info_.AnchorPointCoordinate().heading();
  double tmp_x = 0.0, tmp_y = 0.0, tmp_yaw = 0.0;
  TrajectoryPoint tmp_pt;
  for (size_t i = 0; i < candidate_pos.size(); ++i) {
    tmp_pt = candidate_pos[i];
    vehicle2earth(vel_x, vel_y, vel_yaw, tmp_pt.x(), tmp_pt.y(), tmp_pt.theta(),
                  tmp_x, tmp_y, tmp_yaw);
    tmp_pt.set_x(tmp_x);
    tmp_pt.set_y(tmp_y);
    tmp_pt.set_theta(tmp_yaw);
    trajectory_points_.push_back(tmp_pt);
  }
  return true;
}

bool ParkingFirstPlanGenerator::HybridAStarParkIn() {
  if (park_optimizer_ == nullptr) {
    LOG_ERROR("park_optimizer_ is null");
    return false;
  }

  bool ret = park_optimizer_->Generate(
      park_data_info_.CurrPosOnAnchor(), park_data_info_.ParkingEndPose(),
      park_data_info_.ROIXyBoundary(), park_data_info_.OriginHeading(),
      park_data_info_.OriginPoint(), park_data_info_.ObstaclesEdgesNum(),
      park_data_info_.ObstaclesA(), park_data_info_.ObstaclesB(),
      park_data_info_.ObstaclesVerticesVec(),
      park_data_info_.AnchorPointCoordinate());
  if (ret != true) {
    LOG_ERROR("failed to generate trajectory");
    return false;
  }
  trajectory_points_.clear();
  trajectory_points_ = park_optimizer_->GetOptimizedTrajectory();

  return true;
}

bool ParkingFirstPlanGenerator::GenerateParkOut() {
  ParkingFirstPlanConfig firt_config =
      parking_config_.first_plan_park_out_config();
  bool ret = false;
  if (firt_config.first_plan_type == "geometric") {
    if (firt_config.geo_use_smoothing) {
      LOG_INFO("geometric_park_out_with_smooth");
      ret = GeometricParkOutWithSmooth();
    } else {
      LOG_INFO("geometric_park_out_without_smooth");
      ret = GeometricParkOutWithoutSmooth();
    }
  } else if (firt_config.first_plan_type == "hybrid a") {
    LOG_INFO("hybrid_a_start_park_out");
    ret = HybridAStarParkOut();
  } else {
    LOG_ERROR("park out planner type not defined [{}]",
              firt_config.first_plan_type);
  }
  if (!ret) {
    LOG_ERROR("Generate_Park_out failed, type: {}",
              firt_config.first_plan_type);
    return false;
  }
  return true;
}
bool ParkingFirstPlanGenerator::GeometricParkOutWithoutSmooth() {
  std::vector<Vec2d> carport_coor = park_data_info_.LimitCarportCoorAnchor();
  std::vector<Vec2d> road_oppsite_coor =
      park_data_info_.LimitRoadOppsiteAnchor();
  std::vector<Vec2d> road_rear_coor = park_data_info_.LimitRoadRearAnchor();
  std::vector<Vec2d> road_front_coor = park_data_info_.LimitRoadFrontAnchor();

  // curr_pos find a reference pt in line
  TrajectoryPoint curr_pos = park_data_info_.CurrPosOnAnchor();
  // final pos should be curr_pos
  TrajectoryPoint carport_pos = park_data_info_.FinalPosOnAnchor();

  std::vector<TrajectoryPoint> candidate_pos;

  switch (park_data_info_.ParkSpotType()) {
    case 0:
      LOG_ERROR("parking spot type is not defined");
      break;
    case 1:
      if (!ppp_out_.Initialize(curr_pos, carport_pos, carport_coor,
                               road_oppsite_coor, road_rear_coor,
                               road_front_coor)) {
        LOG_ERROR("ppp_out init failed");
        break;
      }
      if (!ppp_out_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                             park_data_info_.HeadTailIn())) {
        LOG_ERROR("ppp_out Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 2:
      if (!pp_out_.Initialize(curr_pos, carport_pos, carport_coor,
                              road_oppsite_coor, road_rear_coor,
                              road_front_coor)) {
        LOG_ERROR("pp_out init failed");
        break;
      }
      if (!pp_out_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                            park_data_info_.HeadTailIn())) {
        LOG_ERROR("pp_out Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 3:
      LOG_WARN("inclined not developed yet");
      break;
    default:
      LOG_ERROR("parking spot type is not defined");
      break;
  }
  if (candidate_pos.empty()) {
    LOG_ERROR("geometric_park_out_without_smooth failed");
    return false;
  }
  trajectory_points_.clear();
  double vel_x = park_data_info_.AnchorPointCoordinate().x();
  double vel_y = park_data_info_.AnchorPointCoordinate().y();
  double vel_yaw = park_data_info_.AnchorPointCoordinate().heading();
  double tmp_x = 0.0, tmp_y = 0.0, tmp_yaw = 0.0;

  TrajectoryPoint tmp_pt;
  for (size_t i = 0; i < candidate_pos.size(); ++i) {
    tmp_pt = candidate_pos[i];
    vehicle2earth(vel_x, vel_y, vel_yaw, tmp_pt.x(), tmp_pt.y(), tmp_pt.theta(),
                  tmp_x, tmp_y, tmp_yaw);
    tmp_pt.set_x(tmp_x);
    tmp_pt.set_y(tmp_y);
    tmp_pt.set_theta(tmp_yaw);
    trajectory_points_.push_back(tmp_pt);
  }
  return true;
}

bool ParkingFirstPlanGenerator::GeometricParkOutWithSmooth() {
  std::vector<Vec2d> carport_coor = park_data_info_.LimitCarportCoorAnchor();
  std::vector<Vec2d> road_oppsite_coor =
      park_data_info_.LimitRoadOppsiteAnchor();
  std::vector<Vec2d> road_rear_coor = park_data_info_.LimitRoadRearAnchor();
  std::vector<Vec2d> road_front_coor = park_data_info_.LimitRoadFrontAnchor();

  // curr_pos find a reference pt in line
  TrajectoryPoint curr_pos = park_data_info_.CurrPosOnAnchor();
  // final pos should be curr_pos
  TrajectoryPoint carport_pos = park_data_info_.FinalPosOnAnchor();

  std::vector<TrajectoryPoint> candidate_pos;

  switch (park_data_info_.ParkSpotType()) {
    case 0:
      LOG_ERROR("parking spot type is not defined");
      break;
    case 1:
      if (!ppp_out_.Initialize(curr_pos, carport_pos, carport_coor,
                               road_oppsite_coor, road_rear_coor,
                               road_front_coor)) {
        LOG_ERROR("ppp_out init failed");
        break;
      }
      if (!ppp_out_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                             park_data_info_.HeadTailIn())) {
        LOG_ERROR("ppp_out Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 2:
      if (!pp_out_.Initialize(curr_pos, carport_pos, carport_coor,
                              road_oppsite_coor, road_rear_coor,
                              road_front_coor)) {
        LOG_ERROR("pp_out init failed");
        break;
      }
      if (!pp_out_.Generate(park_data_info_.CurrPosOnAnchor(), candidate_pos,
                            park_data_info_.HeadTailIn())) {
        LOG_ERROR("pp_out Generate failed");
        candidate_pos.clear();
        break;
      }
      break;
    case 3:
      LOG_WARN("inclined not developed yet");
      break;
    default:
      LOG_ERROR("parking spot type is not defined");
      break;
  }
  if (candidate_pos.empty()) {
    LOG_ERROR("geometric_park_out_without_smooth failed");
    return false;
  }
  // smoothing
  std::vector<std::vector<TrajectoryPoint>> part_traj;
  if (!TrajectoryPartition(candidate_pos, part_traj)) {
    LOG_ERROR("TrajectoryPartition failed");
    return false;
  }
  if (part_traj.empty()) {
    LOG_ERROR("TrajectoryPartition failed");
    return false;
  }
  candidate_pos.clear();
  DiscretizedTrajectory smoothed_trajectory;
  for (size_t i = 0; i < part_traj.size(); ++i) {
    if (part_traj[i].empty()) {
      LOG_ERROR("TrajectoryPartition [{}] is empty", i);
      return false;
    }
    if (!iterative_anchoring_smoother_->Smooth(
            part_traj[i], park_data_info_.ObstaclesVerticesVec(),
            &smoothed_trajectory)) {
      LOG_ERROR("iterative_anchoring_smoother failed, use raw data");
      // return false;
      double front_s = part_traj[i].front().s();
      for (size_t j = 0; j < part_traj[i].size(); ++j) {
        smoothed_trajectory.mutable_trajectory_points()->push_back(
            part_traj[i][j]);
        smoothed_trajectory.mutable_trajectory_points()->back().set_s(
            smoothed_trajectory.mutable_trajectory_points()->back().s() -
            front_s);
      }
    }
    double last_s = 0.0;
    if (i != 0 && !candidate_pos.empty()) {
      last_s = candidate_pos.back().s();
    }
    for (size_t j = 0; j < smoothed_trajectory.trajectory_points().size();
         ++j) {
      TrajectoryPoint tra_pt;
      if (!smoothed_trajectory.trajectory_point_at(j, tra_pt)) {
        LOG_ERROR("trajectory_point_at {} failed", j);
        continue;
      }
      candidate_pos.push_back(tra_pt);
      candidate_pos.back().set_s(candidate_pos.back().s() + last_s);
    }
  }

  trajectory_points_.clear();

  // transform to anchor
  double vel_x = park_data_info_.AnchorPointCoordinate().x();
  double vel_y = park_data_info_.AnchorPointCoordinate().y();
  double vel_yaw = park_data_info_.AnchorPointCoordinate().heading();
  double tmp_x = 0.0, tmp_y = 0.0, tmp_yaw = 0.0;
  TrajectoryPoint tmp_pt;
  for (size_t i = 0; i < candidate_pos.size(); ++i) {
    tmp_pt = candidate_pos[i];
    vehicle2earth(vel_x, vel_y, vel_yaw, tmp_pt.x(), tmp_pt.y(), tmp_pt.theta(),
                  tmp_x, tmp_y, tmp_yaw);
    tmp_pt.set_x(tmp_x);
    tmp_pt.set_y(tmp_y);
    tmp_pt.set_theta(tmp_yaw);
    trajectory_points_.push_back(tmp_pt);
  }

  return true;
}

bool ParkingFirstPlanGenerator::HybridAStarParkOut() {
  if (park_optimizer_ == nullptr) {
    LOG_ERROR("park_optimizer_ is null");
    return false;
  }

  bool ret = park_optimizer_->Generate(
      park_data_info_.CurrPosOnAnchor(), park_data_info_.ParkingEndPose(),
      park_data_info_.ROIXyBoundary(), park_data_info_.OriginHeading(),
      park_data_info_.OriginPoint(), park_data_info_.ObstaclesEdgesNum(),
      park_data_info_.ObstaclesA(), park_data_info_.ObstaclesB(),
      park_data_info_.ObstaclesVerticesVec(),
      park_data_info_.AnchorPointCoordinate());
  if (ret != true) {
    LOG_ERROR("failed to generate trajectory");
    return false;
  }
  trajectory_points_.clear();
  trajectory_points_ = park_optimizer_->GetOptimizedTrajectory();
  return true;
}

// bool ParkingFirstPlanGenerator::ParkOutStartFinalPosAdjust(
//     TrajectoryPoint& start_pos, ReferenceLinePtr& parking_ref_line,
//     std::array<Vec2d, 4>& spot_corners) {
//   // change start pos to final pos
//   park_data_info_.set_final_pos_flag(true);
//   park_data_info_.set_pre_defined_final_pos(start_pos);

//   // get start pos from reference line using spot_corners

//   ReferencePoint tmp_refer;
//   bool bfind = false;
//   for (size_t i = 0; i < spot_corners.size(); ++i) {
//     Vec2d one_corner(spot_corners[0].x(), spot_corners[0].y());
//     if (!parking_ref_line->GetNearestRefPoint(one_corner, tmp_refer)) {
//       continue;
//     }
//     bfind = true;
//     break;
//   }
//   if (!bfind) {
//     LOG_ERROR("Failed to find appropriate start pos on refer_line");
//     return false;
//   }
//   //
//   park_data_info_.set_curr_gps({tmp_refer.x(), tmp_refer.y()});
//   park_data_info_.set_curr_heading(tmp_refer.heading());

//   return true;
// }

bool ParkingFirstPlanGenerator::TrajectoryPartition(
    std::vector<TrajectoryPoint>& traj,
    std::vector<std::vector<TrajectoryPoint>>& partition_traj) {
  if (traj.empty()) return false;
  int direction = traj.front().direction();
  std::vector<TrajectoryPoint> tmp_tra;
  tmp_tra.clear();
  partition_traj.clear();
  for (size_t i = 0; i < traj.size(); ++i) {
    if (direction != traj[i].direction()) {
      partition_traj.push_back(tmp_tra);
      tmp_tra.clear();
      // if (i >= 1) tmp_tra.push_back(traj[i - 1]); // mchan comment for test
      tmp_tra.push_back(traj[i]);
      direction = traj[i].direction();
    }
    tmp_tra.push_back(traj[i]);
  }
  if (!tmp_tra.empty() && tmp_tra.size() >= 2) {
    partition_traj.push_back(tmp_tra);
  }

  return true;
}

void ParkingFirstPlanGenerator::LogReferLineBound() {
  // x,y,theta, curv, s, left_bound, right_bound
  // test refer line function
  FILE* save;
  if (save = fopen("../test_data/output/parking/refer_test.csv", "w")) {
    for (size_t i = 0; i < parking_ref_line_->ref_points().size(); ++i) {
      auto pt = parking_ref_line_->ref_points().at(i);
      fprintf(save, "%.4f,%.4f,%.4f,0,0,%.4f,%.4f, %.4f,\n ", pt.x(), pt.y(),
              pt.heading(), pt.s(), pt.left_bound(), pt.right_bound());
    }
  }
  fclose(save);
}
void ParkingFirstPlanGenerator::LogParkingResults(
    std::vector<TrajectoryPoint>& traj) {
  FILE* save;
  if (save = fopen("../test_data/output/parking/ParkingResults.csv", "w")) {
    // x,y,theta,kappa,s, direction, index, corners

    for (size_t i = 0; i < traj.size(); ++i) {
      fprintf(save, "%.4f,%.4f,%.4f,%.4f, %.4f, %d,%d,", traj[i].x(),
              traj[i].y(), traj[i].theta(), traj[i].kappa(), traj[i].s(),
              traj[i].direction(), traj[i].segment_index());
      Box2d bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
          {traj[i].x(), traj[i].y()}, traj[i].theta());
      std::vector<Vec2d> corners;
      bounding_box.get_all_corners(&corners);
      for (size_t j = 0; j < corners.size(); ++j) {
        fprintf(save, "%.4f,%.4f,", corners[j].x(), corners[j].y());
      }
      fprintf(save, "\n");
    }
    fclose(save);
  }
}
}  // namespace planning
}  // namespace neodrive

#pragma once

#include "obstacle_map.h"

#include <bitset>

#include "common/visualizer_event/visualizer_event.h"
#include "src/common/coordinate/coodrdinate_convertion.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {
namespace obsmap {

GlobalObsInfo::GlobalObsInfo(const localmap::ObsState &states) {
  info = nullptr;
  trajectory.push_front(State(
      0, {states[8], states[9], states[0], states[1], states[2], states[3]},
      states[7], states[5], nullptr));
}
GlobalObsInfo::GlobalObsInfo(ObsFrameContainer::ObsInfo *binfo,
                             math::Polygon *bNode,
                             const localmap::ObsState &states)
    : info{binfo},
      id{binfo->obs_id},
      birth{obsmap::ObstacleMap::Instance()->time()} {
  Update(binfo, bNode, states, false);
}
void GlobalObsInfo::Update(ObsFrameContainer::ObsInfo *binfo,
                           math::Polygon *bNode,
                           const localmap::ObsState &states, bool do_prediect) {
  if (trajectory.size() >= obsmap::ObstacleMap::Instance()->max_size()) {
    trajectory.pop_back();
    prediction.pop_back();
  }
  uint64_t lid = 0;
  double heading_dev = 0.0;
  if (binfo) {
    lid = binfo->matched_lane_id;
    heading_dev = binfo->matched_lane_heading_deviation;
  }
  trajectory.push_front(
      State(obsmap::ObstacleMap::Instance()->time(),
            {states[8], states[9], states[0], states[1], states[2], states[3]},
            states[7], states[5], bNode, lid, heading_dev));
  if (do_prediect) Predict(bNode == nullptr);
  last_time = obsmap::ObstacleMap::Instance()->time();
}
void GlobalObsInfo::Predict(bool ego) {
  auto &kinematics =
      !ego ? obsmap::ObstacleMap::Instance()->GetObsKinematics(id)
           : *obsmap::ObstacleMap::Instance()->mutable_ego_kinematics();
  std::array<double, 6> ctm_state{
      trajectory.front().x[0], trajectory.front().x[1], trajectory.front().x[2],
      trajectory.front().x[3], trajectory.front().w,    0.0};

  prediction.push_front({});
  prediction.front().emplace("cam",
                             kinematics.ca_kf->Process(trajectory.front().x));
  prediction.front().emplace("cvm",
                             kinematics.cv_kf->Process(trajectory.front().x));
  prediction.front().emplace("ctm", kinematics.ct_kf->Process(ctm_state));

  auto &pred_cam = prediction.front().at("cam");
  auto &pred_cvm = prediction.front().at("cvm");
  auto &pred_ctm = prediction.front().at("ctm");
  pred_cam.conservativeResize(8);
  pred_cvm.conservativeResize(8);
  auto cen_ca = obsmap::ObstacleMap::Instance()->Utm2Odom(
      DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose(),
      DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose(),
      {pred_cam(0, 0), pred_cam(1, 0)}, atan2(pred_cam(1, 0), pred_cam(0, 0)));
  auto cen_cv = obsmap::ObstacleMap::Instance()->Utm2Odom(
      DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose(),
      DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose(),
      {pred_cvm(0, 0), pred_cvm(1, 0)}, atan2(pred_cvm(1, 0), pred_cvm(0, 0)));
  pred_cam(6, 0) = cen_ca.first[0];
  pred_cam(7, 0) = cen_ca.first[1];
  pred_cvm(6, 0) = cen_cv.first[0];
  pred_cvm(7, 0) = cen_cv.first[1];
}
bool GlobalObsInfo::Metabolism() {
  cycle = obsmap::ObstacleMap::Instance()->time() - last_time;
  if (cycle > obsmap::ObstacleMap::Instance()->kLifeSpan) return false;
  while (!trajectory.empty()) {
    if (obsmap::ObstacleMap::Instance()->time() - trajectory.back().frame_id >
        obsmap::ObstacleMap::Instance()->max_size()) {
      trajectory.pop_back();
    } else {
      break;
    }
  }
  return true;
}

ObstacleMap::ObstacleMap() : ObsMap(31, false) {
  ego_state_.dr_accleration = {0, 0};
  ego_state_.dr_est_velocity[0][0] = ego_state_.dr_velocity[0];
  ego_state_.dr_est_velocity[1][0] = ego_state_.dr_velocity[1];
  ego_state_.ego_state = {ego_state_.dr_velocity[0],
                          ego_state_.dr_velocity[1],
                          ego_state_.dr_accleration[0],
                          ego_state_.dr_accleration[1],
                          0,
                          0,
                          0,
                          0.1,
                          0,
                          0};
  ego_state_.ego_state = {ego_state_.dr_velocity[0],
                          ego_state_.dr_velocity[1],
                          ego_state_.dr_accleration[0],
                          ego_state_.dr_accleration[1],
                          0,
                          0,
                          0,
                          0.1,
                          0,
                          0};
  ego_ = std::make_shared<GlobalObsInfo>(ego_state_.ego_state);
  ego_kinematics_.obs_info = ego_;
  ego_kinematics_.cv_kf = math::CVM(ego_->trajectory.front().x, 0.1, 0.0);
  ego_kinematics_.ca_kf = math::CAM(ego_->trajectory.front().x, 0.1, 0.0);
  ego_kinematics_.ct_kf = math::CTM(ego_->trajectory.front().x, 0.1, 0.0);
};

void ObstacleMap::VisObstacles(int frame) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto set_rgba = [](auto &event, auto &rgb) {
    event->set_type(visualizer::Event::k3D);
    event->add_attribute(visualizer::Event::kOdom);
    event->mutable_color()->set_r(rgb[0]);
    event->mutable_color()->set_g(rgb[1]);
    event->mutable_color()->set_b(rgb[2]);
    event->mutable_color()->set_a(rgb[3]);
  };

  auto FillPt = [](auto t, auto x, auto y, auto z) {
    t->set_x(x), t->set_y(y), t->set_z(z);
  };

  auto FillPt2 = [](auto t, auto pt) {
    t->set_x(pt[0]), t->set_y(pt[1]), t->set_z(0);
  };

  auto &currentFrame = At(frame);
  if (currentFrame.Empty()) return;

  std::map<std::string, visualizer::Event *> events;
  std::vector<std::pair<std::string, std::array<double, 4>>> event_names{
      {"__front_ROI", {0.4, 0.152, 0.0, 0.5}},
      {"__behind_ROI", {0.0, 0.297, 0.4, 0.5}},
      {"__rotational_ROI", {0.0, 0.006, 0.4, 0.5}},
      {"__Localmap", {1.0, 0.705, 0.0, 0.9}},
      {"__Localmap_CA", {1.0, 0.0, 0.0, 0.9}},
      {"__Localmap_CV", {0.0, 0.0, 1.0, 0.9}},
      {"__Localmap_CT", {1.0, 0.0, 1.0, 0.9}},
      {"__CurFrame", {0.0, 0.297, 0.4, 0.8}},
      {"$$AgentState", {0.0, 0.297, 0.4, 0.8}}};
  for (auto i = 0; i < event_names.size(); i++) {
    events.insert(std::make_pair(
        event_names[i].first,
        vis::EventSender::Instance()->GetEvent(event_names[i].first)));
    set_rgba(events[event_names[i].first], event_names[i].second);
  }

  int cnt = 0;
  auto planning_map_ptr = PlanningMap::Instance();
  auto data_center_ptr = DataCenter::Instance();
  std::vector<std::weak_ptr<GlobalObsInfo>> vismap{};
  vismap.push_back(ego_);
  for (auto &ptr : localmap_) {
    vismap.push_back(ptr);
  }
  for (int idx = 0; idx < vismap.size(); idx++) {
    auto ptr = vismap[idx].lock();
    if (!ptr) {
      continue;
    }
    auto &traj = ptr->trajectory;
    auto &pred = ptr->prediction;
    LOG_INFO("{} #{}: {}, {}", idx == 0 ? "ego" : "obs", ptr->id, traj.size(),
             pred.size());
    for (int i = 0; i < traj.size(); i++) {
      auto sphere = events["__Localmap"]->add_sphere();
      auto sphere_CA = events["__Localmap_CA"]->add_sphere();
      auto sphere_CV = events["__Localmap_CV"]->add_sphere();
      auto sphere_CT = events["__Localmap_CT"]->add_sphere();
      auto polyline = events["__Localmap"]->add_polyline();
      auto polyline_CA = events["__Localmap_CA"]->add_polyline();
      auto polyline_CV = events["__Localmap_CV"]->add_polyline();
      auto polyline_CT = events["__Localmap_CT"]->add_polyline();
      math::AD2 cen{0, 0};
      if (idx > 0) {
        cen[0] = traj[i].shape->aabox().cen[0];
        cen[1] = traj[i].shape->aabox().cen[1];
      }
      const auto &state = traj[i].x;
      if (i == 0) {
        Eigen::VectorXd state_tmp(7);
        state_tmp << state[0], state[1], state[2], state[3], state[4], state[5],
            traj[0].w;

        /* draw */
        auto &pred_cam = pred.front().at("cam");
        auto &pred_cvm = pred.front().at("cvm");
        auto &pred_ctm = pred.front().at("ctm");

        if (idx > 0) {
          LOG_INFO("obs #{} | lane:{}", ptr->id,
                   planning_map_ptr->GetHashIdString(traj[i].matched_lane_id));
          /* state */
          auto agent_matched_lane_id = events["$$AgentState"]->add_text();

          agent_matched_lane_id->set_text(
              std::to_string(ptr->id) + "\n" +
              planning_map_ptr->GetHashIdString(traj[i].matched_lane_id) +
              "\n" + std::to_string(traj[i].matched_lane_heading_deviation) +
              "\n" + std::to_string(pred_cam(0, 0)) + "\n" +
              std::to_string(pred_cam(1, 0)) + "\n" +
              std::to_string(pred_cam(2, 0)) + "\n" +
              std::to_string(pred_cam(3, 0)) + "\n" +
              std::to_string(pred_cam(4, 0)) + "\n" +
              std::to_string(pred_cam(5, 0)) + "\n" +
              std::to_string(pred_ctm(4, 0)) + "\n");
          FillPt(agent_matched_lane_id->mutable_position(), pred_cam(0, 0),
                 pred_cam(1, 0), 0);
          LOG_INFO("{},{},0,{},{},0,{},{},0,{},0,0,###{},{}", pred_cam(0, 0),
                   pred_cam(1, 0), pred_cam(2, 0), pred_cam(3, 0),
                   pred_cam(4, 0), pred_cam(5, 0), pred_ctm(4, 0),
                   std::to_string(ptr->id),
                   planning_map_ptr->GetHashIdString(traj[i].matched_lane_id));
        } else {
          LOG_INFO("obs #ego");
        }

        std::shared_ptr<neodrive::planning::math::KalmanFilter> ctm =
            idx == 0 ? ego_kinematics_.ct_kf : GetObsKinematics(ptr->id).ct_kf;
        std::vector<std::array<double, 2>> ctm_traj{};
        ctm.get()->S().get()->Predict(30, ctm_traj);
        for (int i = 0; i < ctm_traj.size(); i++) {
          auto step = coordinate_transformations_["utm2odom"](
              this, data_center_ptr->pose_base_link_in_utm_msg.ptr->pose(),
              data_center_ptr->pose_base_link_in_odometry_msg.ptr->pose(),
              {ctm_traj[i][0], ctm_traj[i][1]},
              atan2(ctm_traj[i][1], ctm_traj[i][0]));
          ctm_traj[i][0] = step.first[0];
          ctm_traj[i][1] = step.first[1];
          FillPt(polyline_CT->add_point(), ctm_traj[i][0], ctm_traj[i][1], 0);
        }
        auto cen_ca = coordinate_transformations_["utm2odom"](
            this, data_center_ptr->pose_base_link_in_utm_msg.ptr->pose(),
            data_center_ptr->pose_base_link_in_odometry_msg.ptr->pose(),
            {pred_cam(0, 0) + pred_cam(2, 0) * 1,
             pred_cam(1, 0) + pred_cam(3, 0) * 1},
            atan2(pred_cam(1, 0) + pred_cam(3, 0) * 1,
                  pred_cam(0, 0) + pred_cam(2, 0) * 1));

        auto cen_cv = coordinate_transformations_["utm2odom"](
            this, data_center_ptr->pose_base_link_in_utm_msg.ptr->pose(),
            data_center_ptr->pose_base_link_in_odometry_msg.ptr->pose(),
            {pred_cvm(0, 0) + pred_cvm(2, 0) * 1,
             pred_cvm(1, 0) + pred_cvm(3, 0) * 1},
            atan2(pred_cvm(1, 0) + pred_cvm(3, 0) * 1,
                  pred_cvm(0, 0) + pred_cvm(2, 0) * 1));

        auto vel = coordinate_transformations_["utm2odom"](
            this, data_center_ptr->pose_base_link_in_utm_msg.ptr->pose(),
            data_center_ptr->pose_base_link_in_odometry_msg.ptr->pose(),
            {state[0] + state[2] * 1, state[1] + state[3] * 1},
            atan2(state[1] + state[3] * 1, state[0] + state[2] * 1));
        FillPt(polyline_CA->add_point(), pred_cam(6, 0), pred_cam(7, 0), 0);
        FillPt(polyline_CA->add_point(), cen_ca.first[0], cen_ca.first[1], 0);
        FillPt(polyline_CV->add_point(), pred_cvm(6, 0), pred_cvm(7, 0), 0);
        FillPt(polyline_CV->add_point(), cen_cv.first[0], cen_cv.first[1], 0);
        FillPt(polyline->add_point(), cen[0], cen[1], 0);
        FillPt(polyline->add_point(), vel.first[0], vel.first[1], 0);
      }
      FillPt(sphere->mutable_center(), cen[0], cen[1], 0);
      sphere->set_radius(0.1);
      FillPt(sphere_CA->mutable_center(), pred[i].at("cam")(6, 0),
             pred[i].at("cam")(7, 0), 0);
      sphere_CA->set_radius(0.1);
      FillPt(sphere_CV->mutable_center(), pred[i].at("cvm")(6, 0),
             pred[i].at("cvm")(7, 0), 0);
      sphere_CV->set_radius(0.1);
    }
  }

  for (auto &poly : currentFrame.obs_polygons_vec()) {
    auto id = poly.id;
    auto polygon = events["__CurFrame"]->add_polygon();
    for (auto &p : poly.shape.points()) {
      FillPt(polygon->add_point(), p[0], p[1], 0);
    }
  }
  LOG_INFO("Visualizer finished");
}

std::weak_ptr<GlobalObsInfo> ObstacleMap::GetObs(int id) {
  return idx_mapping_.count(id) > 0 ? idx_mapping_[id].obs_info
                                    : std::weak_ptr<GlobalObsInfo>();
};

ObsKinematicsInfo &ObstacleMap::GetObsKinematics(int id) {
  return idx_mapping_.at(id);
}

std::weak_ptr<GlobalObsInfo> ObstacleMap::ego() {
  return std::weak_ptr<GlobalObsInfo>(ego_);
}

void ObstacleMap::InitROI(const VehicleStateProxy &proxy, double sensorRange) {
  math::AD2 egoPos{{proxy.X(), proxy.Y()}};
  math::AD2 headingVec = {sensorRange * cos(proxy.Heading()),
                          sensorRange * sin(proxy.Heading())};
  math::AD2 extendingVec = {sensorRange * cos(proxy.Heading() - M_PI_2),
                            sensorRange * sin(proxy.Heading() - M_PI_2)};
  roi_[Front] = math::Polygon(
      {{egoPos[0] + headingVec[0] + extendingVec[0],
        egoPos[1] + headingVec[1] + extendingVec[1]},
       {egoPos[0] + headingVec[0] - extendingVec[0],
        egoPos[1] + headingVec[1] - extendingVec[1]},
       {egoPos[0] - extendingVec[0], egoPos[1] - extendingVec[1]},
       {egoPos[0] + extendingVec[0], egoPos[1] + extendingVec[1]}});

  roi_[Behind] = math::Polygon(
      {{egoPos[0] - headingVec[0] + extendingVec[0],
        egoPos[1] - headingVec[1] + extendingVec[1]},
       {egoPos[0] - headingVec[0] - extendingVec[0],
        egoPos[1] - headingVec[1] - extendingVec[1]},
       {egoPos[0] - extendingVec[0], egoPos[1] - extendingVec[1]},
       {egoPos[0] + extendingVec[0], egoPos[1] + extendingVec[1]}});

  roi_[Vortex] = math::Polygon({
      {egoPos[0] + headingVec[0], egoPos[1] + headingVec[1]},
      {egoPos[0] + extendingVec[0] / 2, egoPos[1] + extendingVec[1] / 2},
      {egoPos[0] - headingVec[0], egoPos[1] - headingVec[1]},
      {egoPos[0] - extendingVec[0] / 2, egoPos[1] - extendingVec[1] / 2},
  });
}

void ObstacleMap::InitAttention(const VehicleStateProxy &proxy,
                                const math::AD2 &intersection) {
  const math::AD2 egoPos{{proxy.X(), proxy.Y()}};
  double sensorRange =
      math::Length({egoPos[0] - intersection[0], egoPos[1] - intersection[1]});
  math::Polygon &AttentionZone = roi_[Vortex];
}

void ObstacleMap::CalculateROI(const VehicleStateProxy &proxy, int frame) {
  auto &currentFrame = At(frame);
  auto fill_pt = [](auto t, auto x, auto y, auto z) {
    t->set_x(x), t->set_y(y), t->set_z(z);
  };
  auto clear_obs = [](auto vec, auto isfront, auto ypos) {
    // isfront shall be 0 or 1
    while (!vec.empty()) {
      if ((vec.top()->shape.aabox().cen[1] - ypos) * (isfront * 2 - 1) > 30) {
        vec.pop();
      } else {
        break;
      }
    }
  };
}

bool ObstacleMap::ObstacleParser(const ObjectTable &object_table,
                                 const ObjectTable &object_table_utm,
                                 const std::vector<int> &obstacle_ids) {
  auto BindROI = [](auto &obs, auto &ROI) {
    return math::IsOverlaped(obs, ROI);
  };

  auto Emplace = [this](auto &info, auto node, auto &states) -> ErrorCode {
    LOG_DEBUG("EXIST: {} , EXPIRED: {}", idx_mapping_.count(info.obs_id),
              node.expired());
    if (idx_mapping_.count(info.obs_id) > 0 || node.expired())
      return ErrorCode::PLANNING_ERROR_FAILED;

    std::shared_ptr<GlobalObsInfo> poly(
        new GlobalObsInfo(&info, &(node.lock()->node_.shape), states));
    auto [it, suc] = localmap_.insert(poly);
    if (suc) {
      std::array<std::shared_ptr<math::KalmanFilter>, 3> kf{
          math::CVM(poly->trajectory.front().x, 0.1, 0.0),
          math::CAM(poly->trajectory.front().x, 0.1, 0.0),
          math::CTM(poly->trajectory.front().x, 0.1, 0.0)};
      idx_mapping_.insert(std::make_pair(
          info.obs_id,
          ObsKinematicsInfo{std::weak_ptr(*it), kf[0], kf[1], kf[2]}));

      poly.reset();
      return ErrorCode::PLANNING_OK;
    } else {
      LOG_ERROR("Rbtree insertion exception occurs");
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  };

  auto &currentFrame = obs_frame_container_.front();
  int obs_cnt = obstacle_ids.size();
  std::vector<char> obs_type;
  for (auto i = 0; i < obs_cnt; i++) {
    Obstacle *obstacle = nullptr;
    Obstacle *obstacle_utm = nullptr;
    if (object_table.get_obstacle_slient(obstacle_ids[i], &obstacle)) {
      object_table_utm.get_obstacle_slient(obstacle_ids[i], &obstacle_utm);
      bool is_static = obstacle->prediction_trajectories().empty();
      auto obs_state = obstacle_utm->kinematics_state();
      auto matched_lane_id = obstacle_utm->matched_lane_id();
      double matched_lane_heading_deviation =
          obstacle_utm->matched_lane_heading_deviation();
      Eigen::VectorXd angle(2);
      angle << obs_state[4], obs_state[5];

      std::weak_ptr<GlobalObsInfo> poly = GetObs(obstacle_ids[i]);
      if (!poly.expired() && math::Sign(obs_state[7], 1e-4) == 0 &&
          poly.lock()->trajectory.size() > 1) {
        obs_state[2] = poly.lock()->trajectory[1].x[2];
        obs_state[3] = poly.lock()->trajectory[1].x[3];
        obs_state[5] = poly.lock()->trajectory[1].w;
      }
      currentFrame.AddPolygon(obstacle_ids[i], matched_lane_id,
                              matched_lane_heading_deviation, obstacle->type(),
                              obstacle->polygon(), obs_state);
      obs_type.emplace_back(is_static ? 0 : 1);
    } else {
      LOG_INFO("Failed to get obstacle of {}, continue...", obstacle_ids[i]);
    }
  }
  currentFrame.GenerateFrame();

  auto poly_vec = currentFrame.mutable_obs_polygons_vec();
  auto &nodes = currentFrame.mutable_obs_polygon_tree()->node_map_;
  for (auto cnt = 0; cnt < currentFrame.obs_polygons_vec().size(); cnt++) {
    LOG_DEBUG(
        "------------------------------ local map: #{} in frame {} "
        "------------------------------",
        localmap_.size(), time_);
    auto &info = currentFrame.GetObsInfo(cnt);
    auto &state = currentFrame.GetObsState(cnt);
    if (nodes.count(cnt) == 0) {
      LOG_ERROR("node #{}: not found in curframe", info.obs_id);
      continue;
    } else {
      LOG_DEBUG("node #{}: found in curframe", info.obs_id);
    }
    if (Emplace(info, nodes[cnt], state) == ErrorCode::PLANNING_ERROR_FAILED) {
      std::weak_ptr<GlobalObsInfo> poly = GetObs(info.obs_id);
      if (poly.expired()) {
        LOG_ERROR("obs #{}: error pointer in local map", info.obs_id);
        continue;
      }
      poly.lock()->Update(&info, &(nodes[cnt].lock()->node_.shape), state);
      LOG_DEBUG("obs #{}({}): found in local map", info.obs_id,
                poly.lock()->trajectory.size());
    } else {
      std::weak_ptr<GlobalObsInfo> poly = GetObs(info.obs_id);
      poly.lock()->Predict();
      LOG_DEBUG("obs #{}: not found in local map", info.obs_id);
    }
  }
  return true;
}

void ObstacleMap::Update(ObsFrameContainer &currentFrame) {
  if (IsFull()) Pop();
  Push(currentFrame);
  LOG_DEBUG("ObstacleMap Size:{}, #num: {}", size_,
            obs_frame_container_.front().mutable_obs_polygon_tree()->Length());
}

void ObstacleMap::Update(const ObjectTable &object_table,
                         const ObjectTable &object_table_utm,
                         const std::vector<int> &obstacle_ids,
                         const VehicleStateProxy &proxy) {
  auto t_start = cyber::Time::Now().ToMicrosecond();

  if (IsFull()) {
    Pop();
  }
  Push();

  InitROI(proxy, 30.0);
  LOG_DEBUG("Current ego: Pos -> ( {} , {} ) | Heading -> {}", proxy.X(),
            proxy.Y(), proxy.Heading() / M_PI * 180);

  auto &currentFrame = obs_frame_container_.front();
  auto &poly_vec = *currentFrame.mutable_obs_polygons_vec();
  if (!ObstacleParser(object_table, object_table_utm, obstacle_ids)) {
    LOG_DEBUG("Obstacle parser cannot parse obstacle information.");
    return;
  } else {
    for (auto it = localmap_.begin(); it != localmap_.end();) {
      if (!(*it)->Metabolism()) {
        int id = (*it)->id;
        idx_mapping_.erase(id);
        it = localmap_.erase(it);
        LOG_DEBUG("obs id #{}: dead", id);
      } else {
        it++;
      }
    }
  }

  auto data_center_ptr = DataCenter::Instance();
  if (ego_state_.init) {
    math::AD2 veh_acc =
        data_center_ptr->vehicle_state_proxy().ImuAcceleration();
    ego_state_.imu_accleration = {
        veh_acc[1] * std::cos(data_center_ptr->vehicle_state_utm().Heading()) +
            veh_acc[0] *
                std::sin(data_center_ptr->vehicle_state_utm().Heading()),
        -veh_acc[0] * std::cos(data_center_ptr->vehicle_state_utm().Heading()) +
            veh_acc[1] *
                std::sin(data_center_ptr->vehicle_state_utm().Heading())};
    ego_state_.dr_velocity = {
        data_center_ptr->vehicle_state_utm().Velocity3dX(),
        data_center_ptr->vehicle_state_utm().Velocity3dY()};
    for (int i = 0; i < 2; i++) {
      td_1D_1e1_.Diff(ego_state_.dr_est_velocity[i], ego_state_.dr_velocity[i]);
      ego_state_.dr_accleration[i] = ego_state_.dr_est_velocity[i][1];
    }

    ego_state_.ego_state = {
        data_center_ptr->vehicle_state_utm().Velocity3dX(),
        data_center_ptr->vehicle_state_utm().Velocity3dY(),
        ego_state_.dr_accleration[0],
        ego_state_.dr_accleration[1],
        data_center_ptr->vehicle_state_utm().Heading(),
        data_center_ptr->vehicle_state_utm().LinearVelocity() *
            data_center_ptr->vehicle_state_utm().Curvature(),
        0,
        0.1,
        data_center_ptr->vehicle_state_utm().X(),
        data_center_ptr->vehicle_state_utm().Y()};
    ego_->Update(nullptr, nullptr, ego_state_.ego_state, true);
    LOG_INFO("ego is updated.");
    LOG_INFO("[{},{},{},{},{},{},{},{},{},{},{},{},{},{}]",
             data_center_ptr->vehicle_state_utm().LinearVelocity(),
             data_center_ptr->vehicle_state_utm().Heading(),
             ego_state_.dr_velocity[0], ego_state_.dr_velocity[1],
             ego_state_.dr_est_velocity[0][0], ego_state_.dr_est_velocity[1][0],
             ego_state_.dr_est_velocity[0][1], ego_state_.dr_est_velocity[1][1],
             ego_state_.imu_accleration[0], ego_state_.imu_accleration[1],
             ego_->prediction.front()["cam"](2, 0),
             ego_->prediction.front()["cam"](3, 0),
             ego_->prediction.front()["cam"](4, 0),
             ego_->prediction.front()["cam"](5, 0));
  } else {
    ego_state_.init = true;
    ego_->Predict(true);
  }

  LOG_INFO(
      //"ObstacleMap Size:{}, frame: {}, #all: {}, #static: {}, #dynamic: {}, "
      "ObstacleMap Size:{}, frame: {}, #all: {}, "
      "time cost: {} us",
      size_, currentFrame.frame_id(),
      currentFrame.mutable_obs_polygon_tree()->Length(),
      cyber::Time::Now().ToMicrosecond() - t_start);
}
}  // namespace obsmap
}  // namespace planning
}  // namespace neodrive

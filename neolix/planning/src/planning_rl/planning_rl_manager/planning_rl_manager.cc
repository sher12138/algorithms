#include "planning_rl_manager.h"

namespace neodrive {
namespace planning_rl {

PlanningRLManager::PlanningRLManager() {}

bool PlanningRLManager::Init(
    const std::shared_ptr<neodrive::cyber::Node> &node) {
  if (initialized_) {
    return true;
  }
  node_ = node;

  auto &topics_config =
      config::PlanningRLConfig::Instance()->topics_config().topics;

  planning_pub_ = node_->CreateWriter<ADCTrajectory>(
      topics_config.at("pnc_planning").topic);
  if (planning_pub_ == nullptr) {
    return false;
  }
  planning_rl_pub_ = node_->CreateWriter<ADCTrajectory>(
      topics_config.at("pnc_planning_rl").topic);
  if (planning_rl_pub_ == nullptr) {
    return false;
  }
  LOG_INFO("PlanningRLManager is ready.");
  initialized_ = true;
  return true;
}

ErrorCode PlanningRLManager::Process(ContainerMessageShrPtr &container_msg_) {
  manager_time_log_.ResetStartTime();
  ErrorCode ret = ErrorCode::PLANNING_OK;
  planning_trajectory_published_ = true;
  auto fp = DataTransform(container_msg_);
  manager_time_log_.RegisterTime("Data Transform");
  LOG_INFO("DataTransform Success.");
  PredictInterface(fp, container_msg_);
  manager_time_log_.RegisterTimeAndPrint("PredictInterface");
  LOG_INFO("PredictInterface Success.");
  if (!planning_trajectory_published_) {
    PublishFakeTrajectory();
    LOG_INFO("PublishFakeTrajectory Success.");
  }
  return ret;
}

ErrorCode PlanningRLManager::ProcessUD(ContainerMessageShrPtr &container_msg_) {
  manager_time_log_.ResetStartTime();
  ErrorCode ret = ErrorCode::PLANNING_OK;
  planning_trajectory_published_ = true;
  auto fp = DataTransformUD(container_msg_);
  manager_time_log_.RegisterTime("Data Transform");
  LOG_INFO("DataTransformUD Success.");
  UDPredictInterface(fp, container_msg_);
  manager_time_log_.RegisterTimeAndPrint("PredictInterface");
  LOG_INFO("PredictInterface Success.");
  if (!planning_trajectory_published_) {
    PublishFakeTrajectory();
    LOG_INFO("PublishFakeTrajectory Success.");
  }
  return ret;
}

void PlanningRLManager::PredictInterface(
    FeaturePreparation &fp, ContainerMessageShrPtr &container_msg_) {
  utils::TimeLogger time_log_{"PredictInterface"};
  time_log_.ResetStartTime();
  fp.GetAllData();
  time_log_.RegisterTimeAndPrint("GetAllData");
  auto target_before = fp.target_before();
  time_log_.RegisterTimeAndPrint("target_before");
  // LOG_DEBUG("target:{}", target_before[0][0][0]);
  auto output = ModelPredictTrt(target_before);
  time_log_.RegisterTimeAndPrint("ModelPredict");
  // std::cout << output << std::endl;
  LOG_INFO("model predict done");
  std::cout << "model predict done" << std::endl;
  // auto vec_output = Tensor2Vec(output);
  ModelOutputFix(output, fp.ego_info(), container_msg_);
  std::cout << "fix model predict done" << std::endl;
  std::cout << output << std::endl;
  LOG_INFO("model predict done");
  // LOG_DEBUG("target_before:{}", target_before[0][0][0]);
  // LOG_DEBUG("vec_output:{}", vec_output[0]);
  // LOG_DEBUG("ego_info:{}", fp.ego_info().x);
  // std::cout << "target_before" << std::endl;
  // std::cout << target_before[0] << std::endl;
  // std::cout << target_before.size() << std::endl;
  // std::cout << target_before[0].size() << std::endl;
  // std::cout << target_before[0][0].size() << std::endl;
  // std::cout << output << std::endl;
  // std::cout << fp.ego_info().x << "    " << fp.ego_info().y << "    " <<
  // fp.ego_info().heading << "    " << fp.ego_info().speed << "    " <<
  // fp.ego_info().lon_speed << "    " << fp.ego_info().lat_speed << "    " <<
  // fp.ego_info().acc << "    " << fp.ego_info().lon_acc << "    " <<
  // fp.ego_info().lat_acc << "    " << fp.ego_info().curvature << "    " <<
  // fp.ego_info().step_time << "    " << fp.ego_info().length << "    " <<
  // fp.ego_info().width << std::endl; std::vector<std::vector<double>>
  // target_before_temp (195, std::vector<double> (11, 1.0)); target_before[0] =
  // target_before_temp; std::vector<double> vec_output_temp =
  // {-0.04240580648,-0.01910703443,-0.01111958176,-0.00924955681,0.03761841729,-0.7539520264,1.027939558};
  // vec_output = vec_output_temp;
  // fp.ego_info().x = 1;
  // fp.ego_info().y = 1;
  // fp.ego_info().heading = 1;
  // fp.ego_info().speed = 1;
  // fp.ego_info().lon_speed = 1;
  // fp.ego_info().lat_speed = 1;
  // fp.ego_info().acc = 1;
  // fp.ego_info().lon_acc = 1;
  // fp.ego_info().lat_acc = 1;
  // fp.ego_info().curvature = 1;
  // fp.ego_info().step_time = 1;
  // fp.ego_info().length = 1;
  // fp.ego_info().width = 1;
  auto ego_tmp = EgoInfo();
  // std::cout << "target_before" << std::endl;
  // std::cout << target_before << std::endl;
  // std::cout << target_before[0] << std::endl;
  // std::cout << target_before[4] << std::endl;
  // std::cout << target_before.size() << std::endl;
  // std::cout << target_before[0].size() << std::endl;
  // std::cout << target_before[0][0].size() << std::endl;
  // std::cout << output << std::endl;
  // auto ppr = PostPredict(target_before[0], vec_output, ego_tmp);
  auto ppr = PostPredict(target_before, output, fp.ego_info());
  std::cout << "post predict done" << std::endl;
  LOG_INFO("post predict done");
  std::vector<TrajectoryPoint2D> pred_traj_opt = OptimalTrajectoryRefpoint(
      ppr, CvtVecref2Point(fp.reference_line_points_optj()));
  std::cout << "opttraj done" << std::endl;
  LOG_INFO("opttraj done");
  time_log_.RegisterTimeAndPrint("PostPredict");
  // TransUtm2Odom(pred_traj_opt, container_msg_);
  PublishPlanningRLResult(pred_traj_opt);
  LOG_INFO("publish pl done");
  // WriteTrajJson(pred_traj_opt);
}

void PlanningRLManager::UDPredictInterface(
    FeaturePreparationV2 &fp, ContainerMessageShrPtr &container_msg_) {
  utils::TimeLogger time_log_{"UDPredictInterface"};
  time_log_.ResetStartTime();
  fp.GetAllData();
  time_log_.RegisterTimeAndPrint("GetAllData");
  auto target_before = fp.target_before();
  time_log_.RegisterTimeAndPrint("target_before");
  auto ego_frame = fp.ego_lists().back();
  ego_frames_.push_back(ego_frame);
  if (ego_frames_.size() > 50) {
    ego_frames_.pop_front();
  }
  // LOG_DEBUG("target:{}", target_before[0][0][0]);
  auto output = ud_inference(fp);
  time_log_.RegisterTimeAndPrint("ModelPredict");
  // std::cout << output << std::endl;
  LOG_INFO("model predict done");
  std::cout << "model predict done" << std::endl;

  bool use_step_gap = true;
  CheckSceneState(container_msg_, fp);
  ModelOutputFixUD(output, fp.ego_info(), container_msg_, use_step_gap);
  std::cout << "fix model predict done" << std::endl;
  LOG_INFO("model predict done");

  std::vector<PredState> predicts_odom;
  if (use_step_gap == false) {
    predicts_odom =
        TransPredictsFrenet2Odom(output, fp.ego_info(), fp.refline_points());
  } else {
    predicts_odom = TransPredictsFrenet2OdomStepGap(output, fp.ego_info(),
                                                    fp.refline_points());
  }
  FixPredResult(predicts_odom, container_msg_);

  LOG_INFO("opttraj done");
  time_log_.RegisterTimeAndPrint("PostPredict");
  // TransUtm2Odom(pred_traj_opt, container_msg_);
  PublishPlanningRLUDResult(predicts_odom);
  LOG_INFO("publish ud_traj done");
  // SaveUDResultText(container_msg_, fp, output, predicts_odom);
  // WriteTrajJson(pred_traj_opt);
}

void PlanningRLManager::ModelOutputFix(std::vector<double> &output,
                                       EgoInfo ego_info,
                                       ContainerMessageShrPtr &container_msg_) {
  const double eps = 1e-1;
  // if (fabs(ego_info.lon_speed - 0.0) < eps && fabs(ego_info.lon_acc - 0.0) <
  // eps) {
  //   output[0] = 0.9;
  //   output[3] = 0.7;
  //   output[4] = 0.4;
  //   output[5] = 0.0001;
  // }
  bool scene_start = false;
  if (container_msg_->scene_status == 1 &&
      fabs(ego_info.lon_speed - 0.0) < eps && output[0] < 0.4) {
    scene_start_time_ = container_msg_->timestamp;
    scene_start = true;
  }
  if (container_msg_->scene_status == 2) {
    scene_start = true;
  }
  if (container_msg_->scene_status == 3 &&
      container_msg_->timestamp - scene_start_time_ > -1 &&
      container_msg_->timestamp - scene_start_time_ < 5) {
    scene_start = true;
  }
  if (scene_start == true) {
    output[0] = 0.9;
    output[1] = 0.0;
    output[3] = 0.7;
    output[4] = 0.4;
    output[5] = 0.0001;
  }
  if (container_msg_->scene_status == 3 && scene_start == false) {
    if (output[0] < 0.1) {
      output[0] *= 5;
      // output[1] *= 10;
    }
    if (output[0] < 0.2) {
      output[0] *= 2;
      // output[1] *= 5;
    }
    if (output[0] < 0.4) {
      output[0] *= 2;
      // output[1] *= 2;
    }
  }
  LOG_INFO("scene_start: {}", scene_start);
  LOG_INFO("scene_status: {}", container_msg_->scene_status);
}

int PlanningRLManager::CheckSceneState(ContainerMessageShrPtr &container_msg_,
                                       const FeaturePreparationV2 &fp) {
  int scene_state = 0;
  auto f_ego_frame = ego_frames_.front();
  auto e_ego_frame = ego_frames_.back();
  // check ego move
  if (e_ego_frame.velocity[0] > 0.1 || e_ego_frame.velocity[1] > 0.1) {
    // std::cout << "CheckSceneStateVEL" << std::endl;
    return scene_state;
  }
  double diff_x = f_ego_frame.centroid[0] - e_ego_frame.centroid[0];
  double diff_y = f_ego_frame.centroid[1] - e_ego_frame.centroid[1];
  double diff = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
  auto target_before_back = fp.target_before().back();
  std::vector<std::vector<double>> ego_feature, agents_feature;
  ego_feature.assign(target_before_back.begin(),
                     target_before_back.begin() + 1);
  agents_feature.assign(target_before_back.begin() + 1,
                        target_before_back.begin() + 65);
  bool agent_on_refline = false;
  double nearest_agent_dis = 1000000.0;
  double ego_f_x = ego_feature.front()[0];
  double ego_f_y = ego_feature.front()[1];
  for (auto &agent_f : agents_feature) {
    double agent_x = agent_f[0];
    double agent_y = agent_f[1];
    // check agent near the refline
    if ((agent_x - ego_f_x < 16 && agent_x - ego_f_x > 0) &&
        (agent_y < 1 && agent_y > -1)) {
      agent_on_refline = true;
    }
    double ego2agent =
        sqrt(pow(ego_f_x - agent_x, 2) + pow(ego_f_y - agent_y, 2));
    // check agent near ego
    if ((agent_x - ego_f_x < 16 && agent_x - ego_f_x > 0) &&
        (ego_f_y - agent_y < 5 && ego_f_y - agent_y > -5)) {
      if (ego2agent < nearest_agent_dis) {
        nearest_agent_dis = ego2agent;
      }
    }
  }
  // ego does not move && no agent on refline && no agent near ego, ego starts
  if (diff < 0.1) {
    if (!agent_on_refline && (nearest_agent_dis > 5)) {
      // state 1 -> new scene starts
      scene_state = 1;
      // over 10s past since last starts
      if (container_msg_->timestamp - last_reset_scene_time_ > 10 ||
          container_msg_->timestamp - last_reset_scene_time_ < -10) {
        container_msg_->scene_status = 1;
        last_reset_scene_time_ = container_msg_->timestamp;
      }
    }
  }
  return scene_state;
}

void PlanningRLManager::ModelOutputFixUD(
    std::vector<std::vector<double>> &output, EgoInfo ego_info,
    ContainerMessageShrPtr &container_msg_, bool use_step_gap) {
  const double eps = 1e-1;
  // if (fabs(ego_info.lon_speed - 0.0) < eps && fabs(ego_info.lon_acc - 0.0) <
  // eps) {
  //   output[0] = 0.9;
  //   output[3] = 0.7;
  //   output[4] = 0.4;
  //   output[5] = 0.0001;
  // }
  bool scene_start = false;
  if (container_msg_->scene_status == 1 &&
      fabs(ego_info.lon_speed - 0.0) < eps && output[0][0] < 0.4) {
    scene_start_time_ = container_msg_->timestamp;
    scene_start = true;
  }
  if (container_msg_->scene_status == 2 &&
      fabs(ego_info.lon_speed - 0.0) < eps) {
    scene_start_time_ = container_msg_->timestamp;
    scene_start = true;
  }
  if (container_msg_->scene_status == 3 &&
      container_msg_->timestamp - scene_start_time_ > -1 &&
      container_msg_->timestamp - scene_start_time_ < 0.5) {
    scene_start = true;
  }
  if (scene_start == true) {
    if (use_step_gap) {
      // output[0][0] = 0.0025;
      // output[0][1] = 0.0;
      // output[0][2] = 0.0;
      // output[0][3] = 0.05;
      // output[0][4] = 0.5;
      // output[0][5] = 0.0;
      double gap_speed = 0.0;
      double target_speed = 2.0;
      if (ego_info.lon_speed < target_speed) {
        gap_speed = target_speed - ego_info.lon_speed;
      }
      double gap_acc = 0.0;
      double target_acc = 0.5;
      if (ego_info.lon_acc < target_acc) {
        gap_acc = target_speed - ego_info.lon_acc;
      }
      output[0][0] = 0.2 / 6;
      output[0][1] = 0.0;
      output[0][2] = 0.0;
      output[0][3] = gap_speed / 0.5;
      output[0][4] = gap_acc / 4;
      output[0][5] = 0.0;
      for (int i = 1; i < 80; i++) {
        output[i][1] = 0.0;
        output[i][2] = 0.0;
        output[i][5] = 0.0;
        output[i][4] = 0.0;
        output[i][3] = 0.0;      // 0.05;
        output[i][0] = 0.2 / 6;  //(output[i][3] + output[i - 1][3]) * 0.05;
      }
    } else {
      for (int i = 0; i < 80; i++) {
        output[i][1] = 0.0;
        output[i][2] = 0.0;
        output[i][5] = 0.0;
        output[i][4] = 0.5 / 6;
        output[i][3] = 2.0 / 5;             //(i + 1) * 0.05;
        output[i][0] = 0.2 * (i + 1) / 30;  // 0.0025 * (i + 1) * (i + 1);
      }
    }
  }
  // if (container_msg_->scene_status == 3 && scene_start == false) {
  //   if (output[0] < 0.1) {
  //     output[0] *= 5;
  //   }
  //   if (output[0] < 0.2) {
  //     output[0] *= 2;
  //   }
  //   if (output[0] < 0.4) {
  //     output[0] *= 2;
  //   }
  // }
  // std::cout << "fixed_output:" << std::endl;
  // std::cout << output << std::endl;
  LOG_INFO("scene_start: {}", scene_start);
  LOG_INFO("scene_status: {}", container_msg_->scene_status);
}

FeaturePreparation PlanningRLManager::DataTransform(
    const ContainerMessageShrPtr &container_msg_) {
  FeaturePreparation fp = FeaturePreparation();
  auto &frames = container_msg_->frames->frames();
  std::vector<EgoFrame> ego_lists;
  std::vector<std::vector<AgentFrame>> agents_lists;
  std::vector<std::vector<double>> refline_points;
  std::vector<double> step_time_list;
  std::vector<double> refline_curvatures;
  std::vector<double> thetas;
  for (size_t i = 0; i < frames.size(); i++) {
    step_time_list.emplace_back(frames[i].step_time());
    ego_lists.emplace_back(
        std::vector<double>{frames[i].ego().position().x,
                            frames[i].ego().position().y,
                            frames[i].ego().position().z},
        frames[i].ego().position().heading,
        std::vector<double>{frames[i].ego().velocity().x(),
                            frames[i].ego().velocity().y(),
                            frames[i].ego().velocity().z()},
        std::vector<double>{frames[i].ego().linear_acceleration().x(),
                            frames[i].ego().linear_acceleration().y(),
                            frames[i].ego().linear_acceleration().z()},
        std::vector<double>{
            2.74,
            1.06,
        });
    std::vector<AgentFrame> agentsframe;
    for (size_t j = 0; j < frames[i].agents().agents().size(); j++) {
      agentsframe.emplace_back(
          std::vector<double>{frames[i].agents().agents()[j].position().x,
                              frames[i].agents().agents()[j].position().y,
                              frames[i].agents().agents()[j].position().z},
          frames[i].agents().agents()[j].position().heading,
          std::vector<double>{frames[i].agents().agents()[j].velocity().x(),
                              frames[i].agents().agents()[j].velocity().y(),
                              frames[i].agents().agents()[j].velocity().z()},
          frames[i].agents().agents()[j].extent(),
          frames[i].agents().agents()[j].type_string());
    }
    agents_lists.emplace_back(agentsframe);
  }
  std::cout.precision(10);
  // std::cout << "center_line_ori" << std::endl;
  // for (int i=0; i < container_msg_->reference_line_ptr->size(); i++) {
  //   std::cout << i << std::endl;
  //   std::cout << container_msg_->reference_line_ptr->ref_points()[i].x() <<
  //   std::endl; std::cout <<
  //   container_msg_->reference_line_ptr->ref_points()[i].y() << std::endl;
  //   std::cout << container_msg_->reference_line_ptr->ref_points()[i].s() <<
  //   std::endl;
  // }
  for (size_t i = 0; i < container_msg_->reference_line_ptr->size(); i++) {
    refline_points.emplace_back(std::vector<double>{
        container_msg_->reference_line_ptr->ref_points()[i].x(),
        container_msg_->reference_line_ptr->ref_points()[i].y(),
        container_msg_->reference_line_ptr->ref_points()[i].heading(),
        container_msg_->reference_line_ptr->ref_points()[i].s(),
        container_msg_->reference_line_ptr->ref_points()[i].kappa()});
    refline_curvatures.emplace_back(
        container_msg_->reference_line_ptr->ref_points()[i].kappa());
    thetas.emplace_back(
        container_msg_->reference_line_ptr->ref_points()[i].heading());
  }

  // std::cout << "ego_lists" << ego_lists[0].centroid << std::endl;
  // std::cout << "ego_lists" << ego_lists[0].yaw << std::endl;
  // std::cout << "agents_lists" << agents_lists[0][0].centroid << std::endl;
  // std::cout << "refline_points" << refline_points << std::endl;
  // for (int i = 0; i < refline_points.size(); i++) {
  //   std::cout << "ref_point" << std::endl;
  //   std::cout << i << std::endl;
  //   auto x = refline_points[i];
  //   std::cout << x[0] << std::endl;
  //   std::cout << x[1] << std::endl;
  //   std::cout << x[2] << std::endl;
  //   std::cout << x[3] << std::endl;
  //   std::cout << x[4] << std::endl;
  // }
  // std::cout << "step_time_list" << step_time_list << std::endl;
  // std::cout << "refline_curvatures" << refline_curvatures << std::endl;
  // std::cout << "thetas" << thetas << std::endl;
  fp.GetData(ego_lists, agents_lists, refline_points, step_time_list,
             refline_curvatures, thetas);
  // std::cout << "refline_points" << fp.reference_line_points_optj() <<
  // std::endl; for (int i = 0; i < fp.reference_line_points_optj().size(); i++)
  // {
  //   std::cout << "ref_point" << std::endl;
  //   std::cout << i << std::endl;
  //   auto x = fp.reference_line_points_optj()[i];
  //   std::cout << x[0] << std::endl;
  //   std::cout << x[1] << std::endl;
  //   std::cout << x[2] << std::endl;
  //   std::cout << x[3] << std::endl;
  //   std::cout << x[4] << std::endl;
  // }
  return fp;
}

FeaturePreparationV2 PlanningRLManager::DataTransformUD(
    const ContainerMessageShrPtr &container_msg_) {
  FeaturePreparationV2 fp = FeaturePreparationV2();
  auto &frames = container_msg_->frames->frames();
  std::vector<EgoFrame> ego_lists;
  std::vector<std::vector<AgentFrame>> agents_lists;
  std::vector<std::vector<double>> refline_points;
  std::vector<double> step_time_list;
  std::vector<double> refline_curvatures;
  std::vector<double> thetas;
  for (size_t i = 0; i < frames.size(); i++) {
    step_time_list.emplace_back(frames[i].step_time());
    ego_lists.emplace_back(
        std::vector<double>{frames[i].ego().position().x,
                            frames[i].ego().position().y,
                            frames[i].ego().position().z},
        frames[i].ego().position().heading,
        std::vector<double>{frames[i].ego().velocity().x(),
                            frames[i].ego().velocity().y(),
                            frames[i].ego().velocity().z()},
        std::vector<double>{frames[i].ego().linear_acceleration().x(),
                            frames[i].ego().linear_acceleration().y(),
                            frames[i].ego().linear_acceleration().z()},
        std::vector<double>{
            2.69,
            1.08,
        });
    std::vector<AgentFrame> agentsframe;
    for (size_t j = 0; j < frames[i].agents().agents().size(); j++) {
      agentsframe.emplace_back(
          std::vector<double>{frames[i].agents().agents()[j].position().x,
                              frames[i].agents().agents()[j].position().y,
                              frames[i].agents().agents()[j].position().z},
          frames[i].agents().agents()[j].position().heading,
          std::vector<double>{frames[i].agents().agents()[j].velocity().x(),
                              frames[i].agents().agents()[j].velocity().y(),
                              frames[i].agents().agents()[j].velocity().z()},
          frames[i].agents().agents()[j].extent(),
          frames[i].agents().agents()[j].type_string(),
          frames[i].agents().agents()[j].track_id());
    }
    agents_lists.emplace_back(agentsframe);
  }
  std::cout.precision(10);

  for (size_t i = 0; i < container_msg_->reference_line_ptr->size(); i++) {
    refline_points.emplace_back(std::vector<double>{
        container_msg_->reference_line_ptr->ref_points()[i].x(),
        container_msg_->reference_line_ptr->ref_points()[i].y(),
        container_msg_->reference_line_ptr->ref_points()[i].heading(),
        container_msg_->reference_line_ptr->ref_points()[i].s(),
        container_msg_->reference_line_ptr->ref_points()[i].kappa()});
    refline_curvatures.emplace_back(
        container_msg_->reference_line_ptr->ref_points()[i].kappa());
    thetas.emplace_back(
        container_msg_->reference_line_ptr->ref_points()[i].heading());
  }

  fp.GetData(container_msg_, ego_lists, agents_lists, refline_points,
             step_time_list, refline_curvatures, thetas);
  return fp;
}

void PlanningRLManager::TransUtm2Odom(
    std::vector<TrajectoryPoint2D> &pred_traj_opt,
    const ContainerMessageShrPtr &container_msg_) {
  // auto &utm_pose = container_msg_->perception_obstacles->utm_pose();
  // auto ego_world_x = utm_pose.position().x();
  // auto ego_world_y = utm_pose.position().y();
  // auto ego_world_orientation = utm_pose.orientation();
  // auto ego_world_theta =
  //     planning_rl::GetYawFromQuaternion(ego_world_orientation);
  // auto odom_pose = container_msg_->perception_obstacles->odom_pose();
  // auto ego_odo_x = odom_pose.position().x();
  // auto ego_odo_y = odom_pose.position().y();
  // auto ego_odo_orientation = odom_pose.orientation();
  // auto ego_odo_theta =
  // planning_rl::GetYawFromQuaternion(ego_odo_orientation); auto
  // world_model_odom_pose = container_msg_->odom_pose->pose(); std::cout <<
  // "odom pose world model" << std::endl; std::cout <<
  // world_model_odom_pose.position().x() << std::endl; std::cout <<
  // world_model_odom_pose.position().y() << std::endl; std::cout <<
  // planning_rl::GetYawFromQuaternion(world_model_odom_pose.orientation()) <<
  // std::endl; std::cout << container_msg_->odom_pose->header().timestamp_sec()
  // << std::endl; std::cout << "check position" << std::endl; std::cout <<
  // ego_world_x << std::endl; std::cout << ego_world_y << std::endl; std::cout
  // << ego_world_theta << std::endl; std::cout << ego_odo_x << std::endl;
  // std::cout << ego_odo_y << std::endl;
  // std::cout << ego_odo_theta << std::endl;
  // std::cout << "perception timestamp" << std::endl;
  // std::cout << container_msg_->perception_obstacles->header().timestamp_sec()
  // << std::endl; auto &frames = container_msg_->frames->frames(); for (size_t
  // i = 0; i < frames.size(); i++) {
  //   std::cout << "check position frames " << i << std::endl;
  //   std::cout << frames[i].ego().position().x << std::endl;
  //   std::cout << frames[i].ego().position().y << std::endl;
  //   std::cout << frames[i].ego().position().heading << std::endl;
  //   std::cout << frames[i].timestamp_sec() << std::endl;
  // }
  // auto &frames = container_msg_->frames->frames();
  auto utm_pose = container_msg_->utm_pose->pose();
  auto ego_world_x = utm_pose.position().x();
  auto ego_world_y = utm_pose.position().y();
  auto ego_world_orientation = utm_pose.orientation();
  auto ego_world_theta =
      planning_rl::GetYawFromQuaternion(ego_world_orientation);
  // auto ego_world_x = frames.back().ego().position().x;
  // auto ego_world_y = frames.back().ego().position().y;
  // auto ego_world_theta = frames.back().ego().position().heading;
  auto odom_pose = container_msg_->odom_pose->pose();
  auto ego_odo_x = odom_pose.position().x();
  auto ego_odo_y = odom_pose.position().y();
  auto ego_odo_orientation = odom_pose.orientation();
  auto ego_odo_theta = planning_rl::GetYawFromQuaternion(ego_odo_orientation);
  for (auto &point : pred_traj_opt) {
    double obs_ego_x, obs_ego_y, obs_ego_theta;
    double obs_odo_x, obs_odo_y, obs_odo_theta;
    WorldCoordToVehicleCoord(ego_world_x, ego_world_y, ego_world_theta, point.x,
                             point.y, point.theta, obs_ego_x, obs_ego_y,
                             obs_ego_theta);
    VehicleCoordToOdometryCoord(ego_odo_x, ego_odo_y, ego_odo_theta, obs_ego_x,
                                obs_ego_y, obs_ego_theta, obs_odo_x, obs_odo_y,
                                obs_odo_theta);
    // std::cout << "pointx: " << std::endl;
    // std::cout << point.x << std::endl;
    // std::cout << obs_odo_x << std::endl;
    // std::cout << "pointy: " << std::endl;
    // std::cout << point.y << std::endl;
    // std::cout << obs_odo_y << std::endl;
    // std::cout << "pointtheta: " << std::endl;
    // std::cout << point.theta << std::endl;
    // std::cout << obs_odo_theta << std::endl;
    point.x = obs_odo_x;
    point.y = obs_odo_y;
    point.theta = obs_odo_theta;
  }
}

void PlanningRLManager::PublishFakeTrajectory() {
  auto fake_trajectory = fake_trajectory_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(fake_trajectory);
  fake_trajectory->mutable_header()->set_module_name("planning_rl");
  fake_trajectory->mutable_header()->set_frame_id("ODOM");
  fake_trajectory->mutable_estop()->set_is_estop(true);
  fake_trajectory->set_gear(neodrive::global::status::GEAR_PARKING);
  fake_trajectory->mutable_header()->set_sequence_num(sequence_num_);
  fake_trajectory->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  planning_pub_->Write(fake_trajectory);
}

void PlanningRLManager::PublishRLFakeTrajectory() {
  auto fake_trajectory = fake_trajectory_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(fake_trajectory);
  fake_trajectory->mutable_header()->set_module_name("planning_rl");
  fake_trajectory->mutable_header()->set_frame_id("ODOM");
  fake_trajectory->mutable_estop()->set_is_estop(true);
  fake_trajectory->set_gear(neodrive::global::status::GEAR_PARKING);
  fake_trajectory->mutable_header()->set_sequence_num(sequence_num_);
  fake_trajectory->mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  planning_rl_pub_->Write(fake_trajectory);
}

bool PlanningRLManager::PublishPlanningRLResult(
    const std::vector<TrajectoryPoint2D> &pred_traj_opt) {
  auto response = planning_rl_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(response);
  response->mutable_header()->set_module_name("planning_rl");
  response->mutable_header()->set_sequence_num(0);
  response->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  std::string frame_id = 0 ? "ODOM" : "UTM";
  response->mutable_header()->set_frame_id(frame_id);
  std::vector<double> s_l(pred_traj_opt.size(), 0.0);
  for (int i = 1; i < pred_traj_opt.size(); i++) {
    s_l[i] =
        s_l[i - 1] + sqrt(pow(pred_traj_opt[i].x - pred_traj_opt[i - 1].x, 2) +
                          pow(pred_traj_opt[i].y - pred_traj_opt[i - 1].y, 2));
  }
  int index = 0;
  response->clear_adc_trajectory_point();
  response->mutable_adc_trajectory_point()->Reserve(pred_traj_opt.size());
  for (auto &p : pred_traj_opt) {
    auto add_pt = response->add_adc_trajectory_point();
    add_pt->set_x(p.x);
    add_pt->set_y(p.y);
    add_pt->set_z(0.0);
    add_pt->set_theta(p.theta);
    add_pt->set_curvature(p.kappa);
    add_pt->set_relative_time(p.t + 0.1);
    add_pt->set_speed(p.v);
    add_pt->set_acceleration_s(p.a);
    add_pt->set_curvature_change_rate(0.0);
    add_pt->set_accumulated_s(s_l[index]);
    index++;
  }
  response->clear_adc_path_point();
  response->mutable_adc_path_point()->Reserve(pred_traj_opt.size());
  for (auto &point : pred_traj_opt) {
    auto add_pt = response->add_adc_path_point();
    add_pt->set_x(point.x);
    add_pt->set_y(point.y);
    add_pt->set_z(0.0);
    add_pt->set_heading(point.theta);
    add_pt->set_curvature(point.kappa);
  }
  // speed limit vector
  response->clear_speed_limit_vec();
  for (double t = 0.; t < 2.0; t += 0.2) {
    response->add_speed_limit_vec(6.0);
  }

  response->mutable_estop()->set_is_estop(false);
  // response->mutable_signals()->mutable_signal()->Add(
  //     neodrive::global::planning::ADCSignals::EMERGENCY_LIGHT);
  response->set_gear(neodrive::global::status::GEAR_DRIVE);
  bool pub_flag = planning_rl_pub_->Write(response);
  if (pub_flag) {
    LOG_INFO("PublishPlanningRLResult success");
  } else {
    LOG_INFO("PublishPlanningRLResult failed");
  }
  return pub_flag;
}

bool PlanningRLManager::PublishPlanningRLUDResult(
    const std::vector<PredState> &pred_traj) {
  auto response = planning_rl_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(response);
  response->mutable_header()->set_module_name("planning_rl");
  response->mutable_header()->set_sequence_num(0);
  response->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  std::string frame_id = 0 ? "ODOM" : "UTM";
  response->mutable_header()->set_frame_id(frame_id);
  std::vector<double> s_l(pred_traj.size(), 0.0);
  for (int i = 1; i < pred_traj.size(); i++) {
    s_l[i] = s_l[i - 1] + sqrt(pow(pred_traj[i].x - pred_traj[i - 1].x, 2) +
                               pow(pred_traj[i].y - pred_traj[i - 1].y, 2));
  }
  int index = 0;
  LOG_INFO("Published Traj");
  response->clear_adc_trajectory_point();
  response->mutable_adc_trajectory_point()->Reserve(pred_traj.size());
  for (auto &p : pred_traj) {
    auto add_pt = response->add_adc_trajectory_point();
    add_pt->set_x(p.x);
    add_pt->set_y(p.y);
    add_pt->set_z(0.0);
    add_pt->set_theta(p.heading);
    add_pt->set_curvature(p.curvature);
    add_pt->set_relative_time(index * 0.1 + 0.1);  // tag2
    add_pt->set_speed(p.speed);
    add_pt->set_acceleration_s(p.acc);
    add_pt->set_curvature_change_rate(0.0);
    add_pt->set_accumulated_s(s_l[index]);
    index++;
    // std::cout << "index:" << index << std::endl;
    // std::cout << p.x << std::endl;
    // std::cout << p.y << std::endl;
    // std::cout << p.heading << std::endl;
    // std::cout << p.curvature << std::endl;
    // std::cout << p.speed << std::endl;
    // std::cout << p.acc << std::endl;
    // LOG_INFO("index:{}, x:{}, y:{}, heading:{}, curvature:{}, speed:{},
    // acc:{}", index, p.x, p.y, p.heading, p.curvature, p.speed, p.acc);
  }
  response->clear_adc_path_point();
  response->mutable_adc_path_point()->Reserve(pred_traj.size());
  for (auto &point : pred_traj) {
    auto add_pt = response->add_adc_path_point();
    add_pt->set_x(point.x);
    add_pt->set_y(point.y);
    add_pt->set_z(0.0);
    add_pt->set_heading(point.heading);
    add_pt->set_curvature(point.curvature);
  }
  // speed limit vector
  response->clear_speed_limit_vec();
  for (double t = 0.; t < 2.0; t += 0.2) {
    response->add_speed_limit_vec(6.0);
  }

  response->mutable_estop()->set_is_estop(false);
  // response->mutable_signals()->mutable_signal()->Add(
  //     neodrive::global::planning::ADCSignals::EMERGENCY_LIGHT);
  response->set_gear(neodrive::global::status::GEAR_DRIVE);
  bool pub_flag = planning_rl_pub_->Write(response);
  if (pub_flag) {
    LOG_INFO("PublishPlanningRLUDResult success");
  } else {
    LOG_INFO("PublishPlanningRLUDResult failed");
  }
  return pub_flag;
}

bool PlanningRLManager::ProcessEstopPlanning() {
  auto response = estop_trajectory_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(response);
  response->mutable_header()->set_module_name("planning_rl");
  response->mutable_header()->set_sequence_num(0);
  response->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  std::string frame_id = 0 ? "ODOM" : "UTM";
  response->mutable_header()->set_frame_id(frame_id);

  ADCTrajectoryPoint fake_pt;
  double heading = 0;

  fake_pt.set_theta(heading);
  fake_pt.set_curvature(0.0);
  fake_pt.set_accumulated_s(0.0);
  fake_pt.set_relative_time(0.0);
  fake_pt.set_speed(0.0);
  fake_pt.set_acceleration_s(0.0);
  response->clear_adc_trajectory_point();
  response->mutable_adc_trajectory_point()->Reserve(10);
  for (size_t i = 0; i < 10; ++i) {
    auto add_pt = response->add_adc_trajectory_point();
    add_pt->CopyFrom(fake_pt);
  }

  response->mutable_estop()->set_is_estop(true);
  response->mutable_signals()->mutable_signal()->Add(
      neodrive::global::planning::ADCSignals::EMERGENCY_LIGHT);
  response->set_gear(neodrive::global::status::GEAR_DRIVE);
  bool pub_flag = planning_pub_->Write(response);
  if (pub_flag) {
    LOG_INFO("ProcessEstopPlanning success");
  } else {
    LOG_INFO("ProcessEstopPlanning failed");
  }
  return pub_flag;
}

void PlanningRLManager::FixPredResult(
    const std::vector<PredState> &predicts_odom,
    const ContainerMessageShrPtr &container_msg_) {
  auto time_stamp =
      container_msg_->perception_obstacles->header().timestamp_sec();
  if (time_stamp - last_pred_time_ > 2 || time_stamp - last_pred_time_ < 0) {
    pred_trajs_.clear();
    pred_trajs_.push_back(predicts_odom);
  } else {
    pred_trajs_.push_back(predicts_odom);
    if (pred_trajs_.size() > pred_trajs_size_) {
      pred_trajs_.pop_front();
    }
  }
  last_pred_time_ = time_stamp;
}

void PlanningRLManager::SaveUDResultText(
    const ContainerMessageShrPtr &container_msg_,
    const FeaturePreparationV2 &fp,
    const std::vector<std::vector<double>> &output,
    const std::vector<PredState> &predicts_odom) {
  auto time_stamp =
      container_msg_->perception_obstacles->header().timestamp_sec();
  std::cout.precision(20);
  // std::cout << "time_stamp:" << time_stamp << std::endl;
  auto target_before = fp.target_before();
  // std::cout << "target_before" << std::endl;
  // std::cout << target_before << std::endl;
  // for (int i = 0; i < 1; i++) {
  //   for (int j = 0; j < 165; j++) {
  //     std::cout << 'j:' << j << std::endl;
  //     for (int k = 0; k < 18; k++) {
  //       std::cout << target_before[i][j][k] << "  ";
  //     }
  //     std::cout << std::endl;
  //   }
  // }
  // std::cout << "model_output" << std::endl;
  // std::cout << output << std::endl;
  // std::cout << "predicts_odom" << std::endl;
  // for (auto &pred : predicts_odom) {
  //   std::cout << pred.x << std::endl;
  //   std::cout << pred.y << std::endl;
  //   std::cout << pred.heading << std::endl;
  //   std::cout << pred.curvature << std::endl;
  //   std::cout << pred.speed << std::endl;
  //   std::cout << pred.acc << std::endl;
  // }
}

}  // namespace planning_rl
}  // namespace neodrive

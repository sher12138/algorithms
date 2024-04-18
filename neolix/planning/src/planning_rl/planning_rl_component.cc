#include "planning_rl_component.h"

namespace neodrive {
namespace planning_rl {

bool PlanningRLComponent::Init() {
  INIT_NEOLOG_NAME("planning_rl");
  LOG_INFO("PlanningRLComponent init started.");
  planning_writer_ = node_->CreateWriter<ADCTrajectory>("planning_rl_node");
  // ObstacleManager::Instance();
  container_manager_ = ContainerManager::Instance();
  if (!container_manager_->Init()) {
    return false;
  }
  LOG_INFO("ContainerManager init.");
  if (!PlanningRLManager::Instance()->Init(node_)) {
    LOG_ERROR("PlanningManager init failed!");
    return false;
  }
  std::cout << "PlanningManager init success!" << std::endl;
  planning_rl_manager_ = PlanningRLManager::Instance();
  LOG_INFO("PlanningRLManager init.");
  config::PlanningRLConfig::Instance()->ReLoadConfigFromJson();
  // neodrive::featurenet::FeatureNet_Evaluator::Instance();
  // neodrive::imitation::ImitationNet_Evaluator::Instance();
  // std::cout << "imitationnet" << std::endl;
  neodrive::urbandrivernet::UrbanDriver_Evaluator::Instance();
  std::cout << "urbandriver" << std::endl;
  proc_thread_ = std::make_unique<std::thread>(
      std::bind(&PlanningRLComponent::RunOnce, this));
  neodrive::cyber::scheduler::Instance()->SetInnerThreadAttr(
      "planning_rl_thread", proc_thread_.get());
  return true;
}

void PlanningRLComponent::RunOnce() {
  LOG_WARN("[thread_name]planning_rl");
  LOG_INFO("planning rl start runonce");
  while (!cyber::IsShutdown()) {
    LOG_INFO("planning rl runonce");
    double start_time = cyber::Time::Now().ToSecond();
    if (!Proc()) {
      LOG_ERROR("Process error occurs!");
    }
    double end_time = cyber::Time::Now().ToSecond();
    auto duration_ms = static_cast<long int>((end_time - start_time) * 1000);
    int sleep_ms = 100 - duration_ms;
    if (sleep_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    } else {
      std::this_thread::yield();
    }
  }
  LOG_INFO("cyber Shutdown.");
}

bool PlanningRLComponent::Proc() {
  time_log_.ResetStartTime();
  config::PlanningRLConfig::Instance()->ReLoadConfigFromJson();
  time_log_.RegisterTime("Reload config");
  if (!container_manager_->GetContainerMsg(max_process_msgs_per_interval_,
                                           container_msg_)) {
    time_log_.RegisterTimeAndPrint("Not GetContainerMsg");
    LOG_INFO("GetContainerMsg Failed");
    return true;
  }
  // std::cout << "check_data_1" << std::endl;
  if (!CheckData()) {
    // std::cout << "check_data_2" << std::endl;
    return false;
  }
  // // test
  // auto time_stamp = container_msg_->perception_obstacles->header().timestamp_sec();
  // if (time_stamp > 1685605576.5 || time_stamp < 1685605576.0) {
  //   return true;
  // } 
  time_log_.RegisterTime("Data Prepared");
  DelayDetect();
  PlanningRLManager::Instance()->ProcessUD(container_msg_);
  time_log_.RegisterTimeAndPrint("PlanningRLManager Done");
  // auto perception_obstacles =
  //     container_msg_->perception_obstacles->perception_obstacle_size();
  // auto pose = container_msg_->localization_pose->pose();
  // auto position = pose.position();
  // if (container_msg_->chassis) {
  //   auto speed = container_msg_->chassis->speed_mps();
  //   std::cout << speed << std::endl;
  // } else {
  //   std::cout << "No chasis" << std::endl;
  // }
  return true;
}

bool PlanningRLComponent::CheckData() {
  if (container_msg_->frames->size() < 5) {
    // std::cout << "check_data_3" << std::endl;
    LOG_INFO("Data not Prepared");
    return false;
  }
  if (container_msg_->reference_line_ptr == nullptr) {
    // std::cout << "check_data_4" << std::endl;
    LOG_INFO("Referenceline not Prepared");
    return false;
  }
  return true;
}

bool PlanningRLComponent::WriteJson() {
  Json::Value root = container_msg_->frames->to_json();
  // printf("%s\n", root.toStyledString().c_str());
  Json::StyledWriter sw;
  std::ofstream os;
  os.open("demo.json", std::ios::out | std::ios::app);
  if (!os.is_open())
    std::cout
        << "error:can not find or create the file which named \" demo.json\"."
        << std::endl;
  os << sw.write(root);
  os.close();
  return true;
}

bool PlanningRLComponent::DelayDetect() {
  double start_time_sec = cyber::Time::Now().ToSecond();
  auto sequence_num = container_msg_->sequence_num;
  double delay_time = start_time_sec - container_msg_->timestamp;
  LOG_INFO(
      "process time: {:.4f} msg send time: {:.4f} delay: {:.4f} "
      "sequence_num: {}",
      start_time_sec, container_msg_->timestamp, delay_time, sequence_num);
  return true;
}

}  // namespace planning_rl
}  // namespace neodrive

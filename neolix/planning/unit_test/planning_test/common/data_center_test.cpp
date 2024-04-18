#include "src/planning/common/data_center/data_center.h"

#include <vector>

#include "common_config/config/common_config.h"
#include "gtest/gtest.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

TEST(DataCenterTest, InitFrame) {
  auto planning_config = config::PlanningConfig::Instance();
  auto data_center = DataCenter::Instance();
  auto environment = data_center->mutable_environment();
  uint32_t sequence_num = 0;
  LOG_WARN("test data_center->InitFrame");
  EXPECT_EQ(data_center->InitFrame(sequence_num), ErrorCode::PLANNING_OK);

  sequence_num += 100;
  neodrive::global::status::Chassis chassis;
  chassis.set_speed_mps(1.2);
  DataCenter::Instance()->chassis_msg.ptr->CopyFrom(chassis);
  DataCenter::Instance()->chassis_msg.is_updated = true;
  LOG_WARN("test data_center->InitFrame");
  EXPECT_EQ(data_center->InitFrame(sequence_num), ErrorCode::PLANNING_OK);

  neodrive::global::common::PoseStamped pose;
  neodrive::global::common::TwistStamped twist;
  pose.mutable_pose()->mutable_position()->set_x(0.1);
  pose.mutable_pose()->mutable_position()->set_y(0.2);
  twist.mutable_twist()->mutable_linear()->set_x(1.0);
  twist.mutable_twist()->mutable_linear_acc()->set_x(0.1);
  DataCenter::Instance()->twist_base_link_msg.ptr->CopyFrom(twist);
  DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->CopyFrom(pose);
  DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->CopyFrom(pose);
  DataCenter::Instance()->twist_base_link_msg.is_updated = true;
  DataCenter::Instance()->pose_base_link_in_odometry_msg.is_updated = true;
  DataCenter::Instance()->pose_base_link_in_utm_msg.is_updated = true;
  LOG_WARN("test data_center->InitFrame");
  EXPECT_EQ(data_center->InitFrame(sequence_num), ErrorCode::PLANNING_OK);

  neodrive::global::prediction::PredictionObstacles prediction;
  environment->set_prediction(
      std::make_shared<neodrive::global::prediction::PredictionObstacles>(
          prediction));
  LOG_WARN("test data_center->InitFrame");
  EXPECT_EQ(data_center->InitFrame(sequence_num), ErrorCode::PLANNING_OK);

  auto prediction_obstacle = prediction.add_prediction_obstacle();
  prediction_obstacle->set_id(1);
  prediction_obstacle = prediction.add_prediction_obstacle();
  prediction_obstacle->set_id(2);
  environment->set_prediction(
      std::make_shared<neodrive::global::prediction::PredictionObstacles>(
          prediction));
  LOG_WARN("test data_center->InitFrame");
  EXPECT_EQ(data_center->InitFrame(sequence_num), ErrorCode::PLANNING_OK);
}

TEST(DataCenterTest, CreateShadowFrame) {
  auto data_center = DataCenter::Instance();
  auto frame_ptr = data_center->CreateShadowFrame();
  EXPECT_NE(frame_ptr, nullptr);
}

TEST(DataCenterTest, SaveTask) {
  auto data_center = DataCenter::Instance();
  neodrive::planning::TaskInfo task_info(data_center->CreateShadowFrame(),
                                         data_center->last_frame(), nullptr,
                                         nullptr);
  EXPECT_NE(task_info.current_frame(), nullptr);
  EXPECT_NO_THROW(data_center->SaveTask(task_info));
}

TEST(DataCenterTest, SaveFrame) {
  auto data_center = DataCenter::Instance();
  EXPECT_NO_THROW(data_center->SaveFrame());
}

TEST(DataCenterTest, ClearHistoryFrame) {
  auto data_center = DataCenter::Instance();
  EXPECT_NO_THROW(data_center->ClearHistoryFrame());
}

TEST(DataCenterTest, SetSpeedLimit) {
  auto planning_config = config::PlanningConfig::Instance();
  auto data_center = DataCenter::Instance();
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  data_center->set_drive_strategy_max_speed(
      common_config->drive_strategy_config().non_motorway.max_cruise_speed);
  data_center->mutable_behavior_speed_limits()->set_drive_strategy_max_speed(
      data_center->drive_strategy_max_speed());

  neodrive::global::planning::SpeedLimit limit_1{};
  limit_1.set_source_type(SpeedLimitType::ABNORMAL_WATCH);
  limit_1.add_upper_bounds(1.0);
  limit_1.set_acceleration(0.0);
  limit_1.set_constraint_type(SpeedLimitType::HARD);
  data_center->mutable_behavior_speed_limits()->SetSpeedLimit(limit_1);

  neodrive::global::planning::SpeedLimit limit_2{};
  limit_2.set_source_type(SpeedLimitType::BARRIER_GATE);
  limit_2.add_upper_bounds(2.0);
  limit_2.set_acceleration(0.0);
  limit_2.set_constraint_type(SpeedLimitType::HARD);
  data_center->mutable_behavior_speed_limits()->SetSpeedLimit(limit_2);

  neodrive::global::planning::SpeedLimit limit_3{};
  limit_3.set_source_type(SpeedLimitType::DEGRADATION);
  limit_3.add_upper_bounds(3.0);
  limit_3.set_acceleration(0.0);
  limit_3.set_constraint_type(SpeedLimitType::HARD);
  data_center->mutable_behavior_speed_limits()->SetSpeedLimit(limit_3);

  neodrive::global::planning::SpeedLimit limit_4{};
  limit_4.set_source_type(SpeedLimitType::START_UP);
  limit_4.add_upper_bounds(4.0);
  limit_4.set_acceleration(0.0);
  limit_4.set_constraint_type(SpeedLimitType::HARD);
  data_center->mutable_behavior_speed_limits()->SetSpeedLimit(limit_4);

  EXPECT_EQ(data_center->behavior_speed_limits().speed_limit(), 1.0);
  LOG_INFO("{}", data_center->behavior_speed_limits().AggregateSpeedLimitStr());

  data_center->mutable_behavior_speed_limits()->Reset();
  LOG_INFO("reset speed limit: {}",
           data_center->behavior_speed_limits().AggregateSpeedLimitStr());
  EXPECT_EQ(data_center->behavior_speed_limits().speed_limit(),
            data_center->drive_strategy_max_speed());
}

TEST(DataCenterTest, MonitorString) {
  auto data_center = DataCenter::Instance();
  std::string str = "PILOT_STATE:xxxxxxxxxxxxxxxxxxxxxxxxx";
  data_center->SetMonitorString(str, MonitorItemSource::PILOT_STATE);
  str = "SCENARIO_STATE:xxxxxxxxxxxxxxxxxxxxxxxxx";
  data_center->SetMonitorString(str, MonitorItemSource::SCENARIO_STATE);
  str = "DRIVE_MODE:xxxxxxxxxxxxxxxxxxxxxxxxx";
  data_center->SetMonitorString(str, MonitorItemSource::DRIVE_MODE);
  str = "REFERENCE_LINE:xxxxxxxxxxxxxxxxxxxxxxxxx";
  data_center->SetMonitorString(
      data_center->behavior_speed_limits().AggregateSpeedLimitStr(),
      MonitorItemSource::SPEED_LIMIT);
  data_center->SetMonitorString(str, MonitorItemSource::REFERENCE_LINE);
  str = "MOTION_PLANNING:xxxxxxxxxxxxxxxxxxxxxxxxx";
  data_center->SetMonitorString(str, MonitorItemSource::MOTION_PLANNING);
  data_center->AggregateMonitorString();
  LOG_WARN("monitor_message: {}", data_center->monitor_message().DebugString());
}
}  // namespace planning
}  // namespace neodrive
#include "observer_test.h"

#include "common/data_center/data_center.h"
#include "common/global_data.h"
#include "gtest/gtest.h"
#include "reader_manager/reader_manager.h"
#include "time/time.h"

namespace neodrive {
namespace planning {
ObserverTest observer_test;

void ObserverTest::TestObserveAebMsg() {
  std::shared_ptr<neodrive::cyber::Node> aeb_node =
      std::move(neodrive::cyber::CreateNode("aeb_node", ""));
  auto aeb_cmd_pub = aeb_node->CreateWriter<AebCmd>("/aeb/aeb_cmd");
  observer_->Init(DataCenter::Instance()->node());
  std::shared_ptr<AebCmd> msg = std::make_shared<AebCmd>();
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  msg->mutable_aeb_state()->set_state(AebState::ACTIVE);
  msg->set_brake(-10.);
  aeb_cmd_pub->Write(msg);
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  sleep(1);

  observer_->Observe();
  auto msg1_ptr = DataCenter::Instance()->aeb_cmd_msg.ptr;
  auto &aeb_cmd = *(DataCenter::Instance()->aeb_cmd_msg.ptr);
  EXPECT_EQ(aeb_cmd.aeb_state().state(), AebState::ACTIVE);
  EXPECT_NEAR(aeb_cmd.brake(), -10., 0.01);
  LOG_INFO("receive aeb_cmd msg {}: {}",
           fmt::ptr(DataCenter::Instance()->aeb_cmd_msg.ptr),
           aeb_cmd.DebugString());

  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  msg->mutable_aeb_state()->set_state(AebState::STANDBY);
  aeb_cmd_pub->Write(msg);
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  sleep(1);
  observer_->Observe();
  auto msg2_ptr = DataCenter::Instance()->aeb_cmd_msg.ptr;
  auto &aeb_cmd2 = *(DataCenter::Instance()->aeb_cmd_msg.ptr);
  EXPECT_EQ(msg1_ptr, msg2_ptr);
  EXPECT_EQ(aeb_cmd2.aeb_state().state(), AebState::STANDBY);
  EXPECT_NEAR(aeb_cmd2.brake(), -10., 0.01);
  LOG_INFO("receive aeb_cmd msg {}: {}",
           fmt::ptr(DataCenter::Instance()->aeb_cmd_msg.ptr),
           aeb_cmd2.DebugString());

  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  msg->mutable_aeb_state()->set_state(AebState::STANDBY);
  aeb_cmd_pub->Write(msg);
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  sleep(1);
  observer_->Observe();
  auto &aeb_cmd3 = *(DataCenter::Instance()->aeb_cmd_msg.ptr);
  EXPECT_EQ(aeb_cmd3.aeb_state().state(), AebState::STANDBY);
  LOG_INFO("receive aeb_cmd msg {}: {}",
           fmt::ptr(DataCenter::Instance()->aeb_cmd_msg.ptr),
           aeb_cmd3.DebugString());

  AebCmd tmp_aeb_cmd;
  tmp_aeb_cmd.mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  tmp_aeb_cmd.mutable_aeb_state()->set_state(AebState::ACTIVE);
  tmp_aeb_cmd.set_brake(-10.);
  aeb_cmd_pub->Write(tmp_aeb_cmd);
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(&tmp_aeb_cmd),
           tmp_aeb_cmd.DebugString());
  sleep(1);

  observer_->Observe();
  auto &aeb_cmd4 = *(DataCenter::Instance()->aeb_cmd_msg.ptr);
  auto prev_ptr = DataCenter::Instance()->aeb_cmd_msg.ptr;
  EXPECT_EQ(aeb_cmd4.aeb_state().state(), AebState::ACTIVE);
  EXPECT_NEAR(aeb_cmd4.brake(), -10., 0.01);
  LOG_INFO("receive aeb_cmd msg {} {}: {}", fmt::ptr(prev_ptr),
           fmt::ptr(prev_ptr.get()), aeb_cmd4.DebugString());

  observer_->Observe();
  auto new_ptr = DataCenter::Instance()->aeb_cmd_msg.ptr;
  auto &aeb_cmd5 = *(DataCenter::Instance()->aeb_cmd_msg.ptr);
  EXPECT_EQ(prev_ptr, new_ptr);
  EXPECT_EQ(aeb_cmd5.aeb_state().state(), AebState::ACTIVE);
  EXPECT_NEAR(aeb_cmd5.brake(), -10., 0.01);
  LOG_INFO("receive aeb_cmd msg {} {}: {}", fmt::ptr(new_ptr),
           fmt::ptr(new_ptr.get()), aeb_cmd5.DebugString());

  std::shared_ptr<AebCmd> tmp_msg = std::make_shared<AebCmd>(tmp_aeb_cmd);
  LOG_INFO("tmp_msg {} {} {}: {}", fmt::ptr(&tmp_aeb_cmd), fmt::ptr(tmp_msg),
           fmt::ptr(tmp_msg.get()), tmp_msg->DebugString());
}

void ObserverTest::TestGetNewObservedMessages() {
  std::shared_ptr<neodrive::cyber::Node> aeb_node =
      std::move(neodrive::cyber::CreateNode("aeb_cmd_test", ""));
  auto aeb_cmd_pub = aeb_node->CreateWriter<AebCmd>("/aeb/aeb_cmd_test");
  observer_->Init(DataCenter::Instance()->node());

  LOG_INFO("IsRealityMode: {}",
           neodrive::cyber::common::GlobalData::Instance()->IsRealityMode());

  auto reader_manager = neodrive::common::ReaderManager::Instance();
  neodrive::cyber::ReaderConfig aeb_reader_config;
  aeb_reader_config.channel_name = "/aeb/aeb_cmd_test";
  aeb_reader_config.pending_queue_size = 10;
  auto aeb_reader = reader_manager->CreateReader<AebCmd>(aeb_reader_config);

  std::shared_ptr<AebCmd> msg = std::make_shared<AebCmd>();
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  msg->mutable_aeb_state()->set_state(AebState::ACTIVE);
  msg->set_brake(-10.);
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(1);

  aeb_reader->Observe();
  auto aeb_msg1 = aeb_reader->GetLatestObserved();
  LOG_INFO("receive aeb msg {}: {}", fmt::ptr(aeb_msg1.get()),
           aeb_msg1->DebugString());

  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(0.1);
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(0.1);
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(1);

  LOG_INFO("print all received msgs");
  aeb_reader->Observe();
  for (auto item = aeb_reader->Begin(); item != aeb_reader->End(); item++) {
    auto msg = *item;
    LOG_INFO("receive aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  }

  LOG_INFO("GetNewObservedMessages");
  aeb_reader->Observe();
  auto new_msgs = aeb_reader->GetNewObservedMessages(nullptr);
  EXPECT_EQ(new_msgs.size(), 4);
  for (auto &msg : new_msgs) {
    LOG_INFO("receive aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  }
  LOG_INFO("GetNewObservedMessages");
  aeb_reader->Observe();
  auto new_msgs2 = aeb_reader->GetNewObservedMessages(nullptr);
  EXPECT_EQ(new_msgs2.size(), 4);
  for (auto &msg : new_msgs2) {
    LOG_INFO("receive aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  }
}

void ObserverTest::TestGetNewObservedMessages2() {
  std::shared_ptr<neodrive::cyber::Node> aeb_node =
      std::move(neodrive::cyber::CreateNode("aeb_cmd_test2", ""));
  auto aeb_cmd_pub = aeb_node->CreateWriter<AebCmd>("/aeb/aeb_cmd_test2");
  observer_->Init(DataCenter::Instance()->node());

  LOG_INFO("IsRealityMode: {}",
           neodrive::cyber::common::GlobalData::Instance()->IsRealityMode());

  auto reader_manager = neodrive::common::ReaderManager::Instance();
  neodrive::cyber::ReaderConfig aeb_reader_config;
  aeb_reader_config.channel_name = "/aeb/aeb_cmd_test2";
  aeb_reader_config.pending_queue_size = 1;
  auto aeb_reader = reader_manager->CreateReader<AebCmd>(aeb_reader_config);

  std::shared_ptr<AebCmd> msg = std::make_shared<AebCmd>();
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  msg->mutable_aeb_state()->set_state(AebState::ACTIVE);
  msg->set_brake(-10.);
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(1);

  aeb_reader->Observe();
  auto aeb_msg1 = aeb_reader->GetLatestObserved();
  LOG_INFO("receive aeb msg {}: {}", fmt::ptr(aeb_msg1.get()),
           aeb_msg1->DebugString());

  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(0.1);
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(0.1);
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  LOG_INFO("publish aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  aeb_cmd_pub->Write(msg);
  sleep(1);

  LOG_INFO("print all received msgs");
  aeb_reader->Observe();
  for (auto item = aeb_reader->Begin(); item != aeb_reader->End(); item++) {
    auto msg = *item;
    LOG_INFO("receive aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  }

  LOG_INFO("GetNewObservedMessages");
  aeb_reader->Observe();
  auto new_msgs = aeb_reader->GetNewObservedMessages(nullptr);
  EXPECT_EQ(new_msgs.size(), 1);
  for (auto &msg : new_msgs) {
    LOG_INFO("receive aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  }
  LOG_INFO("GetNewObservedMessages");
  aeb_reader->Observe();
  auto new_msgs2 = aeb_reader->GetNewObservedMessages(nullptr);
  EXPECT_EQ(new_msgs2.size(), 1);
  for (auto &msg : new_msgs2) {
    LOG_INFO("receive aeb msg {}: {}", fmt::ptr(msg.get()), msg->DebugString());
  }
}

TEST(TestObserve, Observe) { observer_test.TestObserveAebMsg(); }

TEST(TestGetNewObservedMessages, Observe) {
  observer_test.TestGetNewObservedMessages();
}

TEST(TestGetNewObservedMessages2, Observe) {
  observer_test.TestGetNewObservedMessages2();
}

}  // namespace planning
}  // namespace neodrive

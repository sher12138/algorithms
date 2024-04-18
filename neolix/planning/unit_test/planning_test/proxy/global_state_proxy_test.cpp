#include "src/planning/proxy/global_state_proxy.h"

#include "common/data_center/data_center.h"
#include "global_adc_status.pb.h"
#include "gtest/gtest.h"

namespace neodrive {
namespace planning {
namespace {

TEST(TestGlobalStateProxy, set_global_state) {
  neodrive::global::status::GlobalState global_state;
  global_state.mutable_header()->set_timestamp_sec(
      common::util::TimeLogger::GetCurrentTimeseocnd());
  global_state.set_state(neodrive::global::status::INIT);
  global_state.mutable_context()->mutable_finish_context()->set_last_state(
      neodrive::global::status::INIT);
  global_state.mutable_context()->mutable_finish_context()->set_result_code(
      neodrive::global::status::RESULT_SUCCESS);
  global_state.set_stop_reason(neodrive::global::status::StopReason::NOT_STOP);
  global_state.set_remote_mode(
      neodrive::global::status::DrivingMode::COMPLETE_MANUAL);
  LOG_INFO("reset global state: {}", global_state.DebugString());
  DataCenter::Instance()->mutable_global_state_proxy()->set_global_state(
      global_state);
}

TEST(TestGlobalStateProxy, set_global_state2) {
  neodrive::global::status::GlobalState global_state;
  global_state.mutable_header()->set_timestamp_sec(
      common::util::TimeLogger::GetCurrentTimeseocnd());
  global_state.set_state(neodrive::global::status::CRUISE);
  global_state.mutable_context()->mutable_finish_context()->set_last_state(
      neodrive::global::status::INIT);
  global_state.mutable_context()->mutable_finish_context()->set_result_code(
      neodrive::global::status::RESULT_FAIL);
  global_state.set_stop_reason(
      neodrive::global::status::StopReason::ESTOP_FROM_PLANNING);
  global_state.set_remote_mode(
      neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE);
  LOG_INFO("reset global state: {}", global_state.DebugString());
  DataCenter::Instance()->mutable_global_state_proxy()->set_global_state(
      std::move(global_state));
}

}  // namespace
}  // namespace planning
}  // namespace neodrive
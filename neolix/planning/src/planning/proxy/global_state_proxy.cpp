#include "global_state_proxy.h"

#include "common/data_center/data_center.h"
#include "common_config/config/common_config.h"

namespace neodrive {
namespace planning {
using neodrive::global::status::GlobalState;
using neodrive::global::status::State_Name;

GlobalStateProxy::GlobalStateProxy() {
  global_state_.mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  global_state_.set_state(neodrive::global::status::INIT);
  global_state_.set_stop_reason(neodrive::global::status::StopReason::NOT_STOP);
  global_state_.mutable_context()->mutable_finish_context()->set_last_state(
      neodrive::global::status::INIT);
  global_state_.mutable_context()->mutable_finish_context()->set_result_code(
      neodrive::global::status::RESULT_SUCCESS);
  global_state_.set_stop_reason(neodrive::global::status::StopReason::NOT_STOP);
  global_state_.set_remote_mode(
      neodrive::global::status::DrivingMode::COMPLETE_MANUAL);
  global_state_.set_reach_station(false);
}

void GlobalStateProxy::SetState(const neodrive::global::status::State state) {
  if (state == global_state_.state()) {
    return;
  }
  LOG_INFO("Update global state: {} -> {}", State_Name(global_state_.state()),
           State_Name(state));
  global_state_.mutable_context()->mutable_finish_context()->set_last_state(
      global_state_.state());
  global_state_.set_state(state);
}

std::string GlobalStateProxy::StateName() const {
  return State_Name(global_state_.state());
}

#define MASTER_STATE_GET_SET_FUNC(VALUE, TYPE)                      \
  bool GlobalStateProxy::is_##VALUE() const {                       \
    return global_state_.state() == neodrive::global::status::TYPE; \
  }                                                                 \
  void GlobalStateProxy::set_##VALUE() {                            \
    SetState(neodrive::global::status::TYPE);                       \
  }

MASTER_STATE_GET_SET_FUNC(init, INIT);
MASTER_STATE_GET_SET_FUNC(cruise, CRUISE);
MASTER_STATE_GET_SET_FUNC(wait, WAIT);
MASTER_STATE_GET_SET_FUNC(finish, FINISH);
MASTER_STATE_GET_SET_FUNC(parking_in, PARKING_IN);
MASTER_STATE_GET_SET_FUNC(parking_out, PARKING_OUT);
MASTER_STATE_GET_SET_FUNC(estop, ESTOP);
MASTER_STATE_GET_SET_FUNC(updc, UPDC);
MASTER_STATE_GET_SET_FUNC(calculate_routing, CALCULATE_ROUTING);
MASTER_STATE_GET_SET_FUNC(restart, RESTART);

#undef MASTER_STATE_GET_SET_FUNC

void GlobalStateProxy::SetFinish(bool success) {
  if (global_state_.state() != neodrive::global::status::FINISH) {
    set_finish();
    if (success) {
      global_state_.mutable_context()
          ->mutable_finish_context()
          ->set_result_code(neodrive::global::status::RESULT_SUCCESS);
    } else {
      global_state_.mutable_context()
          ->mutable_finish_context()
          ->set_result_code(neodrive::global::status::RESULT_FAIL);
    }
    scenario_manager_need_reset_ = true;
    LOG_INFO("update global state: {}", global_state_.DebugString());
  }
}

void GlobalStateProxy::SetHaveTask(bool have_task) {
  global_state_.set_have_task(have_task);
}

bool GlobalStateProxy::HaveTask() const { return global_state_.have_task(); }

void GlobalStateProxy::SetRequestYield(bool request_yield) {
  global_state_.set_request_to_yield(request_yield);
}

void GlobalStateProxy::SetReachStation(bool reach_station) {
  if (reach_station) LOG_INFO("Reach station success!");
  is_station_stop_ = reach_station;
}

void GlobalStateProxy::SyncReachStation() {
  global_state_.set_reach_station(is_station_stop_);
  auto &interface_config =
      config::PlanningConfig::Instance()->plan_config().human_interface;
  if (is_station_stop_ &&
      ++reach_station_count_ >= interface_config.reach_station_keep_cnt) {
    is_station_stop_ = false;
    reach_station_count_ = 0;
  }
}

void GlobalStateProxy::ResetInitState() {
  if (global_state_.state() != neodrive::global::status::INIT) {
    set_init();
    global_state_.mutable_context()->mutable_finish_context()->set_result_code(
        neodrive::global::status::RESULT_SUCCESS);
    global_state_.set_stop_reason(
        neodrive::global::status::StopReason::NOT_STOP);
    LOG_INFO("reset global state: {}", global_state_.DebugString());
  }
}

bool GlobalStateProxy::IsInSpecailState() const {
  return is_parking_in() || is_parking_out() || is_updc() ||
         is_calculate_routing() || is_restart() || is_estop();
}

}  // namespace planning
}  // namespace neodrive

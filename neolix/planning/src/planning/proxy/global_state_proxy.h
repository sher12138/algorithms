#pragma once

#include <mutex>

#include "common/macros.h"
#include "global_adc_status.pb.h"
#include "node/writer.h"

namespace neodrive {
namespace planning {

class GlobalStateProxy {
 public:
  GlobalStateProxy();
  ~GlobalStateProxy() = default;
  void SetState(const neodrive::global::status::State state);
  std::string StateName() const;
  bool is_init() const;
  bool is_cruise() const;
  bool is_wait() const;
  bool is_finish() const;
  bool is_parking_in() const;
  bool is_parking_out() const;
  bool is_estop() const;
  bool is_updc() const;
  bool is_calculate_routing() const;
  bool is_restart() const;

  void set_init();
  void set_cruise();
  void set_wait();
  void set_finish();
  void set_parking_in();
  void set_parking_out();
  void set_estop();
  void set_updc();
  void set_calculate_routing();
  void set_restart();

  void SetFinish(bool success);
  void SetHaveTask(bool have_task);
  void SetRequestYield(bool request_yield);
  void ResetInitState();
  bool IsInSpecailState() const;
  bool HaveTask() const;
  void SetReachStation(bool reach_station);
  void SyncReachStation();

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(neodrive::global::status::GlobalState,
                                       global_state);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, scenario_manager_need_reset);

 private:
  neodrive::global::status::GlobalState global_state_;
  std::atomic<bool> is_station_stop_;
  std::mutex global_mutex_;
  bool scenario_manager_need_reset_{false};
  uint32_t reach_station_count_{0};
};

}  // namespace planning
}  // namespace neodrive

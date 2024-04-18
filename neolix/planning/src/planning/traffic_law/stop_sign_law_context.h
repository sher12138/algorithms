#pragma once

#include <sstream>
#include <string>
#include <unordered_map>

namespace neodrive {
namespace planning {

class RouteStopSign {
 public:
  RouteStopSign() {}
  ~RouteStopSign() {}
  RouteStopSign(const uint64_t id, double start_s, double end_s);

  uint64_t ObjectId() const;
  double StartRouteS() const;
  double EndRouteS() const;
  double StopTime() const;
  bool IsValid() const;

  void SetObjectId(const uint64_t id);
  void SetStartRouteS(double start_s);
  void SetEndRouteS(double end_s);
  void SetStopTime(double stop_time);
  void SetIsValid(bool valid);
  void UpdateS(double start_s, double end_s);

  std::string DebugString() const;

 private:
  uint64_t object_id_;
  double start_route_s_ = 0.0;
  double end_route_s_ = 0.0;
  double stop_time_ = -1.0;
  bool is_valid_ = true;
};

using StopSignTable = std::unordered_map<uint64_t, RouteStopSign>;

enum class StopSignState { NORMAL_CRUISE = 0, PRE_STOP, STOP, CREEP, FINISH };

class StopsignLawContext {
 public:
  StopsignLawContext();
  ~StopsignLawContext();
  const RouteStopSign& StopSign() const;
  RouteStopSign* MutableStopSign();
  StopSignState StopsignState() const;
  double LastStopTime() const;
  double StopLineS() const;

  void SetStopSign(RouteStopSign stop_sign);
  void SetStopSignState(StopSignState state);
  void SetStopLineS(double stop_line_s);
  void SetLastStopTime(double time);

 private:
  RouteStopSign stop_sign_;
  StopSignState stop_sign_state_ = StopSignState::NORMAL_CRUISE;
  double stop_line_s_;
  double last_stop_time_;
};

}  // namespace planning
}  // namespace neodrive

#include "stop_sign_law_context.h"

namespace neodrive {
namespace planning {

RouteStopSign::RouteStopSign(const uint64_t id, double start_s, double end_s)
    : object_id_(id), start_route_s_(start_s), end_route_s_(end_s) {}

uint64_t RouteStopSign::ObjectId() const { return object_id_; }

double RouteStopSign::StartRouteS() const { return start_route_s_; }

double RouteStopSign::EndRouteS() const { return end_route_s_; }

double RouteStopSign::StopTime() const { return stop_time_; }

bool RouteStopSign::IsValid() const { return is_valid_; }

void RouteStopSign::SetObjectId(const uint64_t id) { object_id_ = id; }

void RouteStopSign::SetStartRouteS(double start_s) { start_route_s_ = start_s; }

void RouteStopSign::SetEndRouteS(double end_s) { end_route_s_ = end_s; }

void RouteStopSign::SetStopTime(double stop_time) { stop_time_ = stop_time; }

void RouteStopSign::SetIsValid(bool valid) { is_valid_ = valid; }

void RouteStopSign::UpdateS(double start_s, double end_s) {
  start_route_s_ = start_s;
  end_route_s_ = end_s;
}

std::string RouteStopSign::DebugString() const {
  std::ostringstream ret;
  ret << "id:" << object_id_ << ", start_s:" << start_route_s_
      << ", end_s:" << end_route_s_ << ", stop_time:" << std::fixed
      << stop_time_ << ", is_valid:" << std::boolalpha << is_valid_;
  return ret.str();
}

StopsignLawContext::StopsignLawContext() {}
StopsignLawContext::~StopsignLawContext() {}

const RouteStopSign& StopsignLawContext::StopSign() const { return stop_sign_; }

RouteStopSign* StopsignLawContext::MutableStopSign() { return &stop_sign_; }

double StopsignLawContext::LastStopTime() const { return last_stop_time_; }

StopSignState StopsignLawContext::StopsignState() const {
  return stop_sign_state_;
};

double StopsignLawContext::StopLineS() const { return stop_line_s_; }

void StopsignLawContext::SetStopSign(RouteStopSign stop_sign) {
  stop_sign_ = stop_sign;
}

void StopsignLawContext::SetStopSignState(StopSignState state) {
  stop_sign_state_ = state;
}

void StopsignLawContext::SetStopLineS(double stop_line_s) {
  stop_line_s_ = stop_line_s;
}

void StopsignLawContext::SetLastStopTime(double time) {
  last_stop_time_ = time;
}

}  // namespace planning
}  // namespace neodrive

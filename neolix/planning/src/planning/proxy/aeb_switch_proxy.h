#pragma once

#include "aeb_msgs.pb.h"
#include "planning/config/planning_config.h"

namespace neodrive {
namespace planning {
using ForbidAeb = global::planning::ForbidAeb;

class AebSwitchProxy {
 public:
  AebSwitchProxy() { frobid_aeb_.set_value(false); }
  void SetForbidAeb(bool is_forbid);
  ~AebSwitchProxy() = default;
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ForbidAeb, frobid_aeb);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_updated);

 private:
  ForbidAeb frobid_aeb_;
  bool is_updated_{false};
  config::PlanningConfig* planning_config_ = config::PlanningConfig::Instance();
};

}  // namespace planning
}  // namespace neodrive

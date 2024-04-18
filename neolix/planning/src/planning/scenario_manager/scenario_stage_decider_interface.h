#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "common/macros.h"
#include "common/planning_logger.h"
#include "planning/task/task_info.h"
#include "proto/scenario_manager_msgs.pb.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/planning_code_define.h"
#include "state_machine/state_machine.h"

namespace neodrive {
namespace planning {
class ScenarioStageDeciderInterface {
  using StateMachine = neodrive::common::state_machine::StateMachine;
  using StateMachineShrPtr = std::shared_ptr<StateMachine>;

 public:
  ScenarioStageDeciderInterface() = default;
  virtual ~ScenarioStageDeciderInterface() = default;

 public:
  virtual bool Init() = 0;
  virtual bool Reset() = 0;
  virtual ErrorCode RunOnce() = 0;

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, initialized);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::string, curr_state_str);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::string, prev_state_str);

 public:
  virtual const std::string &Name() const { return name_; }

 protected:
  bool initialized_{false};
  std::string name_{""};
  // state machine
  // StateMachineShrPtr state_machine_{std::make_shared<StateMachine>()};
  StateMachine state_machine_;
  // current state (from state machine)
  std::string curr_state_str_{"DEFAULT"};
  std::string prev_state_str_{"DEFAULT"};
  bool is_state_change_{false};
  DataCenter *data_center_{DataCenter::Instance()};
};

using ScenarioStageDeciderInterfacePtr = ScenarioStageDeciderInterface *;
using ScenarioStageDeciderInterfaceShrPtr =
    std::shared_ptr<ScenarioStageDeciderInterface>;

// macro of reflect factory of singletion FunctionTask;
// todo.
class ScenarioStageDeciderFactory {
 private:
  ScenarioStageDeciderFactory() = default;
  ScenarioStageDeciderFactory(const ScenarioStageDeciderFactory &) = delete;
  ScenarioStageDeciderFactory &operator=(const ScenarioStageDeciderFactory &) =
      delete;

 public:
  ~ScenarioStageDeciderFactory() = default;

 public:
  static ScenarioStageDeciderFactory *Instance() {
    static ScenarioStageDeciderFactory instance;
    return &instance;
  }

 public:
  ScenarioStageDeciderInterface *Instantiate(const std::string &name) {
    auto iter = constructor_map_.find(name);
    if (iter != constructor_map_.end()) {
      return (*iter).second();
    } else {
      LOG_ERROR("ScenarioStageDeciderFactory Instantiate failed: {}", name);
      return nullptr;
    }
  }

  void Register(const std::string &name,
                std::function<ScenarioStageDeciderInterface *()> func_handle) {
    if (constructor_map_.find(name) == constructor_map_.end()) {
      constructor_map_.insert(std::make_pair(name, func_handle));
    }
  }

 private:
  std::unordered_map<std::string,
                     std::function<ScenarioStageDeciderInterface *()>>
      constructor_map_;
};

#define REGISTER_SCENARIO_STAGE_DECIDER(name)                    \
  static inline ScenarioStageDeciderInterface                    \
      *InstantiateScenarioStageDeciderHandler_##name() {         \
    return name::Instance();                                     \
  }                                                              \
                                                                 \
  static struct RegisterHandler_##name {                         \
    RegisterHandler_##name() {                                   \
      ScenarioStageDeciderFactory::Instance()->Register(         \
          #name, InstantiateScenarioStageDeciderHandler_##name); \
    }                                                            \
  } register_handler_##name;

}  // namespace planning
}  // namespace neodrive
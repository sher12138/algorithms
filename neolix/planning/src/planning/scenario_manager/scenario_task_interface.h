#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "common/macros.h"
#include "common/planning_logger.h"
#include "planning/common/data_center/data_center.h"
#include "planning/common/planning_types.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/task/task_info.h"
#include "state_machine/state_machine.h"

namespace neodrive {
namespace planning {
class ScenarioTaskInterface {
 public:
  ScenarioTaskInterface() = default;
  virtual ~ScenarioTaskInterface() = default;

 public:
  // virtual void Init() = 0;
  virtual ErrorCode Execute(TaskInfo &task_info) = 0;
  virtual void SaveTaskResults(TaskInfo &task_info) = 0;
  virtual void Reset() = 0;
  const std::string &Name() { return name_; }
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, initialized);

 protected:
  bool initialized_{false};
  std::string name_{""};
  DataCenter *data_center_{DataCenter::Instance()};
  const VehicleStateProxy &vehicle_state_{data_center_->vehicle_state_proxy()};
};

using ScenarioTaskInterfacePtr = ScenarioTaskInterface *;
using ScenarioTaskInterfaceShrPtr = std::shared_ptr<ScenarioTaskInterface>;

// macro of reflect factory of singletion FunctionTask;
// todo.
class ScenarioTaskFactory {
 private:
  ScenarioTaskFactory() = default;
  ScenarioTaskFactory(const ScenarioTaskFactory &) = delete;
  ScenarioTaskFactory &operator=(const ScenarioTaskFactory &) = delete;

 public:
  ~ScenarioTaskFactory() = default;

 public:
  static ScenarioTaskFactory *Instance() {
    static ScenarioTaskFactory instance;
    return &instance;
  }

  ScenarioTaskInterface *Instantiate(const std::string &name) {
    auto iter = constructor_map_.find(name);
    if (iter != constructor_map_.end()) {
      return (*iter).second();
    } else {
      LOG_ERROR("ScenarioTaskFactory Instantiate failed: {}", name);
      return nullptr;
    }
  }

  void Register(const std::string &name,
                std::function<ScenarioTaskInterface *()> func_handle) {
    if (constructor_map_.find(name) == constructor_map_.end()) {
      constructor_map_.insert(std::make_pair(name, func_handle));
    }
  }

 private:
  std::unordered_map<std::string, std::function<ScenarioTaskInterface *()>>
      constructor_map_;
};

#define REGISTER_SCENARIO_TASK(name)                     \
  static inline ScenarioTaskInterface                    \
      *InstantiateScenarioTaskHandler_##name() {         \
    return name::Instance();                             \
  }                                                      \
                                                         \
  static struct RegisterHandler_##name {                 \
    RegisterHandler_##name() {                           \
      ScenarioTaskFactory::Instance()->Register(         \
          #name, InstantiateScenarioTaskHandler_##name); \
    }                                                    \
  } register_handler_##name;

}  // namespace planning
}  // namespace neodrive
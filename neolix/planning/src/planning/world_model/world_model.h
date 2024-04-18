#pragma once

#include "cyber.h"
#include "world_model/coordinator_transform/coordinator_transform.h"

namespace neodrive {
namespace world_model {

class WorldModel {
  DECLARE_SINGLETON(WorldModel);

 public:
  ~WorldModel();

  bool Init(std::shared_ptr<neodrive::cyber::Node> &node);

  std::string Name() const;

  void RunOnce();

 private:
  std::atomic<bool> initialized_{false};
  std::shared_ptr<neodrive::cyber::Node> node_{nullptr};
  CoordinateTransform *coordinate_tranform_{CoordinateTransform::Instance()};
};

}  // namespace world_model
}  // namespace neodrive

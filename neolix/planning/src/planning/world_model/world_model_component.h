#pragma once

#include <thread>

#include "cyber.h"
#include "world_model/coordinator_transform/coordinator_transform.h"

namespace neodrive {
namespace world_model {

class WorldModelComponent final : public neodrive::cyber::Component<> {
 public:
  WorldModelComponent() = default;
  ~WorldModelComponent();

  bool Init() override;

  std::string Name() const;

 private:
  void Proc();

 private:
  std::atomic<bool> initialized_{false};
  std::unique_ptr<std::thread> proc_thread_;
  CoordinateTransform *coordinate_tranform_{CoordinateTransform::Instance()};
};

CYBER_REGISTER_COMPONENT(WorldModelComponent);

}  // namespace world_model
}  // namespace neodrive

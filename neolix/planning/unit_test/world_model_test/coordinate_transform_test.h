#pragma once

#include "src/planning/world_model/coordinator_transform/coordinator_transform.h"

namespace neodrive {
namespace world_model {

class CoordinateTransformTest {
 public:
  CoordinateTransformTest() = default;
  ~CoordinateTransformTest() = default;
  void TestGetBaseLinkTwistFromUtmPose();
  void TestGetBaseLinkTwistFromImu();
  void TestGetBaseLinkPose1();
  void TestGetBaseLinkPose2();

 private:
  CoordinateTransform *coordinate_transform_{CoordinateTransform::Instance()};
};

}  // namespace world_model
}  // namespace neodrive

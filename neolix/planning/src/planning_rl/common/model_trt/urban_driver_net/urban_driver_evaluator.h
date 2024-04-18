#pragma once

#include <string>
#include <vector>
#include <iostream>
#include "config/planning_rl_config.h"
#include "utils/planning_rl_macros.h"

#include "trt_engine.h"

namespace neodrive {
namespace urbandrivernet {

class UrbanDriver_Evaluator {
  DECLARE_SINGLETON(UrbanDriver_Evaluator);

 public:
  /// Constructor
  // UrbanDriver_Evaluator();

  /// Destructor
  ~UrbanDriver_Evaluator() = default;

 public:
  /// Get the name of the evaluator
  std::string GetName();

  /// Debug info of the evaluator
  std::string DebugString();

  /// Evaluate by model

  std::vector<std::vector<std::vector<double>>> Evaluate(
      float inputs0[1][1][5][18], float inputs1[1][64][5][18], float inputs2[1][100][5][18],
      float inputs3[1][64], float inputs4[1], bool inputs5[1][1][5],
      bool inputs6[1][64][5], bool inputs7[1][100][5]);

 private:
  /// Load neural network model from path
  void LoadModel();

  /// Init models
  void InitModel();

 private:
  std::shared_ptr<ZeroCopyTrtEngine> engine_{nullptr};
  std::string input0_blob_name_{"input0"};
  std::string input1_blob_name_{"input1"};
  std::string input2_blob_name_{"input2"};
  std::string input3_blob_name_{"input3"};
  std::string input4_blob_name_{"input4"};
  std::string input5_blob_name_{"input5"};
  std::string input6_blob_name_{"input6"};
  std::string input7_blob_name_{"input7"};
  std::string output_blob_name_{"outputs"};
};

}  // namespace urbandrivernet
}  // namespace neodrive
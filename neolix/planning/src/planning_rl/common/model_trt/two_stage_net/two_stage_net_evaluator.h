#pragma once

#include <string>
#include <vector>

#include "config/planning_rl_config.h"
#include "trt_engine.h"
#include "utils/planning_rl_macros.h"

namespace neodrive {
namespace two_stage_net {

class TwoStageNet_Evaluator {
 public:
  /// Constructor
  TwoStageNet_Evaluator();

  /// Destructor
  ~TwoStageNet_Evaluator() = default;

 public:
  /// Get the name of the evaluator
  std::string GetName();

  /// Debug info of the evaluator
  std::string DebugString();

  /// Evaluate by model

  std::vector<std::vector<std::vector<float>>> Evaluate(
      float inputs0[16][1][957]);

 private:
  /// Load neural network model from path
  void LoadModel();

  /// Init models
  void InitModel();

 private:
  std::shared_ptr<ZeroCopyTrtEngine> engine_{nullptr};
  std::string input0_blob_name_{"input"};
  std::string output0_blob_name_{"output_short"};
  std::string output1_blob_name_{"output_short_sa"};
  std::string output2_blob_name_{"output_long"};
  std::string output3_blob_name_{"output_long_sa"};
};

}  // namespace two_stage_net
}  // namespace neodrive
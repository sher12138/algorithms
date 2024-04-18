#pragma once

#include <string>
#include <vector>
#include "config/planning_rl_config.h"
#include "utils/planning_rl_macros.h"

#include "trt_engine.h"

namespace neodrive {
namespace imitation {

class ImitationNet_Evaluator {
  DECLARE_SINGLETON(ImitationNet_Evaluator);

 public:
  /// Constructor
  // ImitationNet_Evaluator();

  /// Destructor
  ~ImitationNet_Evaluator() = default;

 public:
  /// Get the name of the evaluator
  std::string GetName();

  /// Debug info of the evaluator
  std::string DebugString();

  /// Evaluate by model
  std::vector<std::vector<double>> Evaluate(float inputs1[1 * 5 * 1815]);

 private:
  /// Load neural network model from path
  void LoadModel();

  /// Init models
  void InitModel();

 private:
  std::shared_ptr<ZeroCopyTrtEngine> engine_{nullptr};
  std::string input1_blob_name_{"input"};
  std::string output_blob_name_{"output"};
};

}  // namespace imitation
}  // namespace neodrive

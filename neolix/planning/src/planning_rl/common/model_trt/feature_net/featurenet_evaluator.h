#pragma once

#include <string>
#include <vector>
#include "config/planning_rl_config.h"
#include "utils/planning_rl_macros.h"

#include "trt_engine.h"

namespace neodrive {
namespace featurenet {

class FeatureNet_Evaluator {
  DECLARE_SINGLETON(FeatureNet_Evaluator);

 public:
  /// Constructor
  // FeatureNet_Evaluator();

  /// Destructor
  ~FeatureNet_Evaluator() = default;

 public:
  /// Get the name of the evaluator
  std::string GetName();

  /// Debug info of the evaluator
  std::string DebugString();

  /// Evaluate by model

  std::vector<std::vector<double>> Evaluate(
      float inputs0[1][100], float inputs1[1][100], float inputs2[1][100],
      float inputs3[100][3], float inputs4[100][2], float inputs5[100][3],
      float inputs6[100][2], float inputs7[1][100], float inputs8[1][100],
      float inputs9[1][100][16], float inputs10[1][100][16], float inputs11[1],
      float inputs12[1][100][3]);

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
  std::string input8_blob_name_{"input8"};
  std::string input9_blob_name_{"input9"};
  std::string input10_blob_name_{"input10"};
  std::string input11_blob_name_{"input11"};
  std::string input12_blob_name_{"input12"};
  std::string output_blob_name_{"output"};
};

}  // namespace featurenet
}  // namespace neodrive
#include "featurenet_evaluator.h"
#include <stdio.h>
#include <iostream>

namespace neodrive {
namespace featurenet {

#define WARMUP (3)

FeatureNet_Evaluator::FeatureNet_Evaluator() {
  LoadModel();
  InitModel();
}

std::string FeatureNet_Evaluator::GetName() { return "FEATURENET_EVALUATOR"; }

std::string FeatureNet_Evaluator::DebugString() { return "TODO"; }

std::vector<std::vector<double>> FeatureNet_Evaluator::Evaluate(
    float inputs0[1][100], float inputs1[1][100], float inputs2[1][100],
    float inputs3[100][3], float inputs4[100][2], float inputs5[100][3],
    float inputs6[100][2], float inputs7[1][100], float inputs8[1][100],
    float inputs9[1][100][16], float inputs10[1][100][16], float inputs11[1],
    float inputs12[1][100][3]) {
  auto input0_blob = engine_->GetBlob(input0_blob_name_);
  auto input1_blob = engine_->GetBlob(input1_blob_name_);
  auto input2_blob = engine_->GetBlob(input2_blob_name_);
  auto input3_blob = engine_->GetBlob(input3_blob_name_);
  auto input4_blob = engine_->GetBlob(input4_blob_name_);
  auto input5_blob = engine_->GetBlob(input5_blob_name_);
  auto input6_blob = engine_->GetBlob(input6_blob_name_);
  auto input7_blob = engine_->GetBlob(input7_blob_name_);
  auto input8_blob = engine_->GetBlob(input8_blob_name_);
  auto input9_blob = engine_->GetBlob(input9_blob_name_);
  auto input10_blob = engine_->GetBlob(input10_blob_name_);
  auto input11_blob = engine_->GetBlob(input11_blob_name_);
  auto input12_blob = engine_->GetBlob(input12_blob_name_);

  input0_blob->CopyIn(inputs0, 100 * sizeof(float));
  input1_blob->CopyIn(inputs1, 100 * sizeof(float));
  input2_blob->CopyIn(inputs2, 100 * sizeof(float));
  input3_blob->CopyIn(inputs3, 300 * sizeof(float));
  input4_blob->CopyIn(inputs4, 200 * sizeof(float));
  input5_blob->CopyIn(inputs5, 300 * sizeof(float));
  input6_blob->CopyIn(inputs6, 200 * sizeof(float));
  input7_blob->CopyIn(inputs7, 100 * sizeof(float));
  input8_blob->CopyIn(inputs8, 100 * sizeof(float));
  input9_blob->CopyIn(inputs9, 1600 * sizeof(float));
  input10_blob->CopyIn(inputs10, 1600 * sizeof(float));
  input11_blob->CopyIn(inputs11, 1 * sizeof(float));
  input12_blob->CopyIn(inputs12, 300 * sizeof(float));

  if (!engine_->Infer()) {
    std::cout << "trt engine infer failed!" << std::endl;
  }

  auto output_blob = engine_->GetBlob(output_blob_name_);
  auto shape = output_blob->GetShape();

  std::cout << "start check output shape ..." << std::endl;
  for (auto x : shape) {
    // std::cout << "output dim:" << x << std::endl;
  }
  std::cout << "end check output shape ..." << std::endl;

  auto* ans = static_cast<float*>(output_blob->HostPtr());
  std::vector<std::vector<double>> result;
  std::cout << "开始预测" << std::endl;
  for (int i = 0; i < 100; i++) {
    std::vector<double> tmp_res;
    for (int j = 0; j < 14; j++) {
      tmp_res.push_back(ans[i * 14 + j]);
      // std::cout << i << "\t" << j << "\t" << ans[i * 100 + j] << std::endl;
    }
    result.push_back(tmp_res);
  }
  std::cout << "结束预测" << std::endl;

  return result;
}

void FeatureNet_Evaluator::LoadModel() {
  auto& model_conf =
      planning_rl::config::PlanningRLConfig::Instance()->model_config();
  auto model_path = model_conf.feature_net.model_path;
  engine_.reset(new ZeroCopyTrtEngine(model_path));
  printf("feature-net model loaded: {%s}\n", "model_trt.engine");
  LOG_INFO("feature-net model loaded: {}", model_path);
}

void FeatureNet_Evaluator::InitModel() {
  auto& logger = Logger::GetTRTLogger();

  if (!engine_->Init(logger)) {
    printf("feature-net init failed!");
  } else {
    printf("feature-net model init: {%s}\n", "model_trt.engine");
  }

  for (int i = 0; i < WARMUP; ++i) {
    engine_->Infer();
  }
}

}  // namespace featurenet
}  // namespace neodrive

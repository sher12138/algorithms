#include "two_stage_net_evaluator.h"

#include <stdio.h>

#include <iostream>

namespace neodrive {
namespace two_stage_net {

#define WARMUP (3)

TwoStageNet_Evaluator::TwoStageNet_Evaluator() {
  LoadModel();
  InitModel();
}

std::string TwoStageNet_Evaluator::GetName() { return "TwoStageNet_EVALUATOR"; }

std::string TwoStageNet_Evaluator::DebugString() { return "TODO"; }

std::vector<std::vector<std::vector<float>>> TwoStageNet_Evaluator::Evaluate(
    float inputs0[16][1][957]) {
  auto input0_blob = engine_->GetBlob(input0_blob_name_);
  input0_blob->CopyIn(inputs0, 16 * 1 * 957 * sizeof(float));

  if (!engine_->Infer()) {
    std::cout << "trt engine infer failed!" << std::endl;
  }

  auto output0_blob = engine_->GetBlob(output0_blob_name_);
  auto output1_blob = engine_->GetBlob(output1_blob_name_);
  auto output2_blob = engine_->GetBlob(output2_blob_name_);
  auto output3_blob = engine_->GetBlob(output3_blob_name_);
  auto shape = output0_blob->GetShape();

  std::cout << "start check output shape ..." << std::endl;
  for (auto x : shape) {
    std::cout << "output dim:" << x << std::endl;
  }
  std::cout << "end check output shape ..." << std::endl;

  auto* ans0 = static_cast<float*>(output0_blob->HostPtr());
  auto* ans1 = static_cast<float*>(output1_blob->HostPtr());
  auto* ans2 = static_cast<float*>(output2_blob->HostPtr());
  auto* ans3 = static_cast<float*>(output3_blob->HostPtr());

  std::vector<std::vector<std::vector<float>>> result;
  std::vector<std::vector<float>> result0;
  for (int i = 0; i < 16; i++) {
    std::vector<float> tmp_res0;
    for (int j = 0; j < 3; j++) {
      tmp_res0.push_back(ans0[i * 16 + j]);
      std::cout << i << "\t" << j << "\t" << ans0[i * 16 + j] << std::endl;
    }
    result0.push_back(tmp_res0);
  }
  std::vector<std::vector<float>> result1;
  for (int i = 0; i < 16; i++) {
    std::vector<float> tmp_res1;
    for (int j = 0; j < 2; j++) {
      std::cout << i << "\t" << j << "\t" << ans1[i * 16 + j] << std::endl;
    }
    result1.push_back(tmp_res1);
  }
  std::vector<std::vector<float>> result2;
  for (int i = 0; i < 16; i++) {
    std::vector<float> tmp_res2;
    for (int j = 0; j < 4; j++) {
      std::cout << i << "\t" << j << "\t" << ans2[i * 16 + j] << std::endl;
    }
    result2.push_back(tmp_res2);
  }
  std::vector<std::vector<float>> result3;
  for (int i = 0; i < 16; i++) {
    std::vector<float> tmp_res3;
    for (int j = 0; j < 2; j++) {
      std::cout << i << "\t" << j << "\t" << ans3[i * 16 + j] << std::endl;
    }
    result3.push_back(tmp_res3);
  }

  result.push_back(result0);
  result.push_back(result1);
  result.push_back(result2);
  result.push_back(result3);

  std::cout << "结束预测" << std::endl;

  return result;
}

void TwoStageNet_Evaluator::LoadModel() {
  auto& model_conf =
      planning_rl::config::PlanningRLConfig::Instance()->model_config();
  auto model_path = model_conf.two_stage_net.model_path;
  engine_.reset(new ZeroCopyTrtEngine(model_path));
  printf("feature-net model loaded: {%s}\n", "model_trt.engine");
}

void TwoStageNet_Evaluator::InitModel() {
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

}  // namespace two_stage_net
}  // namespace neodrive

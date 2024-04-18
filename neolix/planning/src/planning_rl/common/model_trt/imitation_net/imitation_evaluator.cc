#include "imitation_evaluator.h"
#include <stdio.h>
#include <iostream>

namespace neodrive {
namespace imitation {

#define WARMUP (3)

ImitationNet_Evaluator::ImitationNet_Evaluator() {
  LoadModel();
  InitModel();
}

std::string ImitationNet_Evaluator::GetName() { return "IMITATION_EVALUATOR"; }

std::string ImitationNet_Evaluator::DebugString() { return "TODO"; }

std::vector<std::vector<double>> ImitationNet_Evaluator::Evaluate(
    float inputs1[1 * 5 * 1815]) {
  auto input1_blob = engine_->GetBlob(input1_blob_name_);

  input1_blob->CopyIn(inputs1, 1 * 5 * 1815 * sizeof(float));

  if (!engine_->Infer()) {
    std::cout << "trt engine infer failed!" << std::endl;
  }

  auto output_blob = engine_->GetBlob(output_blob_name_);
  auto shape = output_blob->GetShape();

  std::cout << "start check output shape ..." << std::endl;
  for (auto x : shape) {
    std::cout << "output dim:" << x << std::endl;
  }
  std::cout << "end check output shape ..." << std::endl;
  // for (int i = 0; i< 5*1815; i++) {
  //   std::cout << inputs1[i] << std::endl;
  // }

  auto* ans = static_cast<float*>(output_blob->HostPtr());
  std::vector<std::vector<double>> result;

  for (int i = 0; i < 1; i++) {
    std::vector<double> tmp_res;
    for (int j = 0; j < 7; j++) {
      tmp_res.push_back(ans[i * 1 + j]);
      std::cout << i << "\t" << j << "\t" << ans[i * 1 + j] << std::endl;
    }
    result.push_back(tmp_res);
  }

  std::cout << "结束预测" << std::endl;

  return result;
}

void ImitationNet_Evaluator::LoadModel() {
  auto& model_conf =
      planning_rl::config::PlanningRLConfig::Instance()->model_config();
  auto model_path = model_conf.imitation_learning.model_path;
  engine_.reset(new ZeroCopyTrtEngine(model_path));
  printf("ImitationNet model loaded: {%s}\n", "model_trt.engine");
  LOG_INFO("ImitationNet model loaded: {}\n", model_path);
}

void ImitationNet_Evaluator::InitModel() {
  auto& logger = Logger::GetTRTLogger();

  if (!engine_->Init(logger)) {
    printf("ImitationNet init failed!");
  } else {
    printf("ImitationNet model init: {%s}\n", "model_trt.engine");
  }

  for (int i = 0; i < WARMUP; ++i) {
    engine_->Infer();
  }
}

}  // namespace imitation
}  // namespace neodrive

#include "urban_driver_evaluator.h"
#include <stdio.h>

namespace neodrive {
namespace urbandrivernet {

#define WARMUP (3)

UrbanDriver_Evaluator::UrbanDriver_Evaluator() {
  LoadModel();
  InitModel();
}

std::string UrbanDriver_Evaluator::GetName() { return "URBANDRIVER_EVALUATOR"; }

std::string UrbanDriver_Evaluator::DebugString() { return "TODO"; }

std::vector<std::vector<std::vector<double>>> UrbanDriver_Evaluator::Evaluate(
    float inputs0[1][1][5][18], float inputs1[1][64][5][18], float inputs2[1][100][5][18],
    float inputs3[1][64], float inputs4[1], bool inputs5[1][1][5],
    bool inputs6[1][64][5], bool inputs7[1][100][5]) {
  // std::vector<std::vector<std::vector<double>>> result(1, std::vector<std::vector<double>>(80, std::vector<double>(6, 0.0)));
  // return result;
  auto input0_blob = engine_->GetBlob(input0_blob_name_);
  auto input1_blob = engine_->GetBlob(input1_blob_name_);
  auto input2_blob = engine_->GetBlob(input2_blob_name_);
  auto input3_blob = engine_->GetBlob(input3_blob_name_);
  auto input4_blob = engine_->GetBlob(input4_blob_name_);
  auto input5_blob = engine_->GetBlob(input5_blob_name_);
  auto input6_blob = engine_->GetBlob(input6_blob_name_);
  auto input7_blob = engine_->GetBlob(input7_blob_name_);

  input0_blob->CopyIn(inputs0, 90 * sizeof(float));
  input1_blob->CopyIn(inputs1, 5760 * sizeof(float));
  input2_blob->CopyIn(inputs2, 9000* sizeof(float));
  input3_blob->CopyIn(inputs3, 64 * sizeof(float));
  // input4_blob->CopyIn(inputs4, 1 * sizeof(float));
  input5_blob->CopyIn(inputs5, 5 * sizeof(bool));
  input6_blob->CopyIn(inputs6, 320 * sizeof(bool));
  input7_blob->CopyIn(inputs7, 500 * sizeof(bool));

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
  int vec_size = 80;
  int out_length = 6;
  int out_num = vec_size * out_length;
  std::vector<std::vector<std::vector<double>>> result(1, std::vector<std::vector<double>>(vec_size, std::vector<double>(out_length, 0.0)));
  // return result;
  std::cout << "开始预测" << std::endl;
  for (int i = 0; i < 1; i++) {
    for (int j = 0; j < vec_size; j++) {
      for (int k = 0; k < out_length; k++) {
        // std::cout << j*6+k << std::endl;
        // std::cout << ans[j*6+k] << std::endl;
        // std::cout << ans[i][j][k] << std::endl;
        // std::cout << result[i][j][k] << std::endl;
        result[i][j][k] = double(ans[i * out_num + j * out_length + k]); //i=0
      }
    }
  }
  std::cout << "结束预测" << std::endl;

  return result;
}

void UrbanDriver_Evaluator::LoadModel() {
  auto& model_conf =
      planning_rl::config::PlanningRLConfig::Instance()->model_config();
  auto model_path = model_conf.urban_driver.model_path;
  engine_.reset(new ZeroCopyTrtEngine(model_path));
  printf("urban_driver-net model loaded: {%s}\n", "model_trt.engine");
  LOG_INFO("urban_driver-net model loaded: {}", model_path);
}

void UrbanDriver_Evaluator::InitModel() {
  auto& logger = Logger::GetTRTLogger();

  if (!engine_->Init(logger)) {
    printf("urban_driver-net init failed!");
  } else {
    printf("urban_driver-net model init: {%s}\n", "model_trt.engine");
  }

  for (int i = 0; i < WARMUP; ++i) {
    engine_->Infer();
  }
}

}  // namespace urbandrivernet
}  // namespace neodrive

#pragma once

#include <NvInferRuntime.h>
#include <NvInferRuntimeCommon.h>
#include <cuda_runtime_api.h>

#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace neodrive {
namespace two_stage_net {

#define GPU_CHECK(ans)         \
  do {                         \
    auto code = (ans);         \
    if (code != cudaSuccess) { \
      exit(code);              \
    }                          \
  } while (0)

#define KERNEL_LAUNCH_CHECK()      \
  do {                             \
    GPU_CHECK(cudaGetLastError()); \
  } while (0)

class Logger : public nvinfer1::ILogger {
 public:
  static Logger& GetTRTLogger() noexcept;

  void log(Severity severity, const char* msg) noexcept override {
    printf("%s", msg);
  }

  void SetReportableSeverity(Severity severity) noexcept {
    reportableSeverity_ = severity;
  }

  Severity GetReportableSeverity() const { return reportableSeverity_; }

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

 private:
  Logger(Severity severity = Severity::kWARNING)
      : reportableSeverity_(severity) {}

 private:
  Severity reportableSeverity_;
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T>;

size_t GetTensorSize(const std::vector<int>& shape, nvinfer1::DataType dtype);

class Blob {
 public:
  Blob(size_t size);
  ~Blob() = default;

  void CopyIn(const void* ptr, size_t size);
  void CopyOut(void* ptr, size_t size);
  void* DevPtr();
  void* HostPtr();
  size_t GetSize() const;

 private:
  size_t size_;
  std::shared_ptr<void> ptr_;
};

class TensorBuffer {
 public:
  TensorBuffer(const std::vector<int>& dims, nvinfer1::DataType dtype);
  ~TensorBuffer() = default;

  void* DevPtr();
  void* HostPtr();
  size_t GetSize() const;
  std::vector<int> GetShape() const;
  nvinfer1::DataType GetDtype() const;

  void CopyIn(const void* ptr, size_t size);
  void CopyOut(void* ptr, size_t size);

 private:
  Blob blob_;
  std::vector<int> shape_;
  nvinfer1::DataType dtype_;
};

class ZeroCopyTrtEngine {
 public:
  ZeroCopyTrtEngine(const std::string& engine_file);
  ~ZeroCopyTrtEngine();

  bool Init(Logger& logger);
  bool Infer();

  std::shared_ptr<TensorBuffer> GetBlob(const std::string& name);

  ZeroCopyTrtEngine(const ZeroCopyTrtEngine& other) = delete;
  ZeroCopyTrtEngine& operator=(const ZeroCopyTrtEngine& other) = delete;

 private:
  void GetTensorInfo(int i, std::string& name, std::vector<int>& shape,
                     nvinfer1::DataType& dtype);

 private:
  std::vector<std::shared_ptr<TensorBuffer>> inputs_;
  std::vector<std::shared_ptr<TensorBuffer>> outputs_;
  std::map<std::string, std::shared_ptr<TensorBuffer>> blobs_;

  std::string filename_;
  std::shared_ptr<cudaStream_t> stream_{nullptr};

  TrtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TrtUniquePtr<nvinfer1::IExecutionContext> context_;
  std::vector<void*> bindings_;
};

}  // namespace two_stage_net
}  // namespace neodrive

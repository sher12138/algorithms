#include "trt_engine.h"

#include <fstream>
#include <functional>
#include <numeric>
#include <iostream>

namespace neodrive {
namespace urbandrivernet {

// static auto device_settings = cudaSetDeviceFlags(cudaDeviceMapHost);

static auto StreamDeleter = [](cudaStream_t* pStream) {
  if (pStream) {
    cudaStreamDestroy(*pStream);
    delete pStream;
  }
};

size_t GetTensorSize(const std::vector<int>& shape, nvinfer1::DataType dtype) {
  size_t ans = 1;
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
    case nvinfer1::DataType::kINT32:
      ans = 4;
      break;
    case nvinfer1::DataType::kHALF:
      ans = 2;
      break;
    default:  // kINT8 && kBOOL
      break;
  }

  return accumulate(shape.begin(), shape.end(), ans, std::multiplies<size_t>());
}

Logger& Logger::GetTRTLogger() noexcept {
  static Logger logger;
  return logger;
}

Blob::Blob(size_t size) : size_(size) {
  void* ptr = nullptr;
  GPU_CHECK(cudaMallocManaged(&ptr, size));
  ptr_.reset(ptr, [](void* p) { GPU_CHECK(cudaFree(p)); });
}

void Blob::CopyIn(const void* ptr, size_t size) {
  memcpy((void*)ptr_.get(), ptr, size);
}

void Blob::CopyOut(void* ptr, size_t size) {
  memcpy(ptr, (const void*)ptr_.get(), size);
}

void* Blob::DevPtr() { return ptr_.get(); }

void* Blob::HostPtr() { return ptr_.get(); }

size_t Blob::GetSize() const { return size_; }

TensorBuffer::TensorBuffer(const std::vector<int>& dims,
                           nvinfer1::DataType dtype)
    : blob_(GetTensorSize(dims, dtype)), shape_(dims), dtype_(dtype) {}

void* TensorBuffer::DevPtr() { return blob_.DevPtr(); }

void* TensorBuffer::HostPtr() { return blob_.HostPtr(); }

size_t TensorBuffer::GetSize() const { return blob_.GetSize(); }

std::vector<int> TensorBuffer::GetShape() const { return shape_; }

nvinfer1::DataType TensorBuffer::GetDtype() const { return dtype_; }

void TensorBuffer::CopyIn(const void* ptr, size_t size) {
  blob_.CopyIn(ptr, size);
}

void TensorBuffer::CopyOut(void* ptr, size_t size) { blob_.CopyOut(ptr, size); }

ZeroCopyTrtEngine::ZeroCopyTrtEngine(const std::string& file)
    : filename_(file), stream_(new cudaStream_t(), StreamDeleter) {
  // GPU_CHECK(device_settings);
  GPU_CHECK(cudaStreamCreateWithFlags(stream_.get(), cudaStreamNonBlocking));
}

ZeroCopyTrtEngine::~ZeroCopyTrtEngine() {
  if (context_) context_.release();
  if (engine_) engine_.release();
}

bool ZeroCopyTrtEngine::Init(Logger& logger) {
  std::ifstream engine_file(filename_.c_str(), std::ios::binary);
  if (!engine_file.is_open()) {
    exit(-1);
  }

  engine_file.seekg(0, engine_file.end);
  int64_t fsize = engine_file.tellg();
  engine_file.seekg(0, std::ifstream::beg);

  std::vector<char> engine_bytes(fsize);
  engine_file.read(engine_bytes.data(), fsize);
  engine_file.close();

  TrtUniquePtr<nvinfer1::IRuntime> runtime(
      nvinfer1::createInferRuntime(logger));
  if (!runtime) {
    // LOG_ERROR("create tensorrt runtime failed!");
    return false;
  }
  engine_.reset(runtime->deserializeCudaEngine(engine_bytes.data(), fsize));
  if (!engine_) {
    // LOG_ERROR("deserialize cuda engine failed!");
    return false;
  }

  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    // LOG_ERROR("create context failed!");
    return false;
  }

  for (int i = 0; i < engine_->getNbBindings(); ++i) {
    std::string tensor_name;
    std::vector<int> shape;
    nvinfer1::DataType dtype;
    GetTensorInfo(i, tensor_name, shape, dtype);

    std::shared_ptr<TensorBuffer> blob(new TensorBuffer(shape, dtype));

    bindings_.push_back(blob->DevPtr());
    blobs_.insert({tensor_name, blob});

    bool is_input = engine_->bindingIsInput(i);
    if (is_input) {
      inputs_.emplace_back(blob);
    } else {
      outputs_.emplace_back(blob);
    }
  }

  return true;
}

bool ZeroCopyTrtEngine::Infer() {
  KERNEL_LAUNCH_CHECK();
  bool ret = context_->enqueueV2(bindings_.data(), *stream_, nullptr);
  GPU_CHECK(cudaStreamSynchronize(*stream_));
  return ret;
}

std::shared_ptr<TensorBuffer> ZeroCopyTrtEngine::GetBlob(
    const std::string& name) {
  if (blobs_.end() != blobs_.find(name)) {
    return blobs_.at(name);
  }

  return nullptr;
}

void ZeroCopyTrtEngine::GetTensorInfo(int i, std::string& name,
                                      std::vector<int>& shape,
                                      nvinfer1::DataType& dtype) {
  name = engine_->getBindingName(i);
  nvinfer1::Dims dims = engine_->getBindingDimensions(i);
  shape = std::vector<int>(dims.d, dims.d + dims.nbDims);
  dtype = engine_->getBindingDataType(i);
}

}  // namespace urbandrivernet
}  // namespace neodrive
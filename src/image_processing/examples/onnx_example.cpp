#include <ament_index_cpp/get_package_prefix.hpp>
#include <iostream>
#include <onnxruntime_cxx_api.h>

int main() {
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "pose_demo");
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  session_options.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

// Enable CUDA if available
#ifdef USE_CUDA
  OrtCUDAProviderOptions cuda_options;
  session_options.AppendExecutionProvider_CUDA(cuda_options);
#endif

  // Load model
  auto model_path_str =
      ament_index_cpp::get_package_prefix("onnxruntime") +
      "/models/movenet_singlepose_lightning.onnx";
  const char *model_path = model_path_str.c_str();
  Ort::Session session(env, model_path, session_options);

  std::cout << "Model loaded successfully." << std::endl;
  return 0;
}

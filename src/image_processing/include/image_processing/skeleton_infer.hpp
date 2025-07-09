#pragma once

#include "image_processing/utils.hpp"
#include "onnxruntime_c_api.h"
#include "onnxruntime_cxx_api.h"
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

class SkeletonInfer {
public:
  SkeletonInfer(
      std::string model_path, const char *log_id,
      std::vector<int64_t> tensor_shape, int intra_op_num_threads = 4,
      GraphOptimizationLevel opt_level = GraphOptimizationLevel::ORT_ENABLE_ALL,
      OrtLoggingLevel log_level = ORT_LOGGING_LEVEL_WARNING);
  void load_input(cv::Mat raw_img);
  void run(bool img_output);
  cv::Mat output_image() { return output_img; }
  const std::map<int, skeleton_utils::landmark> &get_landmarks() {
    return landmarks;
  }
  ~SkeletonInfer();

private:
  // Model variables
  std::string model_path;
  Ort::SessionOptions session_options;
  Ort::Env session_environment;
  std::unique_ptr<Ort::Session> session;

  // Tensor infos
  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  size_t tensor_element_count;
  std::vector<int64_t> tensor_shape;
  size_t tensor_shape_len;

  // Inference related vars
  std::vector<std::string> input_names_vec;
  std::vector<std::string> output_names_vec;
  std::vector<const char *> input_names;
  std::vector<const char *> output_names;
  size_t num_input_nodes;
  size_t num_output_nodes;

  cv::Mat input_img;
  std::vector<Ort::Value> input_tensors;
  std::map<int, skeleton_utils::landmark> landmarks;
  cv::Mat output_img;
};

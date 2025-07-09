#include "image_processing/skeleton_infer.hpp"
#include "onnxruntime_c_api.h"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <boost/iostreams/categories.hpp>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <map>
#include <onnxruntime_cxx_api.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

const std::vector<std::pair<int, int>> skeleton = {
    {0, 1},   // Nose -> Left Eye
    {0, 2},   // Nose -> Right Eye
    {1, 3},   // Left Eye -> Left Ear
    {2, 4},   // Right Eye -> Right Ear
    {0, 5},   // Nose -> Left Shoulder
    {0, 6},   // Nose -> Right Shoulder
    {5, 7},   // Left Shoulder -> Left Elbow
    {7, 9},   // Left Elbow -> Left Wrist
    {6, 8},   // Right Shoulder -> Right Elbow
    {8, 10},  // Right Elbow -> Right Wrist
    {5, 6},   // Left Shoulder -> Right Shoulder
    {5, 11},  // Left Shoulder -> Left Hip
    {6, 12},  // Right Shoulder -> Right Hip
    {11, 12}, // Left Hip -> Right Hip
    {11, 13}, // Left Hip -> Left Knee
    {13, 15}, // Left Knee -> Left Ankle
    {12, 14}, // Right Hip -> Right Knee
    {14, 16}  // Right Knee -> Right Ankle
};

void draw_skeleton(cv::Mat &image, float *output_data) {
  const int num_keypoints = 17;
  const int stride = 3; // x, y, score
  std::map<int, cv::Point> landmarks;

  for (int i = 0; i < num_keypoints; ++i) {
    float score = output_data[i * stride + 2];
    if (score > 0.2) {
      float y_norm = output_data[i * stride + 0];
      float x_norm = output_data[i * stride + 1];

      int x = static_cast<int>(x_norm * image.cols);
      int y = static_cast<int>(y_norm * image.rows);

      cv::Point landmark = cv::Point(x, y);
      landmarks[i] = landmark;
      cv::circle(image, landmark, 5, cv::Scalar(0, 255, 0), -1);
    }
  }
  auto end = landmarks.end();
  for (std::pair<int, int> segment : skeleton) {
    if (landmarks.find(segment.first) != end &&
        landmarks.find(segment.second) != end) {
      cv::line(image, landmarks[segment.first], landmarks[segment.second],
               cv::Scalar(0, 0, 255), 3);
    }
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: ./pose_infer <image.jpg>" << std::endl;
    return -1;
  }

  std::string model_path = ament_index_cpp::get_package_prefix("onnxruntime") +
                           "/models/movenet_singlepose_lightning.onnx";
  std::string image_path = argv[1];

  // Load and preprocess image with OpenCV
  cv::Mat raw_img = cv::imread(image_path);
  if (raw_img.empty()) {
    std::cerr << "Failed to read image: " << image_path << std::endl;
    return -1;
  }

  // Resize to model input size, e.g. 256x192
  cv::Mat img;
  cv::resize(raw_img, img, cv::Size(192, 192)); // width x height
  //
  img.convertTo(img, CV_32S);

  // // Initialize ONNX Runtime environment
  // Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "pose_infer");
  // Ort::SessionOptions session_options;
  // session_options.SetIntraOpNumThreads(8);
  // session_options.SetGraphOptimizationLevel(
  //     GraphOptimizationLevel::ORT_ENABLE_ALL);
  // // Enable CUDA (optional, only if built with CUDA)
  //
  // // OrtCUDAProviderOptions cuda_options;
  // // cuda_options.device_id = 0;
  // // cuda_options.arena_extend_strategy = 0;
  // // cuda_options.do_copy_in_default_stream = 1;
  // // cuda_options.cudnn_conv_algo_search =
  // OrtCudnnConvAlgoSearch::OrtCudnnConvAlgoSearchExhaustive;
  // // cuda_options.gpu_mem_limit = SIZE_MAX;
  // //
  // // session_options.EnableProfiling("onnxruntime_profile.json");
  // // session_options.AppendExecutionProvider_CUDA(cuda_options);
  // // std::cout << "Caricato CUDA providers";
  //
  // Ort::Session session(env, model_path.c_str(), session_options);
  //
  //
  // // Prepare input tensor shape: (1, 3, H, W)
  // std::vector<int64_t> input_dims = {1, 192, 192, 3};
  //
  // int channels = 3;
  // int height = 192;
  // int width = 192;
  //
  // // Prepare input data vector
  // std::vector<int32_t> input_tensor_values(1 * channels * height * width);
  //
  // // Create memory info
  // Ort::MemoryInfo memory_info =
  //     Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  //
  // // Create input tensor object from data values
  // Ort::Value input_tensor = Ort::Value::CreateTensor<int32_t>(
  //     memory_info, reinterpret_cast<int32_t *>(img.data),
  //     width * height * channels, input_dims.data(), input_dims.size());
  //
  // // Get input/output node names
  // Ort::AllocatorWithDefaultOptions allocator;
  // Ort::RunOptions run_options{nullptr};
  //
  // std::vector<const char *> input_names;
  // std::vector<const char *> output_names;
  // size_t num_input_nodes = session.GetInputCount();
  // size_t num_output_nodes = session.GetOutputCount();
  // input_names.resize(num_input_nodes);
  // output_names.resize(num_output_nodes);
  //
  // std::vector<std::string> input_names_vec = session.GetInputNames();
  // std::vector<std::string> output_names_vec = session.GetOutputNames();
  //
  // for (size_t i = 0; i < num_input_nodes; i++) {
  //   input_names[i] = input_names_vec[i].c_str();
  // }
  //
  // for (size_t i = 0; i < num_output_nodes; i++) {
  //   output_names[i] = output_names_vec[i].c_str();
  // }
  //
  // std::vector<Ort::Value> input_tensors;
  // input_tensors.emplace_back(std::move(input_tensor));
  //
  // // Run inference
  // auto start = std::chrono::high_resolution_clock::now();
  // std::vector<Ort::Value> output_tensors =
  //     session.Run(run_options, input_names.data(), input_tensors.data(),
  //                 num_input_nodes, output_names.data(), num_output_nodes);
  // auto end = std::chrono::high_resolution_clock::now();
  // auto duration_ns =
  //     std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  // std::cout << "Time passed: " << duration_ns.count() * 1e-9 << std::endl;
  //
  // for (int i = 0; i < 5; i++) {
  //   start = std::chrono::high_resolution_clock::now();
  //   output_tensors =
  //       session.Run(run_options, input_names.data(), input_tensors.data(),
  //                   num_input_nodes, output_names.data(), num_output_nodes);
  //   end = std::chrono::high_resolution_clock::now();
  //   duration_ns =
  //       std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  //   std::cout << "Time passed: " << duration_ns.count() * 1e-9 << std::endl;
  // }
  //
  // // Process output tensor (assumed float, shape depends on model)
  // float *output_data = output_tensors.front().GetTensorMutableData<float>();
  // auto shape_type = output_tensors.front().GetTensorTypeAndShapeInfo();
  // std::cout << "Shape: " << std::endl;
  // for (size_t i = 0; i < shape_type.GetShape().size(); i++) {
  //   std::cout << shape_type.GetShape()[i] << std::endl;
  // }
  // int num_keypoints = shape_type.GetShape()[2];
  // int keypoint_dim = shape_type.GetShape()[3];
  //
  // for (int k = 0; k < num_keypoints; ++k) {
  //   int offset = k * keypoint_dim;
  //   float x = output_data[offset];
  //   float y = output_data[offset + 1];
  //   float score = output_data[offset + 2];
  //
  //   std::cout << "Keypoint " << k + 1 << ": (x=" << x << ", y=" << y
  //             << "), score=" << score * 100.0 << "%" << std::endl;
  // }
  //
  // // Post processing on the image
  // draw_skeleton(raw_img, output_data);
  // cv::imshow("Landmarks", raw_img);
  // cv::waitKey();

  std::vector<int64_t> input_dims = {1, 192, 192, 3};
  SkeletonInfer skeleton_infer{model_path, "skel_infer", input_dims};
  skeleton_infer.load_input(raw_img);
  skeleton_infer.run(true);
  cv::Mat output = skeleton_infer.output_image();
  cv::imshow("Landmarks", output);
  cv::waitKey();
  return 0;
}

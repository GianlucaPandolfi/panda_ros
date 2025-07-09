#include "image_processing/skeleton_infer.hpp"
#include "image_processing/constants.hpp"

SkeletonInfer::SkeletonInfer(std::string model_path, const char *log_id,
                             std::vector<int64_t> tensor_shape,
                             int intra_op_num_threads,
                             GraphOptimizationLevel opt_level,
                             OrtLoggingLevel log_level)
    : model_path(model_path), tensor_shape(tensor_shape) {

  this->session_environment = Ort::Env(log_level, log_id);
  this->session_options.SetIntraOpNumThreads(intra_op_num_threads);
  this->session_options.SetGraphOptimizationLevel(opt_level);
  this->session = std::make_unique<Ort::Session>(this->session_environment,
                                                 this->model_path.c_str(),
                                                 this->session_options);
  std::cout << "Created session" << std::endl;
  tensor_element_count = 1;
  for (int64_t elem : tensor_shape) {
    tensor_element_count *= elem;
  }
  tensor_shape_len = tensor_shape.size();

  this->num_input_nodes = session->GetInputCount();
  this->num_output_nodes = session->GetOutputCount();
  this->input_names.resize(num_input_nodes);
  this->output_names.resize(num_output_nodes);

  input_names_vec = session->GetInputNames();
  output_names_vec = session->GetOutputNames();

  for (size_t i = 0; i < num_input_nodes; i++) {
    this->input_names[i] = input_names_vec[i].c_str();
    std::cout << "Input name " << input_names[i] << std::endl;
  }

  for (size_t i = 0; i < num_output_nodes; i++) {
    this->output_names[i] = output_names_vec[i].c_str();
    std::cout << "Output name " << output_names[i] << std::endl;
  }

  input_tensors.resize(tensor_shape[0]);
  std::cout << "Constructed SkeletonInfer obj" << std::endl;
}

void SkeletonInfer::load_input(cv::Mat raw_img) {
  input_img = raw_img;
  cv::Mat img;
  cv::resize(raw_img, img,
             cv::Size(this->tensor_shape[1], this->tensor_shape[2]));
  img.convertTo(img, CV_32S);
  Ort::Value input_tensor = Ort::Value::CreateTensor<int32_t>(
      memory_info, reinterpret_cast<int32_t *>(img.data),
      this->tensor_element_count, this->tensor_shape.data(),
      this->tensor_shape_len);
  input_tensors.front() = std::move(input_tensor);
  // std::cout << "Loaded image preprocessed" << std::endl;
}

void SkeletonInfer::run(bool img_output) {
  Ort::RunOptions run_options{nullptr};
  // std::cout << "Running infer" << std::endl;
  std::vector<Ort::Value> output_tensors = session->Run(
      run_options, this->input_names.data(), this->input_tensors.data(),
      this->num_input_nodes, this->output_names.data(), this->num_output_nodes);
  // std::cout << "Finished running" << std::endl;
  float *output_data = output_tensors.front().GetTensorMutableData<float>();

  for (int k = 0; k < 17; ++k) {
    int offset = k * 3;
    float y = output_data[offset];
    float x = output_data[offset + 1];
    float score = output_data[offset + 2];
    landmarks[k] = skeleton_utils::landmark{x, y, score};
  }
  if (img_output) {
    output_img = input_img;
    skeleton_utils::draw_skeleton(output_img, landmarks,
                                  image_constants::skeleton, 0.3);
  }
}

SkeletonInfer::~SkeletonInfer() {}

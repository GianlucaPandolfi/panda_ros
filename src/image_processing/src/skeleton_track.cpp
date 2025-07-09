#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_processing/constants.hpp"
#include "image_processing/skeleton_infer.hpp"
#include "image_processing/utils.hpp"
#include "panda_interfaces/msg/double_array_stamped.hpp"
#include "panda_interfaces/msg/double_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <boost/fusion/sequence/intrinsic_fwd.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <optional>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>

template <typename message> using Subscription = rclcpp::Subscription<message>;
using sensor_msgs::msg::Image;

using namespace std::chrono_literals;

template <typename type, int STATES, int OUTPUTS> struct kalman_filt_params {
  // Noise process matrix
  Eigen::Matrix<type, STATES, STATES> Q;
  // Noise output matrix
  Eigen::Matrix<type, OUTPUTS, OUTPUTS> R;
  // State transition matrix
  Eigen::Matrix<type, STATES, STATES> F;
  // Output matrix
  Eigen::Matrix<type, OUTPUTS, STATES> H;
};

template <typename type, int STATES> struct state {
  type x, y, z, x_dot, y_dot, z_dot;
};

enum class TargetState {
  // The Skeleton tracker does not track any person inside the scene, all the
  // states are deinitialized
  NO_PERSON,
  // The skeleton tracker is tracking a person consistently inside the scene,
  // the states are up and running
  PERSON_TRACKED,
  // The skeleton tracker does not have a good confidence in the person
  // previously tracked, due to lost keypoints or obstacle in scene obstructing
  // the person. The tracking continues with some criteria, till the person is
  // completely lost
  PERSON_LOST,
};

class SkeletonTracker : public rclcpp::Node {
public:
  SkeletonTracker(std::string model_path, const char *net_log_id,
                  std::vector<int64_t> net_tensor_shape, double min_depth,
                  double max_depth)
      : Node("skeleton_tracker"), net(model_path, net_log_id, net_tensor_shape),
        min_depth(min_depth), max_depth(max_depth) {

    this->declare_parameter<double>("process_noise", 1.0);
    this->declare_parameter<double>("measurement_noise", 0.5);

    // Initializing kalman filter variables
    kalman_params.Q.setIdentity();
    kalman_params.Q *= this->get_parameter("process_noise").as_double();

    kalman_params.R.setIdentity();
    kalman_params.R *= this->get_parameter("measurement_noise").as_double();

    kalman_params.F.setIdentity();

    kalman_params.H.setZero();
    kalman_params.H(0, 0) = 1.0;
    kalman_params.H(1, 1) = 1.0;
    kalman_params.H(2, 2) = 1.0;

    num_landmarks = 17;

    P_vector.resize(num_landmarks);
    for (auto &P : P_vector) {
      P = kalman_params.Q;
    }

    landmark_3d.resize(num_landmarks);
    for (geometry_msgs::msg::Point &point : landmark_3d) {
      point.x = 0.0;
      point.y = 0.0;
      point.z = 0.0;
    }
    current_state.resize(num_landmarks);
    for (auto &state : current_state) {
      state.setZero();
    }

    last_image_stamp = this->now();

    auto rgb_img_cb = [this](const Image::SharedPtr img) {
      std::lock_guard<std::mutex> img_mutex(images_mutex);
      current_rgb_image = img;
      delta_time =
          rclcpp::Time(current_rgb_image->header.stamp) - last_image_stamp;
      last_image_stamp = current_rgb_image->header.stamp;
    };

    auto depth_img_cb = [this](const Image::SharedPtr img) {
      std::lock_guard<std::mutex> img_mutex(images_mutex);
      current_depth_image = img;
    };

    auto camera_info_cb = [this](const sensor_msgs::msg::CameraInfo msg) {
      current_camera_info = msg;
      image_geom.fromCameraInfo(current_camera_info);
    };

    rgb_image_sub = this->create_subscription<Image>(
        image_constants::rgb_image_topic, image_constants::DEFAULT_TOPIC_QOS,
        rgb_img_cb);

    depth_image_sub = this->create_subscription<Image>(
        image_constants::depth_image_topic, image_constants::DEFAULT_TOPIC_QOS,
        depth_img_cb);

    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        image_constants::camera_info_topic, image_constants::DEFAULT_TOPIC_QOS,
        camera_info_cb);

    skeleton_image_pub =
        std::make_shared<realtime_tools::RealtimePublisher<Image>>(
            this->create_publisher<Image>(image_constants::skeleton_image_topic,
                                          image_constants::DEFAULT_TOPIC_QOS));

    tf_skel_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto timer_cb = [this]() {
      // If these two images are not nullptr
      if (current_rgb_image && current_depth_image) {
        auto start = std::chrono::high_resolution_clock::now();
        // Save current messages to not lose future messages
        Image rgb_img, depth_img;
        {
          std::lock_guard<std::mutex> img_mutex(images_mutex);
          rgb_img = *current_rgb_image;
          depth_img = *current_depth_image;
        }
        cv_bridge::CvImagePtr rgb_img_mat =
            cv_bridge::toCvCopy(rgb_img, "rgb8");
        net.load_input(rgb_img_mat->image);
        net.run(false);
        landmarks = net.get_landmarks();
        std::vector<double> depths =
            calculate_depths(cv_bridge::toCvCopy(depth_img)->image);
        std::vector<std::optional<geometry_msgs::msg::Point>> points =
            project_pixels_to_3d(depths);
        this->kalman_predict(points);
        this->publish_keypoints_tf();
        skel_image_output =
            *(cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8",
                                 this->create_skel_img(rgb_img))
                  .toImageMsg());
        skeleton_image_pub->tryPublish(skel_image_output.value());
        debug_print();

        // Set current images to nullptr
        current_rgb_image = nullptr;
        current_depth_image = nullptr;
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Timer callback time passed: " << duration_ns.count() * 1e-9);
      }
    };

    timer = rclcpp::create_timer(this, this->get_clock(), 1ms, timer_cb);

    // DEBUG

    dbg_innovations.resize(num_landmarks, Eigen::Vector3d::Zero());
    dbg_uncertainties.resize(num_landmarks,
                             Eigen::Matrix<double, 6, 1>::Zero());
    dbg_innovation_pubs.resize(num_landmarks);
    dbg_uncertainty_pubs.resize(num_landmarks);
    dbg_state_pubs.resize(num_landmarks);

    dbg_mean_conf_pub =
        this->create_publisher<panda_interfaces::msg::DoubleStamped>(
            "debug/mean_confidence", 10);

    dbg_all_confs_pub =
        this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
            "debug/all_confidences", 10);

    for (int i = 0; i < num_landmarks; ++i) {
      std::string topic_name =
          "debug/landmark_" + image_constants::coco_keypoints[i];
      dbg_state_pubs[i] =
          this->create_publisher<geometry_msgs::msg::PointStamped>(topic_name,
                                                                   10);
    }

    for (int i = 0; i < num_landmarks; ++i) {
      std::string innovation_topic_name = "debug/landmark_" +
                                          image_constants::coco_keypoints[i] +
                                          "/innovation";
      dbg_innovation_pubs[i] =
          this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
              innovation_topic_name, 10);

      std::string uncertainty_topic_name = "debug/landmark_" +
                                           image_constants::coco_keypoints[i] +
                                           "/uncertainty";
      dbg_uncertainty_pubs[i] =
          this->create_publisher<panda_interfaces::msg::DoubleArrayStamped>(
              uncertainty_topic_name, 10);
    }
  }

private:
  // Subscribers
  Subscription<Image>::SharedPtr rgb_image_sub;
  Subscription<Image>::SharedPtr depth_image_sub;
  Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

  // Publishers
  realtime_tools::RealtimePublisher<Image>::SharedPtr skeleton_image_pub{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_skel_publisher;
  rclcpp::TimerBase::SharedPtr timer;

  // DEBUG PUBLISHERS
  rclcpp::Publisher<panda_interfaces::msg::DoubleStamped>::SharedPtr
      dbg_mean_conf_pub;
  rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr
      dbg_all_confs_pub;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr>
      dbg_state_pubs;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>
      dbg_innovation_pubs;
  std::vector<
      rclcpp::Publisher<panda_interfaces::msg::DoubleArrayStamped>::SharedPtr>
      dbg_uncertainty_pubs;

  // Image processing
  SkeletonInfer net;
  std::mutex images_mutex;
  Image::SharedPtr current_rgb_image;
  Image::SharedPtr current_depth_image;
  sensor_msgs::msg::CameraInfo current_camera_info;
  const std::string camera_frame = "/camera_link";
  double min_depth, max_depth;
  int num_landmarks;
  image_geometry::PinholeCameraModel image_geom;

  // Keypoints state variables
  std::optional<Image> skel_image_output;
  std::map<int, skeleton_utils::landmark> landmarks;
  std::vector<geometry_msgs::msg::Point> landmark_3d;
  kalman_filt_params<double, 6, 3> kalman_params;
  // Vector of prediction matrix
  std::vector<Eigen::Matrix<double, 6, 6>> P_vector;
  // Vector representing internal kalman filter state, the first three are the
  // positions and the last are the velocities
  std::vector<Eigen::Vector<double, 6>> current_state;
  rclcpp::Time last_image_stamp;
  rclcpp::Duration delta_time{0, 0};

  // Debug
  std::vector<Eigen::Vector3d> dbg_innovations;
  std::vector<Eigen::Matrix<double, 6, 1>> dbg_uncertainties;

  cv::Mat create_skel_img(Image background_scene = Image());
  std::vector<std::optional<geometry_msgs::msg::Point>>
  project_pixels_to_3d(std::vector<double> depths);
  void publish_keypoints_tf();
  std::vector<double> calculate_depths(cv::Mat depth_image);
  void
  kalman_predict(std::vector<std::optional<geometry_msgs::msg::Point>> points);
  void debug_print();
};

void SkeletonTracker::debug_print() {
  if (landmarks.empty()) {
    return; // Nothing to publish if no landmarks were detected
  }
  auto now = this->now();
  double total_confidence = 0.0;
  auto confs_msg = panda_interfaces::msg::DoubleArrayStamped();
  confs_msg.header.stamp = now;

  for (const auto &pair : landmarks) {
    total_confidence += pair.second.conf;
    confs_msg.data.push_back(pair.second.conf);
  }

  dbg_all_confs_pub->publish(confs_msg);

  auto mean_conf_msg = panda_interfaces::msg::DoubleStamped();
  mean_conf_msg.header.stamp = now;
  mean_conf_msg.data = total_confidence / landmarks.size();
  dbg_mean_conf_pub->publish(mean_conf_msg);

  for (int i = 0; i < num_landmarks; ++i) {
    auto point_msg = geometry_msgs::msg::PointStamped();
    point_msg.header.stamp = now;

    point_msg.header.frame_id = camera_frame;

    point_msg.point.x = current_state[i](0);
    point_msg.point.y = current_state[i](1);
    point_msg.point.z = current_state[i](2);

    dbg_state_pubs[i]->publish(point_msg);
  }

  for (int i = 0; i < num_landmarks; ++i) {
    // Publish Innovation (Measurement Error)
    auto innovation_msg = geometry_msgs::msg::Vector3Stamped();
    innovation_msg.header.stamp = now;
    innovation_msg.vector.x = dbg_innovations[i](0);
    innovation_msg.vector.y = dbg_innovations[i](1);
    innovation_msg.vector.z = dbg_innovations[i](2);
    dbg_innovation_pubs[i]->publish(innovation_msg);

    auto uncertainty_msg = panda_interfaces::msg::DoubleArrayStamped();
    uncertainty_msg.header.stamp = now;
    uncertainty_msg.data = {dbg_uncertainties[i](0), dbg_uncertainties[i](1),
                            dbg_uncertainties[i](2), dbg_uncertainties[i](3),
                            dbg_uncertainties[i](4), dbg_uncertainties[i](5)};
    dbg_uncertainty_pubs[i]->publish(uncertainty_msg);
  }
}

void SkeletonTracker::kalman_predict(
    std::vector<std::optional<geometry_msgs::msg::Point>> points) {
  // Update F matrix of prediction based on time passed
  kalman_params.F(0, 3) = delta_time.seconds();
  kalman_params.F(1, 4) = delta_time.seconds();
  kalman_params.F(2, 5) = delta_time.seconds();
  for (int i = 0; i < num_landmarks; i++) {
    auto point = points[i];

    // Prediction
    Eigen::Vector<double, 6> pred_state = kalman_params.F * current_state[i];
    Eigen::Matrix<double, 6, 6> pred_P =
        kalman_params.Q +
        kalman_params.F * P_vector[i] * kalman_params.F.transpose();

    Eigen::Matrix<double, 3, 3> S =
        kalman_params.H * pred_P * kalman_params.H.transpose() +
        kalman_params.R;
    Eigen::Matrix<double, 6, 3> K_gain =
        pred_P * kalman_params.H.transpose() * S.inverse();

    // Update
    // If the point inferred is valid i include the measurement, otherwise just
    // predict based on model and current state
    if (point.has_value()) {
      Eigen::Vector3d z;
      z(0) = point->x;
      z(1) = point->y;
      z(2) = point->z;
      dbg_innovations[i] = z - kalman_params.H * pred_state;
      current_state[i] = pred_state + K_gain * dbg_innovations[i];
      P_vector[i] = pred_P - K_gain * S * K_gain.transpose();
    } else {
      dbg_innovations[i].setZero();
      current_state[i] = pred_state;
      P_vector[i] = pred_P;
    }

    dbg_uncertainties[i] = P_vector[i].diagonal();
    landmark_3d[i].x = current_state[i].x();
    landmark_3d[i].y = current_state[i].y();
    landmark_3d[i].z = current_state[i].z();
  }
}

void SkeletonTracker::publish_keypoints_tf() {
  geometry_msgs::msg::TransformStamped tf;
  auto parent_frame = camera_frame;
  tf.header.stamp = this->now();
  tf.header.frame_id = parent_frame;

  for (int i = 0; i < num_landmarks; i++) {

    tf.child_frame_id = image_constants::coco_keypoints[i];

    tf.transform.translation.x = landmark_3d[i].x;
    tf.transform.translation.y = landmark_3d[i].y;
    tf.transform.translation.z = landmark_3d[i].z;

    tf_skel_publisher->sendTransform(tf);
  }
}

std::vector<double> SkeletonTracker::calculate_depths(cv::Mat depth_image) {
  std::vector<double> depths;
  double height, width;
  height = depth_image.rows;
  width = depth_image.cols;

  for (size_t i = 0; i < landmarks.size(); i++) {
    depths.push_back(
        depth_image.at<float>(landmarks[i].y * height, landmarks[i].x * width));
  }

  return depths;
}

std::vector<std::optional<geometry_msgs::msg::Point>>
SkeletonTracker::project_pixels_to_3d(std::vector<double> depths) {

  std::vector<std::optional<geometry_msgs::msg::Point>> points;
  geometry_msgs::msg::Point point_in_optical_frame;
  geometry_msgs::msg::Point point_in_camera_frame;
  int height = image_geom.cameraInfo().height;
  int width = image_geom.cameraInfo().width;
  double depth;
  for (size_t i = 0; i < landmarks.size(); i++) {
    cv::Point2d point;
    point.x = landmarks[i].x * width;
    point.y = landmarks[i].y * height;
    cv::Point3d point_3d = image_geom.projectPixelTo3dRay(point);
    if (depths[i] >= min_depth && depths[i] <= max_depth &&
        !std::isnan(depths[i])) {
      depth = depths[i];
      point_in_optical_frame.x = point_3d.x * depth;
      point_in_optical_frame.y = point_3d.y * depth;
      point_in_optical_frame.z = point_3d.z * depth;

      // Rotation from optical frame to camera frame
      point_in_camera_frame.x = point_in_optical_frame.z;
      point_in_camera_frame.y = -point_in_optical_frame.x;
      point_in_camera_frame.z = -point_in_optical_frame.y;

      points.push_back(std::optional(point_in_camera_frame));
    } else {
      points.push_back(std::optional<geometry_msgs::msg::Point>{});
    }
    // std::cout << "3D point of " << image_constants::coco_keypoints[i] <<
    // ":
    // ("
    //           << ros_point.x << ", " << ros_point.y << ", " <<
    //           ros_point.z
    //           << ")" << std::endl;
  }
  return points;
}
cv::Mat SkeletonTracker::create_skel_img(Image background_scene) {
  // cv::Mat img(640, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  // cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  // cv::Mat img_out;

  cv::Mat img = cv_bridge::toCvCopy(background_scene)->image;
  skeleton_utils::draw_skeleton(img, landmarks, image_constants::skeleton, 0.3);

  // if (background_scene != Image()) {
  //   cv::add(img, cv_bridge::toCvCopy(background_scene)->image, img_out);
  // } else {
  //   img_out = img;
  // }

  return img;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  double min_depth = 0.8;
  double max_depth = 4.0;
  std::string model_path = ament_index_cpp::get_package_prefix("onnxruntime") +
                           "/models/movenet_singlepose_lightning.onnx";
  std::vector<int64_t> input_dims = {1, 192, 192, 3};
  rclcpp::spin(std::make_shared<SkeletonTracker>(
      model_path, "skeleton_tracker", input_dims, min_depth, max_depth));
  rclcpp::shutdown();
  return 0;
}

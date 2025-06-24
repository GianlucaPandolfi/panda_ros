#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp> // Required for cv::imwrite
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageSaverNode : public rclcpp::Node {
public:
  ImageSaverNode() : Node("image_saver_one_shot") {
    RCLCPP_INFO(this->get_logger(), "Node '%s' started.", this->get_name());

    // Declare parameters for topic names and output file paths
    this->declare_parameter<std::string>("rgb_topic", "/rgbd_camera/image");
    this->declare_parameter<std::string>("depth_topic",
                                         "/rgbd_camera/depth_image");
    this->declare_parameter<std::string>("output_rgb_path", "captured_rgb.png");
    this->declare_parameter<std::string>("output_depth_path",
                                         "captured_depth.png");

    // Get parameters
    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    output_rgb_path_ = this->get_parameter("output_rgb_path").as_string();
    output_depth_path_ = this->get_parameter("output_depth_path").as_string();

    // Setup subscribers
    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rgb_topic,
        10, // QoS history depth
        std::bind(&ImageSaverNode::rgb_callback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic,
        10, // QoS history depth
        std::bind(&ImageSaverNode::depth_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing to topics '%s' and '%s'...",
                rgb_topic.c_str(), depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Waiting for images...");

    rgb_saved_ = false;
    depth_saved_ = false;
  }

private:
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (rgb_saved_) {
      return; // Already saved, do nothing
    }

    RCLCPP_INFO(this->get_logger(), "Received RGB image. Saving...");

    cv_bridge::CvImagePtr cv_ptr;
    try {
      // Convert ROS Image message to OpenCV image
      // Assuming RGB is bgr8 encoding for OpenCV display
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

      // Save RGB image (PNG is lossless)
      if (cv::imwrite(output_rgb_path_, cv_ptr->image)) {
        RCLCPP_INFO(this->get_logger(), "Saved RGB image to %s",
                    output_rgb_path_.c_str());
        rgb_saved_ = true; // Set flag

        // Check if both images are saved
        check_and_shutdown();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save RGB image to %s",
                     output_rgb_path_.c_str());
        // Optionally shutdown on save error
        // rclcpp::shutdown();
      }

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s",
                   e.what());
      // Optionally shutdown on conversion error
      // rclcpp::shutdown();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception (RGB): %s",
                   e.what());
      // Optionally shutdown on other errors
      // rclcpp::shutdown();
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred (RGB).");
      // Optionally shutdown on unknown errors
      // rclcpp::shutdown();
    }
  }

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (depth_saved_) {
      return; // Already saved, do nothing
    }

    RCLCPP_INFO(this->get_logger(), "Received Depth image. Saving...");

    cv_bridge::CvImagePtr cv_ptr;
    try {
      // Convert ROS Image message to OpenCV image
      // Use passthrough for depth to preserve original encoding (e.g., mono16,
      // 32FC1) cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
      cv_ptr = cv_bridge::toCvCopy(msg);
      RCLCPP_INFO_STREAM(this->get_logger(), "Image type " << cv_ptr->encoding);
      cv::imwrite("depth.tif", cv_ptr->image);
      // cv::Mat img = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      // cv_ptr->image.convertTo(img, CV_8UC1);
      // std::cout << img;

      // // Save Depth image (PNG is good for preserving 16-bit data, TIFF for
      // // float) Check depth encoding and save appropriately
      // if (cv_ptr->encoding == "mono16") {
      //   // For 16-bit PNG, ensure the image type is CV_16U
      //   if (cv_ptr->image.type() == CV_16U) {
      //     if (cv::imwrite(output_depth_path_, cv_ptr->image)) {
      //       RCLCPP_INFO(this->get_logger(), "Saved 16-bit Depth image to %s",
      //                   output_depth_path_.c_str());
      //     } else {
      //       RCLCPP_ERROR(this->get_logger(),
      //                    "Failed to save 16-bit Depth image to %s",
      //                    output_depth_path_.c_str());
      //     }
      //   } else {
      //     RCLCPP_ERROR(this->get_logger(),
      //                  "Depth image encoding is mono16 but OpenCV type is not
      //                  " "CV_16U (%d). Cannot guarantee 16-bit PNG save.",
      //                  cv_ptr->image.type());
      //     if (cv::imwrite(output_depth_path_,
      //                     cv_ptr->image)) { // Attempt save anyway
      //       RCLCPP_INFO(this->get_logger(),
      //                   "Attempted save of Depth image to %s",
      //                   output_depth_path_.c_str());
      //     } else {
      //       RCLCPP_ERROR(this->get_logger(), "Failed to save Depth image to
      //       %s",
      //                    output_depth_path_.c_str());
      //     }
      //   }
      // } else if (cv_ptr->encoding == "32FC1") {
      //   // For 32-bit float depth, TIFF is a better format than PNG
      //   std::string output_tiff_path = output_depth_path_;
      //   size_t dot_pos = output_tiff_path.rfind('.');
      //   if (dot_pos != std::string::npos) {
      //     output_tiff_path = output_tiff_path.substr(0, dot_pos) + ".tiff";
      //   } else {
      //     output_tiff_path += ".tiff";
      //   }
      //   RCLCPP_WARN(this->get_logger(), "Depth image encoding is 32FC1.
      //   Saving "
      //                                   "as TIFF to preserve float data.");
      //   if (cv::imwrite(output_depth_path_, cv_ptr->image)) {
      //     RCLCPP_INFO(this->get_logger(),
      //                 "Saved 32-bit Float Depth image to %s",
      //                 output_tiff_path.c_str());
      //   } else {
      //     RCLCPP_ERROR(this->get_logger(),
      //                  "Failed to save 32-bit Float Depth image to %s",
      //                  output_tiff_path.c_str());
      //   }
      //
      // } else {
      //   // Handle other potential depth encodings or save as default PNG
      //   RCLCPP_WARN(this->get_logger(),
      //               "Unknown Depth image encoding: %s. Attempting to save as
      //               " "standard PNG.", cv_ptr->encoding.c_str());
      //   if (cv::imwrite(output_depth_path_, cv_ptr->image)) {
      //     RCLCPP_INFO(this->get_logger(), "Saved Depth image to %s",
      //                 output_depth_path_.c_str());
      //   } else {
      //     RCLCPP_ERROR(this->get_logger(), "Failed to save Depth image to
      //     %s",
      //                  output_depth_path_.c_str());
      //   }
      // }

      depth_saved_ = true; // Set flag

      // Check if both images are saved
      check_and_shutdown();

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Depth): %s",
                   e.what());
      // Optionally shutdown on conversion error
      // rclcpp::shutdown();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception (Depth): %s",
                   e.what());
      // Optionally shutdown on other errors
      // rclcpp::shutdown();
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred (Depth).");
      // Optionally shutdown on unknown errors
      // rclcpp::shutdown();
    }
  }

  void check_and_shutdown() {
    if (rgb_saved_ && depth_saved_) {
      RCLCPP_INFO(this->get_logger(),
                  "Both images saved. Shutting down node...");
      rclcpp::shutdown(); // Shutdown the ROS 2 context
    }
  }

  // Member variables
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  cv_bridge::CvImage bridge_; // CvBridge instance is needed for conversion

  bool rgb_saved_;
  bool depth_saved_;

  std::string output_rgb_path_;
  std::string output_depth_path_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Create the node and spin it. It will stop spinning when rclcpp::shutdown()
  // is called.
  rclcpp::spin(std::make_shared<ImageSaverNode>());
  // rclcpp::shutdown(); // This will be called from within the node's callback
  return 0;
}

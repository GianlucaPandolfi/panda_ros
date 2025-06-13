#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

template<typename message>
using Subscription = rclcpp::Subscription<message>;
class ImageProcessor : public rclcpp::Node {
public:
  ImageProcessor() : Node("image_processor_example") {
    auto color_image_cb = [this](const sensor_msgs::msg::Image msg){
      color_image = cv_bridge::toCvCopy(msg)->image;
    };
    auto depth_image_cb = [this](const sensor_msgs::msg::Image msg){
      depth_image = cv_bridge::toCvCopy(msg)->image;
    };
  }

private: 
  Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub;
  Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub;
  Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
  cv::Mat color_image;
  cv::Mat depth_image;
};

int main(int argc, char **argv) { return 0; }

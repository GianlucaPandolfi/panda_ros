#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "panda_interfaces/srv/forward_kine.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <cstddef>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/event_handler.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <unistd.h>
#include <vector>
#include "panda_utils/constants.hpp"

struct DHParam {
  double a;
  double alpha;
  double d;
};

std::vector<DHParam> panda_dh = {
    {0.0, -M_PI_2, 0.3330},  {0.0, M_PI_2, 0.0},    {0.0825, M_PI_2, 0.3160},
    {-0.0825, -M_PI_2, 0.0}, {0.0, M_PI_2, 0.3840}, {0.0880, M_PI_2, 0.0},
    {0.0, 0.0, 0.1070}};

// Computes the homogeneous transformation following the standard DH convention
Eigen::Isometry3d dh_transform(DHParam dhparam, double theta) {
  Eigen::Matrix4d T;
  Eigen::Isometry3d iso;
  T << cos(theta), -sin(theta) * cos(dhparam.alpha),
      sin(theta) * sin(dhparam.alpha), dhparam.a * cos(theta),

      sin(theta), cos(theta) * cos(dhparam.alpha),
      -cos(theta) * sin(dhparam.alpha), sin(theta) * dhparam.a,

      0.0, sin(dhparam.alpha), cos(dhparam.alpha), dhparam.d,

      0, 0, 0, 1;
  iso = T;
  return iso;
}

geometry_msgs::msg::TransformStamped
create_tf_stamped(DHParam dhparam, double joint_value, rclcpp::Time now) {

  geometry_msgs::msg::TransformStamped tf_stamped;

  auto tf = dh_transform(dhparam, joint_value);
  geometry_msgs::msg::Transform transform;

  tf_stamped.header.stamp = now;
  transform.translation.x = tf.translation().x();
  transform.translation.y = tf.translation().y();
  transform.translation.z = tf.translation().z();

  Eigen::Quaterniond quat{tf.rotation()};
  quat.normalize();

  transform.rotation.w = quat.w();
  transform.rotation.x = quat.x();
  transform.rotation.y = quat.y();
  transform.rotation.z = quat.z();
  tf_stamped.transform = transform;

  return tf_stamped;
}

class ForwardKineServer : public rclcpp::Node {

public:
  ForwardKineServer() : Node("forward_kine_server") {
    const int service_qos{10};

    auto calculate_forward_kine =
        [this](const std::shared_ptr<panda_interfaces::srv::ForwardKine_Request>
                   request,
               std::shared_ptr<panda_interfaces::srv::ForwardKine_Response>
                   response) {
          // Get joint values and time as of now
          auto joints = request->joints.joint_values;
          auto now = this->get_clock()->now();
          RCLCPP_DEBUG(this->get_logger(), "Got joint values");

          // Construct transforms message
          for (size_t i = 0; i < joints.size(); i++) {
            response->tfs.transforms.push_back(
                create_tf_stamped(panda_dh[i], joints[i], now));
          }
          RCLCPP_DEBUG(this->get_logger(), "Filled transforms");

          return response;
        };
    service = this->create_service<panda_interfaces::srv::ForwardKine>(
        panda_interface_names::forward_kine_service_name, calculate_forward_kine, service_qos);
    RCLCPP_INFO_ONCE(this->get_logger(), "Created service %s",
                     panda_interface_names::forward_kine_service_name.c_str());
  }

private:
  rclcpp::Service<panda_interfaces::srv::ForwardKine>::SharedPtr service;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForwardKineServer>());
  rclcpp::shutdown();
  return 0;
}
